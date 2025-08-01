// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2019-2025, The OpenROAD Authors

#include "cts/TritonCTS.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <ctime>
#include <fstream>
#include <functional>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

#include "Clock.h"
#include "CtsOptions.h"
#include "HTreeBuilder.h"
#include "LevelBalancer.h"
#include "TechChar.h"
#include "TreeBuilder.h"
#include "db_sta/dbNetwork.hh"
#include "db_sta/dbSta.hh"
#include "odb/db.h"
#include "odb/dbShape.h"
#include "rsz/Resizer.hh"
#include "sta/Fuzzy.hh"
#include "sta/Graph.hh"
#include "sta/GraphDelayCalc.hh"
#include "sta/Liberty.hh"
#include "sta/PathAnalysisPt.hh"
#include "sta/PathEnd.hh"
#include "sta/PathExpanded.hh"
#include "sta/PatternMatch.hh"
#include "sta/Sdc.hh"
#include "utl/Logger.h"

namespace cts {

using utl::CTS;

void TritonCTS::init(utl::Logger* logger,
                     odb::dbDatabase* db,
                     sta::dbNetwork* network,
                     sta::dbSta* sta,
                     stt::SteinerTreeBuilder* st_builder,
                     rsz::Resizer* resizer)
{
  logger_ = logger;
  db_ = db;
  network_ = network;
  openSta_ = sta;
  resizer_ = resizer;

  options_ = new CtsOptions(logger_, st_builder);
}

TritonCTS::TritonCTS() = default;

TritonCTS::~TritonCTS()
{
  delete options_;
}

void TritonCTS::runTritonCts()
{
  odb::dbChip* chip = db_->getChip();
  odb::dbBlock* block = chip->getBlock();
  options_->addOwner(block);

  setupCharacterization();
  findClockRoots();
  populateTritonCTS();
  if (builders_.empty()) {
    logger_->warn(CTS, 82, "No valid clock nets in the design.");
  } else {
    checkCharacterization();
    buildClockTrees();
    writeDataToDb();
    setAllClocksPropagated();
    if (options_->getRepairClockNets()) {
      repairClockNets();
    }
    balanceMacroRegisterLatencies();
  }

  // reset
  techChar_.reset();
  builders_.clear();
  staClockNets_.clear();
  visitedClockNets_.clear();
  inst2clkbuf_.clear();
  driver2subnet_.clear();
  numberOfClocks_ = 0;
  numClkNets_ = 0;
  numFixedNets_ = 0;
  dummyLoadIndex_ = 0;
  rootBuffers_.clear();
  sinkBuffers_.clear();
  regTreeRootBufIndex_ = 0;
  delayBufIndex_ = 0;
  options_->removeOwner();
}

TreeBuilder* TritonCTS::addBuilder(CtsOptions* options,
                                   Clock& net,
                                   odb::dbNet* topInputNet,
                                   TreeBuilder* parent,
                                   utl::Logger* logger,
                                   odb::dbDatabase* db)
{
  auto builder
      = std::make_unique<HTreeBuilder>(options, net, parent, logger, db);

  builder->setTopInputNet(topInputNet);
  builders_.emplace_back(std::move(builder));
  return builders_.back().get();
}

int TritonCTS::getBufferFanoutLimit(const std::string& bufferName)
{
  int fanout = std::numeric_limits<int>::max();
  float tempFanout;
  bool existMaxFanout;
  odb::dbMaster* bufferMaster = db_->findMaster(bufferName.c_str());
  sta::Cell* bufferCell = network_->dbToSta(bufferMaster);
  sta::Port* buffer_port = nullptr;
  for (odb::dbMTerm* mterm : bufferMaster->getMTerms()) {
    odb::dbSigType sig_type = mterm->getSigType();
    if (sig_type == odb::dbSigType::GROUND
        || sig_type == odb::dbSigType::POWER) {
      continue;
    }
    odb::dbIoType io_type = mterm->getIoType();
    if (io_type == odb::dbIoType::OUTPUT) {
      buffer_port = network_->dbToSta(mterm);
      break;
    }
  }
  if (buffer_port == nullptr) {
    return 0;
  }

  openSta_->sdc()->fanoutLimit(
      buffer_port, sta::MinMax::max(), tempFanout, existMaxFanout);
  if (existMaxFanout) {
    fanout = std::min(fanout, (int) tempFanout);
  }

  openSta_->sdc()->fanoutLimit(
      bufferCell, sta::MinMax::max(), tempFanout, existMaxFanout);
  if (existMaxFanout) {
    fanout = std::min(fanout, (int) tempFanout);
  }

  sta::LibertyPort* port = network_->libertyPort(buffer_port);
  port->fanoutLimit(sta::MinMax::max(), tempFanout, existMaxFanout);
  if (existMaxFanout) {
    fanout = std::min(fanout, (int) tempFanout);
  } else {
    port->libertyLibrary()->defaultMaxFanout(tempFanout, existMaxFanout);
    if ((existMaxFanout)) {
      fanout = std::min(fanout, (int) tempFanout);
    }
  }
  return fanout;
}

void TritonCTS::setupCharacterization()
{
  // Check if CTS library is valid
  if (options_->isCtsLibrarySet()) {
    sta::Library* lib = network_->findLibrary(options_->getCtsLibrary());
    if (lib == nullptr) {
      logger_->error(CTS,
                     209,
                     "Library {} cannot be found because it is not "
                     "loaded or name is incorrect",
                     options_->getCtsLibrary());
    } else {
      logger_->info(CTS,
                    210,
                    "Clock buffers will be chosen from library {}",
                    options_->getCtsLibrary());
    }
  }

  openSta_->checkFanoutLimitPreamble();
  // Finalize root/sink buffers
  std::string rootBuffer = selectRootBuffer(rootBuffers_);
  options_->setRootBuffer(rootBuffer);
  std::string sinkBuffer = selectSinkBuffer(sinkBuffers_);
  options_->setSinkBuffer(sinkBuffer);

  int sinkMaxFanout = getBufferFanoutLimit(sinkBuffer);
  int rootMaxFanout = getBufferFanoutLimit(rootBuffer);

  if (rootMaxFanout && (options_->getNumMaxLeafSinks() > rootMaxFanout)) {
    options_->setMaxFanout(rootMaxFanout);
  }

  if (sinkMaxFanout) {
    if (options_->getSinkClusteringSize() > sinkMaxFanout) {
      options_->setSinkClusteringSize(sinkMaxFanout);
    }

    if (sinkMaxFanout < options_->getMaxFanout()) {
      options_->setMaxFanout(sinkMaxFanout);
    }
  }

  // A new characteriztion is always created.
  techChar_ = std::make_unique<TechChar>(
      options_, db_, openSta_, resizer_, network_, logger_);
  techChar_->create();

  // Also resets metrics everytime the setup is done
  options_->setNumSinks(0);
  options_->setNumBuffersInserted(0);
  options_->setNumClockRoots(0);
  options_->setNumClockSubnets(0);
}

void TritonCTS::checkCharacterization()
{
  std::unordered_set<std::string> visitedMasters;
  techChar_->forEachWireSegment([&](unsigned idx, const WireSegment& wireSeg) {
    for (int buf = 0; buf < wireSeg.getNumBuffers(); ++buf) {
      const std::string& master = wireSeg.getBufferMaster(buf);
      if (visitedMasters.count(master) == 0) {
        if (masterExists(master)) {
          visitedMasters.insert(master);
        } else {
          logger_->error(CTS, 81, "Buffer {} is not in the loaded DB.", master);
        }
      }
    }
  });

  logger_->info(CTS,
                97,
                "Characterization used {} buffer(s) types.",
                visitedMasters.size());
}

void TritonCTS::findClockRoots()
{
  if (!options_->getClockNets().empty()) {
    logger_->info(CTS,
                  1,
                  "Running TritonCTS with user-specified clock roots: {}.",
                  options_->getClockNets());
  }
}

void TritonCTS::buildClockTrees()
{
  for (auto& builder : builders_) {
    builder->setTechChar(*techChar_);
    builder->setDb(db_);
    builder->setLogger(logger_);
    builder->initBlockages();
    builder->run();
  }

  if (options_->getBalanceLevels()) {
    for (auto& builder : builders_) {
      if (!builder->getParent()
          && !builder->getChildren().empty()
          // don't balance levels for macro cell tree
          && builder->getTreeType() != TreeType::MacroTree) {
        LevelBalancer balancer(
            builder.get(), options_, logger_, techChar_->getLengthUnit());
        balancer.run();
      }
    }
  }
}

void TritonCTS::initOneClockTree(odb::dbNet* driverNet,
                                 odb::dbNet* clkInputNet,
                                 const std::string& sdcClockName,
                                 TreeBuilder* parent)
{
  TreeBuilder* clockBuilder = nullptr;
  if (driverNet->isSpecial()) {
    logger_->info(
        CTS, 116, "Special net \"{}\" skipped.", driverNet->getName());
  } else {
    clockBuilder = initClock(driverNet, clkInputNet, sdcClockName, parent);
  }
  // Treat gated clocks as separate clock trees
  // TODO: include sinks from gated clocks together with other sinks and build
  // one clock tree
  visitedClockNets_.insert(driverNet);
  odb::dbITerm* driver = driverNet->getFirstOutput();
  odb::dbSet<odb::dbITerm> iterms = driverNet->getITerms();
  for (odb::dbITerm* iterm : iterms) {
    if (iterm != driver && iterm->isInputSignal()) {
      if (!isSink(iterm)) {
        odb::dbITerm* outputPin = getSingleOutput(iterm->getInst(), iterm);
        if (outputPin && outputPin->getNet()) {
          odb::dbNet* outputNet = outputPin->getNet();
          if (visitedClockNets_.find(outputNet) == visitedClockNets_.end()
              && !openSta_->sdc()->isLeafPinClock(
                  network_->dbToSta(outputPin))) {
            initOneClockTree(
                outputNet, clkInputNet, sdcClockName, clockBuilder);
          }
        }
      }
    }
  }
}

void TritonCTS::countSinksPostDbWrite(
    TreeBuilder* builder,
    odb::dbNet* net,
    unsigned& sinks_cnt,
    unsigned& leafSinks,
    unsigned currWireLength,
    double& sinkWireLength,
    int& minDepth,
    int& maxDepth,
    int depth,
    bool fullTree,
    const std::unordered_set<odb::dbITerm*>& sinks,
    const std::unordered_set<odb::dbInst*>& dummies)
{
  odb::dbSet<odb::dbITerm> iterms = net->getITerms();
  int driverX = 0;
  int driverY = 0;
  for (odb::dbITerm* iterm : iterms) {
    if (iterm->getIoType() != odb::dbIoType::INPUT) {
      iterm->getAvgXY(&driverX, &driverY);
      break;
    }
  }
  odb::dbSet<odb::dbBTerm> bterms = net->getBTerms();
  for (odb::dbBTerm* bterm : bterms) {
    if (bterm->getIoType() == odb::dbIoType::INPUT) {
      for (odb::dbBPin* pin : bterm->getBPins()) {
        odb::dbPlacementStatus status = pin->getPlacementStatus();
        if (status == odb::dbPlacementStatus::NONE
            || status == odb::dbPlacementStatus::UNPLACED) {
          continue;
        }
        for (odb::dbBox* box : pin->getBoxes()) {
          if (box) {
            driverX = box->xMin();
            driverY = box->yMin();
            break;
          }
        }
        break;
      }
    }
  }
  for (odb::dbITerm* iterm : iterms) {
    if (iterm->getIoType() == odb::dbIoType::INPUT) {
      std::string name = iterm->getInst()->getName();
      int receiverX, receiverY;
      iterm->getAvgXY(&receiverX, &receiverY);
      unsigned dist = abs(driverX - receiverX) + abs(driverY - receiverY);
      odb::dbInst* inst = iterm->getInst();
      bool terminate = fullTree
                           ? (sinks.find(iterm) != sinks.end())
                           : !builder->isAnyTreeBuffer(getClockFromInst(inst));
      odb::dbITerm* outputPin = iterm->getInst()->getFirstOutput();
      bool trueSink = true;
      if (outputPin && outputPin->getNet() == net) {
        // Skip feedback loop.  When input pin and output pin are
        // connected to the same net this can lead to infinite recursion. For
        // example, some designs have Q pin connected to SI pin.
        terminate = true;
        trueSink = false;
      }

      if (!terminate && inst) {
        if (inst->isBlock()) {
          // Skip non-sink macro blocks
          terminate = true;
          trueSink = false;
        } else {
          sta::Cell* masterCell = network_->dbToSta(inst->getMaster());
          if (masterCell) {
            sta::LibertyCell* libCell = network_->libertyCell(masterCell);
            if (libCell) {
              if (libCell->hasSequentials()) {
                // Skip non-sink registers
                terminate = true;
                trueSink = false;
              }
            }
          }
        }
      }

      if (!terminate) {
        // ignore dummy buffer and inverters added to balance loads
        if (outputPin && outputPin->getNet() != nullptr) {
          countSinksPostDbWrite(builder,
                                outputPin->getNet(),
                                sinks_cnt,
                                leafSinks,
                                (currWireLength + dist),
                                sinkWireLength,
                                minDepth,
                                maxDepth,
                                depth + 1,
                                fullTree,
                                sinks,
                                dummies);
        } else {
          std::string cellType = "Complex cell";
          odb::dbInst* inst = iterm->getInst();
          sta::Cell* masterCell = network_->dbToSta(inst->getMaster());
          if (masterCell) {
            sta::LibertyCell* libCell = network_->libertyCell(masterCell);
            if (libCell) {
              if (libCell->isInverter()) {
                cellType = "Inverter";
              } else if (libCell->isBuffer()) {
                cellType = "Buffer";
              }
            }
          }

          if (dummies.find(inst) == dummies.end()) {
            logger_->info(CTS,
                          121,
                          "{} '{}' has unconnected output pin.",
                          cellType,
                          name);
          }
        }
        if (builder->isLeafBuffer(getClockFromInst(iterm->getInst()))) {
          leafSinks++;
        }
      } else if (trueSink) {
        sinks_cnt++;
        double currSinkWl
            = (dist + currWireLength) / double(options_->getDbUnits());
        sinkWireLength += currSinkWl;
        if (depth > maxDepth) {
          maxDepth = depth;
        }
        if ((minDepth > 0 && depth < minDepth) || (minDepth == 0)) {
          minDepth = depth;
        }
      }
    }
  }  // ignoring block pins/feedthrus
}

ClockInst* TritonCTS::getClockFromInst(odb::dbInst* inst)
{
  auto it = inst2clkbuf_.find(inst);
  return it != inst2clkbuf_.end() ? it->second : nullptr;
}

void TritonCTS::writeDataToDb()
{
  std::set<odb::dbNet*> clkLeafNets;
  std::unordered_set<odb::dbInst*> clkDummies;

  for (auto& builder : builders_) {
    writeClockNetsToDb(builder.get(), clkLeafNets);
    if (options_->applyNDR()) {
      writeClockNDRsToDb(clkLeafNets);
    }
    if (options_->dummyLoadEnabled()) {
      writeDummyLoadsToDb(builder->getClock(), clkDummies);
    }
  }

  for (auto& builder : builders_) {
    odb::dbNet* topClockNet = builder->getClock().getNetObj();
    unsigned sinkCount = 0;
    unsigned leafSinks = 0;
    double allSinkDistance = 0.0;
    int minDepth = 0;
    int maxDepth = 0;
    bool reportFullTree = !builder->getParent()
                          && !builder->getChildren().empty()
                          && options_->getBalanceLevels();

    std::unordered_set<odb::dbITerm*> sinks;
    builder->getClock().forEachSink([&sinks](const ClockInst& inst) {
      sinks.insert(inst.getDbInputPin());
    });
    if (sinks.size() < 2) {
      logger_->info(
          CTS, 124, "Clock net \"{}\"", builder->getClock().getName());
      logger_->info(CTS, 125, " Sinks {}", sinks.size());
    } else {
      countSinksPostDbWrite(builder.get(),
                            topClockNet,
                            sinkCount,
                            leafSinks,
                            0,
                            allSinkDistance,
                            minDepth,
                            maxDepth,
                            0,
                            reportFullTree,
                            sinks,
                            clkDummies);
      logger_->info(CTS, 98, "Clock net \"{}\"", builder->getClock().getName());
      logger_->info(CTS, 99, " Sinks {}", sinkCount);
      logger_->info(CTS, 100, " Leaf buffers {}", leafSinks);
      if (sinkCount > 0) {
        double avgWL = allSinkDistance / sinkCount;
        logger_->info(CTS, 101, " Average sink wire length {:.2f} um", avgWL);
      }
      logger_->info(CTS, 102, " Path depth {} - {}", minDepth, maxDepth);
      if (options_->dummyLoadEnabled()) {
        logger_->info(CTS, 207, " Leaf load cells {}", dummyLoadIndex_);
      }
    }
  }
}

void TritonCTS::forEachBuilder(
    const std::function<void(const TreeBuilder*)>& func) const
{
  for (const auto& builder : builders_) {
    func(builder.get());
  }
}

void TritonCTS::reportCtsMetrics()
{
  std::string filename = options_->getMetricsFile();

  if (!filename.empty()) {
    std::ofstream file(filename.c_str());

    if (!file.is_open()) {
      logger_->error(
          CTS, 87, "Could not open output metric file {}.", filename.c_str());
    }

    file << "Total number of Clock Roots: " << options_->getNumClockRoots()
         << ".\n";
    file << "Total number of Buffers Inserted: "
         << options_->getNumBuffersInserted() << ".\n";
    file << "Total number of Clock Subnets: " << options_->getNumClockSubnets()
         << ".\n";
    file << "Total number of Sinks: " << options_->getNumSinks() << ".\n";

    file << "Buffers used:\n";
    for (const auto& [master, count] : options_->getBufferCount()) {
      file << "  " << master->getName() << ": " << count << "\n";
    }
    if (!options_->getDummyCount().empty()) {
      file << "Dummys used:\n";
      for (const auto& [master, count] : options_->getDummyCount()) {
        file << "  " << master->getName() << ": " << count << "\n";
      }
    }
    file.close();

  } else {
    logger_->report("Total number of Clock Roots: {}.",
                    options_->getNumClockRoots());
    logger_->report("Total number of Buffers Inserted: {}.",
                    options_->getNumBuffersInserted());
    logger_->report("Total number of Clock Subnets: {}.",
                    options_->getNumClockSubnets());
    logger_->report("Total number of Sinks: {}.", options_->getNumSinks());

    logger_->report("Cells used:");
    for (const auto& [master, count] : options_->getBufferCount()) {
      logger_->report("  {}: {}", master->getName(), count);
    }
    if (!options_->getDummyCount().empty()) {
      logger_->report("Dummys used:");
      for (const auto& [master, count] : options_->getDummyCount()) {
        logger_->report("  {}: {}", master->getName(), count);
      }
    }
  }
}

int TritonCTS::setClockNets(const char* names)
{
  odb::dbChip* chip = db_->getChip();
  odb::dbBlock* block = chip->getBlock();

  options_->setClockNets(names);
  std::stringstream ss(names);
  std::istream_iterator<std::string> begin(ss);
  std::istream_iterator<std::string> end;
  std::vector<std::string> nets(begin, end);

  std::vector<odb::dbNet*> netObjects;

  for (const std::string& name : nets) {
    odb::dbNet* net = block->findNet(name.c_str());
    bool netFound = false;
    if (net != nullptr) {
      // Since a set is unique, only the nets not found by dbSta are added.
      netObjects.push_back(net);
      netFound = true;
    } else {
      // User input was a pin, transform it into an iterm if possible
      odb::dbITerm* iterm = block->findITerm(name.c_str());
      if (iterm != nullptr) {
        net = iterm->getNet();
        if (net != nullptr) {
          // Since a set is unique, only the nets not found by dbSta are added.
          netObjects.push_back(net);
          netFound = true;
        }
      }
    }
    if (!netFound) {
      return 1;
    }
  }
  options_->setClockNetsObjs(netObjects);
  return 0;
}

void TritonCTS::setBufferList(const char* buffers)
{
  std::stringstream ss(buffers);
  std::istream_iterator<std::string> begin(ss);
  std::istream_iterator<std::string> end;
  std::vector<std::string> bufferList(begin, end);
  if (bufferList.empty()) {
    inferBufferList(bufferList);
  } else {
    for (const std::string& buffer : bufferList) {
      if (db_->findMaster(buffer.c_str()) == nullptr) {
        logger_->error(
            CTS, 126, "No physical master cell found for buffer {}.", buffer);
      }
    }
  }
  options_->setBufferList(bufferList);
}

void TritonCTS::inferBufferList(std::vector<std::string>& buffers)
{
  sta::Vector<sta::LibertyCell*> selected_buffers;

  // first, look for buffers with "is_clock_cell: true" cell attribute
  sta::LibertyLibraryIterator* lib_iter = network_->libertyLibraryIterator();
  while (lib_iter->hasNext()) {
    sta::LibertyLibrary* lib = lib_iter->next();
    if (options_->isCtsLibrarySet()
        && strcmp(lib->name(), options_->getCtsLibrary()) != 0) {
      continue;
    }
    for (sta::LibertyCell* buffer : *lib->buffers()) {
      if (buffer->isClockCell() && isClockCellCandidate(buffer)) {
        // "is_clock_cell: true"
        selected_buffers.emplace_back(buffer);
        debugPrint(logger_,
                   CTS,
                   "buffering",
                   1,
                   "{} has clock cell attribute",
                   buffer->name());
      }
    }
  }
  delete lib_iter;

  // second, look for buffers with an input port that has
  // LEF USE as "CLOCK"
  if (selected_buffers.empty()) {
    sta::LibertyLibraryIterator* lib_iter = network_->libertyLibraryIterator();
    while (lib_iter->hasNext()) {
      sta::LibertyLibrary* lib = lib_iter->next();
      if (options_->isCtsLibrarySet()
          && strcmp(lib->name(), options_->getCtsLibrary()) != 0) {
        continue;
      }
      for (sta::LibertyCell* buffer : *lib->buffers()) {
        odb::dbMaster* master = db_->findMaster(buffer->name());
        for (odb::dbMTerm* mterm : master->getMTerms()) {
          if (mterm->getIoType() == odb::dbIoType::INPUT
              && mterm->getSigType() == odb::dbSigType::CLOCK
              && isClockCellCandidate(buffer)) {
            // input port with LEF USE as "CLOCK"
            selected_buffers.emplace_back(buffer);
            debugPrint(logger_,
                       CTS,
                       "buffering",
                       1,
                       "{} has input port {} with LEF USE as CLOCK",
                       buffer->name(),
                       mterm->getName());
          }
        }
      }
    }
    delete lib_iter;
  }

  // third, look for all buffers with name CLKBUF or clkbuf
  if (selected_buffers.empty()) {
    sta::PatternMatch patternClkBuf(".*CLKBUF.*",
                                    /* is_regexp */ true,
                                    /* nocase */ true,
                                    /* Tcl_interp* */ nullptr);
    sta::LibertyLibraryIterator* lib_iter = network_->libertyLibraryIterator();
    while (lib_iter->hasNext()) {
      sta::LibertyLibrary* lib = lib_iter->next();
      if (options_->isCtsLibrarySet()
          && strcmp(lib->name(), options_->getCtsLibrary()) != 0) {
        continue;
      }
      for (sta::LibertyCell* buffer :
           lib->findLibertyCellsMatching(&patternClkBuf)) {
        if (buffer->isBuffer() && isClockCellCandidate(buffer)) {
          debugPrint(logger_,
                     CTS,
                     "buffering",
                     1,
                     "{} found by 'CLKBUF' pattern match",
                     buffer->name());
          selected_buffers.emplace_back(buffer);
        }
      }
    }
    delete lib_iter;
  }

  // fourth, look for all buffers with name BUF or buf
  if (selected_buffers.empty()) {
    sta::PatternMatch patternBuf(".*BUF.*",
                                 /* is_regexp */ true,
                                 /* nocase */ true,
                                 /* Tcl_interp* */ nullptr);
    lib_iter = network_->libertyLibraryIterator();
    while (lib_iter->hasNext()) {
      sta::LibertyLibrary* lib = lib_iter->next();
      if (options_->isCtsLibrarySet()
          && strcmp(lib->name(), options_->getCtsLibrary()) != 0) {
        continue;
      }
      for (sta::LibertyCell* buffer :
           lib->findLibertyCellsMatching(&patternBuf)) {
        if (buffer->isBuffer() && isClockCellCandidate(buffer)) {
          debugPrint(logger_,
                     CTS,
                     "buffering",
                     1,
                     "{} found by 'BUF' pattern match",
                     buffer->name());
          selected_buffers.emplace_back(buffer);
        }
      }
    }
    delete lib_iter;
  }

  // abandon attributes & name patterns, just look for all buffers
  if (selected_buffers.empty()) {
    debugPrint(logger_,
               CTS,
               "buffering",
               1,
               "No buffers with clock atributes or name patterns found, using "
               "all buffers");
    lib_iter = network_->libertyLibraryIterator();
    while (lib_iter->hasNext()) {
      sta::LibertyLibrary* lib = lib_iter->next();
      if (options_->isCtsLibrarySet()
          && strcmp(lib->name(), options_->getCtsLibrary()) != 0) {
        continue;
      }
      for (sta::LibertyCell* buffer : *lib->buffers()) {
        if (isClockCellCandidate(buffer)) {
          selected_buffers.emplace_back(buffer);
        }
      }
    }
    delete lib_iter;

    if (selected_buffers.empty()) {
      logger_->error(
          CTS,
          110,
          "No clock buffer candidates could be found from any libraries.");
    }
  }

  resizer_->setClockBuffersList(selected_buffers);

  for (sta::LibertyCell* buffer : selected_buffers) {
    buffers.emplace_back(buffer->name());
    debugPrint(logger_,
               CTS,
               "buffering",
               1,
               "{} has been inferred as clock buffer",
               buffer->name());
  }

  options_->setBufferListInferred(true);
}

std::string toLowerCase(std::string str)
{
  std::transform(str.begin(), str.end(), str.begin(), [](unsigned char c) {
    return std::tolower(c);
  });
  return str;
}

bool TritonCTS::isClockCellCandidate(sta::LibertyCell* cell)
{
  return (!cell->dontUse() && !resizer_->dontUse(cell) && !cell->alwaysOn()
          && !cell->isIsolationCell() && !cell->isLevelShifter());
}

void TritonCTS::setRootBuffer(const char* buffers)
{
  std::stringstream ss(buffers);
  std::istream_iterator<std::string> begin(ss);
  std::istream_iterator<std::string> end;
  std::vector<std::string> bufferList(begin, end);
  for (const std::string& buffer : bufferList) {
    if (db_->findMaster(buffer.c_str()) == nullptr) {
      logger_->error(
          CTS, 127, "No physical master cell found for buffer {}.", buffer);
    }
  }
  rootBuffers_ = std::move(bufferList);
}

std::string TritonCTS::selectRootBuffer(std::vector<std::string>& buffers)
{
  // if -root_buf is not specified, choose from the buffer list
  if (buffers.empty()) {
    buffers = options_->getBufferList();
  }

  if (buffers.size() == 1) {
    return buffers.front();
  }

  options_->setRootBufferInferred(true);
  // estimate wire cap for root buffer
  // assume sink buffer needs to drive clk buffers at two far ends of chip
  // at midpoint
  //
  //  --------------
  //  |      .     |
  //  |   ===x===  |
  //  |      .     |
  //  --------------
  odb::dbBlock* block = db_->getChip()->getBlock();
  odb::Rect coreArea = block->getCoreArea();
  float sinkWireLength
      = static_cast<float>(std::max(coreArea.dx(), coreArea.dy()))
        / block->getDbUnitsPerMicron();
  sta::Corner* corner = openSta_->cmdCorner();
  float rootWireCap
      = resizer_->wireSignalCapacitance(corner) * 1e-6 * sinkWireLength / 2.0;
  std::string rootBuf = selectBestMaxCapBuffer(buffers, rootWireCap);
  return rootBuf;
}

void TritonCTS::setSinkBuffer(const char* buffers)
{
  std::stringstream ss(buffers);
  std::istream_iterator<std::string> begin(ss);
  std::istream_iterator<std::string> end;
  std::vector<std::string> bufferList(begin, end);
  sinkBuffers_ = std::move(bufferList);
}

std::string TritonCTS::selectSinkBuffer(std::vector<std::string>& buffers)
{
  // if -sink_clustering_buf is not specified, choose from the buffer list
  if (buffers.empty()) {
    buffers = options_->getBufferList();
  }

  if (buffers.size() == 1) {
    return buffers.front();
  }

  options_->setSinkBufferInferred(true);
  // estimate wire cap for sink buffer
  // assume sink buffer needs to drive clk buffers at two far ends of chip
  // to account for unknown pin caps
  //
  //  --------------
  //  |======x=====|
  //  |      .     |
  //  |----- .-----|
  //  |      .     |
  //  --------------
  odb::dbBlock* block = db_->getChip()->getBlock();
  odb::Rect coreArea = block->getCoreArea();
  float sinkWireLength
      = static_cast<float>(std::max(coreArea.dx(), coreArea.dy()))
        / block->getDbUnitsPerMicron();
  sta::Corner* corner = openSta_->cmdCorner();
  float sinkWireCap
      = resizer_->wireSignalCapacitance(corner) * 1e-6 * sinkWireLength;

  std::string sinkBuf = selectBestMaxCapBuffer(buffers, sinkWireCap);
  // clang-format off
  debugPrint(logger_, CTS, "buffering", 1, "{} has been selected as sink "
             "buffer to drive sink wire cap of {:0.2e}", sinkBuf, sinkWireCap);
  // clang-format on
  return sinkBuf;
}

// pick the smallest buffer that can drive total cap
// if no such buffer exists, pick one that has the largest max cap
std::string TritonCTS::selectBestMaxCapBuffer(
    const std::vector<std::string>& buffers,
    float totalCap)
{
  std::string bestBuf, nextBestBuf;
  float bestArea = std::numeric_limits<float>::max();
  float bestCap = 0.0;

  for (const std::string& name : buffers) {
    odb::dbMaster* master = db_->findMaster(name.c_str());
    if (master == nullptr) {
      logger_->error(
          CTS, 117, "Physical master could not be found for cell '{}'", name);
    }
    sta::Cell* masterCell = network_->dbToSta(master);
    sta::LibertyCell* libCell = network_->libertyCell(masterCell);
    if (libCell == nullptr) {
      logger_->error(
          CTS, 112, "Liberty cell could not be found for cell '{}'", name);
    }
    sta::LibertyPort *in, *out;
    libCell->bufferPorts(in, out);
    float area = libCell->area();
    float maxCap = 0.0;
    bool maxCapExists = false;
    out->capacitanceLimit(sta::MinMax::max(), maxCap, maxCapExists);
    // clang-format off
    debugPrint(logger_, CTS, "buffering", 1, "{} has cap limit:{}"
               " vs. total cap:{}, derate:{}", name,
               maxCap * options_->getSinkBufferMaxCapDerate(), totalCap,
               options_->getSinkBufferMaxCapDerate());
    // clang-format on
    if (maxCapExists
        && ((maxCap * options_->getSinkBufferMaxCapDerate()) > totalCap)
        && area < bestArea) {
      bestBuf = name;
      bestArea = area;
    }
    if (maxCap > bestCap) {
      nextBestBuf = name;
      bestCap = maxCap;
    }
  }

  if (bestBuf.empty()) {
    bestBuf = std::move(nextBestBuf);
  }

  return bestBuf;
}

// db functions

void TritonCTS::populateTritonCTS()
{
  block_ = db_->getChip()->getBlock();
  options_->setDbUnits(block_->getDbUnitsPerMicron());

  clearNumClocks();

  // Use dbSta to find all clock nets in the design.
  std::vector<std::pair<std::set<odb::dbNet*>, std::string>> clockNetsInfo;

  // Checks the user input in case there are other nets that need to be added to
  // the set.
  std::vector<odb::dbNet*> inputClkNets = options_->getClockNetsObjs();

  if (!inputClkNets.empty()) {
    std::set<odb::dbNet*> clockNets;
    for (odb::dbNet* net : inputClkNets) {
      // Since a set is unique, only the nets not found by dbSta are added.
      clockNets.insert(net);
    }
    clockNetsInfo.emplace_back(clockNets, "");
  } else {
    std::set<odb::dbNet*> allClkNets;
    staClockNets_ = openSta_->findClkNets();
    sta::Sdc* sdc = openSta_->sdc();
    for (auto clk : *sdc->clocks()) {
      std::string clkName = clk->name();
      std::set<odb::dbNet*> clkNets;
      findClockRoots(clk, clkNets);
      for (auto net : clkNets) {
        if (allClkNets.find(net) != allClkNets.end()) {
          logger_->error(
              CTS, 114, "Clock {} overlaps a previous clock.", clkName);
        }
      }
      clockNetsInfo.emplace_back(clkNets, clkName);
      allClkNets.insert(clkNets.begin(), clkNets.end());
    }
  }

  // Iterate over all the nets found by the user-input and dbSta
  for (const auto& clockInfo : clockNetsInfo) {
    std::set<odb::dbNet*> clockNets = clockInfo.first;
    std::string clkName = clockInfo.second;
    for (odb::dbNet* net : clockNets) {
      if (net != nullptr) {
        if (clkName.empty()) {
          logger_->info(CTS, 95, "Net \"{}\" found.", net->getName());
        } else {
          logger_->info(CTS,
                        7,
                        "Net \"{}\" found for clock \"{}\".",
                        net->getName(),
                        clkName);
        }
        // Initializes the net in TritonCTS. If the number of sinks is less than
        // 2, the net is discarded.
        if (visitedClockNets_.find(net) == visitedClockNets_.end()) {
          initOneClockTree(net, net, clkName, nullptr);
        }
      } else {
        logger_->warn(
            CTS,
            40,
            "Net was not found in the design for {}, please check. Skipping...",
            clkName);
      }
    }
  }

  if (getNumClocks() == 0) {
    logger_->warn(CTS, 83, "No clock nets have been found.");
  }

  logger_->info(CTS, 8, "TritonCTS found {} clock nets.", getNumClocks());
  options_->setNumClockRoots(getNumClocks());
}

TreeBuilder* TritonCTS::initClock(odb::dbNet* firstNet,
                                  odb::dbNet* clkInputNet,
                                  const std::string& sdcClock,
                                  TreeBuilder* parentBuilder)
{
  std::string driver;
  odb::dbITerm* iterm = firstNet->getFirstOutput();
  int xPin, yPin;
  if (iterm == nullptr) {
    odb::dbBTerm* bterm = firstNet->get1stBTerm();  // Clock pin
    if (bterm == nullptr) {
      logger_->info(
          CTS,
          122,
          "Clock net \"{}\" is skipped for CTS because it is not "
          "connected to any output instance pin or input block terminal.",
          firstNet->getName());
      return nullptr;
    }
    driver = bterm->getConstName();
    bterm->getFirstPinLocation(xPin, yPin);
  } else {
    odb::dbInst* inst = iterm->getInst();
    odb::dbMTerm* mterm = iterm->getMTerm();
    driver = std::string(inst->getConstName()) + "/"
             + std::string(mterm->getConstName());
    int xTmp, yTmp;
    computeITermPosition(iterm, xTmp, yTmp);
    xPin = xTmp;
    yPin = yTmp;
  }

  // Initialize clock net
  Clock clockNet(firstNet->getConstName(), driver, sdcClock, xPin, yPin);
  clockNet.setDriverPin(iterm);

  // Build a set of all the clock buffers' masters
  std::unordered_set<odb::dbMaster*> buffer_masters;
  for (const std::string& name : options_->getBufferList()) {
    auto master = db_->findMaster(name.c_str());
    if (master) {
      buffer_masters.insert(master);
    }
  }

  // Add the root buffer
  {
    const std::string& name = options_->getRootBuffer();
    auto master = db_->findMaster(name.c_str());
    if (master) {
      buffer_masters.insert(master);
    }
  }

  // Build a clock tree to drive macro cells with insertion delays
  // separated from registers or leaves without insertion delays
  TreeBuilder* builder = initClockTreeForMacrosAndRegs(
      firstNet, clkInputNet, buffer_masters, clockNet, parentBuilder);
  return builder;
}

// Build a separate clock tree to pull macro cells with insertion delays
// ahead of cells without insertion delays.  If sinks consist of
// both macros and FFs, clock tree for macros is built first.  A new net and a
// new buffer are created to drive cells without insertion delays.   New
// buffer will be sized later based on macro cell insertion delays.
//
//                |----|>----[] cells with insertion delays
//       firstNet |
//            |   |----|>----[]
//            v   |
//   [root]-------|                  |---|>----[] cells without insertion
//                |
//                |----|>------------|
//                      ^        ^   |
//                      |        |   |
//               new buffer secondNet|---|>----[]
//
TreeBuilder* TritonCTS::initClockTreeForMacrosAndRegs(
    odb::dbNet*& firstNet,
    odb::dbNet* clkInputNet,
    const std::unordered_set<odb::dbMaster*>& buffer_masters,
    Clock& clockNet,
    TreeBuilder* parentBuilder)
{
  // Separate sinks into two buckets: one with insertion delays and another
  // without
  std::vector<std::pair<odb::dbInst*, odb::dbMTerm*>> macroSinks;
  std::vector<std::pair<odb::dbInst*, odb::dbMTerm*>> registerSinks;
  if (!separateMacroRegSinks(
          firstNet, clockNet, buffer_masters, registerSinks, macroSinks)) {
    return nullptr;
  }

  if (!options_->insertionDelayEnabled() || macroSinks.empty()
      || registerSinks.empty()) {
    // There is no need for separate clock trees
    for (odb::dbITerm* iterm : firstNet->getITerms()) {
      odb::dbInst* inst = iterm->getInst();
      if (iterm->isInputSignal() && inst->isPlaced()) {
        odb::dbMTerm* mterm = iterm->getMTerm();
        std::string name = std::string(inst->getConstName()) + "/"
                           + std::string(mterm->getConstName());
        int x, y;
        computeITermPosition(iterm, x, y);
        float insDelay = computeInsertionDelay(name, inst, mterm);
        clockNet.addSink(name, x, y, iterm, getInputPinCap(iterm), insDelay);
      }
    }
    if (clockNet.getNumSinks() < 2) {
      logger_->warn(CTS,
                    41,
                    "Net \"{}\" has {} sinks. Skipping...",
                    clockNet.getName(),
                    clockNet.getNumSinks());
      return nullptr;
    }
    logger_->info(CTS,
                  10,
                  " Clock net \"{}\" has {} sinks.",
                  firstNet->getConstName(),
                  clockNet.getNumSinks());
    int totalSinks = options_->getNumSinks() + clockNet.getNumSinks();
    options_->setNumSinks(totalSinks);
    incrementNumClocks();
    clockNet.setNetObj(firstNet);
    return addBuilder(
        options_, clockNet, clkInputNet, parentBuilder, logger_, db_);
  }

  // add macro sinks to existing firstNet
  TreeBuilder* firstBuilder = addClockSinks(
      clockNet, clkInputNet, firstNet, macroSinks, parentBuilder, "macros");
  if (firstBuilder) {
    firstBuilder->setTreeType(TreeType::MacroTree);
  }

  // create a new net 'secondNet' to drive register sinks
  odb::dbNet* secondNet;
  std::string topBufferName;
  Clock clockNet2 = forkRegisterClockNetwork(
      clockNet, registerSinks, firstNet, secondNet, topBufferName);

  // add register sinks to secondNet
  TreeBuilder* secondBuilder
      = addClockSinks(clockNet2,
                      clkInputNet,
                      secondNet,
                      registerSinks,
                      firstBuilder ? firstBuilder : parentBuilder,
                      "registers");
  if (secondBuilder) {
    secondBuilder->setTreeType(TreeType::RegisterTree);
    secondBuilder->setTopBufferName(std::move(topBufferName));
    secondBuilder->setDrivingNet(firstNet);
  }

  return firstBuilder;
}

// Separate sinks into registers (no insertion delay) and macros (insertion
// delay)
bool TritonCTS::separateMacroRegSinks(
    odb::dbNet*& net,
    Clock& clockNet,
    const std::unordered_set<odb::dbMaster*>& buffer_masters,
    std::vector<std::pair<odb::dbInst*, odb::dbMTerm*>>& registerSinks,
    std::vector<std::pair<odb::dbInst*, odb::dbMTerm*>>& macroSinks)
{
  for (odb::dbITerm* iterm : net->getITerms()) {
    odb::dbInst* inst = iterm->getInst();

    if (buffer_masters.find(inst->getMaster()) != buffer_masters.end()
        && inst->getSourceType() == odb::dbSourceType::TIMING) {
      logger_->warn(CTS,
                    105,
                    "Net \"{}\" already has clock buffer {}. Skipping...",
                    clockNet.getName(),
                    inst->getName());
      return false;
    }

    if (iterm->isInputSignal() && inst->isPlaced()) {
      odb::dbMTerm* mterm = iterm->getMTerm();
      // Treat clock gaters like macro sink
      if (hasInsertionDelay(inst, mterm) || !isSink(iterm) || inst->isBlock()) {
        macroSinks.emplace_back(inst, mterm);
      } else {
        registerSinks.emplace_back(inst, mterm);
      }
    }
  }

  return true;
}

TreeBuilder* TritonCTS::addClockSinks(
    Clock& clockNet,
    odb::dbNet* topInputNet,
    odb::dbNet* physicalNet,
    const std::vector<std::pair<odb::dbInst*, odb::dbMTerm*>>& sinks,
    TreeBuilder* parentBuilder,
    const std::string& macrosOrRegs)
{
  for (auto elem : sinks) {
    odb::dbInst* inst = elem.first;
    odb::dbMTerm* mterm = elem.second;
    std::string name = std::string(inst->getConstName()) + "/"
                       + std::string(mterm->getConstName());
    int x, y;
    odb::dbITerm* iterm = inst->getITerm(mterm);
    computeITermPosition(iterm, x, y);
    float insDelay = computeInsertionDelay(name, inst, mterm);
    clockNet.addSink(name, x, y, iterm, getInputPinCap(iterm), insDelay);
  }
  logger_->info(CTS,
                11,
                " Clock net \"{}\" for {} has {} sinks.",
                physicalNet->getConstName(),
                macrosOrRegs,
                clockNet.getNumSinks());
  int totalSinks = options_->getNumSinks() + clockNet.getNumSinks();
  options_->setNumSinks(totalSinks);
  incrementNumClocks();
  clockNet.setNetObj(physicalNet);
  return addBuilder(
      options_, clockNet, topInputNet, parentBuilder, logger_, db_);
}

Clock TritonCTS::forkRegisterClockNetwork(
    Clock& clockNet,
    const std::vector<std::pair<odb::dbInst*, odb::dbMTerm*>>& registerSinks,
    odb::dbNet*& firstNet,
    odb::dbNet*& secondNet,
    std::string& topBufferName)
{
  // create a new clock net to drive register sinks
  std::string newClockName = clockNet.getName() + "_regs";
  secondNet = odb::dbNet::create(block_, newClockName.c_str());
  secondNet->setSigType(odb::dbSigType::CLOCK);

  sta::Pin* first_pin_driver = nullptr;
  odb::dbModule* first_net_module = network_->getNetDriverParentModule(
      network_->dbToSta(firstNet), first_pin_driver);
  (void) first_pin_driver;
  sta::Pin* second_pin_driver = nullptr;
  odb::dbModule* second_net_module = network_->getNetDriverParentModule(
      network_->dbToSta(secondNet), second_pin_driver);
  (void) second_pin_driver;
  odb::dbModule* target_module = nullptr;
  if ((first_net_module != nullptr)
      && (first_net_module == second_net_module)) {
    target_module = first_net_module;
  }

  // move register sinks from previous clock net to new clock net
  for (auto elem : registerSinks) {
    odb::dbInst* inst = elem.first;
    odb::dbMTerm* mterm = elem.second;
    odb::dbITerm* iterm = inst->getITerm(mterm);
    iterm->disconnect();
    iterm->connect(secondNet);
  }

  // create a new clock buffer
  odb::dbMaster* master = db_->findMaster(options_->getRootBuffer().c_str());
  topBufferName = "clkbuf_regs_" + std::to_string(regTreeRootBufIndex_++) + "_"
                  + clockNet.getSdcName();
  odb::dbInst* clockBuf = odb::dbInst::create(
      block_, master, topBufferName.c_str(), false, target_module);

  // place new clock buffer near center of mass for registers
  odb::Rect bbox = secondNet->getTermBBox();
  clockBuf->setSourceType(odb::dbSourceType::TIMING);
  clockBuf->setLocation(bbox.xCenter(), bbox.yCenter());
  clockBuf->setPlacementStatus(odb::dbPlacementStatus::PLACED);

  // connect root buffer to clock net
  odb::dbITerm* inputTerm = getFirstInput(clockBuf);
  odb::dbITerm* outputTerm = clockBuf->getFirstOutput();
  inputTerm->connect(firstNet);
  outputTerm->connect(secondNet);

  // initialize new clock net
  std::string driver = std::string(clockBuf->getConstName()) + "/"
                       + std::string(outputTerm->getMTerm()->getConstName());
  int xPin, yPin;
  computeITermPosition(outputTerm, xPin, yPin);
  Clock clockNet2(
      secondNet->getConstName(), driver, clockNet.getSdcName(), xPin, yPin);
  clockNet2.setDriverPin(outputTerm);

  return clockNet2;
}

void TritonCTS::computeITermPosition(odb::dbITerm* term, int& x, int& y) const
{
  odb::dbITermShapeItr itr;

  odb::dbShape shape;
  x = 0;
  y = 0;
  unsigned numShapes = 0;
  for (itr.begin(term); itr.next(shape);) {
    if (!shape.isVia()) {
      x += shape.xMin() + (shape.xMax() - shape.xMin()) / 2;
      y += shape.yMin() + (shape.yMax() - shape.yMin()) / 2;
      ++numShapes;
    }
  }
  if (numShapes > 0) {
    x /= numShapes;
    y /= numShapes;
  }
};

void TritonCTS::writeClockNetsToDb(TreeBuilder* builder,
                                   std::set<odb::dbNet*>& clkLeafNets)
{
  Clock& clockNet = builder->getClock();
  odb::dbNet* topClockNet = clockNet.getNetObj();
  // gets the module for the driver for the net
  sta::Pin* pin_driver = nullptr;
  odb::dbModule* top_module = network_->getNetDriverParentModule(
      network_->dbToSta(topClockNet), pin_driver);
  (void) pin_driver;

  disconnectAllSinksFromNet(topClockNet);

  // re-connect top buffer that separates macros from registers
  if (builder->getTreeType() == TreeType::RegisterTree) {
    odb::dbInst* topRegBuffer
        = block_->findInst(builder->getTopBufferName().c_str());
    if (topRegBuffer) {
      odb::dbITerm* topRegBufferInputPin = getFirstInput(topRegBuffer);
      topRegBufferInputPin->connect(builder->getDrivingNet());
    }
  }

  createClockBuffers(clockNet, top_module);

  // connect top buffer on the clock pin
  std::string topClockInstName = "clkbuf_0_" + clockNet.getName();
  odb::dbInst* topClockInst = block_->findInst(topClockInstName.c_str());
  odb::dbITerm* topClockInstInputPin = getFirstInput(topClockInst);
  topClockInstInputPin->connect(topClockNet);
  topClockNet->setSigType(odb::dbSigType::CLOCK);

  std::map<int, odb::uint> fanoutcount;

  // create subNets
  numClkNets_ = 0;
  numFixedNets_ = 0;
  ClockSubNet* rootSubNet = nullptr;
  std::set<ClockInst*> removedSinks;
  clockNet.forEachSubNet([&](ClockSubNet& subNet) {
    bool outputPinFound = true;
    bool inputPinFound = true;
    bool leafLevelNet = subNet.isLeafLevel();
    if (("clknet_0_" + clockNet.getName()) == subNet.getName()) {
      rootSubNet = &subNet;
    }
    odb::dbNet* clkSubNet
        = odb::dbNet::create(block_, subNet.getName().c_str());

    ++numClkNets_;
    clkSubNet->setSigType(odb::dbSigType::CLOCK);

    odb::dbInst* driver = subNet.getDriver()->getDbInst();
    odb::dbITerm* driverInputPin = getFirstInput(driver);
    odb::dbNet* inputNet = driverInputPin->getNet();
    odb::dbITerm* outputPin = driver->getFirstOutput();
    if (outputPin == nullptr) {
      outputPinFound = false;
    }
    if (outputPinFound) {
      outputPin->connect(clkSubNet);
    }

    if (subNet.getNumSinks() == 0) {
      inputPinFound = false;
    }

    subNet.forEachSink([&](ClockInst* inst) {
      odb::dbITerm* inputPin = nullptr;
      if (inst->isClockBuffer()) {
        odb::dbInst* sink = inst->getDbInst();
        inputPin = getFirstInput(sink);
      } else {
        inputPin = inst->getDbInputPin();
      }
      if (inputPin == nullptr) {
        inputPinFound = false;
      } else {
        if (!inputPin->getInst()->isPlaced()) {
          inputPinFound = false;
        }
      }

      if (inputPinFound) {
        inputPin->connect(clkSubNet);
        // get module for input pin
        // resolve connection in hierarchy
        if (network_->hasHierarchy()) {
          network_->hierarchicalConnect(
              outputPin, inputPin, clkSubNet->getName().c_str());
        }
      }
    });

    if (leafLevelNet) {
      // Report fanout values only for sink nets
      if (fanoutcount.find(subNet.getNumSinks()) == fanoutcount.end()) {
        fanoutcount[subNet.getNumSinks()] = 0;
      }
      fanoutcount[subNet.getNumSinks()] = fanoutcount[subNet.getNumSinks()] + 1;
      clkLeafNets.insert(clkSubNet);
    }

    if (!inputPinFound || !outputPinFound) {
      // Net not fully connected. Removing it.
      disconnectAllPinsFromNet(clkSubNet);
      odb::dbNet::destroy(clkSubNet);
      ++numFixedNets_;
      --numClkNets_;
      odb::dbInst::destroy(driver);
      removedSinks.insert(subNet.getDriver());
      checkUpstreamConnections(inputNet);
    }
  });

  if (!rootSubNet) {
    logger_->error(
        CTS, 85, "Could not find the root of {}", clockNet.getName());
  }

  int minPath = std::numeric_limits<int>::max();
  int maxPath = std::numeric_limits<int>::min();
  rootSubNet->forEachSink([&](ClockInst* inst) {
    // skip removed sinks
    if (removedSinks.find(inst) == removedSinks.end()) {
      if (inst->isClockBuffer()) {
        std::pair<int, int> resultsForBranch
            = branchBufferCount(inst, 1, clockNet);
        if (resultsForBranch.first < minPath) {
          minPath = resultsForBranch.first;
        }
        if (resultsForBranch.second > maxPath) {
          maxPath = resultsForBranch.second;
        }
      }
    } else {
      rootSubNet->removeSinks(removedSinks);
    }
  });

  logger_->info(
      CTS, 12, "    Minimum number of buffers in the clock path: {}.", minPath);
  logger_->info(
      CTS, 13, "    Maximum number of buffers in the clock path: {}.", maxPath);

  if (numFixedNets_ > 0) {
    logger_->info(
        CTS, 14, "    {} clock nets were removed/fixed.", numFixedNets_);
  }

  logger_->info(CTS, 15, "    Created {} clock nets.", numClkNets_);
  int totalNets = options_->getNumClockSubnets() + numClkNets_;
  options_->setNumClockSubnets(totalNets);

  std::string fanout;
  for (auto const& x : fanoutcount) {
    fanout += std::to_string(x.first) + ':' + std::to_string(x.second) + ", ";
  }

  logger_->info(CTS,
                16,
                "    Fanout distribution for the current clock = {}.",
                fanout.substr(0, fanout.size() - 2) + ".");
  logger_->info(
      CTS, 17, "    Max level of the clock tree: {}.", clockNet.getMaxLevel());
}

void TritonCTS::writeClockNDRsToDb(const std::set<odb::dbNet*>& clkLeafNets)
{
  char ruleName[64];
  int ruleIndex = 0;
  odb::dbTechNonDefaultRule* clockNDR;

  // create a new non-default rule in *block* not tech
  while (ruleIndex >= 0) {
    snprintf(ruleName, 64, "CTS_NDR_%d", ruleIndex++);
    clockNDR = odb::dbTechNonDefaultRule::create(block_, ruleName);
    if (clockNDR) {
      break;
    }
  }
  assert(clockNDR != nullptr);

  // define NDR for all routing layers
  odb::dbTech* tech = db_->getTech();
  for (int i = 1; i <= tech->getRoutingLayerCount(); i++) {
    odb::dbTechLayer* layer = tech->findRoutingLayer(i);
    odb::dbTechLayerRule* layerRule = clockNDR->getLayerRule(layer);
    if (!layerRule) {
      layerRule = odb::dbTechLayerRule::create(clockNDR, layer);
    }
    assert(layerRule != nullptr);
    int defaultSpace = layer->getSpacing();
    int defaultWidth = layer->getWidth();
    layerRule->setSpacing(defaultSpace * 2);
    layerRule->setWidth(defaultWidth);
    // clang-format off
    debugPrint(logger_, CTS, "clustering", 1, "  NDR rule set to layer {} {} as "
	       "space={} width={} vs. default space={} width={}",
	       i, layer->getName(),
	       layerRule->getSpacing(), layerRule->getWidth(),
	       defaultSpace, defaultWidth);
    // clang-format on
  }

  // apply NDR to all non-leaf clock nets
  int clkNets = 0;
  for (odb::dbNet* net : block_->getNets()) {
    if (net->getSigType() == odb::dbSigType::CLOCK
        && (clkLeafNets.find(net) == clkLeafNets.end())) {
      net->setNonDefaultRule(clockNDR);
      clkNets++;
    }
  }

  logger_->info(CTS,
                202,
                "Non-default rule {} for double spacing has been applied to {} "
                "clock nets",
                ruleName,
                clkNets);
}

std::pair<int, int> TritonCTS::branchBufferCount(ClockInst* inst,
                                                 int bufCounter,
                                                 Clock& clockNet)
{
  odb::dbInst* sink = inst->getDbInst();
  odb::dbITerm* outITerm = sink->getFirstOutput();
  int minPath = std::numeric_limits<int>::max();
  int maxPath = std::numeric_limits<int>::min();
  for (odb::dbITerm* sinkITerms : outITerm->getNet()->getITerms()) {
    if (sinkITerms != outITerm) {
      ClockInst* clockInst
          = clockNet.findClockByName(sinkITerms->getInst()->getName());
      if (clockInst == nullptr) {
        int newResult = bufCounter + 1;
        if (newResult > maxPath) {
          maxPath = newResult;
        }
        if (newResult < minPath) {
          minPath = newResult;
        }
      } else {
        std::pair<int, int> newResults
            = branchBufferCount(clockInst, bufCounter + 1, clockNet);
        if (newResults.first < minPath) {
          minPath = newResults.first;
        }
        if (newResults.second > maxPath) {
          maxPath = newResults.second;
        }
      }
    }
  }
  std::pair<int, int> results(minPath, maxPath);
  return results;
}

void TritonCTS::disconnectAllSinksFromNet(odb::dbNet* net)
{
  odb::dbSet<odb::dbITerm> iterms = net->getITerms();
  for (odb::dbITerm* iterm : iterms) {
    if (iterm->getIoType() == odb::dbIoType::INPUT) {
      iterm->disconnect();
    }
  }
}

void TritonCTS::disconnectAllPinsFromNet(odb::dbNet* net)
{
  odb::dbSet<odb::dbITerm> iterms = net->getITerms();
  for (odb::dbITerm* iterm : iterms) {
    iterm->disconnect();
  }
}

void TritonCTS::checkUpstreamConnections(odb::dbNet* net)
{
  while (net->getITermCount() <= 1) {
    // Net is incomplete, only 1 pin.
    odb::dbITerm* firstITerm = net->get1stITerm();
    if (firstITerm == nullptr) {
      disconnectAllPinsFromNet(net);
      odb::dbNet::destroy(net);
      break;
    }
    odb::dbInst* bufferInst = firstITerm->getInst();
    odb::dbITerm* driverInputPin = getFirstInput(bufferInst);
    disconnectAllPinsFromNet(net);
    odb::dbNet::destroy(net);
    net = driverInputPin->getNet();
    ++numFixedNets_;
    --numClkNets_;
    odb::dbInst::destroy(bufferInst);
  }
}

void TritonCTS::createClockBuffers(Clock& clockNet, odb::dbModule* parent)
{
  unsigned numBuffers = 0;
  clockNet.forEachClockBuffer([&](ClockInst& inst) {
    odb::dbMaster* master = db_->findMaster(inst.getMaster().c_str());
    odb::dbInst* newInst = odb::dbInst::create(
        block_, master, inst.getName().c_str(), false, parent);
    newInst->setSourceType(odb::dbSourceType::TIMING);
    inst.setInstObj(newInst);
    inst2clkbuf_[newInst] = &inst;
    inst.setInputPinObj(getFirstInput(newInst));
    newInst->setLocation(inst.getX(), inst.getY());
    newInst->setPlacementStatus(odb::dbPlacementStatus::PLACED);
    ++numBuffers;
  });
  logger_->info(CTS, 18, "    Created {} clock buffers.", numBuffers);
  int totalBuffers = options_->getNumBuffersInserted() + numBuffers;
  options_->setNumBuffersInserted(totalBuffers);
}

odb::dbITerm* TritonCTS::getFirstInput(odb::dbInst* inst) const
{
  odb::dbSet<odb::dbITerm> iterms = inst->getITerms();
  for (odb::dbITerm* iterm : iterms) {
    if (iterm->isInputSignal()) {
      return iterm;
    }
  }

  return nullptr;
}

odb::dbITerm* TritonCTS::getSingleOutput(odb::dbInst* inst,
                                         odb::dbITerm* input) const
{
  odb::dbSet<odb::dbITerm> iterms = inst->getITerms();
  odb::dbITerm* output = nullptr;
  for (odb::dbITerm* iterm : iterms) {
    if (iterm != input && iterm->isOutputSignal()) {
      odb::dbNet* net = iterm->getNet();
      if (net) {
        if (staClockNets_.find(net) != staClockNets_.end()) {
          output = iterm;
          break;
        }
      }
    }
  }
  return output;
}
bool TritonCTS::masterExists(const std::string& master) const
{
  return db_->findMaster(master.c_str());
};

void TritonCTS::findClockRoots(sta::Clock* clk,
                               std::set<odb::dbNet*>& clockNets)
{
  for (const sta::Pin* pin : clk->leafPins()) {
    odb::dbITerm* instTerm;
    odb::dbBTerm* port;
    odb::dbModITerm* moditerm;
    network_->staToDb(pin, instTerm, port, moditerm);
    odb::dbNet* net = instTerm ? instTerm->getNet() : port->getNet();
    clockNets.insert(net);
  }
}

float TritonCTS::getInputPinCap(odb::dbITerm* iterm)
{
  odb::dbInst* inst = iterm->getInst();
  sta::Cell* masterCell = network_->dbToSta(inst->getMaster());
  sta::LibertyCell* libertyCell = network_->libertyCell(masterCell);
  if (!libertyCell) {
    return 0.0;
  }

  sta::LibertyPort* inputPort
      = libertyCell->findLibertyPort(iterm->getMTerm()->getConstName());
  if (inputPort) {
    return inputPort->capacitance();
  }

  return 0.0;
}

bool TritonCTS::isSink(odb::dbITerm* iterm)
{
  odb::dbInst* inst = iterm->getInst();
  sta::Cell* masterCell = network_->dbToSta(inst->getMaster());
  sta::LibertyCell* libertyCell = network_->libertyCell(masterCell);
  if (!libertyCell) {
    return true;
  }

  if (inst->isBlock()) {
    return true;
  }

  sta::LibertyPort* inputPort
      = libertyCell->findLibertyPort(iterm->getMTerm()->getConstName());
  if (inputPort) {
    return inputPort->isRegClk();
  }

  return false;
}

bool TritonCTS::hasInsertionDelay(odb::dbInst* inst, odb::dbMTerm* mterm)
{
  if (options_->insertionDelayEnabled()) {
    sta::LibertyCell* libCell = network_->libertyCell(network_->dbToSta(inst));
    if (libCell) {
      sta::LibertyPort* libPort
          = libCell->findLibertyPort(mterm->getConstName());
      if (libPort) {
        const float rise = libPort->clkTreeDelay(
            0.0, sta::RiseFall::rise(), sta::MinMax::max());
        const float fall = libPort->clkTreeDelay(
            0.0, sta::RiseFall::fall(), sta::MinMax::max());

        if (rise != 0 || fall != 0) {
          return true;
        }
      }
    }
  }
  return false;
}

double TritonCTS::computeInsertionDelay(const std::string& name,
                                        odb::dbInst* inst,
                                        odb::dbMTerm* mterm)
{
  double insDelayPerMicron = 0.0;

  if (!options_->insertionDelayEnabled()) {
    return insDelayPerMicron;
  }

  sta::LibertyCell* libCell = network_->libertyCell(network_->dbToSta(inst));
  if (libCell) {
    sta::LibertyPort* libPort = libCell->findLibertyPort(mterm->getConstName());
    if (libPort) {
      const float rise = libPort->clkTreeDelay(
          0.0, sta::RiseFall::rise(), sta::MinMax::max());
      const float fall = libPort->clkTreeDelay(
          0.0, sta::RiseFall::fall(), sta::MinMax::max());

      if (rise != 0 || fall != 0) {
        // use average of max rise and max fall
        // TODO: do we need to look at min insertion delays?
        double delayPerSec = (rise + fall) / 2.0;
        // convert delay to length because HTree uses lengths
        sta::Corner* corner = openSta_->cmdCorner();
        double capPerMicron = resizer_->wireSignalCapacitance(corner) * 1e-6;
        double resPerMicron = resizer_->wireSignalResistance(corner) * 1e-6;
        if (sta::fuzzyEqual(capPerMicron, 1e-18)
            || sta::fuzzyEqual(resPerMicron, 1e-18)) {
          logger_->warn(CTS,
                        203,
                        "Insertion delay cannot be used because unit "
                        "capacitance or unit resistance is zero.  Check "
                        "layer RC settings.");
          return 0.0;
        }
        insDelayPerMicron = delayPerSec / (capPerMicron * resPerMicron);
        // clang-format off
          debugPrint(logger_, CTS, "clustering", 1, "sink {} has ins "
                     "delay={:.2e} and micron leng={:0.1f} dbUnits/um={}",
                     name, delayPerSec, insDelayPerMicron,
                     block_->getDbUnitsPerMicron());
          debugPrint(logger_, CTS, "clustering", 1, "capPerMicron={:.2e} "
                     "resPerMicron={:.2e}", capPerMicron, resPerMicron);
        // clang-format on
      }
    }
  }
  return insDelayPerMicron;
}

float getInputCap(const sta::LibertyCell* cell)
{
  sta::LibertyPort *in, *out;
  cell->bufferPorts(in, out);
  if (in != nullptr) {
    return in->capacitance();
  }
  return 0.0;
}

sta::LibertyCell* findBestDummyCell(
    const std::vector<sta::LibertyCell*>& dummyCandidates,
    float deltaCap)
{
  float minDiff = std::numeric_limits<float>::max();
  sta::LibertyCell* bestCell = nullptr;
  for (sta::LibertyCell* cell : dummyCandidates) {
    float diff = std::abs(getInputCap(cell) - deltaCap);
    if (diff < minDiff) {
      minDiff = diff;
      bestCell = cell;
    }
  }
  return bestCell;
}

void TritonCTS::writeDummyLoadsToDb(Clock& clockNet,
                                    std::unordered_set<odb::dbInst*>& dummies)
{
  // Traverse clock tree and compute ideal output caps for clock
  // buffers in the same level
  if (!computeIdealOutputCaps(clockNet)) {
    // No cap adjustment is needed
    return;
  }

  // Find suitable candidate cells for dummy loads
  std::vector<sta::LibertyCell*> dummyCandidates;
  findCandidateDummyCells(dummyCandidates);

  clockNet.forEachSubNet([&](ClockSubNet& subNet) {
    subNet.forEachSink([&](ClockInst* inst) {
      if (inst->isClockBuffer()
          && !sta::fuzzyEqual(inst->getOutputCap(),
                              inst->getIdealOutputCap())) {
        odb::dbInst* dummyInst
            = insertDummyCell(clockNet, inst, dummyCandidates);
        if (dummyInst != nullptr) {
          dummies.insert(dummyInst);
        }
      }
    });
  });

  if (logger_->debugCheck(utl::CTS, "dummy load", 1)) {
    printClockNetwork(clockNet);
  }
}

// Return true if any clock buffers need cap adjustment; false otherwise
bool TritonCTS::computeIdealOutputCaps(Clock& clockNet)
{
  bool needAdjust = false;

  // pass 1: compute actual output caps seen by each clock instance
  clockNet.forEachSubNet([&](ClockSubNet& subNet) {
    // build driver -> subNet map
    ClockInst* driver = subNet.getDriver();
    driver2subnet_[driver] = &subNet;
    float sinkCapTotal = 0.0;
    subNet.forEachSink([&](ClockInst* inst) {
      odb::dbITerm* inputPin = inst->isClockBuffer()
                                   ? getFirstInput(inst->getDbInst())
                                   : inst->getDbInputPin();
      float cap = getInputPinCap(inputPin);
      // TODO: include wire caps?
      sinkCapTotal += cap;
    });
    driver->setOutputCap(sinkCapTotal);
  });

  // pass 2: compute ideal output caps for perfectly balanced tree
  clockNet.forEachSubNet([&](const ClockSubNet& subNet) {
    ClockInst* driver = subNet.getDriver();
    float maxCap = std::numeric_limits<float>::min();
    subNet.forEachSink([&](ClockInst* inst) {
      if (inst->isClockBuffer() && inst->getOutputCap() > maxCap) {
        maxCap = inst->getOutputCap();
      }
    });
    subNet.forEachSink([&](ClockInst* inst) {
      if (inst->isClockBuffer()) {
        inst->setIdealOutputCap(maxCap);
        float cap = inst->getOutputCap();
        if (!sta::fuzzyEqual(cap, maxCap)) {
          needAdjust = true;
          // clang-format off
          debugPrint(logger_, CTS, "dummy load", 1, "{} => {} "
                     "cap:{:0.2e} idealCap:{:0.2e} delCap:{:0.2e}",
                     driver->getName(), inst->getName(), cap, maxCap,
                     maxCap-cap);
          // clang-format on
        }
      }
    });
  });

  return needAdjust;
}

// Find clock buffers and inverters to use as dummy loads
void TritonCTS::findCandidateDummyCells(
    std::vector<sta::LibertyCell*>& dummyCandidates)
{
  // Add existing buffer list
  for (const std::string& buffer : options_->getBufferList()) {
    odb::dbMaster* master = db_->findMaster(buffer.c_str());

    if (master) {
      sta::Cell* masterCell = network_->dbToSta(master);
      if (masterCell) {
        sta::LibertyCell* libCell = network_->libertyCell(masterCell);
        if (libCell) {
          dummyCandidates.emplace_back(libCell);
        }
      }
    }
  }

  // Add additional inverter cells
  // first, look for inverters with "is_clock_cell: true" cell attribute
  std::vector<sta::LibertyCell*> inverters;
  sta::LibertyLibraryIterator* lib_iter = network_->libertyLibraryIterator();
  while (lib_iter->hasNext()) {
    sta::LibertyLibrary* lib = lib_iter->next();
    for (sta::LibertyCell* inv : *lib->inverters()) {
      if (inv->isClockCell() && isClockCellCandidate(inv)) {
        inverters.emplace_back(inv);
        dummyCandidates.emplace_back(inv);
      }
    }
  }
  delete lib_iter;

  // second, look for all inverters with name CLKINV or clkinv
  if (inverters.empty()) {
    sta::PatternMatch patternClkInv("*CLKINV*",
                                    /* is_regexp */ true,
                                    /* nocase */ true,
                                    /* Tcl_interp* */ nullptr);
    lib_iter = network_->libertyLibraryIterator();
    while (lib_iter->hasNext()) {
      sta::LibertyLibrary* lib = lib_iter->next();
      for (sta::LibertyCell* inv :
           lib->findLibertyCellsMatching(&patternClkInv)) {
        if (inv->isInverter() && isClockCellCandidate(inv)) {
          inverters.emplace_back(inv);
          dummyCandidates.emplace_back(inv);
        }
      }
    }
    delete lib_iter;
  }

  // third, look for all inverters with name INV or inv
  if (inverters.empty()) {
    sta::PatternMatch patternInv("*INV*",
                                 /* is_regexp */ true,
                                 /* nocase */ true,
                                 /* Tcl_interp* */ nullptr);
    lib_iter = network_->libertyLibraryIterator();
    while (lib_iter->hasNext()) {
      sta::LibertyLibrary* lib = lib_iter->next();
      for (sta::LibertyCell* inv : lib->findLibertyCellsMatching(&patternInv)) {
        if (inv->isInverter() && isClockCellCandidate(inv)) {
          inverters.emplace_back(inv);
          dummyCandidates.emplace_back(inv);
        }
      }
    }
    delete lib_iter;
  }

  // abandon attributes & name patterns, just look for all inverters
  if (inverters.empty()) {
    lib_iter = network_->libertyLibraryIterator();
    while (lib_iter->hasNext()) {
      sta::LibertyLibrary* lib = lib_iter->next();
      for (sta::LibertyCell* inv : *lib->inverters()) {
        if (isClockCellCandidate(inv)) {
          inverters.emplace_back(inv);
          dummyCandidates.emplace_back(inv);
        }
      }
    }
    delete lib_iter;
  }

  // Sort cells in ascending order of input cap
  std::sort(dummyCandidates.begin(),
            dummyCandidates.end(),
            [](const sta::LibertyCell* cell1, const sta::LibertyCell* cell2) {
              return (getInputCap(cell1) < getInputCap(cell2));
            });

  if (logger_->debugCheck(utl::CTS, "dummy load", 1)) {
    for (const sta::LibertyCell* libCell : dummyCandidates) {
      // clang-format off
      logger_->debug(CTS, "dummy load",
                     "  {} is a dummy cell candidate with input cap={:0.3e}",
                     libCell->name(), getInputCap(libCell));
      // clang-format on
    }
  }
}

odb::dbInst* TritonCTS::insertDummyCell(
    Clock& clockNet,
    ClockInst* inst,
    const std::vector<sta::LibertyCell*>& dummyCandidates)
{
  ClockSubNet* subNet = driver2subnet_[inst];
  if (subNet->getNumSinks() == options_->getMaxFanout()) {
    return nullptr;
  }
  float deltaCap = inst->getIdealOutputCap() - inst->getOutputCap();
  sta::LibertyCell* dummyCell = findBestDummyCell(dummyCandidates, deltaCap);
  // clang-format off
  debugPrint(logger_, CTS, "dummy load", 1, "insertDummyCell {} at {}",
             inst->getName(), dummyCell->name());
  // clang-format on
  odb::dbInst* dummyInst = nullptr;
  ClockInst& dummyClock = placeDummyCell(clockNet, inst, dummyCell, dummyInst);
  if (driver2subnet_.find(inst) == driver2subnet_.end()) {
    logger_->error(
        CTS, 120, "Subnet was not found for clock buffer {}.", inst->getName());
    return nullptr;
  }
  connectDummyCell(inst, dummyInst, *subNet, dummyClock);
  return dummyInst;
}

ClockInst& TritonCTS::placeDummyCell(Clock& clockNet,
                                     const ClockInst* inst,
                                     const sta::LibertyCell* dummyCell,
                                     odb::dbInst*& dummyInst)
{
  odb::dbMaster* master = network_->staToDb(dummyCell);
  if (master == nullptr) {
    logger_->error(CTS,
                   118,
                   "No phyiscal master cell found for dummy cell {}.",
                   dummyCell->name());
  }
  std::string cellName
      = options_->getDummyLoadPrefix() + std::to_string(dummyLoadIndex_++);
  dummyInst = odb::dbInst::create(block_, master, cellName.c_str());
  dummyInst->setSourceType(odb::dbSourceType::TIMING);
  dummyInst->setLocation(inst->getX(), inst->getY());
  dummyInst->setPlacementStatus(odb::dbPlacementStatus::PLACED);
  ClockInst& dummyClock = clockNet.addClockBuffer(
      cellName, master->getName(), inst->getX(), inst->getY());
  // clang-format off
  debugPrint(logger_, CTS, "dummy load", 1, "  placed dummy instance {} at {}",
             dummyInst->getName(), dummyInst->getLocation());
  return dummyClock;
  // clang-format on
}

void TritonCTS::connectDummyCell(const ClockInst* inst,
                                 odb::dbInst* dummyInst,
                                 ClockSubNet& subNet,
                                 ClockInst& dummyClock)
{
  odb::dbInst* sinkInst = inst->getDbInst();
  if (sinkInst == nullptr) {
    logger_->error(
        CTS, 119, "Phyiscal instance {} is not found.", inst->getName());
  }
  odb::dbITerm* iTerm = sinkInst->getFirstOutput();
  odb::dbNet* sinkNet = iTerm->getNet();
  odb::dbITerm* dummyInputPin = getFirstInput(dummyInst);
  dummyInputPin->connect(sinkNet);
  dummyClock.setInputPinObj(dummyInputPin);
  subNet.addInst(dummyClock);
}

void TritonCTS::printClockNetwork(const Clock& clockNet) const
{
  clockNet.forEachSubNet([&](const ClockSubNet& subNet) {
    ClockInst* driver = subNet.getDriver();
    logger_->report("{} has {} sinks", driver->getName(), subNet.getNumSinks());
    subNet.forEachSink([&](const ClockInst* inst) {
      logger_->report("{} -> {}", driver->getName(), inst->getName());
    });
  });
}

void TritonCTS::setAllClocksPropagated()
{
  // Compute ideal buffer delay to use in delay insertion
  if (options_->insertionDelayEnabled()) {
    for (auto& iter : builders_) {
      TreeBuilder* builder = iter.get();
      computeTopBufferDelay(builder);
    }
  }
  sta::Sdc* sdc = openSta_->sdc();
  for (sta::Clock* clk : *sdc->clocks()) {
    openSta_->setPropagatedClock(clk);
  }
  resizer_->estimateParasitics(rsz::ParasiticsSrc::placement);
}

void TritonCTS::repairClockNets()
{
  double max_wire_length
      = resizer_->findMaxWireLength(/* don't issue error */ false);
  if (max_wire_length > 0.0) {
    resizer_->repairClkNets(max_wire_length);
  }
}

// Balance macro cell latencies with register latencies.
// This is needed only if special insertion delay handling
// is invoked.
void TritonCTS::balanceMacroRegisterLatencies()
{
  if (!options_->insertionDelayEnabled()) {
    return;
  }

  // Visit builders from bottom up such that latencies are adjusted near bottom
  // trees first
  rsz::IncrementalParasiticsGuard parasitics_guard(resizer_);
  openSta_->ensureClkNetwork();
  openSta_->ensureClkArrivals();
  sta::Graph* graph = openSta_->graph();
  for (auto& iter : builders_) {
    TreeBuilder* registerBuilder = iter.get();
    if (registerBuilder->getTreeType() == TreeType::RegisterTree) {
      TreeBuilder* macroBuilder = registerBuilder->getParent();
      if (macroBuilder) {
        // Update graph information after possible buffers inserted
        computeAveSinkArrivals(registerBuilder, graph);
        computeAveSinkArrivals(macroBuilder, graph);
        adjustLatencies(macroBuilder, registerBuilder);
        parasitics_guard.update();
        openSta_->updateTiming(false);
      }
    }
  }
}

float TritonCTS::getVertexClkArrival(sta::Vertex* sinkVertex,
                                     odb::dbNet* topNet,
                                     odb::dbITerm* iterm)
{
  sta::VertexPathIterator pathIter(sinkVertex, openSta_);
  float clkPathArrival = 0.0;
  while (pathIter.hasNext()) {
    sta::Path* path = pathIter.next();
    const sta::ClockEdge* clock_edge = path->clkEdge(openSta_);
    if (clock_edge == nullptr) {
      continue;
    }

    if (clock_edge->transition() != sta::RiseFall::rise()) {
      // only populate with rising edges
      continue;
    }

    if (path->dcalcAnalysisPt(openSta_)->delayMinMax() != sta::MinMax::max()) {
      continue;
      // only populate with max delay
    }

    const sta::Clock* clock = path->clock(openSta_);
    if (clock) {
      sta::PathExpanded expand(path, openSta_);
      const sta::Path* start = expand.startPath();

      odb::dbNet* pathStartNet = nullptr;

      odb::dbITerm* term;
      odb::dbBTerm* port;
      odb::dbModITerm* modIterm;
      network_->staToDb(start->pin(openSta_), term, port, modIterm);
      if (term) {
        pathStartNet = term->getNet();
      }
      if (port) {
        pathStartNet = port->getNet();
      }
      if (pathStartNet == topNet) {
        clkPathArrival = path->arrival();
        return clkPathArrival;
      }
    }
  }
  logger_->warn(CTS, 2, "No paths found for pin {}.", iterm->getName());
  return clkPathArrival;
}

void TritonCTS::computeAveSinkArrivals(TreeBuilder* builder, sta::Graph* graph)
{
  Clock clock = builder->getClock();
  odb::dbNet* topInputClockNet = clock.getNetObj();
  if (builder->getTopInputNet() != nullptr) {
    topInputClockNet = builder->getTopInputNet();
  }
  // compute average input arrival at all sinks
  float sumArrivals = 0.0;
  unsigned numSinks = 0;
  clock.forEachSink([&](const ClockInst& sink) {
    odb::dbITerm* iterm = sink.getDbInputPin();
    computeSinkArrivalRecur(
        topInputClockNet, iterm, sumArrivals, numSinks, graph);
  });
  float aveArrival = sumArrivals / (float) numSinks;
  builder->setAveSinkArrival(aveArrival);
  debugPrint(logger_,
             CTS,
             "insertion delay",
             1,
             "{} {}: average sink arrival is {:0.3e}",
             (builder->getTreeType() == TreeType::MacroTree) ? "macro tree"
                                                             : "register tree",
             clock.getName(),
             builder->getAveSinkArrival());
}

void TritonCTS::computeSinkArrivalRecur(odb::dbNet* topClokcNet,
                                        odb::dbITerm* iterm,
                                        float& sumArrivals,
                                        unsigned& numSinks,
                                        sta::Graph* graph)
{
  if (iterm) {
    odb::dbInst* inst = iterm->getInst();
    if (inst) {
      if (isSink(iterm)) {
        // either register or macro input pin
        sta::Pin* pin = network_->dbToSta(iterm);
        if (pin) {
          sta::Vertex* sinkVertex = graph->pinDrvrVertex(pin);
          float arrival = getVertexClkArrival(sinkVertex, topClokcNet, iterm);
          // add insertion delay
          float insDelay = 0.0;
          sta::LibertyCell* libCell
              = network_->libertyCell(network_->dbToSta(inst));
          odb::dbMTerm* mterm = iterm->getMTerm();
          if (libCell && mterm) {
            sta::LibertyPort* libPort
                = libCell->findLibertyPort(mterm->getConstName());
            if (libPort) {
              const float rise = libPort->clkTreeDelay(
                  0.0, sta::RiseFall::rise(), sta::MinMax::max());
              const float fall = libPort->clkTreeDelay(
                  0.0, sta::RiseFall::fall(), sta::MinMax::max());

              if (rise != 0 || fall != 0) {
                insDelay = (rise + fall) / 2.0;
              }
            }
          }
          sumArrivals += (arrival + insDelay);
          numSinks++;
        }
      } else {
        // not a sink, but a clock gater
        odb::dbITerm* outTerm = inst->getFirstOutput();
        if (outTerm) {
          odb::dbNet* outNet = outTerm->getNet();
          bool propagate = propagateClock(iterm);
          if (outNet && propagate) {
            odb::dbSet<odb::dbITerm> iterms = outNet->getITerms();
            odb::dbSet<odb::dbITerm>::iterator iter;
            for (iter = iterms.begin(); iter != iterms.end(); ++iter) {
              odb::dbITerm* inTerm = *iter;
              if (inTerm->getIoType() == odb::dbIoType::INPUT) {
                computeSinkArrivalRecur(
                    topClokcNet, inTerm, sumArrivals, numSinks, graph);
              }
            }
          }
        }
      }
    }
  }
}

bool TritonCTS::propagateClock(odb::dbITerm* input)
{
  odb::dbInst* inst = input->getInst();
  sta::Cell* masterCell = network_->dbToSta(inst->getMaster());
  sta::LibertyCell* libertyCell = network_->libertyCell(masterCell);

  if (!libertyCell) {
    return false;
  }
  // Clock tree buffers
  if (libertyCell->isInverter() || libertyCell->isBuffer()) {
    return true;
  }
  // Combinational components
  if (!libertyCell->hasSequentials()) {
    return true;
  }
  sta::LibertyPort* inputPort
      = libertyCell->findLibertyPort(input->getMTerm()->getConstName());

  // Clock Gater / Latch improvised as clock gater
  if (inputPort) {
    return inputPort->isClockGateClock() || inputPort->isLatchData();
  }

  return false;
}

// Balance latencies between macro tree and register tree
// by adding delay buffers to one tree
void TritonCTS::adjustLatencies(TreeBuilder* macroBuilder,
                                TreeBuilder* registerBuilder)
{
  float latencyDiff = macroBuilder->getAveSinkArrival()
                      - registerBuilder->getAveSinkArrival();
  int numBuffers = 0;
  TreeBuilder* builder = nullptr;
  if (latencyDiff > 0) {
    // add buffers to register tree
    numBuffers = (int) (latencyDiff / registerBuilder->getTopBufferDelay());
    builder = registerBuilder;
  } else {
    // add buffers to macro tree (not common but why not?)
    numBuffers
        = (int) (std::abs(latencyDiff) / macroBuilder->getTopBufferDelay());
    builder = macroBuilder;
  }

  // We don't want to add more delay buffers than needed because
  // wire delays are not considered.  The fewer the delay buffers, the better.
  numBuffers = numBuffers * options_->getDelayBufferDerate();
  if (numBuffers == 0) {
    // clang-format off
    debugPrint(logger_, CTS, "insertion delay", 1, "no delay buffers are needed"
               " to adjust latencies");
    // clang-format on
    return;
  }
  // clang-format off
  debugPrint(logger_, CTS, "insertion delay", 1, "{} delay buffers are needed"
             " to adjust latencies at {} tree", numBuffers,
             (builder->getTreeType() == TreeType::MacroTree)? "macro" : "register");
  // clang-format on

  // disconnect driver output
  odb::dbInst* driver = builder->getTopBuffer();
  odb::dbITerm* driverOutputTerm = driver->getFirstOutput();
  odb::dbNet* outputNet = driverOutputTerm->getNet();

  // hierarchy support:
  // Get the hierarchical net if any and propagate to end of chain
  sta::Pin* op_pin = network_->dbToSta(driverOutputTerm);
  odb::dbModNet* candidate_hier_net = network_->hasHierarchicalElements()
                                          ? network_->hierNet(op_pin)
                                          : nullptr;
  odb::dbNet* orig_flat_net = network_->flatNet(op_pin);

  // get bbox of current load pins without driver output pin
  driverOutputTerm->disconnect();
  odb::Rect bbox = outputNet->getTermBBox();
  int destX = bbox.xCenter();
  int destY = bbox.yCenter();
  int sourceX, sourceY;
  driver->getLocation(sourceX, sourceY);
  float offsetX = (float) (destX - sourceX) / (numBuffers + 1);
  float offsetY = (float) (destY - sourceY) / (numBuffers + 1);

  double scalingFactor = techChar_->getLengthUnit();
  for (int i = 0; i < numBuffers; i++) {
    double locX = (double) (sourceX + offsetX * (i + 1)) / scalingFactor;
    double locY = (double) (sourceY + offsetY * (i + 1)) / scalingFactor;
    Point<double> bufferLoc(locX, locY);
    Point<double> legalBufferLoc
        = builder->legalizeOneBuffer(bufferLoc, options_->getRootBuffer());
    odb::dbInst* buffer
        = insertDelayBuffer(driver,
                            builder->getClock().getSdcName(),
                            legalBufferLoc.getX() * scalingFactor,
                            legalBufferLoc.getY() * scalingFactor);
    driver = buffer;
  }
  // take care of output pin connections
  // driver is now the last delay buffer
  driverOutputTerm = driver->getFirstOutput();
  driverOutputTerm->disconnect();
  // hierarchical fix. guarded by network has hierarchy
  if (candidate_hier_net && network_->hasHierarchy()) {
    network_->connectPin((sta::Pin*) driverOutputTerm,
                         (sta::Net*) orig_flat_net,
                         (sta::Net*) candidate_hier_net);
  } else {
    driverOutputTerm->connect(outputNet);
  }
}

void TritonCTS::computeTopBufferDelay(TreeBuilder* builder)
{
  Clock clock = builder->getClock();
  odb::dbInst* topBuffer
      = block_->findInst(builder->getTopBufferName().c_str());
  if (topBuffer) {
    builder->setTopBuffer(topBuffer);
    odb::dbITerm* inputTerm = getFirstInput(topBuffer);
    odb::dbITerm* outputTerm = topBuffer->getFirstOutput();
    sta::Pin* inputPin = network_->dbToSta(inputTerm);
    sta::Pin* outputPin = network_->dbToSta(outputTerm);

    float inputArrival = openSta_->pinArrival(
        inputPin, sta::RiseFall::rise(), sta::MinMax::max());
    float outputArrival = openSta_->pinArrival(
        outputPin, sta::RiseFall::rise(), sta::MinMax::max());
    float bufferDelay = outputArrival - inputArrival;
    // add a 10% increase on the buffer delay as this is an ideal model
    // TODO: compute the exact delay adding a buffer adds,
    // removing the need for the derate
    builder->setTopBufferDelay(bufferDelay * 1.1);
    debugPrint(logger_,
               CTS,
               "insertion delay",
               1,
               "top buffer delay for {} {} is {:0.3e}",
               (builder->getTreeType() == TreeType::MacroTree)
                   ? "macro tree"
                   : "register tree",
               topBuffer->getName(),
               builder->getTopBufferDelay());
  }
}

// Create a new delay buffer and connect output pin of driver to input pin of
// new buffer. Output pin of new buffer will be connected later.
odb::dbInst* TritonCTS::insertDelayBuffer(odb::dbInst* driver,
                                          const std::string& clockName,
                                          int locX,
                                          int locY)

{
  // creat a new input net
  std::string newNetName
      = "delaynet_" + std::to_string(delayBufIndex_) + "_" + clockName;

  // hierarchy fix, make the net in the right scope
  odb::dbModule* module = driver->getModule();
  if (module == nullptr) {
    // if none put in top level
    module = block_->getTopModule();
  }
  sta::Instance* scope
      = (module == nullptr || (module == block_->getTopModule()))
            ? network_->topInstance()
            : (sta::Instance*) (module->getModInst());
  odb::dbNet* newNet
      = network_->staToDb(network_->makeNet(newNetName.c_str(), scope));

  newNet->setSigType(odb::dbSigType::CLOCK);

  // create a new delay buffer
  std::string newBufName
      = "delaybuf_" + std::to_string(delayBufIndex_++) + "_" + clockName;
  odb::dbMaster* master = db_->findMaster(options_->getRootBuffer().c_str());

  // fix: make buffer in same hierarchical module as driver

  odb::dbInst* newBuf
      = odb::dbInst::create(block_, master, newBufName.c_str(), false, module);

  newBuf->setSourceType(odb::dbSourceType::TIMING);
  newBuf->setLocation(locX, locY);
  newBuf->setPlacementStatus(odb::dbPlacementStatus::PLACED);

  // connect driver output with new buffer input
  odb::dbITerm* driverOutTerm = driver->getFirstOutput();
  odb::dbITerm* newBufInTerm = getFirstInput(newBuf);

  driverOutTerm->disconnect();
  driverOutTerm->connect(newNet);
  newBufInTerm->connect(newNet);

  debugPrint(logger_,
             CTS,
             "insertion delay",
             1,
             "new delay buffer {} is inserted at ({} {})",
             newBuf->getName(),
             locX,
             locY);

  return newBuf;
}
}  // namespace cts
