// SPDX-License-Identifier: BSD-3-Clause
// Copyright (c) 2021-2025, The OpenROAD Authors

#include "object.h"

#include <algorithm>
#include <boost/random/uniform_int_distribution.hpp>
#include <cmath>
#include <iterator>
#include <map>
#include <memory>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "odb/db.h"
#include "odb/dbTypes.h"
#include "util.h"
#include "utl/Logger.h"

namespace mpl {
using utl::MPL;

///////////////////////////////////////////////////////////////////////
// Metrics Class
Metrics::Metrics(unsigned int num_std_cell,
                 unsigned int num_macro,
                 float std_cell_area,
                 float macro_area)
{
  num_std_cell_ = num_std_cell;
  num_macro_ = num_macro;
  std_cell_area_ = std_cell_area;
  macro_area_ = macro_area;
}

void Metrics::addMetrics(const Metrics& metrics)
{
  num_std_cell_ += metrics.num_std_cell_;
  num_macro_ += metrics.num_macro_;
  std_cell_area_ += metrics.std_cell_area_;
  macro_area_ += metrics.macro_area_;
}

std::pair<unsigned int, unsigned int> Metrics::getCountStats() const
{
  return std::pair<unsigned int, unsigned int>(num_std_cell_, num_macro_);
}

std::pair<float, float> Metrics::getAreaStats() const
{
  return std::pair<float, float>(std_cell_area_, macro_area_);
}

unsigned int Metrics::getNumMacro() const
{
  return num_macro_;
}

unsigned int Metrics::getNumStdCell() const
{
  return num_std_cell_;
}

float Metrics::getStdCellArea() const
{
  return std_cell_area_;
}

float Metrics::getMacroArea() const
{
  return macro_area_;
}

float Metrics::getArea() const
{
  return std_cell_area_ + macro_area_;
}

bool Metrics::empty() const
{
  return num_macro_ == 0 && num_std_cell_ == 0;
}

///////////////////////////////////////////////////////////////////////
// Cluster Class
// Constructors and Destructors
Cluster::Cluster(int cluster_id, utl::Logger* logger)
{
  id_ = cluster_id;
  logger_ = logger;
}

Cluster::Cluster(int cluster_id,
                 const std::string& cluster_name,
                 utl::Logger* logger)
{
  id_ = cluster_id;
  name_ = cluster_name;
  logger_ = logger;
}

// cluster id
int Cluster::getId() const
{
  return id_;
}

const std::string& Cluster::getName() const
{
  return name_;
}

void Cluster::setName(const std::string& name)
{
  name_ = name;
}

// cluster type
void Cluster::setClusterType(const ClusterType& cluster_type)
{
  type_ = cluster_type;
}

ClusterType Cluster::getClusterType() const
{
  return type_;
}

// Instances (Here we store dbModule to reduce memory)
void Cluster::addDbModule(odb::dbModule* db_module)
{
  db_modules_.push_back(db_module);
}

void Cluster::addLeafStdCell(odb::dbInst* leaf_std_cell)
{
  leaf_std_cells_.push_back(leaf_std_cell);
}

void Cluster::addLeafMacro(odb::dbInst* leaf_macro)
{
  leaf_macros_.push_back(leaf_macro);
}

void Cluster::addLeafInst(odb::dbInst* inst)
{
  if (inst->isBlock()) {
    addLeafMacro(inst);
  } else {
    addLeafStdCell(inst);
  }
}

void Cluster::specifyHardMacros(std::vector<HardMacro*>& hard_macros)
{
  hard_macros_ = hard_macros;
}

std::vector<odb::dbModule*> Cluster::getDbModules() const
{
  return db_modules_;
}

std::vector<odb::dbInst*> Cluster::getLeafStdCells() const
{
  return leaf_std_cells_;
}

std::vector<odb::dbInst*> Cluster::getLeafMacros() const
{
  return leaf_macros_;
}

std::vector<HardMacro*> Cluster::getHardMacros() const
{
  return hard_macros_;
}

void Cluster::clearDbModules()
{
  db_modules_.clear();
}

void Cluster::clearLeafStdCells()
{
  leaf_std_cells_.clear();
}

void Cluster::clearLeafMacros()
{
  leaf_macros_.clear();
}

void Cluster::clearHardMacros()
{
  hard_macros_.clear();
}

std::string Cluster::getClusterTypeString() const
{
  std::string cluster_type;

  if (is_io_bundle_) {
    return "IO Bundle";
  }

  if (is_cluster_of_unconstrained_io_pins_) {
    return "Unconstrained IOs";
  }

  if (is_cluster_of_unplaced_io_pins_) {
    return "Unplaced IOs";
  }

  if (is_io_pad_cluster_) {
    return "IO Pad";
  }

  switch (type_) {
    case StdCellCluster:
      cluster_type = "StdCell";
      break;
    case MixedCluster:
      cluster_type = "Mixed";
      break;
    case HardMacroCluster:
      cluster_type = "Macro";
      break;
  }

  return cluster_type;
}

std::string Cluster::getIsLeafString() const
{
  std::string is_leaf_string;

  if (!isIOCluster() && children_.empty()) {
    is_leaf_string = "Leaf";
  }

  return is_leaf_string;
}

// copy instances based on cluster Type
void Cluster::copyInstances(const Cluster& cluster)
{
  // clear firstly
  db_modules_.clear();
  leaf_std_cells_.clear();
  leaf_macros_.clear();
  hard_macros_.clear();
  // insert new elements
  if (type_ == HardMacroCluster) {
    leaf_macros_.insert(leaf_macros_.end(),
                        cluster.leaf_macros_.begin(),
                        cluster.leaf_macros_.end());
  } else if (type_ == StdCellCluster) {
    leaf_std_cells_.insert(leaf_std_cells_.end(),
                           cluster.leaf_std_cells_.begin(),
                           cluster.leaf_std_cells_.end());
    db_modules_.insert(db_modules_.end(),
                       cluster.db_modules_.begin(),
                       cluster.db_modules_.end());
  } else {  // type_ == MixedCluster
    leaf_macros_.insert(leaf_macros_.end(),
                        cluster.leaf_macros_.begin(),
                        cluster.leaf_macros_.end());
    leaf_std_cells_.insert(leaf_std_cells_.end(),
                           cluster.leaf_std_cells_.begin(),
                           cluster.leaf_std_cells_.end());
    db_modules_.insert(db_modules_.end(),
                       cluster.db_modules_.begin(),
                       cluster.db_modules_.end());
  }
}

void Cluster::setAsClusterOfUnplacedIOPins(
    const std::pair<float, float>& pos,
    const float width,
    const float height,
    const bool is_cluster_of_unconstrained_io_pins)
{
  is_cluster_of_unplaced_io_pins_ = true;
  is_cluster_of_unconstrained_io_pins_ = is_cluster_of_unconstrained_io_pins;
  soft_macro_ = std::make_unique<SoftMacro>(pos, name_, width, height, this);
}

void Cluster::setAsIOPadCluster(const std::pair<float, float>& pos,
                                float width,
                                float height)
{
  is_io_pad_cluster_ = true;
  soft_macro_ = std::make_unique<SoftMacro>(pos, name_, width, height, this);
}

void Cluster::setAsIOBundle(const Point& pos, float width, float height)
{
  is_io_bundle_ = true;
  soft_macro_ = std::make_unique<SoftMacro>(pos, name_, width, height, this);
}

bool Cluster::isIOCluster() const
{
  return is_cluster_of_unplaced_io_pins_ || is_io_pad_cluster_ || is_io_bundle_;
}

bool Cluster::isClusterOfUnconstrainedIOPins() const
{
  return is_cluster_of_unconstrained_io_pins_;
}

bool Cluster::isClusterOfUnplacedIOPins() const
{
  return is_cluster_of_unplaced_io_pins_;
}

void Cluster::setAsArrayOfInterconnectedMacros()
{
  is_array_of_interconnected_macros_ = true;
}

bool Cluster::isArrayOfInterconnectedMacros() const
{
  return is_array_of_interconnected_macros_;
}

bool Cluster::isEmpty() const
{
  return getLeafStdCells().empty() && getLeafMacros().empty()
         && getDbModules().empty();
}

bool Cluster::correspondsToLogicalModule() const
{
  return getLeafStdCells().empty() && getLeafMacros().empty()
         && (getDbModules().size() == 1);
}

// Metrics Support and Statistics
void Cluster::setMetrics(const Metrics& metrics)
{
  metrics_ = metrics;
}

const Metrics& Cluster::getMetrics() const
{
  return metrics_;
}

int Cluster::getNumStdCell() const
{
  if (type_ == HardMacroCluster) {
    return 0;
  }
  return metrics_.getNumStdCell();
}

int Cluster::getNumMacro() const
{
  if (type_ == StdCellCluster) {
    return 0;
  }
  return metrics_.getNumMacro();
}

float Cluster::getArea() const
{
  return getStdCellArea() + getMacroArea();
}

float Cluster::getStdCellArea() const
{
  if (type_ == HardMacroCluster) {
    return 0.0;
  }

  return metrics_.getStdCellArea();
}

float Cluster::getMacroArea() const
{
  if (type_ == StdCellCluster) {
    return 0.0;
  }

  return metrics_.getMacroArea();
}

// Physical location support
float Cluster::getWidth() const
{
  if (!soft_macro_) {
    return 0.0;
  }

  return soft_macro_->getWidth();
}

float Cluster::getHeight() const
{
  if (!soft_macro_) {
    return 0.0;
  }

  return soft_macro_->getHeight();
}

float Cluster::getX() const
{
  if (!soft_macro_) {
    return 0.0;
  }

  return soft_macro_->getX();
}

float Cluster::getY() const
{
  if (!soft_macro_) {
    return 0.0;
  }

  return soft_macro_->getY();
}

void Cluster::setX(float x)
{
  if (soft_macro_) {
    soft_macro_->setX(x);
  }
}

void Cluster::setY(float y)
{
  if (soft_macro_) {
    soft_macro_->setY(y);
  }
}

std::pair<float, float> Cluster::getLocation() const
{
  if (!soft_macro_) {
    return {0, 0};
  }

  return soft_macro_->getLocation();
}

Rect Cluster::getBBox() const
{
  return soft_macro_->getBBox();
}

Point Cluster::getCenter() const
{
  return {getX() + getWidth() / 2.0, getY() + getHeight() / 2.0};
}

// Hierarchy Support
void Cluster::setParent(Cluster* parent)
{
  parent_ = parent;
}

void Cluster::addChild(std::unique_ptr<Cluster> child)
{
  children_.push_back(std::move(child));
}

std::unique_ptr<Cluster> Cluster::releaseChild(const Cluster* candidate)
{
  auto it = std::find_if(
      children_.begin(), children_.end(), [candidate](const auto& child) {
        return child.get() == candidate;
      });

  if (it != children_.end()) {
    std::unique_ptr<Cluster> released_child = std::move(*it);
    children_.erase(it);
    return released_child;
  }

  return nullptr;
}

void Cluster::addChildren(UniqueClusterVector children)
{
  std::move(children.begin(), children.end(), std::back_inserter(children_));
}

UniqueClusterVector Cluster::releaseChildren()
{
  UniqueClusterVector released_children = std::move(children_);
  children_.clear();

  return released_children;
}

Cluster* Cluster::getParent() const
{
  return parent_;
}

const UniqueClusterVector& Cluster::getChildren() const
{
  return children_;
}

bool Cluster::isLeaf() const
{
  return children_.empty();
}

bool Cluster::attemptMerge(Cluster* incomer, bool& incomer_deleted)
{
  if (parent_ != incomer->parent_) {
    return false;
  }

  // If the ownership is not passed to the receiver. The incomer
  // is destroyed.
  std::unique_ptr<Cluster> released_incomer = parent_->releaseChild(incomer);

  metrics_.addMetrics(incomer->metrics_);
  name_ += "||" + incomer->name_;

  leaf_macros_.insert(leaf_macros_.end(),
                      incomer->leaf_macros_.begin(),
                      incomer->leaf_macros_.end());
  leaf_std_cells_.insert(leaf_std_cells_.end(),
                         incomer->leaf_std_cells_.begin(),
                         incomer->leaf_std_cells_.end());
  db_modules_.insert(db_modules_.end(),
                     incomer->db_modules_.begin(),
                     incomer->db_modules_.end());
  incomer_deleted = true;

  // If the receiver is not a leaf, the incomer becomes another
  // one of its children.
  if (!children_.empty()) {
    incomer->setParent(this);
    children_.push_back(std::move(released_incomer));
    incomer_deleted = false;
  }
  return true;
}

// Connection signature support
void Cluster::initConnection()
{
  connection_map_.clear();
}

void Cluster::addConnection(int cluster_id, float weight)
{
  if (connection_map_.find(cluster_id) == connection_map_.end()) {
    connection_map_[cluster_id] = weight;
  } else {
    connection_map_[cluster_id] += weight;
  }
}

std::map<int, float> Cluster::getConnection() const
{
  return connection_map_;
}

// The connection signature is based on connection topology
// if the number of connnections between two clusters is larger than the
// net_threshold we think the two clusters are connected, otherwise they are
// disconnected.
bool Cluster::isSameConnSignature(const Cluster& cluster, float net_threshold)
{
  std::vector<int> neighbors;          // neighbors of current cluster
  std::vector<int> cluster_neighbors;  // neighbors of the input cluster
  for (auto& [cluster_id, weight] : connection_map_) {
    if ((cluster_id != id_) && (cluster_id != cluster.id_)
        && (weight >= net_threshold)) {
      neighbors.push_back(cluster_id);
    }
  }

  if (neighbors.empty()) {
    return false;
  }

  for (auto& [cluster_id, weight] : cluster.connection_map_) {
    if ((cluster_id != id_) && (cluster_id != cluster.id_)
        && (weight >= net_threshold)) {
      cluster_neighbors.push_back(cluster_id);
    }
  }

  if (neighbors.size() != cluster_neighbors.size()) {
    return false;
  }
  std::sort(neighbors.begin(), neighbors.end());
  std::sort(cluster_neighbors.begin(), cluster_neighbors.end());
  for (int i = 0; i < neighbors.size(); i++) {
    if (neighbors[i] != cluster_neighbors[i]) {
      return false;
    }
  }

  return true;
}

// Directly connected macros should be grouped in an array of macros.
// The number of connections must be greater than the threshold in order for
// the macros to be considered part of the same array.
bool Cluster::hasMacroConnectionWith(const Cluster& cluster,
                                     float net_threshold)
{
  if (id_ != cluster.getId()) {
    for (const auto& [cluster_id, num_of_conn] : connection_map_) {
      if (cluster_id == cluster.getId() && num_of_conn > net_threshold) {
        return true;
      }
    }
  }

  return false;
}

// Get closely-connected cluster if such cluster exists
// For example, if a small cluster A is closely connected to a
// well-formed cluster B, (there are also other well-formed clusters
// C, D), A is only connected to B and A has no connection with C, D
// Candidate clusters always need to be merged.
// A non candidate cluster is a well-formed cluster.
int Cluster::getCloseCluster(const std::vector<int>& candidate_clusters,
                             float net_threshold)
{
  int closely_cluster = -1;
  int num_closely_clusters = 0;
  for (auto& [cluster_id, num_nets] : connection_map_) {
    debugPrint(logger_,
               MPL,
               "multilevel_autoclustering",
               2,
               "cluster_id: {}, nets: {}",
               cluster_id,
               num_nets);
    if (num_nets > net_threshold
        && std::find(
               candidate_clusters.begin(), candidate_clusters.end(), cluster_id)
               == candidate_clusters.end()) {
      num_closely_clusters++;
      closely_cluster = cluster_id;
    }
  }

  if (num_closely_clusters == 1) {
    return closely_cluster;
  }
  return -1;
}

// Print Basic Information
// Normally we call this after macro placement is done
void Cluster::printBasicInformation(utl::Logger* logger) const
{
  std::string line = "\n";
  line += std::string(80, '*') + "\n";
  line += "[INFO] cluster_name :  " + name_ + "  ";
  line += "cluster_id : " + std::to_string(id_) + "  \n";
  line += "num_std_cell : " + std::to_string(getNumStdCell()) + "  ";
  line += "num_macro : " + std::to_string(getNumMacro()) + "\n";
  line += "width : " + std::to_string(getWidth()) + "  ";
  line += "height : " + std::to_string(getHeight()) + "  ";
  line += "location :  ( " + std::to_string((getLocation()).first) + " , ";
  line += std::to_string((getLocation()).second) + " )\n";
  for (const auto& hard_macro : hard_macros_) {
    line += "\t macro_name : " + hard_macro->getName();
    line += "\t width : " + std::to_string(hard_macro->getRealWidth());
    line += "\t height : " + std::to_string(hard_macro->getRealHeight());
    line += "\t lx : " + std::to_string(hard_macro->getRealX());
    line += "\t ly : " + std::to_string(hard_macro->getRealY());
    line += "\n";
  }

  logger->report(line);
}

// Macro Placement Support
void Cluster::setSoftMacro(std::unique_ptr<SoftMacro> soft_macro)
{
  soft_macro_.reset();
  soft_macro_ = std::move(soft_macro);
}

SoftMacro* Cluster::getSoftMacro() const
{
  return soft_macro_.get();
}

void Cluster::setTilings(const TilingList& tilings)
{
  tilings_ = tilings;
}

const TilingList& Cluster::getTilings() const
{
  return tilings_;
}

// Virtual Connections
std::vector<std::pair<int, int>> Cluster::getVirtualConnections() const
{
  return virtual_connections_;
}

void Cluster::addVirtualConnection(int src, int target)
{
  virtual_connections_.emplace_back(src, target);
}

///////////////////////////////////////////////////////////////////////
// HardMacro
HardMacro::HardMacro(std::pair<float, float> location,
                     const std::string& name,
                     float width,
                     float height,
                     Cluster* cluster)
{
  width_ = width;
  height_ = height;
  name_ = name;
  pin_x_ = 0.0;
  pin_y_ = 0.0;
  x_ = location.first;
  y_ = location.second;
  cluster_ = cluster;
}

HardMacro::HardMacro(float width, float height, const std::string& name)
{
  width_ = width;
  height_ = height;
  name_ = name;
  pin_x_ = width / 2.0;
  pin_y_ = height / 2.0;
}

HardMacro::HardMacro(odb::dbInst* inst, float halo_width, float halo_height)
{
  inst_ = inst;
  block_ = inst->getBlock();
  name_ = inst->getName();

  halo_width_ = halo_width;
  halo_height_ = halo_height;

  odb::dbMaster* master = inst->getMaster();
  width_ = block_->dbuToMicrons(master->getWidth()) + 2 * halo_width;
  height_ = block_->dbuToMicrons(master->getHeight()) + 2 * halo_height;

  // Set the position of virtual pins
  odb::Rect bbox;
  bbox.mergeInit();
  for (odb::dbMTerm* mterm : master->getMTerms()) {
    if (mterm->getSigType() == odb::dbSigType::SIGNAL) {
      for (odb::dbMPin* mpin : mterm->getMPins()) {
        for (odb::dbBox* box : mpin->getGeometry()) {
          odb::Rect rect = box->getBox();
          bbox.merge(rect);
        }
      }
    }
  }
  pin_x_
      = block_->dbuToMicrons((bbox.xMin() + bbox.xMax()) / 2.0) + halo_width_;
  pin_y_
      = block_->dbuToMicrons((bbox.yMin() + bbox.yMax()) / 2.0) + halo_height_;
}

// overload the comparison operators
// based on area, width, height order
// When we compare, we also consider the effect of halo_width
bool HardMacro::operator<(const HardMacro& macro) const
{
  if (width_ * height_ != macro.width_ * macro.height_) {
    return width_ * height_ < macro.width_ * macro.height_;
  }
  if (width_ != macro.width_) {
    return width_ < macro.width_;
  }
  return height_ < macro.height_;
}

bool HardMacro::operator==(const HardMacro& macro) const
{
  return (width_ == macro.width_) && (height_ == macro.height_);
}

// Cluster support to identify if a fixed terminal correponds
// to a cluster of unplaced IO pins when running HardMacro SA.
bool HardMacro::isClusterOfUnplacedIOPins() const
{
  if (!cluster_) {
    return false;
  }

  return cluster_->isClusterOfUnplacedIOPins();
}

// Cluster support to identify if a fixed terminal correponds
// to the cluster of unconstrained IO pins when running HardMacro SA.
bool HardMacro::isClusterOfUnconstrainedIOPins() const
{
  if (!cluster_) {
    return false;
  }

  return cluster_->isClusterOfUnconstrainedIOPins();
}

// Get Physical Information
// Note that the default X and Y include halo_width
void HardMacro::setLocation(const std::pair<float, float>& location)
{
  if (width_ * height_ <= 0.0) {
    return;
  }
  x_ = location.first;
  y_ = location.second;
}

void HardMacro::setX(float x)
{
  if (width_ * height_ <= 0.0) {
    return;
  }
  x_ = x;
}

void HardMacro::setY(float y)
{
  if (width_ * height_ <= 0.0) {
    return;
  }
  y_ = y;
}

std::pair<float, float> HardMacro::getLocation() const
{
  return std::pair<float, float>(x_, y_);
}

// Note that the real X and Y does NOT include halo_width
void HardMacro::setRealLocation(const std::pair<float, float>& location)
{
  if (width_ * height_ <= 0.0) {
    return;
  }

  x_ = location.first - halo_width_;
  y_ = location.second - halo_height_;
}

void HardMacro::setRealX(float x)
{
  if (width_ * height_ <= 0.0) {
    return;
  }

  x_ = x - halo_width_;
}

void HardMacro::setRealY(float y)
{
  if (width_ * height_ <= 0.0) {
    return;
  }

  y_ = y - halo_height_;
}

std::pair<float, float> HardMacro::getRealLocation() const
{
  return std::pair<float, float>(x_ + halo_width_, y_ + halo_height_);
}

float HardMacro::getRealX() const
{
  return x_ + halo_width_;
}

float HardMacro::getRealY() const
{
  return y_ + halo_height_;
}

float HardMacro::getRealWidth() const
{
  return width_ - 2 * halo_width_;
}

float HardMacro::getRealHeight() const
{
  return height_ - 2 * halo_height_;
}

// Orientation support
odb::dbOrientType HardMacro::getOrientation() const
{
  return orientation_;
}

// We do not allow rotation of macros
// This may violate the direction of metal layers
void HardMacro::flip(bool flip_horizontal)
{
  if (flip_horizontal) {
    orientation_ = orientation_.flipX();
    pin_y_ = height_ - pin_y_;
  } else {
    orientation_ = orientation_.flipY();
    pin_x_ = width_ - pin_x_;
  }
}

// Interfaces with OpenDB
odb::dbInst* HardMacro::getInst() const
{
  return inst_;
}

const std::string& HardMacro::getName() const
{
  return name_;
}

std::string HardMacro::getMasterName() const
{
  if (inst_ == nullptr) {
    return name_;
  }
  return inst_->getMaster()->getName();
}

///////////////////////////////////////////////////////////////////////

// Represent a "regular" cluster (Mixed, StdCell or Macro).
SoftMacro::SoftMacro(Cluster* cluster)
{
  name_ = cluster->getName();
  cluster_ = cluster;
}

// Represent a blockage.
SoftMacro::SoftMacro(float width, float height, const std::string& name)
{
  name_ = name;
  width_ = width;
  height_ = height;
  area_ = width * height;
  cluster_ = nullptr;
}

// Represent an IO cluster or fixed terminal.
SoftMacro::SoftMacro(const std::pair<float, float>& location,
                     const std::string& name,
                     float width,
                     float height,
                     Cluster* cluster)
{
  name_ = name;
  x_ = location.first;
  y_ = location.second;
  width_ = width;
  height_ = height;

  // Even though clusters of unplaced IOs have shapes, i.e., are not
  // just points, their area should be zero, because we use the area
  // to check whether or not a SoftMacro if a fixed terminal or cluster
  // of unplaced IOs inside SA. Ideally we should check the fixed flag.
  area_ = 0.0f;

  cluster_ = cluster;
  fixed_ = true;
}

// name
const std::string& SoftMacro::getName() const
{
  return name_;
}

// Physical Information
void SoftMacro::setX(float x)
{
  if (!fixed_) {
    x_ = x;
  }
}

void SoftMacro::setY(float y)
{
  if (!fixed_) {
    y_ = y;
  }
}

void SoftMacro::setLocation(const std::pair<float, float>& location)
{
  if (fixed_) {
    return;
  }
  x_ = location.first;
  y_ = location.second;
}

// Find the index of the interval to which 'value' belongs.
int SoftMacro::findIntervalIndex(const IntervalList& interval_list,
                                 float& value,
                                 bool increasing_list)
{
  // We assume the value is within the range of list
  int idx = 0;
  if (increasing_list) { /* Width Intervals */
    while ((idx < interval_list.size()) && (interval_list[idx].max < value)) {
      idx++;
    }
    if (interval_list[idx].min > value) {
      value = interval_list[idx].min;
    }
  } else { /* Height Intervals */
    while ((idx < interval_list.size()) && (interval_list[idx].min > value)) {
      idx++;
    }
    if (interval_list[idx].max < value) {
      value = interval_list[idx].max;
    }
  }
  return idx;
}

void SoftMacro::setWidth(float width)
{
  if (width <= 0.0 || area_ == 0.0
      || width_intervals_.size() != height_intervals_.size()
      || width_intervals_.empty() || cluster_ == nullptr
      || cluster_->getClusterType() == HardMacroCluster
      || cluster_->isIOCluster()) {
    return;
  }

  // The width intervals are sorted in nondecreasing order.
  if (width <= width_intervals_.front().min) {
    width_ = width_intervals_.front().min;
    height_ = height_intervals_.front().max;
    area_ = width_ * height_;
  } else if (width >= width_intervals_.back().max) {
    width_ = width_intervals_.back().max;
    height_ = height_intervals_.back().min;
    area_ = width_ * height_;
  } else {
    width_ = width;
    int idx = findIntervalIndex(width_intervals_, width_, true);
    area_ = width_intervals_[idx].max * height_intervals_[idx].min;
    height_ = area_ / width_;
  }
}

void SoftMacro::setHeight(float height)
{
  if (height <= 0.0 || area_ == 0.0
      || width_intervals_.size() != height_intervals_.size()
      || width_intervals_.empty() || cluster_ == nullptr
      || cluster_->getClusterType() == HardMacroCluster
      || cluster_->isIOCluster()) {
    return;
  }

  // The height intervals are sorted in nonincreasing order.
  if (height >= height_intervals_.front().max) {
    height_ = height_intervals_.front().max;
    width_ = width_intervals_.front().min;
    area_ = width_ * height_;
  } else if (height <= height_intervals_.back().min) {
    height_ = height_intervals_.back().min;
    width_ = width_intervals_.back().max;
    area_ = width_ * height_;
  } else {
    height_ = height;
    int idx = findIntervalIndex(height_intervals_, height_, false);
    area_ = width_intervals_[idx].max * height_intervals_[idx].min;
    width_ = area_ / height_;
  }
}

void SoftMacro::setArea(float area)
{
  if (area_ == 0.0 || width_intervals_.size() != height_intervals_.size()
      || width_intervals_.empty() || cluster_ == nullptr
      || cluster_->getClusterType() == HardMacroCluster
      || cluster_->isIOCluster()
      || area <= width_intervals_.front().min * height_intervals_.front().max) {
    return;
  }

  // area must be larger than area_
  IntervalList width_intervals;
  IntervalList height_intervals;
  for (int i = 0; i < width_intervals_.size(); i++) {
    const float min_width = width_intervals_[i].min;
    const float min_height = height_intervals_[i].min;
    const float max_width = area / min_height;
    const float max_height = area / min_width;
    if (width_intervals.empty() || min_width > width_intervals.back().max) {
      width_intervals.emplace_back(min_width, max_width);
      height_intervals.emplace_back(min_height, max_height);
    } else {
      width_intervals.back().max = max_width;
      height_intervals.back().min = min_height;
    }
  }

  width_intervals_ = std::move(width_intervals);
  height_intervals_ = std::move(height_intervals);
  area_ = area;
  width_ = width_intervals_.front().min;
  height_ = height_intervals_.front().max;
}

// Method to set the shape curve for Macro clusters.
// The shape curve is discrete.
void SoftMacro::setShapes(const TilingList& tilings, bool force)
{
  if (!force
      && (tilings.empty() || cluster_ == nullptr
          || cluster_->getClusterType() != HardMacroCluster)) {
    return;
  }

  // Here we do not need to sort the intervals.
  for (auto& tiling : tilings) {
    width_intervals_.emplace_back(tiling.width(), tiling.width());
    height_intervals_.emplace_back(tiling.height(), tiling.height());
  }

  width_ = tilings.front().width();
  height_ = tilings.front().height();
  area_ = width_ * height_;
}

// Method to set the shape curve for the following cluster types:
// - Mixed
// - Std Cell
//
// The shape curve is piecewise.
void SoftMacro::setShapes(const IntervalList& width_intervals, float area)
{
  if (width_intervals.empty() || area <= 0.0 || cluster_ == nullptr
      || cluster_->isIOCluster()
      || cluster_->getClusterType() == HardMacroCluster) {
    return;
  }

  width_intervals_.clear();
  height_intervals_.clear();

  // Copy & sort the width intervals list.
  IntervalList old_width_intervals = width_intervals;
  std::sort(old_width_intervals.begin(),
            old_width_intervals.end(),
            isMinWidthSmaller);

  // Merge the overlapping intervals.
  for (auto& old_width_interval : old_width_intervals) {
    if (width_intervals_.empty()
        || old_width_interval.min > width_intervals_.back().max) {
      width_intervals_.push_back(old_width_interval);
    } else if (old_width_interval.max > width_intervals_.back().max) {
      width_intervals_.back().max = old_width_interval.max;
    }
  }

  // Set height intervals based on the new width intervals.
  for (auto& width_interval : width_intervals_) {
    height_intervals_.emplace_back(area / width_interval.max /* min */,
                                   area / width_interval.min /* max */);
  }

  width_ = width_intervals_.front().min;
  height_ = height_intervals_.front().max;
  area_ = area;
}

float SoftMacro::getArea() const
{
  return area_ > 0.01 ? area_ : 0.0;
}

Rect SoftMacro::getBBox() const
{
  return Rect(x_, y_, x_ + width_, y_ + height_);
}

// Num Macros
bool SoftMacro::isMacroCluster() const
{
  if (cluster_ == nullptr) {
    return false;
  }
  return (cluster_->getClusterType() == HardMacroCluster);
}

bool SoftMacro::isStdCellCluster() const
{
  if (cluster_ == nullptr) {
    return false;
  }

  return (cluster_->getClusterType() == StdCellCluster);
}

int SoftMacro::getNumMacro() const
{
  if (cluster_ == nullptr) {
    return 0;
  }

  return cluster_->getNumMacro();
}

void SoftMacro::resizeRandomly(
    std::uniform_real_distribution<float>& distribution,
    std::mt19937& generator)
{
  if (width_intervals_.empty()) {
    return;
  }

  boost::random::uniform_int_distribution<> index_distribution(
      0, width_intervals_.size() - 1);
  const int idx = index_distribution(generator);

  const float min_width = width_intervals_[idx].min;
  const float max_width = width_intervals_[idx].max;
  width_ = min_width + distribution(generator) * (max_width - min_width);
  area_ = width_intervals_[idx].min * height_intervals_[idx].max;
  height_ = area_ / width_;
}

// Align Flag support
void SoftMacro::setAlignFlag(bool flag)
{
  align_flag_ = flag;
}

bool SoftMacro::getAlignFlag() const
{
  return align_flag_;
}

// cluster
Cluster* SoftMacro::getCluster() const
{
  return cluster_;
}

// Calculate macro utilization
float SoftMacro::getMacroUtil() const
{
  if (cluster_ == nullptr || area_ == 0.0) {
    return 0.0;
  }

  if (cluster_->getClusterType() == HardMacroCluster) {
    return 1.0;
  }

  if (cluster_->getClusterType() == MixedCluster) {
    return cluster_->getMacroArea() / area_;
  }

  return 0.0;
}

bool SoftMacro::isMixedCluster() const
{
  if (cluster_ == nullptr) {
    return false;
  }

  return (cluster_->getClusterType() == MixedCluster);
}

// Cluster support to identify if a fixed terminal correponds
// to a cluster of unplaced IO pins when running SoftMacro SA.
bool SoftMacro::isClusterOfUnplacedIOPins() const
{
  if (!cluster_) {
    return false;
  }

  return cluster_->isClusterOfUnplacedIOPins();
}

// Cluster support to identify if a fixed terminal correponds
// to the cluster of unconstrained IO pins when running SoftMacro SA.
bool SoftMacro::isClusterOfUnconstrainedIOPins() const
{
  if (!cluster_) {
    return false;
  }

  return cluster_->isClusterOfUnconstrainedIOPins();
}

void SoftMacro::setLocationF(float x, float y)
{
  x_ = x;
  y_ = y;
}

void SoftMacro::setShapeF(float width, float height)
{
  width_ = width;
  height_ = height;
  area_ = width * height;
}

}  // namespace mpl
