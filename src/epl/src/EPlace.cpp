#include "epl/EPlace.h"

#include <random>

#include "EDensity.h"
#include "gpl/placerBase.h"
#include "sta/StaMain.hh"

namespace sta {
// Tcl files encoded into strings.
extern const char* eplace_tcl_inits[];
}  // namespace sta

namespace epl {

using utl::EPL;

extern "C" {
extern int Epl_Init(Tcl_Interp* interp);
}

EPlace::EPlace()
{
}

EPlace::~EPlace()
{
  clear();
}

void EPlace::init(odb::dbDatabase* db, utl::Logger* logger)
{
  clear();
  db_ = db;
  log_ = logger;
}

void EPlace::clear()
{
  // clear instances
  nesterov_.reset();
  wa_wirelength_.reset();
  e_density_vec_.clear();
  pbc_.reset();
  pbVec_.clear();
  gui_.reset();
}

bool EPlace::initEPlace(float density, bool uniform_density)
{
  if (nesterov_) {
    log_->warn(EPL, 3, "EPlacer already initialized.");
    return true;
  }

  if (!initPlacer()) {
    return false;
  }

  // Init wa_wirelength_
  debugPrint(log_, EPL, "initEPlace", 3, "Init wa_wirelength_");
  WAwirelengthVars waVars;
  int threads = 1;
  wa_wirelength_ = std::make_shared<WAwirelength>(waVars, pbc_, log_, threads);

  // Init e_density
  debugPrint(log_, EPL, "initEPlace", 1, "Init e_density");
  EDensityVars edVars;
  edVars.target_density = density;
  edVars.uniform_density = uniform_density;
  for (auto pb : pbVec_) {
    e_density_vec_.push_back(
        std::make_shared<EDensity>(edVars, pb, wa_wirelength_, log_));
  }

  // Init nesterov_
  debugPrint(log_, EPL, "initEPlace", 1, "Init nesterov_");
  nesterov_ = std::make_shared<NesterovOptimizer>(
      wa_wirelength_, e_density_vec_, log_);

  return true;
}

bool EPlace::initPlacer()
{
  if (pbc_) {
    log_->warn(EPL, 2, "Placer already initialized.");
    return true;
  }
  // Init PlacerBaseCommon
  int padLeft = 0;
  int padRight = 0;
  bool skipIoMode = false;
  gpl::PlacerBaseVars pbVars(padLeft, padRight, skipIoMode, true);

  pbc_ = std::make_shared<gpl::PlacerBaseCommon>(db_, pbVars, log_);
  if (pbc_->placeInsts().size() == 0) {
    log_->warn(EPL, 1, "No placeable instances - skipping placement.");
    return false;
  }

  // Init PlacerBase
  pbVec_.push_back(std::make_shared<gpl::PlacerBase>(db_, pbc_, log_, false));
  for (auto pd : db_->getChip()->getBlock()->getPowerDomains()) {
    if (pd->getGroup()) {
      pbVec_.push_back(
          std::make_shared<gpl::PlacerBase>(db_, pbc_, log_, pd->getGroup()));
    }
  }
  return true;
}

void EPlace::set_debug(bool draw_bins,
                       bool disable_wirelength,
                       bool disable_density)
{
  debug_ = true;
  draw_bins_ = draw_bins;
  disable_wirelength_ = disable_wirelength;
  disable_density_ = disable_density;
}

void EPlace::place(int threads,
                   float density,
                   bool uniform_density,
                   float density_penalty,
                   int iterations)
{
  debugPrint(log_, EPL, "place", 1, "place: number of threads {}", threads);
  if (!initEPlace(density, uniform_density)) {
    return;
  }

  int gif_key = 0;
  if (debug_ && gui::Gui::enabled()) {
    gui_ = std::make_unique<Graphics>(log_);
    gui_->debug(this,
                pbc_,
                wa_wirelength_,
                nesterov_,
                pbVec_,
                e_density_vec_,
                draw_bins_);
    for (auto ed : e_density_vec_) {
      ed->updateForce();
    }
    gui_->cellPlot(true);
    gif_key = gui_->gifStart("ePlace.gif");
  }

  debugPrint(log_,
             EPL,
             "place",
             1,
             "core size: ({} {}) ({} {})",
             pbc_->getDie().coreLx(),
             pbc_->getDie().coreLy(),
             pbc_->getDie().coreUx(),
             pbc_->getDie().coreUy());

  wa_wirelength_->setGamma(1.0);
  // bool debug = true;
  int lst_step = 0;
  for (int i = 0; i < iterations;) {
    debugPrint(log_, EPL, "place", 1, "nesterov_step: {}", i);
    wa_wirelength_->update();
    // eDensity gradient calc
    for (auto ed : e_density_vec_) {
      ed->updateForce();
    }
    updateGradient(density_penalty, disable_wirelength_, disable_density_);
    std::cout << "total cost: " << cost_ << " WA: " << wa_wirelength_->getWA()
              << " density: " << density_cost_ << std::endl;
    lst_step = i;
    i = nesterov_->step(i);

    // eDensity density calc
    for (auto ed : e_density_vec_) {
      ed->updateDensity();
    }

    if (gui_ && gui_->enabled() && (lst_step != i)) {
      gui_->cellPlot(true);
      odb::Rect region;
      odb::Rect bbox = pbc_->db()->getChip()->getBlock()->getBBox()->getBox();
      int max_dim = std::max(bbox.dx(), bbox.dy());
      double dbu_per_pixel = static_cast<double>(max_dim) / 1000.0;
      gui_->gifAddFrame(gif_key, region, 500, dbu_per_pixel, 20);
    }
  }

  // update_db
  auto insts = pbc_->placeInsts();
  int n_inst = insts.size();
#pragma omp parallel for num_threads(threads)
  for (int i = 0; i < n_inst; i++) {
    auto inst = insts[i];
    inst->dbSetLocation();
    inst->dbSetPlaced();
  }

  if (gui_ && gui_->enabled()) {
    gui_->gifEnd(gif_key);
    gui_->setDebugOn(false);
    gui_->cellPlot(false);
  }
  gui_.reset();
}

void EPlace::updateGradient(float density_penalty,
                            bool disable_wirelength,
                            bool disable_density,
                            bool use_density_field,
                            bool use_preconditioning)
{
  // update the gradient on each instance
  density_cost_ = 0;
  int idx = 0;
  for (auto& ed : e_density_vec_) {
    float filler_area = 1;
    if (use_density_field) {
      filler_area = ed->defaultFillerArea();
    }
    for (auto& inst : nesterov_->nesterovInsts()[idx++]) {
      // Density Gradient
      float force_e_x = 0, force_e_y = 0;
      float preconditioner = 0;
      if (!disable_density) {
        std::tie(force_e_x, force_e_y) = ed->getElectroForce(inst.gplInst());
        if (use_density_field) {
          float area_ratio = filler_area / inst.gplInst()->getArea();
          force_e_x = force_e_x * area_ratio;
          force_e_y = force_e_y * area_ratio;
        }
        preconditioner = density_penalty * inst.gplInst()->getArea();
      }

      // WA gradient
      float gradient_wa_x = 0, gradient_wa_y = 0;
      if (!disable_wirelength) {
        for (auto* pin : inst.gplInst()->getPins()) {
          auto [tmp_x, tmp_y] = wa_wirelength_->getGradient(pin);
          gradient_wa_x = tmp_x;
          gradient_wa_y = tmp_y;
          preconditioner += pin->getNet()->getPins().size();
        }
      }

      // Compute the total gradient
      float gradient_x = gradient_wa_x - density_penalty * force_e_x;
      float gradient_y = gradient_wa_y - density_penalty * force_e_y;
      if (use_preconditioning) {
        gradient_x *= preconditioner;
        gradient_y *= preconditioner;
      }
      inst.setGradient(gradient_x, gradient_y);

      // calculate cost
      density_cost_ += ed->getPotentialEnergy(inst.gplInst());
    }
  }
  cost_ = wa_wirelength_->getWA() + density_penalty *  density_cost_;
}

void EPlace::randomPlace(int threads)
{
  debugPrint(log_,
             EPL,
             "random_place",
             1,
             "random_place: number of threads {}",
             threads);
  if (!initPlacer()) {
    return;
  }

  odb::dbBlock* block = db_->getChip()->getBlock();
  odb::Rect coreRect = block->getCoreArea();

  auto insts = pbc_->placeInsts();
  int n_inst = insts.size();
  std::default_random_engine generator(42);
  std::uniform_int_distribution<int> distrib_x(coreRect.xMin(),
                                               coreRect.xMax());
  std::uniform_int_distribution<int> distrib_y(coreRect.yMin(),
                                               coreRect.yMax());
  std::vector<int> pos_x(n_inst), pos_y(n_inst);
  std::generate(
      pos_x.begin(), pos_x.end(), [&]() { return distrib_x(generator); });
  std::generate(
      pos_y.begin(), pos_y.end(), [&]() { return distrib_y(generator); });

#pragma omp parallel for num_threads(threads)
  for (int i = 0; i < n_inst; i++) {
    auto inst = insts[i];
    inst->dbSetLocation(pos_x[i], pos_y[i]);
    inst->dbSetPlaced();
  }
}

}  // namespace epl
