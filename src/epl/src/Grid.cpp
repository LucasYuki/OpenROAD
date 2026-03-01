#include "Grid.h"

#include "gpl/placerBase.h"
#include "odb/db.h"
#include "odb/geom.h"

namespace gpl {
class FFT;
}  // namespace gpl

namespace odb {
class Rect;
}  // namespace odb

namespace epl {

Grid::Grid(utl::Logger* log,
           int binCntX,
           int binCntY,
           float binSizeX,
           float binSizeY,
           gpl::Die* die)
    : gpl::FFT(binCntX, binCntY, binSizeX, binSizeY), log_(log), die_(die)
{
  log_->info(utl::EPL,
             11,
             "Grid size: ({}, {}). Bin size: ({}, {}) um",
             binCntX,
             binCntY,
             binSizeX,
             binSizeY);
  binAreaFixed_ = new int64_t*[binCntX_];
  binAreaFixedMacro_ = new int64_t*[binCntX_];

  for (int x = 0; x < binCntX_; x++) {
    binAreaFixed_[x] = new int64_t[binCntY_];
    binAreaFixedMacro_[x] = new int64_t[binCntY_];

    for (int y = 0; y < binCntY_; y++) {
      binAreaFixed_[x][y] = 0.0f;
      binAreaFixedMacro_[x][y] = 0.0f;
    }
  }
}

Grid::~Grid()
{
  for (int x = 0; x < binCntX_; x++) {
    delete[] binAreaFixed_[x];
    delete[] binAreaFixedMacro_[x];
  }
  delete[] binAreaFixed_;
  delete[] binAreaFixedMacro_;
}

void Grid::clearMovable()
{
  for (int x = 0; x < binCntX_; x++) {
    for (int y = 0; y < binCntY_; y++) {
      binDensity_[x][y]
          = (binAreaFixed_[x][y] + binAreaFixedMacro_[x][y] * target_density_)
            / getBin(x, y).area();
    }
  }
}

void Grid::addFixedInst(const gpl::Instance* inst)
{
  std::pair<int, int> idxX = getMinMaxIdxX(inst);
  std::pair<int, int> idxY = getMinMaxIdxY(inst);
  odb::Rect inst_rect{inst->lx(), inst->ly(), inst->ux(), inst->uy()};
  int64_t** bin_area = binAreaFixed_;
  if (inst->isMacro()) {
    bin_area = binAreaFixedMacro_;
  }
  for (int x = idxX.first; x < idxX.second; x++) {
    for (int y = idxY.first; y < idxY.second; y++) {
      bin_area[x][y] += inst_rect.intersect(getBin(x, y)).area();
    }
  }
}

void Grid::addMovableInst(const gpl::Instance* inst)
{
  std::pair<int, int> idxX = getMinMaxIdxX(inst);
  std::pair<int, int> idxY = getMinMaxIdxY(inst);
  const auto [scaling, inst_rect] = smoothScaleInst(inst, idxX, idxY);

  for (int x = idxX.first; x < idxX.second; x++) {
    for (int y = idxY.first; y < idxY.second; y++) {
      binDensity_[x][y] += inst_rect.intersect(getBin(x, y)).area() * scaling
                           / getBin(x, y).area();
    }
  }
}

std::pair<float, float> Grid::getElectroForce(gpl::Instance* inst) const
{
  std::pair<int, int> idxX = getMinMaxIdxX(inst);
  std::pair<int, int> idxY = getMinMaxIdxY(inst);
  const auto [scaling, inst_rect] = smoothScaleInst(inst, idxX, idxY);

  float force_x = 0, force_y = 0;
  for (int x = idxX.first; x < idxX.second; x++) {
    for (int y = idxY.first; y < idxY.second; y++) {
      float intersect_ratio
          = inst_rect.intersect(getBin(x, y)).area() / getBin(x, y).area();
      force_x += electroForceX_[x][y] * intersect_ratio;
      force_y += electroForceY_[x][y] * intersect_ratio;
    }
  }
  return std::make_pair<float, float>(force_x * scaling, force_y * scaling);
}

std::pair<int, int> Grid::getMinMaxIdxX(const gpl::Instance* inst) const
{
  int lowerIdx = (inst->lx() - die_->coreLx()) / binSizeX_;
  int upperIdx = std::ceil((inst->ux() - die_->coreLx()) / binSizeX_);

  return std::make_pair(std::max(lowerIdx, 0), std::min(upperIdx, binCntX_));
}

std::pair<int, int> Grid::getMinMaxIdxY(const gpl::Instance* inst) const
{
  int lowerIdx = (inst->ly() - die_->coreLy()) / binSizeY_;
  int upperIdx = std::ceil((inst->uy() - die_->coreLy()) / binSizeY_);

  return std::make_pair(std::max(lowerIdx, 0), std::min(upperIdx, binCntY_));
}

std::pair<float, odb::Rect> Grid::smoothScaleInst(
    const gpl::Instance* inst,
    const std::pair<int, int>& idxX,
    const std::pair<int, int>& idxY) const
{
  // Local smoothness over discrete grids

  // Makes the instance at least the size of one bin
  odb::Rect inst_rect{inst->lx(), inst->ly(), inst->ux(), inst->uy()};
  if (inst_rect.dx() < binSizeX_) {
    inst_rect.set_xlo(inst_rect.xCenter() - binSizeX_ / 2);
    inst_rect.set_xhi(inst_rect.xMin() + binSizeX_);
  }
  if (inst_rect.dy() < binSizeY_) {
    inst_rect.set_ylo(inst_rect.yCenter() - binSizeY_ / 2);
    inst_rect.set_yhi(inst_rect.yMin() + binSizeY_);
  }

  // Calculate the ratio between the original size and the inflated size
  double scaling = inst->getArea() / static_cast<double>(inst_rect.area());
  if (inst->isMacro()) {
    scaling *= target_density_;
  }

  return std::make_pair(scaling, inst_rect);
}

}  // namespace epl