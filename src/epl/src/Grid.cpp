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
  binDensityFixed_ = new float*[binCntX_];

  for (int i = 0; i < binCntX_; i++) {
    binDensityFixed_[i] = new float[binCntY_];

    for (int j = 0; j < binCntY_; j++) {
      binDensityFixed_[i][j] = 0.0f;
    }
  }
}

Grid::~Grid()
{
  for (int i = 0; i < binCntX_; i++) {
    delete[] binDensityFixed_[i];
  }
  delete[] binDensityFixed_;
}

void Grid::clearMovable()
{
  for (int i = 0; i < binCntX_; i++) {
    for (int j = 0; j < binCntY_; j++) {
      binDensity_[i][j] = binDensityFixed_[i][j];
    }
  }
}

void Grid::addFixedInst(const gpl::Instance* inst)
{
  std::pair<int, int> idxX = getMinMaxIdxX(inst);
  std::pair<int, int> idxY = getMinMaxIdxY(inst);
  odb::Rect inst_rect{inst->lx(), inst->ly(), inst->ux(), inst->uy()};
  for (int x = idxX.first; x < idxX.second; x++) {
    for (int y = idxY.first; y < idxY.second; y++) {
      binDensityFixed_[x][y] += inst_rect.intersect(getBin(x, y)).area();
    }
  }
}

void Grid::addMovableInst(const gpl::Instance* inst)
{
  std::pair<int, int> idxX = getMinMaxIdxX(inst);
  std::pair<int, int> idxY = getMinMaxIdxY(inst);
  odb::Rect inst_rect{inst->lx(), inst->ly(), inst->ux(), inst->uy()};
  if (inst_rect.dx() < binSizeX_) {
    inst_rect.set_xlo(inst_rect.xCenter() - binSizeX_ / 2);
    inst_rect.set_xhi(inst_rect.xMin() + binSizeX_);
  }
  if (inst_rect.dy() < binSizeY_) {
    inst_rect.set_ylo(inst_rect.yCenter() - binSizeY_ / 2);
    inst_rect.set_yhi(inst_rect.yMin() + binSizeY_);
  }
  double area_ratio = inst->area() / static_cast<double>(inst_rect.area());

  for (int x = idxX.first; x < idxX.second; x++) {
    for (int y = idxY.first; y < idxY.second; y++) {
      binDensity_[x][y] += inst_rect.intersect(getBin(x, y)).area() * area_ratio;
    }
  }
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

}  // namespace epl