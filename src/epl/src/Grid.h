#pragma once

#include "gpl/fft.h"
#include "gpl/placerBase.h"
#include "odb/geom.h"

namespace gpl {
class FFT;
class Instance;
class Die;
}  // namespace gpl

namespace odb {
class Rect;
}  // namespace odb

namespace epl {

class Grid : public gpl::FFT
{
 public:
  Grid(utl::Logger* log,
       int binCntX,
       int binCntY,
       float binSizeX,
       float binSizeY,
       const odb::Rect region);
  ~Grid();

  void doFFT();
  void clearMovable();
  void addFixedInst(const gpl::Instance* inst);
  void addMovableInst(const gpl::Instance* inst);
  std::pair<float, float> getElectroForce(gpl::Instance* inst) const;
  float getDensity(int x, int y) const { return binArea_[x][y] / getBin(x, y).area(); };
  float total_overflow() const;

  void setTargetDensity(float density) { target_density_ = density; };

  const odb::Rect getBin(int x, int y) const
  {
    return odb::Rect(region_.xMin() + std::round(x * binSizeX_),
                     region_.yMin() + std::round(y * binSizeY_),
                     region_.xMin() + std::round((x + 1) * binSizeX_),
                     region_.yMin() + std::round((y + 1) * binSizeY_));
  }

  int binCntX() const { return binCntX_; };
  int binCntY() const { return binCntY_; };
  float binSizeX() const { return binSizeX_; };
  float binSizeY() const { return binSizeY_; };

 private:
  std::pair<int, int> getMinMaxIdxX(const gpl::Instance* inst) const;
  std::pair<int, int> getMinMaxIdxY(const gpl::Instance* inst) const;
  std::pair<float, odb::Rect> smoothScaleInst(
      const gpl::Instance* inst,
      const std::pair<int, int>& idxX,
      const std::pair<int, int>& idxY) const;

 private:
  utl::Logger* log_;
  odb::Rect region_;
  float** binArea_ = nullptr;
  int64_t** binAreaFixed_ = nullptr;
  int64_t** binAreaFixedMacro_ = nullptr;
  float target_density_ = 0;
};
}  // namespace epl