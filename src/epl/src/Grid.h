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

class Grid : gpl::FFT
{
 public:
  Grid(utl::Logger* log,
       int binCntX,
       int binCntY,
       float binSizeX,
       float binSizeY,
       gpl::Die* die);
  ~Grid();

  void clearMovable();
  void addFixedInst(const gpl::Instance* inst);
  void addMovableInst(const gpl::Instance* inst);

 private:
  std::pair<int, int> getMinMaxIdxX(const gpl::Instance* inst) const;
  std::pair<int, int> getMinMaxIdxY(const gpl::Instance* inst) const;

  odb::Rect getBin(int x, int y) const
  {
    return odb::Rect(die_->coreLx() + std::round(x * binSizeX_),
                     die_->coreLy() + std::round(y * binSizeY_),
                     die_->coreLx() + std::round((x + 1) * binSizeX_),
                     die_->coreLy() + std::round((y + 1) * binSizeY_));
  }

 private:
  utl::Logger* log_;
  gpl::Die* die_;
  float** binDensityFixed_ = nullptr;
};
}  // namespace epl