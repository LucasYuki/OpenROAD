#pragma once

#include "gpl/fft.h"

namespace gpl {
class FFT;
}  // namespace gpl

namespace epl {

class Grid : gpl::FFT
{
 public:
  Grid(int binCntX, int binCntY, float binSizeX, float binSizeY);
};
}  // namespace epl