#include "Grid.h"

namespace gpl {
class FFT;
}  // namespace gpl

namespace epl {

Grid::Grid(int binCntX, int binCntY, float binSizeX, float binSizeY)
    : gpl::FFT(binCntX, binCntY, binSizeX, binSizeY)
{

}
}  // namespace epl