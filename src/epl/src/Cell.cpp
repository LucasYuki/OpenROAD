#pragma once

#include <vector>

#include "odb/db.h"
#include "epl/Cell.h"

namespace epl {

class Cell
{
public:
  Cell(std::string name,
           int x,
           int y,
           float delay,
           float power)
    : name_(name),
      x_(x),
      y_(y),
      delay_(delay),
      power_(power)
  {}

  ~Cell() = default;
  

private:
  std::string name_;
  int x_;
  int y_;
  float delay_;
  float power_;


};
}