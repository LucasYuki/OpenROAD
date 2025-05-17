#pragma once

#include <vector>

#include "odb/db.h"

namespace epl {

enum FlopPinInput {
    D,
    D0,
    D1,
    D2,
    D3,
    SI,
    CLK
};

enum FlopPinOutput {
    Q,
    Q0,
    Q1,
    Q2,
    Q3,
    SO
};

class FlipFlop
{
public:
  FlipFlop(std::string name,
           int x,
           int y,
           int size,
           float delay,
           float power,
           std::vector<FlopPinInput> inputs,
           std::vector<FlopPinOutput> outputs)
    : name_(name),
      x_(x),
      y_(y),
      size_(size),
      delay_(delay),
      power_(power),
      inputs_(inputs),
      outputs_(outputs)
  {}

  ~FlipFlop() = default;
  

private:
  std::string name_;
  int x_;
  int y_;
  int size_;
  float delay_;
  float power_;
  std::vector<FlopPinInput> inputs_;
  std::vector<FlopPinOutput> outputs_;


};
}