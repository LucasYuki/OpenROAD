// Copyright (c) 2021, The Regents of the University of California
// All rights reserved.
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#pragma once

#include <unordered_map>

#include "gpl/Replace.h"
#include "gpl/placerBase.h"
#include "odb/db.h"

namespace gpl {
class PlacerBaseCommon;
class Net;
}  // namespace gpl

namespace epl {

class WAwirelength
{
 public:
  WAwirelength(utl::Logger* log, int num_threads);

  void clear();
  void update(const std::vector<gpl::Net*>& nets);

  void setGamma(float gamma) { gamma_ = gamma; };
  float getGamma() const { return gamma_; };
  int64_t getHPWL() const { return hpwl_; };
  float getWA() const { return wa_; };

  std::pair<float, float> getGradient(gpl::Pin* pin)
  {
    return wa_gradient_[pin];
  };

 private:
  utl::Logger* log_;
  int num_threads_;

  int64_t hpwl_ = 0;
  float wa_ = 0;
  float gamma_ = 1.;
  std::unordered_map<gpl::Pin*, std::pair<float, float>> wa_gradient_;
};

}  // namespace epl
