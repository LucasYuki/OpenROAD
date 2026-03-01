#pragma once

#include <unordered_map>
#include <vector>

#include "EDensity.h"
#include "Grid.h"
#include "odb/db.h"
namespace gpl {
class Instance;
}  // namespace gpl

namespace epl {
class WAwirelength;
class EDensity;

class NesterovInst
{
 public:
  NesterovInst(gpl::Instance* inst) : inst_(inst)
  {
    u_ = v_ = std::make_pair(inst_->cx(), inst_->cy());
  };
  ~NesterovInst(){};

  gpl::Instance* gplInst() { return inst_; };
  std::pair<float, float> getForce() { return force_; };
  std::pair<float, float> getPos() { return u_; };
  std::pair<float, float> getRef() { return v_; };
  void setForce(float x, float y) { force_ = std::make_pair(x, y); };
  void setPos(float x, float y)
  {
    u_ = std::make_pair(x, y);
    inst_->setCenterLocation(u_.first, u_.second);
  };
  void setRef(float x, float y) { v_ = std::make_pair(x, y); };

 private:
  gpl::Instance* inst_;

  // location in gpl::Instance* is int
  // but the fft calc and force calc is in float
  std::pair<float, float> force_ = {0, 0};
  std::pair<float, float> u_;
  std::pair<float, float> v_;
};

class NesterovOptimizer
{
 public:
  NesterovOptimizer(  // tconst NesterovOptimizer& npVars,
      const std::shared_ptr<WAwirelength>& wa_wirelength,
      const std::vector<std::shared_ptr<EDensity>>& e_density_vec,
      utl::Logger* log);
  ~NesterovOptimizer(){};

  bool step();

 private:
  void init();

 private:
  std::shared_ptr<WAwirelength> wa_wirelength_;
  std::vector<std::shared_ptr<EDensity>> e_density_vec_;
  utl::Logger* log_;

  std::vector<std::vector<NesterovInst>> inst_ed_vec_;
};

}  // namespace epl