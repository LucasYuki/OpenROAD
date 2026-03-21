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
    u_x_ = v_x_ = inst_->cx();
    u_y_ = v_y_ = inst_->cy();
  };
  ~NesterovInst(){};

  gpl::Instance* gplInst() const { return inst_; };
  std::pair<float, float> const getForce()
  {
    return std::make_pair(force_x_, force_y_);
  };
  std::pair<float, float> getPos() const { return std::make_pair(u_x_, u_y_); };
  std::pair<float, float> getRef() const { return std::make_pair(v_x_, v_y_); };
  void setForce(float x, float y)
  {
    force_x_ = x;
    force_y_ = y;
  };
  void setPos(float x, float y)
  {
    u_x_ = x;
    u_y_ = y;
    inst_->setCenterLocation(x, y);
  };
  void setRef(float x, float y)
  {
    v_x_ = x;
    v_y_ = y;
  };

 private:
  gpl::Instance* inst_;

  // location in gpl::Instance* is int
  // but the fft calc and force calc is in float
  float force_x_ = 0, force_y_ = 0;
  float u_x_ = 0, u_y_ = 0;
  float v_x_ = 0, v_y_ = 0;
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
  const std::vector<std::vector<NesterovInst>>& nesterovInsts() const
  {
    return inst_ed_vec_;
  };
  std::pair<float, float> snapPosition(const odb::Rect& region,
                                       float x,
                                       float y,
                                       gpl::Instance* inst) const;

 private:
  void init();

 private:
  std::shared_ptr<WAwirelength> wa_wirelength_;
  std::vector<std::shared_ptr<EDensity>> e_density_vec_;
  utl::Logger* log_;

  std::vector<std::vector<NesterovInst>> inst_ed_vec_;
};

}  // namespace epl