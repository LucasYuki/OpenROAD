#include "WAwirelength.h"

#include <climits>
#include <cmath>

namespace epl {

WAwirelength::WAwirelength(utl::Logger* log, int num_threads)
    : log_(log), num_threads_(num_threads)
{
}

void WAwirelength::update(const std::vector<gpl::Net*>& nets)
{
  hpwl_ = 0;
  wa_ = 0;
  const float gamma_inv = 1. / gamma_;

  // using DreamPlace gradient formulation
  for (auto* net : nets) {
    auto pins = net->getPins();
    int n_pins = pins.size();
    if (n_pins < 2) {
      for (auto* pin : net->getPins()) {
        wa_gradient_[pin] = std::make_pair(0, 0);
      }
      continue;
    }

    int x_min = INT_MAX, x_max = INT_MIN;
    int y_min = INT_MAX, y_max = INT_MIN;
    for (auto* pin : pins) {
      x_min = std::min(x_min, pin->cx());
      x_max = std::max(x_max, pin->cx());
      y_min = std::min(y_min, pin->cy());
      y_max = std::max(y_max, pin->cy());
    }
    if ((x_max - x_min) < 0 || (y_max - y_min) < 0) {
      std::cout << "net: " << net->getDbNet()->getName() << " x_max: " << x_max
                << " x_min: " << x_min << " y_max: " << y_max
                << " y_min: " << y_min << std::endl;
    }
    hpwl_ += (x_max - x_min) + (y_max - y_min);

    float x_a_pos[n_pins], x_a_neg[n_pins];
    float y_a_pos[n_pins], y_a_neg[n_pins];
    float x_b_pos = 0, x_b_neg = 0;
    float y_b_pos = 0, y_b_neg = 0;
    float x_c_pos = 0, x_c_neg = 0;
    float y_c_pos = 0, y_c_neg = 0;
    int i = 0;
    for (auto* pin : pins) {
      x_a_pos[i] = std::exp((pin->cx() - x_max) * gamma_inv);
      x_a_neg[i] = std::exp(-(pin->cx() - x_min) * gamma_inv);
      y_a_pos[i] = std::exp((pin->cy() - y_max) * gamma_inv);
      y_a_neg[i] = std::exp(-(pin->cy() - y_min) * gamma_inv);

      x_b_pos += x_a_pos[i];
      x_b_neg += x_a_neg[i];
      y_b_pos += y_a_pos[i];
      y_b_neg += y_a_neg[i];

      x_c_pos += x_a_pos[i] * pin->cx();
      x_c_neg += x_a_neg[i] * pin->cx();
      y_c_pos += y_a_pos[i] * pin->cy();
      y_c_neg += y_a_neg[i] * pin->cy();

      i++;
    }

    wa_ += (x_c_pos / x_b_pos - x_c_neg / x_b_neg)
           + (y_c_pos / y_b_pos - y_c_neg / y_b_neg);
    i = 0;
    for (auto* pin : pins) {
      float x_grad = 0;
      if (x_b_pos != 0) {
        x_grad = ((1 + pin->cx() * gamma_inv) * x_b_pos - gamma_inv * x_c_pos)
                     * x_a_pos[i] / x_b_pos / x_b_pos
                 - ((1 - pin->cx() * gamma_inv) * x_b_neg + gamma_inv * x_c_neg)
                       * x_a_neg[i] / x_b_neg / x_b_neg;
      }

      float y_grad = 0;
      if (y_b_pos != 0) {
        y_grad = ((1 + pin->cy() * gamma_inv) * y_b_pos - gamma_inv * y_c_pos)
                     * y_a_pos[i] / y_b_pos / y_b_pos
                 - ((1 - pin->cy() * gamma_inv) * y_b_neg + gamma_inv * y_c_neg)
                       * y_a_neg[i] / y_b_neg / y_b_neg;
      }
      wa_gradient_[pin] = std::make_pair(x_grad, y_grad);
      i++;
    }
  }
}

}  // namespace epl