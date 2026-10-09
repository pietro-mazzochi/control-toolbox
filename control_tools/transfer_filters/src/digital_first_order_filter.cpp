// Copyright (c) 2025, ISI SIM
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#include <cmath>
#include <stdexcept>

#include "transfer_filters/digital_first_order_filter.hpp"

namespace TransferFilters
{
DigitalFirstOrderFilter::DigitalFirstOrderFilter(const double pole) : pole_(pole)
{
  if (std::fabs(pole) > 1)
  {
    throw std::invalid_argument("Resulting pole is unstable");
  }
}

DigitalFirstOrderFilter::DigitalFirstOrderFilter(const double poleContinuous, const double sampleTime)
  : DigitalFirstOrderFilter(std::exp(poleContinuous * sampleTime))
{
  if (sampleTime <= 0)
  {
    throw std::invalid_argument("Sampling time must be positive");
  }
}

double DigitalFirstOrderFilter::compute(const double input)
{
  return output_ = (1 - pole_) * input + pole_ * output_;
}
}  // namespace TransferFilters
