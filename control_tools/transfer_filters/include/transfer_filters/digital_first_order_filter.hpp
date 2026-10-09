// Copyright (c) 2025, ISI SIM
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#ifndef TRANSFER_FILTERS__DIGITAL_FIRST_ORDER_FILTER_HPP_
#define TRANSFER_FILTERS__DIGITAL_FIRST_ORDER_FILTER_HPP_

#include <memory>

namespace TransferFilters
{
class DigitalFirstOrderFilter
{
public:
  DigitalFirstOrderFilter(const double pole);
  DigitalFirstOrderFilter(const double poleContinuous, const double sampleTime);

  double compute(const double input);

  inline void reset()
  {
    output_ = 0;
  }

private:
  const double pole_ = 0.5;
  double output_ = 0;
};

using DigitalFirstOrderFilterPtr = std::shared_ptr<DigitalFirstOrderFilter>;
}  // namespace TransferFilters

#endif  // TRANSFER_FILTERS__DIGITAL_FIRST_ORDER_FILTER_HPP_
