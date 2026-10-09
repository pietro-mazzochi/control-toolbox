// Copyright (c) 2024, ISI SIM
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#ifndef IDENT_SIGNALS__PRBS_GENERATOR_HPP_
#define IDENT_SIGNALS__PRBS_GENERATOR_HPP_

#include <vector>
#include <cstdint>
#include <stdexcept>

class PRBSGenerator
{
public:
    PRBSGenerator(uint8_t num_bits, uint32_t seed = 0x1);

    void setNumBits(uint8_t num_bits);
    void setSeed(uint32_t seed);

    uint8_t getNextValue();
    void reset();

private:
    uint8_t num_bits_;
    uint32_t shift_register_;
    uint32_t mask_;
    size_t current_index_;
    std::vector<uint8_t> sequence_;

    uint32_t calculateMask(uint8_t num_bits);
    void generateSequence();
};

#endif // IDENT_SIGNALS__PRBS_GENERATOR_HPP_
