// Copyright (c) 2024, ISI SIM
// All rights reserved.
//
// Proprietary License
//
// Unauthorized copying of this file, via any medium is strictly prohibited.
// The file is considered confidential.

#include "ident_signals/prbs_generator.hpp"

PRBSGenerator::PRBSGenerator(uint8_t num_bits, uint32_t seed)
    : num_bits_(num_bits), shift_register_(seed), current_index_(0)
{
    if (num_bits_ < 2 || num_bits_ > 31)
    {
        throw std::invalid_argument("Number of bits must be between 2 and 31.");
    }
    mask_ = calculateMask(num_bits_);
    generateSequence();
    if (sequence_.empty())
    {
        throw std::runtime_error("Sequence generation failed, sequence is empty.");
    }
}

void PRBSGenerator::setNumBits(uint8_t num_bits)
{
    if (num_bits < 2 || num_bits > 31)
    {
        throw std::invalid_argument("Number of bits must be between 2 and 31.");
    }
    num_bits_ = num_bits;
    mask_ = calculateMask(num_bits_);
    generateSequence();
    reset();
}

void PRBSGenerator::setSeed(uint32_t seed)
{
    if (seed == 0)
    {
        throw std::invalid_argument("Seed cannot be zero.");
    }
    shift_register_ = seed;
    generateSequence();
    reset();
}

void PRBSGenerator::reset()
{
    current_index_ = 0;
}

uint8_t PRBSGenerator::getNextValue()
{
    if (sequence_.empty())
    {
        throw std::runtime_error("Sequence is empty. Generate the sequence first.");
    }
    uint8_t value = sequence_[current_index_];
    current_index_ = (current_index_ + 1) % sequence_.size();
    return value;
}

uint32_t PRBSGenerator::calculateMask(uint8_t num_bits)
{
    switch (num_bits)
    {
    case 2:
        return 0x3;
    case 3:
        return 0x6;
    case 4:
        return 0xC;
    case 5:
        return 0x14;
    case 6:
        return 0x30;
    case 7:
        return 0x60;
    case 8:
        return 0xB8;
    case 9:
        return 0x110;
    case 10:
        return 0x240;
    // Add more sequences
    default:
        return 0xB8;
    }
}

void PRBSGenerator::generateSequence()
{
    sequence_.clear();
    shift_register_ = shift_register_;

    for (size_t i = 0; i < (1u << num_bits_) - 1; ++i)
    {
        uint8_t new_bit = shift_register_ & 0x1;
        sequence_.push_back(new_bit);
        shift_register_ >>= 1;
        if (new_bit)
        {
            shift_register_ ^= mask_;
        }
    }
    if (sequence_.empty())
    {
        throw std::runtime_error("Sequence generation failed, sequence is empty.");
    }
    reset();
}
