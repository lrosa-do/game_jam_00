#pragma once

namespace prisma3d
{

// SMAA's precomputed tables as PNG: the area texture (160x560, two channels) and the search
// texture (64x16, one channel).
extern const unsigned char kSmaaAreaTexPng[];
extern const unsigned int kSmaaAreaTexPngSize;
extern const unsigned char kSmaaSearchTexPng[];
extern const unsigned int kSmaaSearchTexPngSize;

} // namespace prisma3d
