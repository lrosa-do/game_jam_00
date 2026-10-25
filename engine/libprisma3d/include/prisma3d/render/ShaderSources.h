#pragma once

#include <cstddef>

namespace prisma3d
{

struct ShaderSourceEntry
{
    const char* name;
    unsigned offset;
    unsigned size;
};

extern const unsigned char kShaderSourcesPacked[];
extern const unsigned kShaderSourcesPackedSize;
extern const unsigned kShaderSourcesUnpackedSize;
extern const ShaderSourceEntry kShaderSources[];
extern const unsigned kShaderSourceCount;

const char* shaderSource(const char* name, std::size_t& size);
unsigned long long shaderSourcesHash();

} // namespace prisma3d
