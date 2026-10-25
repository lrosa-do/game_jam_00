#pragma once

#include "prisma/rhi/ShaderBlob.h"

#include <ct/string.hpp>
#include <ct/vector.hpp>

#include <cstddef>
#include <cstdint>

namespace prisma3d
{

struct ShaderVariant
{
    const char* file = nullptr;
    prisma::ShaderStage stage = prisma::ShaderStage::Vertex;
    ct::String defines;
};

struct ShaderCompilerStats
{
    std::uint32_t compiled = 0;
    std::uint32_t loaded = 0;
    std::uint32_t failed = 0;
    double compileMilliseconds = 0.0;
};

const prisma::ShaderBlob* compileShader(const ShaderVariant& variant, const prisma::Caps& caps);
prisma::ShaderDesc shaderDesc(const ShaderVariant& variant, const prisma::Caps& caps);
constexpr const char* kShaderCachePackFolder = "shader_cache";

void setShaderCacheDirectory(const ct::String& directory);
const ct::String& shaderCacheDirectory();
void shaderCacheFiles(ct::Vector<ct::String>& out);
void setShaderCacheEnabled(bool enabled);
const ShaderCompilerStats& shaderCompilerStats();
void releaseShaderCompiler();

} // namespace prisma3d
