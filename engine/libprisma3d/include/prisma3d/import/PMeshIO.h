#pragma once

#include "mathc.h"

#include <ct/string.hpp>
#include <ct/vector.hpp>

#include <cstddef>
#include <cstdint>

namespace prisma3d
{
namespace pmesh
{

class Reader
{
public:
    Reader(const std::uint8_t* data, std::size_t size);

    std::size_t tell() const;
    std::size_t remaining() const;
    bool available(std::uint64_t amount) const;
    bool fits(std::uint64_t count, std::uint64_t recordSize) const;

    bool u8(std::uint8_t& value);
    bool u16(std::uint16_t& value);
    bool u32(std::uint32_t& value);
    bool s32(std::int32_t& value);
    bool u64(std::uint64_t& value);
    bool f32(float& value);
    bool f32Finite(float& value);
    bool bytes(void* out, std::size_t size);
    bool skip(std::uint64_t size);
    bool string(ct::String& value);

    bool vec3(Math::Vec3& value);
    bool box(Math::Vec3& low, Math::Vec3& high);
    bool floats(float* out, std::size_t count);
    bool words(std::uint32_t* out, std::size_t count);
    bool mat4(Math::Mat4& value);

    bool header(std::uint32_t expectedMagic, std::uint32_t& flags);
    bool next(std::uint32_t& id, std::uint64_t& size);
    bool enter(std::uint64_t size);
    void leave();

    bool fail(const char* what);
    bool failed() const;
    const ct::String& error() const;

    static ct::String chunkName(std::uint32_t id);

private:
    const std::uint8_t* mData;
    std::size_t mPos;
    std::size_t mLimit;
    std::size_t mParentLimit;
    std::size_t mChunkEnd;
    std::size_t mChunkStart;
    std::uint32_t mChunkId;
    bool mInChunk;
    ct::String mError;
};

class Writer
{
public:
    explicit Writer(ct::Vector<std::uint8_t>& output);

    std::size_t tell() const;
    void header(std::uint32_t magic, std::uint32_t flags);
    std::size_t beginChunk(std::uint32_t id);
    void endChunk(std::size_t marker);

    void u8(std::uint8_t value);
    void u16(std::uint16_t value);
    void u32(std::uint32_t value);
    void s32(std::int32_t value);
    void u64(std::uint64_t value);
    void f32(float value);
    void bytes(const void* data, std::size_t size);
    void string(const ct::String& value);
    void floats(const float* data, std::size_t count);
    void mat4(const Math::Mat4& value);

private:
    ct::Vector<std::uint8_t>& mOutput;
};

} // namespace pmesh
} // namespace prisma3d
