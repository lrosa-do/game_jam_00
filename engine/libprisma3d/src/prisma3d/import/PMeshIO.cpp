#include "prisma3d/import/PMeshIO.h"

#include "prisma3d/import/PMeshFormat.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

namespace prisma3d
{
namespace pmesh
{

namespace
{

bool hostLittleEndian()
{
    const std::uint32_t probe = 1;
    std::uint8_t first = 0;
    memcpy(&first, &probe, 1);
    return first == 1;
}

std::uint32_t load32(const std::uint8_t* p)
{
    return static_cast<std::uint32_t>(p[0]) | (static_cast<std::uint32_t>(p[1]) << 8) |
           (static_cast<std::uint32_t>(p[2]) << 16) | (static_cast<std::uint32_t>(p[3]) << 24);
}

} // namespace

Reader::Reader(const std::uint8_t* data, std::size_t size)
        : mData(data),
          mPos(0),
          mLimit(size),
          mParentLimit(0),
          mChunkEnd(0),
          mChunkStart(0),
          mChunkId(0),
          mInChunk(false)
{
}

std::size_t Reader::tell() const
{
    return mPos;
}

std::size_t Reader::remaining() const
{
    return mPos <= mLimit ? mLimit - mPos : 0;
}

bool Reader::available(std::uint64_t amount) const
{
    return amount <= remaining();
}

bool Reader::fits(std::uint64_t count, std::uint64_t recordSize) const
{
    return recordSize == 0 || count <= remaining() / recordSize;
}

bool Reader::u8(std::uint8_t& value)
{
    if (!available(1)) return fail("truncated");
    value = mData[mPos++];
    return true;
}

bool Reader::u16(std::uint16_t& value)
{
    if (!available(2)) return fail("truncated");
    value = static_cast<std::uint16_t>(mData[mPos] | (mData[mPos + 1] << 8));
    mPos += 2;
    return true;
}

bool Reader::u32(std::uint32_t& value)
{
    if (!available(4)) return fail("truncated");
    value = load32(mData + mPos);
    mPos += 4;
    return true;
}

bool Reader::s32(std::int32_t& value)
{
    std::uint32_t raw = 0;
    if (!u32(raw)) return false;
    value = static_cast<std::int32_t>(raw);
    return true;
}

bool Reader::u64(std::uint64_t& value)
{
    if (!available(8)) return fail("truncated");
    value = static_cast<std::uint64_t>(load32(mData + mPos)) |
            (static_cast<std::uint64_t>(load32(mData + mPos + 4)) << 32);
    mPos += 8;
    return true;
}

bool Reader::f32(float& value)
{
    std::uint32_t bits = 0;
    if (!u32(bits)) return false;
    memcpy(&value, &bits, sizeof(value));
    return true;
}

bool Reader::f32Finite(float& value)
{
    if (!f32(value)) return false;
    if (value - value != 0.0f) return fail("value is not finite");
    return true;
}

bool Reader::bytes(void* out, std::size_t size)
{
    if (!available(size)) return fail("truncated");
    if (size != 0) memcpy(out, mData + mPos, size);
    mPos += size;
    return true;
}

bool Reader::skip(std::uint64_t size)
{
    if (!available(size)) return fail("truncated");
    mPos += static_cast<std::size_t>(size);
    return true;
}

bool Reader::string(ct::String& value)
{
    std::uint32_t size = 0;
    if (!u32(size)) return false;
    if (size > kMaxString) return fail("string is too long");
    if (!available(size)) return fail("truncated string");
    value = size == 0 ? ct::String()
                      : ct::String(reinterpret_cast<const char*>(mData + mPos), size);
    mPos += size;
    return true;
}

bool Reader::vec3(Math::Vec3& value)
{
    return f32Finite(value.x) && f32Finite(value.y) && f32Finite(value.z);
}

bool Reader::box(Math::Vec3& low, Math::Vec3& high)
{
    if (!vec3(low) || !vec3(high)) return false;
    if (low.x > high.x || low.y > high.y || low.z > high.z) return fail("inverted bounds");
    return true;
}

bool Reader::floats(float* out, std::size_t count)
{
    if (!fits(count, 4)) return fail("truncated");
    const std::size_t size = count * 4;
    if (hostLittleEndian())
    {
        if (size != 0) memcpy(out, mData + mPos, size);
    }
    else
    {
        for (std::size_t i = 0; i < count; ++i)
        {
            const std::uint32_t bits = load32(mData + mPos + i * 4);
            memcpy(out + i, &bits, 4);
        }
    }
    mPos += size;
    return true;
}

bool Reader::words(std::uint32_t* out, std::size_t count)
{
    if (!fits(count, 4)) return fail("truncated");
    const std::size_t size = count * 4;
    if (hostLittleEndian())
    {
        if (size != 0) memcpy(out, mData + mPos, size);
    }
    else
    {
        for (std::size_t i = 0; i < count; ++i) out[i] = load32(mData + mPos + i * 4);
    }
    mPos += size;
    return true;
}

bool Reader::mat4(Math::Mat4& value)
{
    return floats(value.Data(), 16);
}

ct::String Reader::chunkName(std::uint32_t id)
{
    char text[8];
    for (int i = 0; i < 4; ++i)
    {
        const char c = static_cast<char>((id >> (8 * i)) & 0xFFu);
        text[i] = (c >= 32 && c < 127) ? c : '?';
    }
    text[4] = 0;
    return ct::String(text);
}

bool Reader::header(std::uint32_t expectedMagic, std::uint32_t& flags)
{
    std::uint32_t magic = 0;
    std::uint32_t version = 0;
    if (!available(kHeaderSize)) return fail("file is smaller than the header");
    u32(magic);
    u32(version);
    u32(flags);
    if (magic != expectedMagic) return fail("wrong magic number");
    if ((version >> 16) != kVersionMajor) return fail("unsupported major version");
    if ((flags & LittleEndian) == 0) return fail("not a little-endian file");
    return true;
}

bool Reader::next(std::uint32_t& id, std::uint64_t& size)
{
    mChunkStart = mPos;
    if (remaining() < kChunkHeaderSize) return fail("truncated chunk header");
    u32(id);
    u64(size);
    mChunkId = id;
    mInChunk = true;
    if (size > remaining()) return fail("chunk is larger than the rest of the file");
    return true;
}

bool Reader::enter(std::uint64_t size)
{
    if (mChunkEnd != 0 || size > remaining()) return fail("cannot enter chunk");
    mParentLimit = mLimit;
    mChunkEnd = mPos + static_cast<std::size_t>(size);
    mLimit = mChunkEnd;
    return true;
}

void Reader::leave()
{
    if (mChunkEnd == 0) return;
    mPos = mChunkEnd;
    mLimit = mParentLimit;
    mParentLimit = 0;
    mChunkEnd = 0;
    mInChunk = false;
}

bool Reader::fail(const char* what)
{
    if (!mError.empty()) return false;
    char text[160];
    if (mInChunk)
        snprintf(text, sizeof(text), "chunk %s at offset %zu, read at offset %zu: %s",
                chunkName(mChunkId).c_str(), mChunkStart, mPos, what);
    else
        snprintf(text, sizeof(text), "at offset %zu: %s", mPos, what);
    mError = text;
    return false;
}

bool Reader::failed() const
{
    return !mError.empty();
}

const ct::String& Reader::error() const
{
    return mError;
}

Writer::Writer(ct::Vector<std::uint8_t>& output)
        : mOutput(output)
{
}

std::size_t Writer::tell() const
{
    return mOutput.size();
}

void Writer::header(std::uint32_t magic, std::uint32_t flags)
{
    u32(magic);
    u32(kVersion);
    u32(flags | LittleEndian);
}

std::size_t Writer::beginChunk(std::uint32_t id)
{
    u32(id);
    const std::size_t marker = mOutput.size();
    u64(0);
    return marker;
}

void Writer::endChunk(std::size_t marker)
{
    const std::uint64_t size = mOutput.size() - marker - 8;
    for (int i = 0; i < 8; ++i) mOutput[marker + i] = static_cast<std::uint8_t>(size >> (8 * i));
}

void Writer::u8(std::uint8_t value)
{
    mOutput.push_back(value);
}

void Writer::u16(std::uint16_t value)
{
    const std::uint8_t raw[2] = { static_cast<std::uint8_t>(value),
        static_cast<std::uint8_t>(value >> 8) };
    bytes(raw, 2);
}

void Writer::u32(std::uint32_t value)
{
    const std::uint8_t raw[4] = { static_cast<std::uint8_t>(value),
        static_cast<std::uint8_t>(value >> 8), static_cast<std::uint8_t>(value >> 16),
        static_cast<std::uint8_t>(value >> 24) };
    bytes(raw, 4);
}

void Writer::s32(std::int32_t value)
{
    u32(static_cast<std::uint32_t>(value));
}

void Writer::u64(std::uint64_t value)
{
    u32(static_cast<std::uint32_t>(value));
    u32(static_cast<std::uint32_t>(value >> 32));
}

void Writer::f32(float value)
{
    std::uint32_t bits = 0;
    memcpy(&bits, &value, sizeof(bits));
    u32(bits);
}

void Writer::bytes(const void* data, std::size_t size)
{
    if (size == 0) return;
    const std::size_t at = mOutput.size();
    mOutput.resize(at + size);
    memcpy(mOutput.data() + at, data, size);
}

void Writer::string(const ct::String& value)
{
    u32(static_cast<std::uint32_t>(value.size()));
    bytes(value.c_str(), value.size());
}

void Writer::floats(const float* data, std::size_t count)
{
    if (hostLittleEndian())
    {
        bytes(data, count * 4);
        return;
    }
    for (std::size_t i = 0; i < count; ++i) f32(data[i]);
}

void Writer::mat4(const Math::Mat4& value)
{
    floats(value.Data(), 16);
}

} // namespace pmesh
} // namespace prisma3d
