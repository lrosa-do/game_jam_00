#include "prisma3d/core/ChaCha20.h"

#include <string.h>

namespace prisma3d
{

namespace
{

constexpr std::uint32_t kDeriveRounds = 4096;

std::uint32_t rotateLeft(std::uint32_t value, int bits)
{
    return (value << bits) | (value >> (32 - bits));
}

void quarterRound(std::uint32_t& a, std::uint32_t& b, std::uint32_t& c, std::uint32_t& d)
{
    a += b;
    d = rotateLeft(d ^ a, 16);
    c += d;
    b = rotateLeft(b ^ c, 12);
    a += b;
    d = rotateLeft(d ^ a, 8);
    c += d;
    b = rotateLeft(b ^ c, 7);
}

std::uint32_t readLE32(const std::uint8_t* bytes)
{
    return static_cast<std::uint32_t>(bytes[0]) | (static_cast<std::uint32_t>(bytes[1]) << 8) |
           (static_cast<std::uint32_t>(bytes[2]) << 16) |
           (static_cast<std::uint32_t>(bytes[3]) << 24);
}

void writeLE32(std::uint8_t* bytes, std::uint32_t value)
{
    bytes[0] = static_cast<std::uint8_t>(value);
    bytes[1] = static_cast<std::uint8_t>(value >> 8);
    bytes[2] = static_cast<std::uint8_t>(value >> 16);
    bytes[3] = static_cast<std::uint8_t>(value >> 24);
}

void writeLE64(std::uint8_t* bytes, std::uint64_t value)
{
    writeLE32(bytes, static_cast<std::uint32_t>(value));
    writeLE32(bytes + 4, static_cast<std::uint32_t>(value >> 32));
}

std::uint64_t foldPassphrase(const ct::String& passphrase, std::uint64_t seed)
{
    std::uint64_t hash = 14695981039346656037ull ^ seed;
    for (std::size_t i = 0; i < passphrase.size(); ++i)
    {
        hash ^= static_cast<std::uint8_t>(passphrase[i]);
        hash *= 1099511628211ull;
    }
    hash ^= passphrase.size();
    hash *= 1099511628211ull;
    return hash;
}

} // namespace

ChaCha20::ChaCha20()
{
    memset(mState, 0, sizeof(mState));
}

void ChaCha20::setKey(const std::uint8_t key[kKeySize])
{
    mState[0] = 0x61707865u;
    mState[1] = 0x3320646eu;
    mState[2] = 0x79622d32u;
    mState[3] = 0x6b206574u;
    for (std::uint32_t i = 0; i < 8; ++i) mState[4 + i] = readLE32(key + i * 4);
}

void ChaCha20::setNonce(const std::uint8_t nonce[kNonceSize], std::uint32_t counter)
{
    mState[12] = counter;
    mState[13] = readLE32(nonce);
    mState[14] = readLE32(nonce + 4);
    mState[15] = readLE32(nonce + 8);
}

void ChaCha20::block(const std::uint32_t state[16], std::uint8_t out[kBlockSize])
{
    std::uint32_t working[16];
    for (std::uint32_t i = 0; i < 16; ++i) working[i] = state[i];

    for (std::uint32_t i = 0; i < 10; ++i)
    {
        quarterRound(working[0], working[4], working[8], working[12]);
        quarterRound(working[1], working[5], working[9], working[13]);
        quarterRound(working[2], working[6], working[10], working[14]);
        quarterRound(working[3], working[7], working[11], working[15]);
        quarterRound(working[0], working[5], working[10], working[15]);
        quarterRound(working[1], working[6], working[11], working[12]);
        quarterRound(working[2], working[7], working[8], working[13]);
        quarterRound(working[3], working[4], working[9], working[14]);
    }

    for (std::uint32_t i = 0; i < 16; ++i) writeLE32(out + i * 4, working[i] + state[i]);
}

void ChaCha20::process(std::uint8_t* data, std::size_t size)
{
    std::uint8_t keystream[kBlockSize];
    std::size_t done = 0;
    while (done < size)
    {
        block(mState, keystream);
        ++mState[12];

        const std::size_t chunk = size - done < kBlockSize ? size - done : kBlockSize;
        for (std::size_t i = 0; i < chunk; ++i) data[done + i] ^= keystream[i];
        done += chunk;
    }
}

void ChaCha20::deriveKey(const ct::String& passphrase, const std::uint8_t salt[kSaltSize],
        std::uint8_t key[kKeySize])
{
    std::uint8_t seed[kKeySize];
    for (std::uint32_t lane = 0; lane < 4; ++lane)
        writeLE64(seed + lane * 8, foldPassphrase(passphrase, lane * 0x9e3779b97f4a7c15ull));

    std::uint8_t nonce[kNonceSize];
    memcpy(nonce, salt, kNonceSize);

    ChaCha20 stage;
    std::uint8_t keystream[kBlockSize];
    for (std::uint32_t round = 0; round < kDeriveRounds; ++round)
    {
        stage.setKey(seed);
        stage.setNonce(nonce, round);
        memset(keystream, 0, sizeof(keystream));
        stage.process(keystream, sizeof(keystream));
        memcpy(seed, keystream, kKeySize);
        for (std::size_t i = 0; i < kSaltSize; ++i) seed[i] ^= salt[i];
    }

    memcpy(key, seed, kKeySize);
}

} // namespace prisma3d
