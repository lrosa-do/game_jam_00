#pragma once

#include <ct/string.hpp>

#include <cstddef>
#include <cstdint>

namespace prisma3d
{

class ChaCha20
{
public:
    static constexpr std::size_t kKeySize = 32;
    static constexpr std::size_t kNonceSize = 12;
    static constexpr std::size_t kBlockSize = 64;
    static constexpr std::size_t kSaltSize = 16;

    ChaCha20();

    void setKey(const std::uint8_t key[kKeySize]);
    void setNonce(const std::uint8_t nonce[kNonceSize], std::uint32_t counter);

    void process(std::uint8_t* data, std::size_t size);

    static void deriveKey(const ct::String& passphrase, const std::uint8_t salt[kSaltSize],
            std::uint8_t key[kKeySize]);

private:
    static void block(const std::uint32_t state[16], std::uint8_t out[kBlockSize]);

    std::uint32_t mState[16];
};

} // namespace prisma3d
