#pragma once

#include <ct/hashmap.hpp>
#include <ct/vector.hpp>

#include <cstdint>

namespace prisma3d
{

class ShadowAtlas
{
public:
    static constexpr std::uint32_t kQuadrantShift = 27;
    static constexpr std::uint32_t kOmniFlag = 1u << 26;
    static constexpr std::uint32_t kIndexMask = kOmniFlag - 1;
    static constexpr std::uint64_t kReallocToleranceMsec = 500;

    struct Slot
    {
        std::uint32_t quadrant = 0;
        std::uint32_t shadow = 0;
        bool omni = false;
    };

    ShadowAtlas();

    void setSize(std::uint32_t size, bool sixteenBits);
    void setQuadrantSubdivision(std::uint32_t quadrant, std::uint32_t subdivision);
    std::uint32_t getSize() const;
    bool getSixteenBits() const;
    std::uint32_t getSubdivision(std::uint32_t quadrant) const;
    std::uint32_t getSmallestSubdivision() const;

    void markVisible(std::uint32_t light, std::uint64_t scenePass);
    bool updateLight(std::uint32_t light, bool omni, float coverage, std::uint64_t tickMsec,
            std::uint64_t scenePass);
    void releaseLight(std::uint32_t light);
    void clearOwners();

    bool ownsLight(std::uint32_t light) const;
    bool getSlot(std::uint32_t light, Slot& slot) const;
    std::uint32_t getShadowSize(std::uint32_t light) const;
    bool getPixelRect(std::uint32_t light, std::uint32_t rect[4]) const;
    bool getRect(std::uint32_t light, float rect[4], int omniOffset[2]) const;
    std::uint32_t getOwnerCount() const;

private:
    struct Shadow
    {
        std::uint32_t owner = 0;
        std::uint64_t allocTick = 0;
    };

    struct Quadrant
    {
        std::uint32_t subdivision = 0;
        ct::Vector<Shadow> shadows;
    };

    struct Owner
    {
        std::uint32_t key = 0;
        std::uint64_t lastScenePass = 0;
    };

    bool findShadow(const int* quadrants, int quadrantCount, int currentSubdivision,
            std::uint64_t tick, std::uint64_t scenePass, int& quadrant, int& shadow) const;
    bool findOmniShadows(const int* quadrants, int quadrantCount, int currentSubdivision,
            std::uint64_t tick, std::uint64_t scenePass, int& quadrant, int& shadow) const;
    void invalidateShadow(std::uint32_t quadrant, std::uint32_t index);

    std::uint32_t mSize;
    bool mSixteenBits;
    std::uint32_t mSmallestSubdivision;
    int mSizeOrder[4];
    Quadrant mQuadrants[4];
    ct::HashMap<std::uint32_t, Owner> mOwners;
};

} // namespace prisma3d
