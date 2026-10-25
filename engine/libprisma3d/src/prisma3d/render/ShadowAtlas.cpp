#include "prisma3d/render/ShadowAtlas.h"

namespace prisma3d
{

namespace
{

std::uint32_t nextPowerOfTwo(std::uint32_t value)
{
    if (value == 0) return 0;
    --value;
    value |= value >> 1;
    value |= value >> 2;
    value |= value >> 4;
    value |= value >> 8;
    value |= value >> 16;
    return value + 1;
}

std::uint32_t integerSqrt(std::uint32_t value)
{
    std::uint32_t root = 0;
    while ((root + 1) * (root + 1) <= value) ++root;
    return root;
}

} // namespace

ShadowAtlas::ShadowAtlas()
        : mSize(4096),
          mSixteenBits(false),
          mSmallestSubdivision(0),
          mSizeOrder{ 0, 1, 2, 3 }
{
    setQuadrantSubdivision(0, 4);
    setQuadrantSubdivision(1, 4);
    setQuadrantSubdivision(2, 16);
    setQuadrantSubdivision(3, 64);
}

void ShadowAtlas::setSize(std::uint32_t size, bool sixteenBits)
{
    size = nextPowerOfTwo(size);
    if (size == mSize && sixteenBits == mSixteenBits) return;

    for (Quadrant& quadrant: mQuadrants)
    {
        quadrant.shadows.clear();
        quadrant.shadows.resize(quadrant.subdivision * quadrant.subdivision);
    }
    mOwners.clear();
    mSize = size;
    mSixteenBits = sixteenBits;
}

void ShadowAtlas::setQuadrantSubdivision(std::uint32_t quadrant, std::uint32_t subdivision)
{
    if (quadrant >= 4 || subdivision >= 16384) return;

    std::uint32_t rounded = nextPowerOfTwo(subdivision);
    if (rounded & 0xaaaaaaaau) rounded <<= 1;
    const std::uint32_t side = integerSqrt(rounded);

    if (mQuadrants[quadrant].subdivision == side) return;

    for (const Shadow& shadow: mQuadrants[quadrant].shadows)
        if (shadow.owner != 0) mOwners.erase(shadow.owner);

    mQuadrants[quadrant].shadows.clear();
    mQuadrants[quadrant].shadows.resize(side * side);
    mQuadrants[quadrant].subdivision = side;

    mSmallestSubdivision = 1u << 30;
    for (const Quadrant& item: mQuadrants)
        if (item.subdivision != 0 && item.subdivision < mSmallestSubdivision)
            mSmallestSubdivision = item.subdivision;
    if (mSmallestSubdivision == (1u << 30)) mSmallestSubdivision = 0;

    int swaps = 0;
    do
    {
        swaps = 0;
        for (int i = 0; i < 3; ++i)
        {
            if (mQuadrants[mSizeOrder[i]].subdivision < mQuadrants[mSizeOrder[i + 1]].subdivision)
            {
                const int swap = mSizeOrder[i];
                mSizeOrder[i] = mSizeOrder[i + 1];
                mSizeOrder[i + 1] = swap;
                ++swaps;
            }
        }
    } while (swaps > 0);
}

std::uint32_t ShadowAtlas::getSize() const
{
    return mSize;
}

bool ShadowAtlas::getSixteenBits() const
{
    return mSixteenBits;
}

std::uint32_t ShadowAtlas::getSubdivision(std::uint32_t quadrant) const
{
    return quadrant < 4 ? mQuadrants[quadrant].subdivision : 0;
}

std::uint32_t ShadowAtlas::getSmallestSubdivision() const
{
    return mSmallestSubdivision;
}

void ShadowAtlas::markVisible(std::uint32_t light, std::uint64_t scenePass)
{
    if (Owner* owner = mOwners.find(light)) owner->lastScenePass = scenePass;
}

bool ShadowAtlas::findShadow(const int* quadrants, int quadrantCount, int currentSubdivision,
        std::uint64_t tick, std::uint64_t scenePass, int& quadrant, int& shadow) const
{
    for (int i = quadrantCount - 1; i >= 0; --i)
    {
        const int index = quadrants[i];
        if (mQuadrants[index].subdivision == static_cast<std::uint32_t>(currentSubdivision))
            return false;

        const std::size_t count = mQuadrants[index].shadows.size();
        const Shadow* shadows = mQuadrants[index].shadows.data();

        int freeIndex = -1;
        int usedIndex = -1;
        std::uint64_t minPass = 0;

        for (std::size_t j = 0; j < count; ++j)
        {
            if (shadows[j].owner == 0)
            {
                freeIndex = static_cast<int>(j);
                break;
            }

            const Owner* owner = mOwners.find(shadows[j].owner);
            if (!owner) continue;

            if (owner->lastScenePass != scenePass)
            {
                if (tick - shadows[j].allocTick < kReallocToleranceMsec) continue;

                if (usedIndex == -1 || owner->lastScenePass < minPass)
                {
                    usedIndex = static_cast<int>(j);
                    minPass = owner->lastScenePass;
                }
            }
        }

        if (freeIndex == -1 && usedIndex == -1) continue;
        if (freeIndex == -1 && usedIndex != -1) freeIndex = usedIndex;

        quadrant = index;
        shadow = freeIndex;
        return true;
    }
    return false;
}

bool ShadowAtlas::findOmniShadows(const int* quadrants, int quadrantCount, int currentSubdivision,
        std::uint64_t tick, std::uint64_t scenePass, int& quadrant, int& shadow) const
{
    for (int i = quadrantCount - 1; i >= 0; --i)
    {
        const int index = quadrants[i];
        if (mQuadrants[index].subdivision == static_cast<std::uint32_t>(currentSubdivision))
            return false;

        const int count = static_cast<int>(mQuadrants[index].shadows.size());
        const Shadow* shadows = mQuadrants[index].shadows.data();

        int found = -1;
        std::uint64_t minPass = 0;

        for (int j = 0; j < count - 1; ++j)
        {
            std::uint64_t pass = 0;
            bool usable = true;

            for (int k = 0; k < 2 && usable; ++k)
            {
                if (shadows[j + k].owner == 0) continue;
                const Owner* owner = mOwners.find(shadows[j + k].owner);
                if (!owner) continue;
                if (owner->lastScenePass == scenePass ||
                        tick - shadows[j + k].allocTick < kReallocToleranceMsec)
                {
                    usable = false;
                    break;
                }
                pass += owner->lastScenePass;
            }
            if (!usable) continue;

            if (found == -1 || pass < minPass)
            {
                found = j;
                minPass = pass;
                if (pass == 0) break;
            }
        }

        if (found == -1) continue;

        quadrant = index;
        shadow = found;
        return true;
    }
    return false;
}

void ShadowAtlas::invalidateShadow(std::uint32_t quadrant, std::uint32_t index)
{
    Shadow& shadow = mQuadrants[quadrant].shadows[index];
    if (shadow.owner == 0) return;

    const Owner* owner = mOwners.find(shadow.owner);
    if (owner && (owner->key & kOmniFlag))
    {
        const std::uint32_t first = owner->key & kIndexMask;
        const std::uint32_t other = index + (first == index ? 1 : static_cast<std::uint32_t>(-1));
        mQuadrants[quadrant].shadows[other].owner = 0;
    }
    mOwners.erase(shadow.owner);
    shadow.owner = 0;
}

bool ShadowAtlas::updateLight(std::uint32_t light, bool omni, float coverage,
        std::uint64_t tickMsec, std::uint64_t scenePass)
{
    if (mSize == 0 || mSmallestSubdivision == 0) return false;

    const std::uint32_t quadSize = mSize >> 1;
    const std::uint32_t coverageFit =
            nextPowerOfTwo(static_cast<std::uint32_t>(static_cast<float>(quadSize) * coverage));
    const std::uint32_t largestFit = quadSize / mSmallestSubdivision;
    const std::uint32_t desiredFit = largestFit < coverageFit ? largestFit : coverageFit;

    int validQuadrants[4];
    int validCount = 0;
    int bestSize = -1;
    int bestSubdivision = -1;

    for (int i = 0; i < 4; ++i)
    {
        const int q = mSizeOrder[i];
        const int sd = static_cast<int>(mQuadrants[q].subdivision);
        if (sd == 0) continue;

        const int maxFit = static_cast<int>(quadSize) / sd;
        if (bestSize != -1 && maxFit > bestSize) break;

        validQuadrants[validCount++] = q;
        bestSubdivision = sd;

        if (maxFit >= static_cast<int>(desiredFit)) bestSize = maxFit;
    }

    if (validCount == 0) return false;

    std::uint32_t oldKey = 0xffffffffu;
    std::uint32_t oldQuadrant = 0xffffffffu;
    std::uint32_t oldShadow = 0xffffffffu;
    int oldSubdivision = -1;
    bool shouldRealloc = false;

    if (Owner* owner = mOwners.find(light))
    {
        oldKey = owner->key;
        oldQuadrant = (oldKey >> kQuadrantShift) & 0x3;
        oldShadow = oldKey & kIndexMask;
        shouldRealloc = static_cast<int>(mQuadrants[oldQuadrant].subdivision) != bestSubdivision &&
                        (tickMsec - mQuadrants[oldQuadrant].shadows[oldShadow].allocTick >
                                kReallocToleranceMsec);
        if (!shouldRealloc) return true;
        oldSubdivision = static_cast<int>(mQuadrants[oldQuadrant].subdivision);
    }

    int newQuadrant = -1;
    int newShadow = -1;
    const bool found = omni ? findOmniShadows(validQuadrants, validCount, oldSubdivision, tickMsec,
                                      scenePass, newQuadrant, newShadow)
                            : findShadow(validQuadrants, validCount, oldSubdivision, tickMsec,
                                      scenePass, newQuadrant, newShadow);

    if (found)
    {
        if (oldQuadrant != 0xffffffffu)
        {
            mQuadrants[oldQuadrant].shadows[oldShadow].owner = 0;
            if (oldKey & kOmniFlag) mQuadrants[oldQuadrant].shadows[oldShadow + 1].owner = 0;
        }

        std::uint32_t newKey = static_cast<std::uint32_t>(newQuadrant) << kQuadrantShift;
        newKey |= static_cast<std::uint32_t>(newShadow);

        invalidateShadow(static_cast<std::uint32_t>(newQuadrant),
                static_cast<std::uint32_t>(newShadow));
        Shadow& first = mQuadrants[newQuadrant].shadows[static_cast<std::size_t>(newShadow)];
        first.owner = light;
        first.allocTick = tickMsec;

        if (omni)
        {
            newKey |= kOmniFlag;
            invalidateShadow(static_cast<std::uint32_t>(newQuadrant),
                    static_cast<std::uint32_t>(newShadow + 1));
            Shadow& second =
                    mQuadrants[newQuadrant].shadows[static_cast<std::size_t>(newShadow + 1)];
            second.owner = light;
            second.allocTick = tickMsec;
        }

        Owner owner;
        owner.key = newKey;
        owner.lastScenePass = scenePass;
        mOwners[light] = owner;
        return true;
    }

    return oldKey != 0xffffffffu;
}

void ShadowAtlas::releaseLight(std::uint32_t light)
{
    const Owner* owner = mOwners.find(light);
    if (!owner) return;

    const std::uint32_t quadrant = (owner->key >> kQuadrantShift) & 0x3;
    const std::uint32_t shadow = owner->key & kIndexMask;
    mQuadrants[quadrant].shadows[shadow].owner = 0;
    if (owner->key & kOmniFlag) mQuadrants[quadrant].shadows[shadow + 1].owner = 0;
    mOwners.erase(light);
}

void ShadowAtlas::clearOwners()
{
    for (Quadrant& quadrant: mQuadrants)
    {
        quadrant.shadows.clear();
        quadrant.shadows.resize(quadrant.subdivision * quadrant.subdivision);
    }
    mOwners.clear();
}

bool ShadowAtlas::ownsLight(std::uint32_t light) const
{
    return mOwners.find(light) != nullptr;
}

bool ShadowAtlas::getSlot(std::uint32_t light, Slot& slot) const
{
    const Owner* owner = mOwners.find(light);
    if (!owner) return false;
    slot.quadrant = (owner->key >> kQuadrantShift) & 0x3;
    slot.shadow = owner->key & kIndexMask;
    slot.omni = (owner->key & kOmniFlag) != 0;
    return true;
}

std::uint32_t ShadowAtlas::getShadowSize(std::uint32_t light) const
{
    Slot slot;
    if (!getSlot(light, slot)) return 0;
    return (mSize >> 1) / mQuadrants[slot.quadrant].subdivision;
}

bool ShadowAtlas::getPixelRect(std::uint32_t light, std::uint32_t rect[4]) const
{
    Slot slot;
    if (!getSlot(light, slot)) return false;

    const std::uint32_t quadrantSize = mSize >> 1;
    const std::uint32_t subdivision = mQuadrants[slot.quadrant].subdivision;
    const std::uint32_t shadowSize = quadrantSize / subdivision;
    rect[0] = (slot.quadrant & 1) * quadrantSize + (slot.shadow % subdivision) * shadowSize;
    rect[1] = (slot.quadrant >> 1) * quadrantSize + (slot.shadow / subdivision) * shadowSize;
    rect[2] = shadowSize;
    rect[3] = shadowSize;
    return true;
}

bool ShadowAtlas::getRect(std::uint32_t light, float rect[4], int omniOffset[2]) const
{
    std::uint32_t pixels[4];
    Slot slot;
    if (!getSlot(light, slot) || !getPixelRect(light, pixels)) return false;

    const float size = static_cast<float>(mSize);
    rect[0] = static_cast<float>(pixels[0]) / size;
    rect[1] = static_cast<float>(pixels[1]) / size;
    rect[2] = static_cast<float>(pixels[2]) / size;
    rect[3] = static_cast<float>(pixels[3]) / size;

    omniOffset[0] = 0;
    omniOffset[1] = 0;
    if (slot.omni)
    {
        const std::uint32_t subdivision = mQuadrants[slot.quadrant].subdivision;
        if ((slot.shadow + 1) % subdivision == 0)
        {
            omniOffset[0] = 1 - static_cast<int>(subdivision);
            omniOffset[1] = 1;
        }
        else
        {
            omniOffset[0] = 1;
            omniOffset[1] = 0;
        }
    }
    return true;
}

std::uint32_t ShadowAtlas::getOwnerCount() const
{
    return static_cast<std::uint32_t>(mOwners.size());
}

} // namespace prisma3d
