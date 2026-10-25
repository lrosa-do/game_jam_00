#pragma once

#include "mathc.h"

namespace prisma3d
{

class TerrainObserver
{
public:
    virtual ~TerrainObserver() = default;
    virtual void onTerrainChanged(const Math::Box& worldBox) = 0;
    virtual void onTerrainDestroyed() = 0;
};

} // namespace prisma3d
