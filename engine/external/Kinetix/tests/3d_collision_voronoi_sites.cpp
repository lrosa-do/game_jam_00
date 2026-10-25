#include "check.h"

#include "geometry/VoronoiShatter.h"

#include <kinetix/core/Math.h>

using namespace kinetix::Geometry;
using kinetix::f32;
namespace KM = kinetix::Math;

namespace
{

bool finite(const KM::vec3& v)
{
    return isfinite(v.x) && isfinite(v.y) && isfinite(v.z);
}

void checkShards(const ct::Vector<Shard>& shards, f32 expectedVolume)
{
    f32 total = 0.0f;
    for (const Shard& shard : shards)
    {
        CHECK(isfinite(shard.volume) && shard.volume > 0.0f);
        CHECK(finite(shard.centroid));
        for (const KM::vec3& vertex : shard.vertices)
            CHECK(finite(vertex));
        total += shard.volume;
    }
    CHECK_NEAR(total, expectedVolume, 1e-3f);
}

}

int main()
{
    ct::Vector<KM::vec3> cube;
    for (int i = 0; i < 8; ++i)
        cube.push_back(KM::vec3(i & 1 ? 1.0f : -1.0f, i & 2 ? 1.0f : -1.0f, i & 4 ? 1.0f : -1.0f));
    ct::Vector<Shard> shards;

    VoronoiShatter::shatter(cube, {KM::vec3(-0.5f, 0, 0), KM::vec3(0.5f, 0, 0),
                                   KM::vec3(0.5f, 0, 0)},
                            shards);
    CHECK(shards.size() == 2);
    checkShards(shards, 8.0f);

    VoronoiShatter::shatter(cube, {KM::vec3(0.2f, 0.1f, 0), KM::vec3(0.2f, 0.1f, 0),
                                   KM::vec3(0.2f, 0.1f, 0)},
                            shards);
    CHECK(shards.size() == 1);
    checkShards(shards, 8.0f);

    // A site outside the cube whose bisector with the inner one is exactly
    // the cube's face: its cell is that face, flat, and gives no shard.
    VoronoiShatter::shatter(cube, {KM::vec3(0.0f), KM::vec3(2.0f, 0.0f, 0.0f)}, shards);
    CHECK(shards.size() == 1);
    checkShards(shards, 8.0f);
    return TEST_RESULT();
}
