#include "check.h"

#include "softbody/SoftBody.h"

#include <kinetix/core/Math.h>

using kinetix::u32;
using kinetix::Physics::SoftBody;

int main()
{
    const kinetix::Math::vec3 positions[4] = {{0, 0, 0}, {1, 0, 0}, {0, 0, 1}, {1, 0, 1}};
    const u32 quad[6] = {0, 1, 2, 1, 3, 2};

    for (SoftBody::BendType type : {SoftBody::BendType::Dihedral, SoftBody::BendType::Distance})
    {
        SoftBody body;
        body.setParticles(positions, 4, 1.0f);
        body.buildFromMesh(quad, 6, 0.0f, 0.0f, type);
        const u32 distance = body.constraintCount();
        const u32 bend = body.bendConstraintCount();
        body.buildFromMesh(quad, 6, 0.0f, 0.0f, type);
        CHECK(body.constraintCount() == distance);
        CHECK(body.bendConstraintCount() == bend);
        CHECK(body.faceCount() == 2);
        if (type == SoftBody::BendType::Dihedral)
        {
            CHECK(distance == 5);
            CHECK(bend == 1);
        }
        else
        {
            CHECK(distance == 6);
            CHECK(bend == 0);
        }
    }

    // A triangle naming a particle that does not exist is dropped, not read:
    // it shares edge 0-1 with a good one, so it would also have become the
    // far side of a dihedral bend.
    {
        SoftBody body;
        body.setParticles(positions, 3, 1.0f);
        const u32 bad[9] = {0, 1, 2, 1, 0, 40, 2, 2, 1};
        body.buildFromMesh(bad, 9, 0.0f, 0.0f, SoftBody::BendType::Dihedral);
        CHECK(body.faceCount() == 1);
        CHECK(body.constraintCount() == 3);
        CHECK(body.bendConstraintCount() == 0);

        body.setPressure(1.0f);
        body.step(1.0f / 60.0f, 2);
        CHECK(body.volumeTimesSix() == body.volumeTimesSix());
    }

    return TEST_RESULT();
}
