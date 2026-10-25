#include "Check.h"

#include "prisma3d/scene/Node3D.h"
#include "prisma3d/scene/RibbonTrail3D.h"
#include "prisma3d/scene/SceneTree.h"

#include <math.h>
#include <string.h>

using namespace prisma3d;

namespace
{

const float kTolerance = 1e-4f;

bool near(float a, float b)
{
    return fabsf(a - b) <= kTolerance;
}

bool nearVec3(const Math::Vec3& a, float x, float y, float z)
{
    return near(a.x, x) && near(a.y, y) && near(a.z, z);
}

Node3D* addNode(Node* parent, const char* name, float x, float y, float z)
{
    Node3D* node = parent->createChild<Node3D>();
    node->setName(name);
    node->setPosition(Math::Vec3(x, y, z));
    return node;
}

Math::Vec3 midpoint(const SurfaceData& geometry, std::size_t a, std::size_t b)
{
    return (geometry.positions[a] + geometry.positions[b]) * 0.5f;
}

bool newestCentredOn(const SurfaceData& geometry, float x, float y, float z)
{
    const std::size_t count = geometry.positions.size();
    if (count < 2) return false;
    return nearVec3(midpoint(geometry, count - 2, count - 1), x, y, z);
}

float pairHalfWidth(const SurfaceData& geometry, std::size_t pair)
{
    return (geometry.positions[pair * 2 + 1] - geometry.positions[pair * 2]).Length() * 0.5f;
}

float distance(const Math::Vec3& a, const Math::Vec3& b)
{
    return (a - b).Length();
}

Math::Vec3 sample(const SurfaceData& geometry, int index, int smoothing)
{
    const std::size_t first = static_cast<std::size_t>(index * smoothing) * 2;
    return midpoint(geometry, first, first + 1);
}

bool close(const Math::Vec3& a, float x, float y, float z)
{
    return fabsf(a.x - x) < 2e-3f && fabsf(a.y - y) < 2e-3f && fabsf(a.z - z) < 2e-3f;
}

void testRibbonBaking()
{
    SceneTree tree;
    tree.initialize();
    Node3D* emitter = addNode(tree.getRoot(), "Emitter", 0.0f, 5.0f, 3.0f);
    RibbonTrail3D* trail = tree.getRoot()->createChild<RibbonTrail3D>();

    CHECK(trail->castTo<RibbonTrail3D>() == trail);
    CHECK(trail->castTo<VisualInstance3D>() == trail);
    CHECK(trail->castTo<MeshInstance3D>() == nullptr);
    CHECK(trail->getMaxElements() == 48);
    CHECK(trail->getSmoothing() == 4);
    CHECK(near(trail->getTrailLength(), 1.2f));
    CHECK(trail->isEmitting());

    CHECK(trail->addChain("") == -1);
    CHECK(trail->addChain("../Emitter") == 0);
    CHECK(trail->getChainCount() == 1);
    CHECK(trail->getElementCount(0) == 1);
    CHECK(trail->getElementCount(1) == 0);
    trail->update(0.0);
    CHECK(trail->getGeometry().positions.empty());

    emitter->setPosition(Math::Vec3(0.01f, 5.0f, 3.0f));
    trail->update(0.0);
    CHECK(trail->getElementCount(0) == 1);
    CHECK(trail->getGeometry().positions.empty());

    emitter->setPosition(Math::Vec3(0.3f, 5.0f, 3.0f));
    trail->update(0.0);
    CHECK(trail->getElementCount(0) == 2);
    CHECK(trail->getGeometry().positions.size() == 5 * 2);
    CHECK(trail->getGeometry().indices.size() == 4 * 6);
    CHECK(nearVec3(sample(trail->getGeometry(), 0, 4), 0.275f, 5.0f, 3.0f));
    CHECK(newestCentredOn(trail->getGeometry(), 0.3f, 5.0f, 3.0f));
    CHECK(near(pairHalfWidth(trail->getGeometry(), 4), 0.7f));

    emitter->setPosition(Math::Vec3(0.31f, 5.0f, 3.0f));
    trail->update(0.0);
    CHECK(trail->getElementCount(0) == 2);
    CHECK(newestCentredOn(trail->getGeometry(), 0.31f, 5.0f, 3.0f));

    emitter->setPosition(Math::Vec3(1.3f, 5.0f, 3.0f));
    trail->update(0.0);
    CHECK(trail->getElementCount(0) == 3);
    CHECK(nearVec3(sample(trail->getGeometry(), 0, 4), 0.275f, 5.0f, 3.0f));
    CHECK(nearVec3(sample(trail->getGeometry(), 1, 4), 1.275f, 5.0f, 3.0f));
    CHECK(nearVec3(sample(trail->getGeometry(), 2, 4), 1.3f, 5.0f, 3.0f));

    for (int i = 1; i <= 60; ++i)
    {
        emitter->setPosition(Math::Vec3(1.3f + 0.1f * static_cast<float>(i), 5.0f, 3.0f));
        trail->update(0.0);
        CHECK(trail->getElementCount(0) == (i + 3 < 48 ? i + 3 : 48));
    }
    CHECK(trail->getGeometry().positions.size() == (47 * 4 + 1) * 2);
    CHECK(newestCentredOn(trail->getGeometry(), 7.3f, 5.0f, 3.0f));
    CHECK(close(sample(trail->getGeometry(), 0, 4), 2.675f, 5.0f, 3.0f));

    trail->setMaxElements(60);
    CHECK(trail->getMaxElements() == 60);
    CHECK(trail->getElementCount(0) == 1);
    trail->setMaxElements(1);
    CHECK(trail->getMaxElements() == 2);
    trail->setMaxElements(500);
    CHECK(trail->getMaxElements() == 64);
    trail->setSmoothing(0);
    CHECK(trail->getSmoothing() == 1);
    trail->setSmoothing(20);
    CHECK(trail->getSmoothing() == 8);
    trail->setTrailLength(-1.0f);
    CHECK(near(trail->getTrailLength(), 0.01f));
}

struct Blade
{
    SceneTree tree;
    Node3D* sword;
    RibbonTrail3D* trail;

    Blade()
            : tree(),
              sword(nullptr),
              trail(nullptr)
    {
        tree.initialize();
        sword = addNode(tree.getRoot(), "Sword", 0.0f, 0.0f, 0.0f);
        addNode(sword, "Base", 0.0f, 0.0f, 0.0f);
        addNode(sword, "Tip", 0.0f, 1.0f, 0.0f);
        trail = tree.getRoot()->createChild<RibbonTrail3D>();
        trail->setTrailLength(0.02f);
    }
};

void testBladeGeometry()
{
    Blade blade;
    RibbonTrail3D* trail = blade.trail;
    CHECK(trail->addBladeChain("../Sword/Base", "") == -1);
    CHECK(trail->addBladeChain("../Sword/Base", "../Sword/Tip") == 0);
    CHECK(trail->getElementCount(0) == 1);

    for (int i = 1; i <= 4; ++i)
    {
        blade.sword->setPosition(Math::Vec3(0.6f * static_cast<float>(i), 0.0f, 0.0f));
        trail->update(0.0);
        CHECK(trail->getElementCount(0) == i + 1);
    }
    const int n = trail->getElementCount(0);
    const SurfaceData& straight = trail->getGeometry();
    const std::size_t edges = static_cast<std::size_t>((n - 1) * 4 + 1);
    CHECK(straight.positions.size() == edges * 2);
    CHECK(straight.normals.size() == edges * 2);
    CHECK(straight.colors.size() == edges * 2);
    CHECK(straight.uvs.size() == edges * 2);
    CHECK(straight.indices.size() == (edges - 1) * 6);
    for (std::size_t e = 0; e < edges; ++e)
    {
        CHECK(near(straight.positions[e * 2].y, 0.0f) && near(straight.positions[e * 2].z, 0.0f));
        CHECK(near(straight.positions[e * 2 + 1].y, 1.0f));
        CHECK(near(straight.positions[e * 2 + 1].z, 0.0f));
        CHECK(near(straight.uvs[e * 2].x, 0.0f) && near(straight.uvs[e * 2 + 1].x, 1.0f));
        CHECK(near(straight.uvs[e * 2].y, static_cast<float>(e) / static_cast<float>(edges - 1)));
    }
    CHECK(close(straight.positions[1], 0.6f, 1.0f, 0.0f));
    CHECK(close(straight.positions[4 * 2 + 1], 1.2f, 1.0f, 0.0f));
    CHECK(close(straight.positions[8 * 2 + 1], 1.8f, 1.0f, 0.0f));
    CHECK(nearVec3(straight.positions[edges * 2 - 2], 2.4f, 0.0f, 0.0f));
    CHECK(nearVec3(straight.positions[edges * 2 - 1], 2.4f, 1.0f, 0.0f));
    for (std::size_t i = 0; i < straight.indices.size(); ++i)
        CHECK(straight.indices[i] < straight.positions.size());

    Blade arc;
    CHECK(arc.trail->addBladeChain("../Sword/Base", "../Sword/Tip") == 0);
    for (int i = 1; i <= 9; ++i)
    {
        arc.sword->setRotation(Math::Vec3(0.0f, 0.0f, 0.174532925f * static_cast<float>(i)));
        arc.trail->update(0.0);
    }
    CHECK(arc.trail->getElementCount(0) == 10);
    const SurfaceData& curved = arc.trail->getGeometry();
    const Math::Vec3 centre(0.0f, 0.0f, 0.0f);
    const Math::Vec3 from = curved.positions[(4 * 2) * 2 + 1];
    const Math::Vec3 to = curved.positions[(4 * 3) * 2 + 1];
    const Math::Vec3 middle = curved.positions[(4 * 2 + 2) * 2 + 1];
    CHECK(fabsf(distance(from, to) - 0.174311f) < 2e-3f);
    CHECK(distance(middle, centre) > distance((from + to) * 0.5f, centre) + 1e-3f);
    CHECK(fabsf(distance(middle, centre) - 1.0f) < 2e-3f);
    for (std::size_t e = 0; e * 2 < curved.positions.size(); ++e)
        CHECK(nearVec3(curved.positions[e * 2], 0.0f, 0.0f, 0.0f));
}

void testFadeAndEmission()
{
    Blade blade;
    RibbonTrail3D* trail = blade.trail;
    CHECK(trail->addBladeChain("../Sword/Base", "../Sword/Tip", Math::Vec4(1.0f, 1.0f, 1.0f, 1.0f),
                  Math::Vec4(1.0f, 1.0f, 1.0f, 0.0f), 0.4f) == 0);
    blade.sword->setPosition(Math::Vec3(0.6f, 0.0f, 0.0f));
    trail->update(0.0);
    blade.sword->setPosition(Math::Vec3(1.2f, 0.0f, 0.0f));
    trail->update(0.0);
    CHECK(trail->getElementCount(0) == 3);

    float previousSpan = 1.0f;
    float previousAlpha = 1.0f;
    for (int i = 1; i <= 3; ++i)
    {
        trail->update(0.1);
        const SurfaceData& geometry = trail->getGeometry();
        CHECK(!geometry.positions.empty());
        if (geometry.positions.empty()) return;
        const float span = distance(geometry.positions[0], geometry.positions[1]);
        const float alpha = geometry.colors[0].w;
        const float expected = 1.0f - 0.25f * static_cast<float>(i);
        CHECK(near(span, expected));
        CHECK(near(alpha, expected));
        CHECK(span < previousSpan && alpha < previousAlpha);
        previousSpan = span;
        previousAlpha = alpha;
        const std::size_t last = geometry.positions.size() - 2;
        CHECK(near(distance(geometry.positions[last], geometry.positions[last + 1]), 1.0f));
        CHECK(near(geometry.colors[last].w, 1.0f));
    }
    trail->update(0.1);
    trail->update(0.1);
    CHECK(trail->getElementCount(0) == 1);
    CHECK(trail->getGeometry().positions.empty());

    blade.sword->setPosition(Math::Vec3(1.8f, 0.0f, 0.0f));
    trail->update(0.0);
    blade.sword->setPosition(Math::Vec3(2.4f, 0.0f, 0.0f));
    trail->update(0.0);
    CHECK(trail->getElementCount(0) == 3);
    trail->setEmitting(false);
    CHECK(!trail->isEmitting());
    blade.sword->setPosition(Math::Vec3(9.0f, 0.0f, 0.0f));
    trail->update(0.1);
    CHECK(trail->getElementCount(0) == 3);
    const std::size_t last = trail->getGeometry().positions.size() - 2;
    CHECK(nearVec3(trail->getGeometry().positions[last], 2.4f, 0.0f, 0.0f));
    CHECK(near(trail->getGeometry().colors[last].w, 0.75f));
    for (int i = 0; i < 4; ++i) trail->update(0.1);
    CHECK(trail->getElementCount(0) == 1);
    CHECK(trail->getGeometry().positions.empty());

    trail->setEmitting(true);
    CHECK(trail->getElementCount(0) == 1);
    trail->update(0.0);
    CHECK(trail->getGeometry().positions.empty());
    blade.sword->setPosition(Math::Vec3(9.2f, 0.0f, 0.0f));
    trail->update(0.0);
    blade.sword->setPosition(Math::Vec3(9.4f, 0.0f, 0.0f));
    trail->update(0.0);
    CHECK(trail->getElementCount(0) == 3);
    CHECK(close(trail->getGeometry().positions[1], 9.2f, 1.0f, 0.0f));
    CHECK(newestCentredOn(trail->getGeometry(), 9.4f, 0.5f, 0.0f));
}

void testPathsAndSpace()
{
    SceneTree tree;
    tree.initialize();
    RibbonTrail3D* trail = tree.getRoot()->createChild<RibbonTrail3D>();
    trail->setPosition(Math::Vec3(10.0f, 0.0f, 0.0f));
    trail->setTrailLength(0.02f);

    CHECK(trail->addBladeChain("../Later/Base", "../Later/Tip") == 0);
    CHECK(trail->getElementCount(0) == 0);
    trail->update(0.1);
    CHECK(trail->getElementCount(0) == 0);
    CHECK(trail->getGeometry().positions.empty());

    Node3D* later = addNode(tree.getRoot(), "Later", 1.0f, 2.0f, 3.0f);
    addNode(later, "Base", 0.0f, 0.0f, 0.0f);
    trail->update(0.1);
    CHECK(trail->getElementCount(0) == 0);
    addNode(later, "Tip", 0.0f, 1.0f, 0.0f);
    trail->update(0.0);
    CHECK(trail->getElementCount(0) == 1);

    later->setPosition(Math::Vec3(1.3f, 2.0f, 3.0f));
    trail->update(0.0);
    later->setPosition(Math::Vec3(1.6f, 2.0f, 3.0f));
    trail->update(0.0);
    const SurfaceData& geometry = trail->getGeometry();
    CHECK(geometry.positions.size() == 9 * 2);
    if (geometry.positions.size() != 18) return;
    CHECK(close(geometry.positions[1], 1.3f - 10.0f, 3.0f, 3.0f));
    CHECK(nearVec3(geometry.positions[16], 1.6f - 10.0f, 2.0f, 3.0f));
    CHECK(nearVec3(geometry.positions[17], 1.6f - 10.0f, 3.0f, 3.0f));

    later->queueFree();
    tree.process(0.1);
    trail->update(0.1);
    trail->update(1.0);
    CHECK(trail->getElementCount(0) == 1);
    CHECK(trail->getGeometry().positions.empty());
}

void testSeveralChains()
{
    SceneTree tree;
    tree.initialize();
    Node3D* first = addNode(tree.getRoot(), "First", 0.0f, 5.0f, 3.0f);
    Node3D* second = addNode(tree.getRoot(), "Second", 0.0f, 8.0f, 3.0f);
    RibbonTrail3D* trail = tree.getRoot()->createChild<RibbonTrail3D>();

    CHECK(trail->addChain("../First") == 0);
    CHECK(trail->addChain("../Second") == 1);
    CHECK(trail->addChain("../First") == 2);
    CHECK(trail->addChain("../Second") == 3);
    CHECK(trail->addChain("../First") == -1);
    CHECK(trail->addBladeChain("../First", "../Second") == -1);
    trail->clearChains();
    CHECK(trail->getChainCount() == 0);

    CHECK(trail->addChain("../First") == 0);
    CHECK(trail->addChain("../Second") == 1);
    first->setPosition(Math::Vec3(0.2f, 5.0f, 3.0f));
    second->setPosition(Math::Vec3(0.3f, 8.0f, 3.0f));
    trail->update(0.0);
    second->setPosition(Math::Vec3(0.6f, 8.0f, 3.0f));
    trail->update(0.0);
    CHECK(trail->getElementCount(0) == 2);
    CHECK(trail->getElementCount(1) == 3);
    const SurfaceData& geometry = trail->getGeometry();
    const std::uint32_t firstVertices = 5 * 2;
    const std::uint32_t secondVertices = 9 * 2;
    CHECK(geometry.positions.size() == firstVertices + secondVertices);
    CHECK(geometry.indices.size() == (4 + 8) * 6);
    for (std::size_t i = 0; i < geometry.indices.size(); ++i)
    {
        if (i < 4 * 6) CHECK(geometry.indices[i] < firstVertices);
        else
            CHECK(geometry.indices[i] >= firstVertices &&
                    geometry.indices[i] < firstVertices + secondVertices);
    }
    CHECK(near(geometry.positions[0].y + geometry.positions[1].y, 10.0f));
    CHECK(near(geometry.positions[firstVertices].y + geometry.positions[firstVertices + 1].y,
            16.0f));

    trail->clearChains();
    CHECK(trail->getGeometry().positions.empty());
    trail->update(0.0);
    CHECK(trail->getGeometry().positions.empty());
}

} // namespace

int main()
{
    testRibbonBaking();
    testBladeGeometry();
    testFadeAndEmission();
    testPathsAndSpace();
    testSeveralChains();

    if (failures) printf("%d checks failed\n", failures);
    else
        printf("ok\n");
    return failures ? 1 : 0;
}
