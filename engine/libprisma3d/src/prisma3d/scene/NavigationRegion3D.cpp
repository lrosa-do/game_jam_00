#include "prisma3d/scene/NavigationRegion3D.h"

#include "prisma3d/render/AssetLoader.h"
#include "prisma3d/render/MeshStore.h"
#include "prisma3d/scene/Collision.h"
#include "prisma3d/scene/CollisionShape3D.h"
#include "prisma3d/scene/ManualMesh3D.h"
#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/PhysicsBody3D.h"

#include "platform.h"

#include <NavMesh.h>
#include <NavMeshBuilder.h>

namespace prisma3d
{

namespace
{

// Frames to wait for the source to settle when assets are still streaming before baking anyway.
constexpr float kMaxWaitFrames = 600.0f;

// How far above and below a query point the surface is searched.
const Math::Vec3 kSearchExtents(2.0f, 4.0f, 2.0f);

// Static bodies in this group are left out of the bake.
const ct::String kIgnoreGroup("navigation_ignore");

// Debug triangles sit this far above the floor so they do not fight with it.
constexpr float kDebugLift = 0.04f;

} // namespace

struct NavigationRegion3D::State
{
    kinetix::AI::NavMesh mesh;
    int triangles = 0;
    double milliseconds = 0.0;
};

NavigationRegion3D::NavigationRegion3D()
        : mState(new State()),
          mDebug(nullptr),
          mCellSize(0.2f),
          mAgentHeight(1.8f),
          mAgentRadius(0.35f),
          mAgentMaxClimb(0.4f),
          mAgentMaxSlope(45.0f),
          mBakeOnReady(true),
          mShowDebug(false),
          mWaited(0.0f),
          mRandomSeed(0x9e3779b9u)
{
}

NavigationRegion3D::~NavigationRegion3D()
{
    delete mState;
}

void NavigationRegion3D::setSourcePath(const ct::String& path)
{
    mSourcePath = path;
}

ct::String NavigationRegion3D::getSourcePath() const
{
    return mSourcePath;
}

void NavigationRegion3D::setSeedPath(const ct::String& path)
{
    mSeedPath = path;
}

ct::String NavigationRegion3D::getSeedPath() const
{
    return mSeedPath;
}

void NavigationRegion3D::setCellSize(float size)
{
    mCellSize = size < 0.05f ? 0.05f : size;
}

float NavigationRegion3D::getCellSize() const
{
    return mCellSize;
}

void NavigationRegion3D::setAgentHeight(float height)
{
    mAgentHeight = height < 0.1f ? 0.1f : height;
}

float NavigationRegion3D::getAgentHeight() const
{
    return mAgentHeight;
}

void NavigationRegion3D::setAgentRadius(float radius)
{
    mAgentRadius = radius < 0.0f ? 0.0f : radius;
}

float NavigationRegion3D::getAgentRadius() const
{
    return mAgentRadius;
}

void NavigationRegion3D::setAgentMaxClimb(float climb)
{
    mAgentMaxClimb = climb < 0.0f ? 0.0f : climb;
}

float NavigationRegion3D::getAgentMaxClimb() const
{
    return mAgentMaxClimb;
}

void NavigationRegion3D::setAgentMaxSlope(float degrees)
{
    mAgentMaxSlope = degrees < 0.0f ? 0.0f : (degrees > 89.0f ? 89.0f : degrees);
}

float NavigationRegion3D::getAgentMaxSlope() const
{
    return mAgentMaxSlope;
}

void NavigationRegion3D::setBakeOnReady(bool bake)
{
    mBakeOnReady = bake;
}

bool NavigationRegion3D::getBakeOnReady() const
{
    return mBakeOnReady;
}

void NavigationRegion3D::setShowDebug(bool show)
{
    if (mShowDebug == show) return;
    mShowDebug = show;
    updateDebug();
}

bool NavigationRegion3D::getShowDebug() const
{
    return mShowDebug;
}

void NavigationRegion3D::collect(Node* node, ct::Vector<float>& vertices,
        ct::Vector<int>& indices) const
{
    if (node == mDebug) return;
    // Bodies a script marks as passable (a door that can be blown open) stay out of the floor.
    if (node->isInGroup(kIgnoreGroup)) return;
    const CollisionShape3D* shape = node->castTo<CollisionShape3D>();
    if (shape && !shape->isDisabled() && shape->getParent() &&
            shape->getParent()->castTo<StaticBody3D>())
    {
        const Math::Mat4 transform = shape->getGlobalTransform();
        auto add = [&](const Math::Vec3& local)
        {
            const Math::Vec3 point = transform.TransformPoint(local);
            vertices.push_back(point.x);
            vertices.push_back(point.y);
            vertices.push_back(point.z);
            return static_cast<int>(vertices.size() / 3 - 1);
        };
        if (shape->getKind() == CollisionShapeKind::ConcaveMesh)
        {
            const MeshStore* store = MeshStore::singleton();
            const CollisionMesh* mesh = store ? store->collision(shape->getConcaveMesh()) : nullptr;
            const std::uint32_t count = mesh ? mesh->triangleCount() : 0;
            for (std::uint32_t t = 0; t < count; ++t)
            {
                Math::Vec3 a;
                Math::Vec3 b;
                Math::Vec3 c;
                mesh->triangle(t, a, b, c);
                indices.push_back(add(a));
                indices.push_back(add(b));
                indices.push_back(add(c));
            }
        }
        else if (shape->getKind() == CollisionShapeKind::Box)
        {
            const Math::Vec3 h = shape->getBoxSize() * 0.5f;
            int corner[8];
            for (int i = 0; i < 8; ++i)
                corner[i] =
                        add(Math::Vec3(i & 1 ? h.x : -h.x, i & 2 ? h.y : -h.y, i & 4 ? h.z : -h.z));
            // Six faces, counter-clockwise from outside.
            static const int kFaces[12][3] = { { 0, 4, 6 }, { 0, 6, 2 }, { 1, 3, 7 }, { 1, 7, 5 },
                { 0, 1, 5 }, { 0, 5, 4 }, { 2, 6, 7 }, { 2, 7, 3 }, { 0, 2, 3 }, { 0, 3, 1 },
                { 4, 5, 7 }, { 4, 7, 6 } };
            for (const auto& face: kFaces)
                for (int k = 0; k < 3; ++k) indices.push_back(corner[face[k]]);
        }
    }
    for (std::size_t i = 0; i < node->getChildCount(); ++i)
        collect(node->getChild(i), vertices, indices);
}

bool NavigationRegion3D::bake()
{
    const std::uint64_t start = time_nanos();
    clear();
    Node* source = mSourcePath.empty() ? getParent() : getNode(mSourcePath);
    if (!source)
    {
        log_error("navigation: '%s' has no source node", getName().c_str());
        return false;
    }

    ct::Vector<float> vertices;
    ct::Vector<int> indices;
    collect(source, vertices, indices);
    if (indices.empty())
    {
        log_warn("navigation: '%s' found no static collision under '%s'", getName().c_str(),
                source->getName().c_str());
        return false;
    }

    kinetix::AI::NavMeshBuildConfig config;
    config.cellSize = mCellSize;
    config.agentHeight = mAgentHeight;
    config.agentRadius = mAgentRadius;
    config.agentMaxClimb = mAgentMaxClimb;
    config.agentMaxSlope = mAgentMaxSlope;
    ct::Vector<float> floorVertices;
    ct::Vector<int> floorIndices;
    kinetix::AI::NavMeshBuildStats stats;
    if (!kinetix::AI::NavMeshBuilder::build(vertices.data(), static_cast<int>(vertices.size() / 3),
                indices.data(), static_cast<int>(indices.size() / 3), config, floorVertices,
                floorIndices, &stats))
    {
        log_warn("navigation: '%s' has no floor an agent of %.2f m fits on", getName().c_str(),
                mAgentHeight);
        return false;
    }

    kinetix::AI::NavMeshConfig meshConfig;
    meshConfig.agentMaxClimb = mAgentMaxClimb;
    meshConfig.agentMaxSlope = mAgentMaxSlope;
    meshConfig.edgeWeldDistance = stats.cellSize * 0.25f;
    meshConfig.agentRadius = 0.0f; // the builder already kept the floor clear of the walls
    Math::Vec3 seed;
    const Node* seedTarget = mSeedPath.empty() ? nullptr : getNode(mSeedPath);
    const Node3D* seedNode = seedTarget ? seedTarget->castTo<Node3D>() : nullptr;
    if (seedNode) seed = seedNode->getGlobalTransform().GetTranslation();
    if (!mState->mesh.build(floorVertices.data(), static_cast<int>(floorVertices.size() / 3),
                floorIndices.data(), static_cast<int>(floorIndices.size() / 3), meshConfig,
                seedNode ? &seed : nullptr))
        return false;

    mState->triangles = static_cast<int>(mState->mesh.debugTriangles().size() / 3);
    mState->milliseconds = static_cast<double>(time_nanos() - start) * 1e-6;
    log_info("navigation: '%s' baked %d triangles from %zu in %.1f ms (%ux%u cells of %.2f m, %u "
             "floor, %u kept)",
            getName().c_str(), mState->triangles, indices.size() / 3, mState->milliseconds,
            stats.columnsX, stats.columnsZ, stats.cellSize, stats.floorCells, stats.keptCells);
    updateDebug();
    return true;
}

void NavigationRegion3D::clear()
{
    mState->mesh.release();
    mState->triangles = 0;
    mState->milliseconds = 0.0;
    updateDebug();
}

bool NavigationRegion3D::isBaked() const
{
    return mState->mesh.valid();
}

int NavigationRegion3D::getTriangleCount() const
{
    return mState->triangles;
}

double NavigationRegion3D::getBakeMilliseconds() const
{
    return mState->milliseconds;
}

bool NavigationRegion3D::findPath(const Math::Vec3& from, const Math::Vec3& to,
        ct::Vector<Math::Vec3>& path, bool* partial) const
{
    path.clear();
    if (partial) *partial = false;
    if (!isBaked()) return false;
    return mState->mesh.findPath(from, to, path, kSearchExtents, partial);
}

bool NavigationRegion3D::nearestPoint(const Math::Vec3& point, Math::Vec3& out) const
{
    if (!isBaked()) return false;
    return mState->mesh.nearestPoint(point, out, kSearchExtents);
}

bool NavigationRegion3D::moveAlongSurface(const Math::Vec3& from, const Math::Vec3& to,
        Math::Vec3& out) const
{
    if (!isBaked()) return false;
    return mState->mesh.moveAlongSurface(from, to, out, kSearchExtents);
}

bool NavigationRegion3D::raycast(const Math::Vec3& from, const Math::Vec3& to,
        float& fraction) const
{
    fraction = 1.0f;
    if (!isBaked()) return false;
    Math::Vec3 normal;
    return mState->mesh.raycast(from, to, fraction, normal, kSearchExtents);
}

bool NavigationRegion3D::randomPoint(Math::Vec3& out)
{
    if (!isBaked()) return false;
    std::uint32_t seed = mRandomSeed;
    const bool found = mState->mesh.randomPoint(seed, out);
    mRandomSeed = seed;
    return found;
}

void NavigationRegion3D::updateDebug()
{
    const bool show = mShowDebug && isBaked() && isInsideTree();
    if (!show)
    {
        if (mDebug) mDebug->setVisible(false);
        return;
    }
    if (!mDebug)
    {
        mDebug = createChild<ManualMesh3D>();
        mDebug->setInternal(true);
        mDebug->setName("NavigationDebug");
    }
    mDebug->clearSurfaces();
    mDebug->setVisible(true);
    // The surface is in global space; the debug mesh undoes this node's transform.
    mDebug->setGlobalTransform(Math::Mat4::Identity());
    const ct::Vector<Math::Vec3>& triangles = mState->mesh.debugTriangles();
    SurfaceData data;
    data.topology = prisma::Topology::Triangles;
    for (std::size_t i = 0; i + 2 < triangles.size(); i += 3)
    {
        for (int k = 0; k < 3; ++k)
        {
            data.positions.push_back(triangles[i + k] + Math::Vec3(0.0f, kDebugLift, 0.0f));
            data.normals.push_back(Math::Vec3(0.0f, 1.0f, 0.0f));
            data.colors.push_back(Math::Vec4(0.1f, 0.85f, 1.0f, 1.0f));
        }
    }
    if (!data.positions.empty()) mDebug->addSurface(data);
}

void NavigationRegion3D::notificationHandler(int what)
{
    Node3D::notificationHandler(what);
    switch (what)
    {
        case kNotificationEnterTree:
            addToGroup("navigation");
            break;
        case kNotificationReady:
            mWaited = 0.0f;
            if (mBakeOnReady && !isBaked()) setProcess(true);
            break;
        case kNotificationProcess:
        {
            if (!mBakeOnReady || isBaked())
            {
                setProcess(false);
                break;
            }
            // Wait for the models under the source to finish streaming in, which is when their
            // collision exists.
            const AssetLoader* loader = AssetLoader::singleton();
            mWaited += 1.0f;
            if (loader && loader->pendingRequests() > 0 && mWaited < kMaxWaitFrames) break;
            setProcess(false);
            bake();
            break;
        }
        default:
            break;
    }
}

void NavigationRegion3D::bindProperties()
{
    using DB = NodeClassDB;
    using P = NavigationRegion3D;
    DB::addProperty<P>(PropertyInfo::nodePath("source", "Node"), &P::setSourcePath,
            &P::getSourcePath);
    DB::addProperty<P>(PropertyInfo::nodePath("seed", "Node3D"), &P::setSeedPath, &P::getSeedPath);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "cell_size", 0.05, 2.0, 0.01),
            &P::setCellSize, &P::getCellSize);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "agent_height", 0.1, 10.0, 0.01),
            &P::setAgentHeight, &P::getAgentHeight);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "agent_radius", 0.0, 5.0, 0.01),
            &P::setAgentRadius, &P::getAgentRadius);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "agent_max_climb", 0.0, 5.0, 0.01),
            &P::setAgentMaxClimb, &P::getAgentMaxClimb);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "agent_max_slope", 0.0, 89.0, 0.1),
            &P::setAgentMaxSlope, &P::getAgentMaxSlope);
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "bake_on_ready"), &P::setBakeOnReady,
            &P::getBakeOnReady);
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "show_debug"), &P::setShowDebug,
            &P::getShowDebug);
}

} // namespace prisma3d
