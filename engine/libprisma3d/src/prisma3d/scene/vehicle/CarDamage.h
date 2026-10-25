#pragma once

#include "prisma3d/render/MaterialStore.h"
#include "prisma3d/render/MeshStore.h"

#include "ct/string.hpp"
#include "ct/vector.hpp"

#include "mathc.h"

namespace prisma3d
{

struct CarPieceSurface
{
    ct::String material;
    SurfaceData surface;
};

struct BreakPart
{
    enum class Kind
    {
        Mirror,
        Bumper,
        Door,
        Hood
    };
    Kind kind = Kind::Mirror;
    Math::Vec3 centre = Math::Vec3(0.0f, 0.0f, 0.0f);
    Math::Vec3 halfExtents = Math::Vec3(0.1f, 0.1f, 0.1f);
    float mass = 1.0f;
    float breakForce = 1000.0f;
    float side = 1.0f;
    Math::Vec3 hingePoint = Math::Vec3(0.0f, 0.0f, 0.0f);
    Math::Vec3 hingeAxis = Math::Vec3(0.0f, 1.0f, 0.0f);
    float openMin = 0.0f;
    float openMax = 0.0f;
    float latchForce = 2500.0f;
};

struct LoosePiece
{
    ct::Vector<SurfaceData> surfaces;
    ct::Vector<MaterialId> materials;
};

struct BrokenGlass
{
    ct::Vector<Math::Vec3> points;
    ct::Vector<Math::Vec3> normals;
};

class CarDamage
{
public:
    struct BulletHit
    {
        float distance = 0.0f;
        Math::Vec3 point = Math::Vec3(0.0f, 0.0f, 0.0f);
        Math::Vec3 normal = Math::Vec3(0.0f, 1.0f, 0.0f);
        std::size_t piece = 0;
        std::size_t part = 0;
        std::uint32_t triangle = 0;
        int loose = -1;
        bool glass = false;
    };

    enum class PieceKind
    {
        Body,
        Interior,
        Glass
    };

    MeshId addPiece(const ct::String& name, PieceKind kind,
            const ct::Vector<CarPieceSurface>& parts, const ct::Vector<MaterialId>& materials,
            const Math::Mat4& bake);

    void finish();

    void hit(const Math::Vec3& point, float impulse);

    bool raycast(const Math::Vec3& origin, const Math::Vec3& direction, float maxDistance,
            BulletHit& out) const;
    void bullet(const BulletHit& hit);

    void reset();

    bool update(float step);

    const ct::Vector<BreakPart>& parts() const
    {
        return mBreakParts;
    }
    bool detachPart(std::size_t index, LoosePiece& out);
    bool partOff(std::size_t index) const
    {
        return mLoose[index].gone;
    }

    float damageNear(const Math::Vec3& point, float radius) const;

    struct VertexRef
    {
        std::size_t piece = 0;
        std::size_t part = 0;
        std::size_t vertex = 0;
    };
    bool nearestBodyVertex(const Math::Vec3& point, VertexRef& out) const;
    Math::Vec3 vertexMoved(const VertexRef& ref) const;

    void takeBrokenGlass(BrokenGlass& out);

    float peak() const
    {
        return mPeak;
    }
    std::size_t touched() const;

    float noseCrush() const
    {
        return mNose;
    }
    float tailCrush() const
    {
        return mTail;
    }

    const Math::Vec3& low() const
    {
        return mLow;
    }
    const Math::Vec3& high() const
    {
        return mHigh;
    }

private:
    struct Part
    {
        SurfaceData base;
        SurfaceData shown;
        MaterialId material;
        ct::Vector<Math::Vec3> displacement;
        ct::Vector<float> weight;
        ct::Vector<float> drawn;
        ct::Vector<unsigned char> gone;
        ct::Vector<int> looseOf;
        ct::Vector<Math::Vec3> dentVec;
        ct::Vector<float> dentMark;
        ct::Vector<float> mix;
        std::uint32_t surface = 0;
        bool reindex = false;
    };
    struct Piece
    {
        MeshId mesh;
        PieceKind kind = PieceKind::Body;
        ct::Vector<Part> parts;
    };
    struct Slice
    {
        std::size_t piece = 0;
        std::size_t part = 0;
        ct::Vector<std::uint32_t> triangles;
    };
    struct Loose
    {
        ct::Vector<Slice> slices;
        bool gone = false;
    };

    Math::Vec3 crumple(const Math::Vec3& p) const;
    void findLoose();
    void breakGlass(Piece& piece);

    ct::Vector<Piece> mPieces;
    ct::Vector<BreakPart> mBreakParts;
    ct::Vector<Loose> mLoose;
    BrokenGlass mBroken;
    Math::Vec3 mLow = Math::Vec3(0.0f, 0.0f, 0.0f);
    Math::Vec3 mHigh = Math::Vec3(0.0f, 0.0f, 0.0f);
    bool mDirty = false;
    float mPeak = 0.0f;
    float mNose = 0.0f;
    float mTail = 0.0f;
};

} // namespace prisma3d
