#ifndef KINETIX_PHYSICS_BEAM_STRUCTURE_H
#define KINETIX_PHYSICS_BEAM_STRUCTURE_H

#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include <ct/vector.hpp>

namespace kinetix::Physics
{

//  A structure of point masses (nodes) joined by springs with a damper (beams). A beam bends
//  for good once the force on it passes its yield, and breaks once it passes its strength;
//  triangles over the nodes are what other structures, and the structure itself, collide with.
//  The step is meant to be small (half a millisecond), as the springs are stiff.

//  How a surface answers to a node touching it.
struct ContactSurface
{
    f32 adhesionVelocity = 3.0f;  // below this slip speed the contact may hold (va)
    f32 staticFriction = 1.2f;    // (ms)
    f32 slidingFriction = 0.75f;  // (mc)
    f32 hydrodynamicFriction = 0.01f; // (t2) friction that grows with the slip speed
    f32 stribeckVelocity = 6.0f;  // (vs)
    f32 alpha = 2.0f;
    f32 strength = 1.0f;
};

//  What the structures stand on.
class BeamGround
{
public:
    virtual ~BeamGround() = default;
    virtual f32 heightAt(f32 x, f32 z) const = 0;
    virtual Math::vec3 normalAt(f32 x, f32 height, f32 z) const = 0;
    virtual const ContactSurface& surfaceAt(f32 x, f32 z) const = 0;
};

enum class BeamKind : u8
{
    Normal,  // can yield and break
    Hydro,   // a shock or a hydraulic: breaks, never yields
    Virtual  // holds the shape together, outside the mass of the structure
};

enum class BeamBound : u8
{
    None,
    Shock,   // a stiffer spring and damper outside the bounds, a shock or a tyre
    Rope,    // pulls and never pushes
    Support  // never pushes back, breaks once stretched past its limit
};

struct BeamNode
{
    Math::vec3 position{0.0f};
    Math::vec3 velocity{0.0f};
    Math::vec3 force{0.0f};
    f32 mass = 10.0f;
    f32 frictionCoef = 1.0f;
    f32 volumeCoef = 1.0f;
    f32 surfaceCoef = 1.0f;
    f32 minimass = 50.0f;
    f32 overrideMass = -1.0f; // a load set by the definition, negative for none
    bool loaded = false;      // takes a share of the load
    bool noGroundContact = false;
    bool tyre = false;
    bool contacter = false;   // collides with the triangles of other structures
    bool contactable = false; // is a point the triangles of other structures are tested with
    bool cabNode = false;     // one of the nodes of a collision triangle
    bool hasGroundContact = false;
    bool hasMeshContact = false;
};

struct Beam
{
    u32 a = 0;
    u32 b = 0;
    BeamKind kind = BeamKind::Normal;
    BeamBound bound = BeamBound::None;
    f32 spring = 0.0f;
    f32 damping = 0.0f;
    f32 length = 0.0f;       // the rest length, changed by yielding
    f32 referenceLength = 0.0f;
    f32 strength = 0.0f;
    f32 maxPositiveStress = 0.0f;
    f32 maxNegativeStress = 0.0f;
    f32 minMaxStress = 0.0f;
    f32 plasticCoef = 0.0f;
    f32 shortBound = 0.0f;
    f32 longBound = 0.0f;
    f32 boundSpring = 9000000.0f; // what a shock goes over to outside its bounds
    f32 boundDamping = 12000.0f;
    f32 stress = 0.0f;
    bool broken = false;
    bool disabled = false;
};

struct BeamTriangle
{
    u32 a = 0;
    u32 b = 0;
    u32 c = 0;
    bool collides = false;
    //  How often a triangle is looked at, to spare the ones with nothing near them.
    int interRate = 0;
    int interDistance = 0;
    int intraRate = 0;
    int intraDistance = 0;
};

class BeamStructure
{
public:
    //  Definition. Positions are in the world, the spring and damper in N/m and Ns/m.
    u32 addNode(const Math::vec3& position);
    u32 addBeam(u32 a, u32 b, BeamKind kind, BeamBound bound, f32 spring, f32 damping, f32 strength,
                f32 deformThreshold, f32 plasticCoef);
    void addTriangle(u32 a, u32 b, u32 c, bool collides);
    BeamNode& node(u32 index)
    {
        return mNodes[index];
    }
    Beam& beam(u32 index)
    {
        return mBeams[index];
    }

    //  Once everything is added: spreads the dry mass over the beams, the load over the loaded
    //  nodes, lifts the nodes to their minimum mass, and works out which nodes touch what.
    void finish(f32 dryMass, f32 loadMass, f32 collisionRange, bool minimassSkipsLoaded);

    //  What one step of the structure itself is made of, in the order it is run in.
    void stepNodes(f32 dt, const BeamGround& ground, const Math::vec3& gravity);
    void stepBeams();
    void stepIntraCollisions(f32 dt, const ContactSurface& surface);
    void updateBounds();

    //  The nodes of the other structures that can collide, against the triangles of this one.
    void resolveInterCollisions(f32 dt, const ContactSurface& surface,
                                const ct::Vector<BeamStructure*>& structures);

    //  Moves every node by `offset` and gives them `velocity`.
    void place(const Math::vec3& offset, const Math::vec3& velocity);

    u32 nodeCount() const
    {
        return static_cast<u32>(mNodes.size());
    }
    u32 beamCount() const
    {
        return static_cast<u32>(mBeams.size());
    }
    u32 triangleCount() const
    {
        return static_cast<u32>(mTriangles.size());
    }
    const BeamNode& node(u32 index) const
    {
        return mNodes[index];
    }
    const Beam& beam(u32 index) const
    {
        return mBeams[index];
    }
    const BeamTriangle& triangle(u32 index) const
    {
        return mTriangles[index];
    }
    f32 totalMass() const
    {
        return mTotalMass;
    }
    u32 brokenBeams() const;
    //  The largest the rest length of a beam has changed, over its first, as a fraction.
    f32 worstYield() const;
    const Math::vec3& boundsMin() const
    {
        return mBoundsMin;
    }
    const Math::vec3& boundsMax() const
    {
        return mBoundsMax;
    }
    Math::vec3 centre() const;
    Math::vec3 averageVelocity() const;
    bool finite() const;

private:
    void countNeighbours();
    f32 random();

    ct::Vector<BeamNode> mNodes;
    ct::Vector<Beam> mBeams;
    ct::Vector<BeamTriangle> mTriangles;
    ct::Vector<ct::Vector<u32>> mNeighbours;
    ct::Vector<ct::Vector<u32>> mNodeBeams;
    f32 mCollisionRange = 0.02f;
    f32 mTotalMass = 0.0f;
    u32 mRandomState = 0x9E3779B9u;
    Math::vec3 mBoundsMin{0.0f};
    Math::vec3 mBoundsMax{0.0f};
};

//  A set of structures and what they stand on, stepped together.
class BeamWorld
{
public:
    static constexpr f32 kStep = 0.0005f;

    void setGround(const BeamGround* ground)
    {
        mGround = ground;
    }
    void setGravity(const Math::vec3& gravity)
    {
        mGravity = gravity;
    }
    void setStructureSurface(const ContactSurface& surface)
    {
        mStructureSurface = surface;
    }
    void add(BeamStructure* structure)
    {
        mStructures.push_back(structure);
    }
    void clear()
    {
        mStructures.clear();
    }
    //  One step of kStep.
    void step();

private:
    ct::Vector<BeamStructure*> mStructures;
    const BeamGround* mGround = nullptr;
    Math::vec3 mGravity{0.0f, -9.807f, 0.0f};
    ContactSurface mStructureSurface;
};

} // namespace kinetix::Physics

#endif // KINETIX_PHYSICS_BEAM_STRUCTURE_H
