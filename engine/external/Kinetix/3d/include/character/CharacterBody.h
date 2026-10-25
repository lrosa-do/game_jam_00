#ifndef KINETIX_PHYSICS_CHARACTER_BODY_H
#define KINETIX_PHYSICS_CHARACTER_BODY_H

#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include <ct/vector.hpp>
#include <math.h>

namespace kinetix::Physics
{

class TrimeshShape;
struct ContactManifold;

struct CharacterSweepHit
{
    u32 triangle = 0;
    // Fraction of the velocity travelled before contact, in [0, 1]. Negative
    // means the shape already overlapped, and the value is minus the depth
    // in world units along `normal`: moving the centre by -normal * t clears
    // the surface.
    f32 t = 0.0f;
    // Unit length, world space.
    Math::vec3 normal{0.0f, 1.0f, 0.0f};
};

// Whatever the character is swept through, in world space.
class CharacterSweepSource
{
public:
    virtual ~CharacterSweepSource() = default;

    // Moves an ellipsoid from `centre` along `velocity` and reports the first
    // thing it meets. Its half-extents are `radii`, measured along the
    // columns of `axes` (orthonormal, world space): the character's own
    // frame, with its up direction as the second column.
    virtual bool sweepEllipsoid(const Math::vec3& centre, const Math::mat3& axes,
                                const Math::vec3& radii, const Math::vec3& velocity,
                                CharacterSweepHit& hit) const = 0;
};

// `meshTransform` may be any invertible affine transform. While the ellipsoid
// stays axis-aligned in the mesh's own space the mesh's own sweep runs;
// otherwise the same unit-sphere sweep runs on the triangles taken into the
// ellipsoid's space.
class TrimeshSweepSource final : public CharacterSweepSource
{
public:
    TrimeshSweepSource(const TrimeshShape& mesh, const Math::mat4& meshTransform)
        : mMesh(mesh), mTransform(meshTransform)
    {
    }

    bool sweepEllipsoid(const Math::vec3& centre, const Math::mat3& axes, const Math::vec3& radii,
                        const Math::vec3& velocity, CharacterSweepHit& hit) const override;

private:
    const TrimeshShape& mMesh;
    Math::mat4 mTransform;
};

// A player or an NPC, moved by collide-and-slide against static world geometry.
class CharacterBody
{
public:
    struct MoveResult
    {
        bool collided = false;
        bool grounded = false;
        // A surface too steep to walk on stopped the horizontal move.
        bool blocked = false;
        bool onWall = false;
        bool onCeiling = false;
        // Flattest ground found this move - the one the character is really
        // standing on when a move touches two. Up when airborne.
        Math::vec3 groundNormal{0.0f, 1.0f, 0.0f};
        // The steepest wall and the ceiling touched this move, if any.
        Math::vec3 wallNormal{0.0f};
        Math::vec3 ceilingNormal{0.0f};
        // What the slide could not deliver, in world units.
        Math::vec3 remaining{0.0f};
    };

    // `radius` and `height` describe a capsule: height is the INNER segment,
    // so the whole thing stands 2*(radius + height/2) tall. The position is
    // its centre.
    void setShape(f32 radius, f32 height);
    f32 radius() const
    {
        return mRadius;
    }
    f32 height() const
    {
        return mHeight;
    }

    void setPosition(const Math::vec3& position)
    {
        mPosition = position;
    }
    const Math::vec3& position() const
    {
        return mPosition;
    }
    Math::mat4 transform() const;

    // Ground steeper than this is a wall: it blocks instead of carrying.
    void setSlopeLimit(f32 degrees);
    f32 slopeLimit() const
    {
        return mSlopeLimitDegrees;
    }
    // Kept this far off every surface, so a capsule resting on the floor is
    // never exactly touching it and the contact does not flicker.
    void setSkinWidth(f32 width)
    {
        mSkinWidth = Math::max(width, 0.0f);
    }
    // Slide passes per move. A corner needs more than one: sliding off the
    // first wall drives into the second.
    void setMaxIterations(u32 iterations)
    {
        mMaxIterations = Math::max(iterations, 1u);
    }

    // Moves by `displacement`, sliding along whatever it meets. The sweep is
    // continuous, so a fast move cannot pass through a wall.
    MoveResult move(const Math::vec3& displacement, const TrimeshShape& mesh,
                    const Math::mat4& meshTransform);
    MoveResult move(const Math::vec3& displacement, const CharacterSweepSource& source);

    // Applies gravity, then moves by the horizontal input; move() is the raw one.
    MoveResult update(f32 deltaTime, const TrimeshShape& mesh, const Math::mat4& meshTransform);
    MoveResult update(f32 deltaTime, const CharacterSweepSource& source);

    // Caller-owned drive: set the full velocity (gravity included) and call
    // moveAndSlide(); update() is the wrapper that owns gravity and input.
    void setVelocity(const Math::vec3& velocity)
    {
        mVelocity = velocity;
    }
    MoveResult moveAndSlide(f32 deltaTime, const TrimeshShape& mesh,
                            const Math::mat4& meshTransform);
    MoveResult moveAndSlide(f32 deltaTime, const CharacterSweepSource& source);
    // No grounded check, so a double jump is the caller's call.
    void setVerticalSpeed(f32 speed)
    {
        mVerticalSpeed = speed;
    }

    // The up axis the slope limit, ground snap and ceiling test are measured
    // against. Default (0,1,0).
    void setUpDirection(const Math::vec3& up)
    {
        mUpDirection = Math::normalize(up);
        mMoveInput -= mUpDirection * Math::dot(mMoveInput, mUpDirection);
    }
    const Math::vec3& upDirection() const
    {
        return mUpDirection;
    }

    // Stop the body when its motion is straight along -up on a slope, instead
    // of letting the slide carry it down the face. On by default.
    void setFloorStopOnSlope(bool on)
    {
        mFloorStopOnSlope = on;
    }
    // On the floor, pressing into a wall within this angle of head-on stops
    // the horizontal motion instead of sliding along the wall. Zero (the
    // default) disables it.
    void setWallMinSlideAngle(f32 degrees)
    {
        mWallMinSlideAngleDegrees = Math::clamp(degrees, 0.0f, 90.0f);
        mWallMinSlideAngleCosine = cosf(Math::radians(mWallMinSlideAngleDegrees));
    }
    f32 wallMinSlideAngle() const
    {
        return mWallMinSlideAngleDegrees;
    }
    // Keep sliding along a ceiling instead of stopping against it.
    void setSlideOnCeiling(bool on)
    {
        mSlideOnCeiling = on;
    }

    // Horizontal velocity the character is trying to hold, world units per
    // second. The component along upDirection() is dropped - gravity and
    // jump own that axis.
    void setMoveInput(const Math::vec3& velocity)
    {
        mMoveInput = velocity - mUpDirection * Math::dot(velocity, mUpDirection);
    }
    const Math::vec3& moveInput() const
    {
        return mMoveInput;
    }

    void setGravity(f32 gravity)
    {
        mGravity = gravity;
    }
    f32 gravity() const
    {
        return mGravity;
    }
    void setMaxFallSpeed(f32 speed)
    {
        mMaxFallSpeed = speed;
    }

    // How far below his feet the character still counts as standing. One
    // frame of falling covers less than the skin width, so without this
    // `grounded` flickers at rest.
    void setGroundSnapDistance(f32 distance)
    {
        mGroundSnapDistance = Math::max(distance, 0.0f);
    }
    f32 groundSnapDistance() const
    {
        return mGroundSnapDistance;
    }

    // How high a ledge the character climbs instead of being stopped by it.
    void setStepOffset(f32 offset)
    {
        mStepOffset = Math::max(offset, 0.0f);
    }
    f32 stepOffset() const
    {
        return mStepOffset;
    }

    // Ignored while airborne, so jump cannot be repeated in mid-air;
    // setVerticalSpeed() is the no-check alternative.
    void jump(f32 speed);
    void teleport(const Math::vec3& position);

    // Probe down by the snap distance and settle onto whatever is there,
    // without moving sideways. Called by update() when the slide loses the
    // floor.
    bool applyFloorSnap(const TrimeshShape& mesh, const Math::mat4& meshTransform);
    bool applyFloorSnap(const CharacterSweepSource& source);

    // Read-only applyFloorSnap(): whether a walkable floor sits within
    // `maxDistance` below. grounded() can flicker false for a frame on a small
    // step; use this for "close enough to the ground not to look airborne".
    bool isNearGround(const TrimeshShape& mesh, const Math::mat4& meshTransform,
                      f32 maxDistance) const;
    bool isNearGround(const CharacterSweepSource& source, f32 maxDistance) const;

    // Contact state, kept up to date by update() and moveAndSlide().
    bool isOnFloor() const
    {
        return mGrounded;
    }
    bool isOnWall() const
    {
        return mOnWall;
    }
    bool isOnCeiling() const
    {
        return mOnCeiling;
    }
    const Math::vec3& floorNormal() const
    {
        return mGroundNormal;
    }
    const Math::vec3& wallNormal() const
    {
        return mWallNormal;
    }
    const Math::vec3& ceilingNormal() const
    {
        return mCeilingNormal;
    }

    bool grounded() const
    {
        return mGrounded;
    }
    const Math::vec3& groundNormal() const
    {
        return mGroundNormal;
    }
    // Degrees between the ground and horizontal; 0 when airborne.
    f32 slopeAngle() const;
    // What the last update actually achieved, per second.
    const Math::vec3& velocity() const
    {
        return mVelocity;
    }
    f32 verticalSpeed() const
    {
        return mVerticalSpeed;
    }

private:
    struct Slide
    {
        Math::vec3 centre{0.0f};
        Math::vec3 velocity{0.0f};
        Math::vec3 groundNormal{0.0f, 0.0f, 0.0f};
        Math::vec3 wallNormal{0.0f};
        Math::vec3 ceilingNormal{0.0f};
        bool collided = false;
        bool grounded = false;
        bool onWall = false;
        bool onCeiling = false;
        bool steepBlock = false;
    };

    // The ellipsoid half-extents this capsule is swept as, along axes().
    Math::vec3 radii() const;
    Math::mat3 axes() const;

    Slide slide(const Math::vec3& startCentre, const Math::vec3& displacement,
                const CharacterSweepSource& source) const;
    // Up, across, down - the phase order both references use. False when the
    // raised path is blocked or there is nothing to land on.
    bool stepUp(const Math::vec3& startCentre, const Math::vec3& horizontal,
                const CharacterSweepSource& source, Slide& out) const;
    bool snapToGround(const CharacterSweepSource& source);

    Math::vec3 mPosition{0.0f};
    Math::vec3 mUpDirection{0.0f, 1.0f, 0.0f};
    Math::vec3 mGroundNormal{0.0f, 1.0f, 0.0f};
    Math::vec3 mWallNormal{0.0f};
    Math::vec3 mCeilingNormal{0.0f};
    Math::vec3 mMoveInput{0.0f};
    Math::vec3 mVelocity{0.0f};
    f32 mRadius = 0.4f;
    f32 mHeight = 1.2f;
    f32 mSlopeLimitDegrees = 45.0f;
    f32 mSlopeLimitCosine = 0.70710678f;
    f32 mSkinWidth = 0.02f;
    // More than one frame of falling and more than the skin, so resting
    // contact never lapses; small enough not to pull him onto ledges.
    f32 mGroundSnapDistance = 0.15f;
    f32 mStepOffset = 0.35f;
    f32 mGravity = -20.0f;
    f32 mMaxFallSpeed = -50.0f;
    f32 mVerticalSpeed = 0.0f;
    f32 mWallMinSlideAngleDegrees = 0.0f;
    f32 mWallMinSlideAngleCosine = 1.0f;
    bool mFloorStopOnSlope = true;
    bool mSlideOnCeiling = true;
    bool mOnWall = false;
    bool mOnCeiling = false;
    // Eight leaves residual penetration in a corner, where sliding off one
    // wall drives into the next.
    u32 mMaxIterations = 16;
    bool mGrounded = false;
};

} // namespace kinetix::Physics

#endif // KINETIX_PHYSICS_CHARACTER_BODY_H
