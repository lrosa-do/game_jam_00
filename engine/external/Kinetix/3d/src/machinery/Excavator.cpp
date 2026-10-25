#include <kinetix/core/Log.h>
#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include "machinery/Excavator.h"

#include "dynamics/PhysicsWorld.h"

#include <math.h>

namespace kinetix::Physics
{

namespace
{

constexpr f32 kPi = 3.14159265358979f;

ArmPoint operator+(const ArmPoint& a, const ArmPoint& b)
{
    return ArmPoint{a.z + b.z, a.y + b.y};
}

ArmPoint operator-(const ArmPoint& a, const ArmPoint& b)
{
    return ArmPoint{a.z - b.z, a.y - b.y};
}

ArmPoint scaled(const ArmPoint& a, f32 s)
{
    return ArmPoint{a.z * s, a.y * s};
}

f32 length(const ArmPoint& a)
{
    return sqrtf(a.z * a.z + a.y * a.y);
}

f32 angleOf(const ArmPoint& a)
{
    return atan2f(a.y, a.z);
}

// a x b in the plane (+ when b is counter-clockwise of a).
f32 cross(const ArmPoint& a, const ArmPoint& b)
{
    return a.z * b.y - a.y * b.z;
}

ArmPoint rotated(const ArmPoint& p, const ArmPoint& centre, f32 angle)
{
    const f32 c = cosf(angle);
    const f32 s = sinf(angle);
    const ArmPoint d = p - centre;
    return ArmPoint{centre.z + c * d.z - s * d.y, centre.y + s * d.z + c * d.y};
}

f32 wrap(f32 angle)
{
    while (angle > kPi)
        angle -= 2.0f * kPi;
    while (angle < -kPi)
        angle += 2.0f * kPi;
    return angle;
}

// Intersection of two circles on the given side of the line from the first centre to the second
// (+1: counter-clockwise).
bool circles(const ArmPoint& c0, f32 r0, const ArmPoint& c1, f32 r1, f32 side, ArmPoint* out)
{
    const ArmPoint d = c1 - c0;
    const f32 distance = length(d);
    if (!(distance > 1.0e-6f))
        return false;
    const f32 a = (r0 * r0 - r1 * r1 + distance * distance) / (2.0f * distance);
    const f32 h2 = r0 * r0 - a * a;
    if (h2 < 0.0f)
        return false;
    const f32 h = sqrtf(h2);
    const ArmPoint unit = scaled(d, 1.0f / distance);
    const ArmPoint perpendicular{-unit.y, unit.z};
    *out = c0 + scaled(unit, a) + scaled(perpendicular, side >= 0.0f ? h : -h);
    return true;
}

f32 sideOf(const ArmPoint& c0, const ArmPoint& c1, const ArmPoint& p)
{
    return cross(c1 - c0, p - c0) >= 0.0f ? 1.0f : -1.0f;
}

// Joint angle from the cylinder length (law of cosines): the angle the pivot-to-rod arm makes with
// the pivot-to-barrel line, signed as at rest.
f32 triangleJointAngle(const ArmPoint& pivot, const ArmPoint& barrel, const ArmPoint& rod, f32 cylinderLength)
{
    const ArmPoint toBarrel = barrel - pivot;
    const ArmPoint toRod = rod - pivot;
    const f32 a = length(toBarrel);
    const f32 c = length(toRod);
    const f32 rest = atan2f(cross(toBarrel, toRod), toBarrel.z * toRod.z + toBarrel.y * toRod.y);
    const f32 cosine = Math::clamp((a * a + c * c - cylinderLength * cylinderLength) / (2.0f * a * c), -1.0f, 1.0f);
    const f32 opening = acosf(cosine);
    return (rest >= 0.0f ? opening : -opening) - rest;
}

ArmPoint restToothFrom(const ExcavatorGeometry& g)
{
    const ArmPoint back{cosf(g.bucketBackAngle), sinf(g.bucketBackAngle)};
    // The lip turns towards the side the bucket curls to (clockwise).
    const ArmPoint lip{back.y, -back.z};
    return g.bucketPin + scaled(back, g.bucketBackLength) + scaled(lip, g.bucketLipLength);
}

ArmPoint restBackEnd(const ExcavatorGeometry& g)
{
    return g.bucketPin + ArmPoint{cosf(g.bucketBackAngle) * g.bucketBackLength,
                                  sinf(g.bucketBackAngle) * g.bucketBackLength};
}

}

ExcavatorKinematics::ExcavatorKinematics(const ExcavatorGeometry& geometry) : mGeometry(geometry)
{
    const ExcavatorGeometry& g = mGeometry;
    mBoomLength = length(g.stickPin - g.boomFoot);
    mStickLength = length(g.bucketPin - g.stickPin);
    mBoomRestAngle = angleOf(g.stickPin - g.boomFoot);
    mStickRestAngle = angleOf(g.bucketPin - g.stickPin);
    mRockerLength = length(g.linkPin - g.rockerPin);
    mLinkLength = length(g.bucketLinkPin - g.linkPin);
    mBucketLinkRadius = length(g.bucketLinkPin - g.bucketPin);
    mRockerRestAngle = angleOf(g.linkPin - g.rockerPin);
    mBucketLinkRestAngle = angleOf(g.bucketLinkPin - g.bucketPin);
    mRockerSide = sideOf(g.bucketBarrel, g.rockerPin, g.linkPin);
    mBucketSide = sideOf(g.linkPin, g.bucketPin, g.bucketLinkPin);
}

f32 ExcavatorKinematics::boomCylinderLength(f32 boomAngle) const
{
    const ExcavatorGeometry& g = mGeometry;
    return length(rotated(g.boomRod, g.boomFoot, boomAngle) - g.boomBarrel);
}

f32 ExcavatorKinematics::stickCylinderLength(f32 stickAngle) const
{
    const ExcavatorGeometry& g = mGeometry;
    return length(rotated(g.stickRod, g.stickPin, stickAngle) - g.stickBarrel);
}

f32 ExcavatorKinematics::boomAngle(f32 cylinderLength) const
{
    const ExcavatorGeometry& g = mGeometry;
    return triangleJointAngle(g.boomFoot, g.boomBarrel, g.boomRod, cylinderLength);
}

f32 ExcavatorKinematics::stickAngle(f32 cylinderLength) const
{
    const ExcavatorGeometry& g = mGeometry;
    return triangleJointAngle(g.stickPin, g.stickBarrel, g.stickRod, cylinderLength);
}

bool ExcavatorKinematics::bucketLinkage(f32 cylinderLength, f32* rockerAngle, f32* bucketAngle) const
{
    const ExcavatorGeometry& g = mGeometry;
    // Rocker pin: the cylinder's reach from its barrel meets the rocker's circle, on the side it was built on.
    ArmPoint link;
    if (!circles(g.bucketBarrel, cylinderLength, g.rockerPin, mRockerLength, mRockerSide, &link))
        return false;
    ArmPoint bucketLink;
    if (!circles(link, mLinkLength, g.bucketPin, mBucketLinkRadius, mBucketSide, &bucketLink))
        return false;
    if (rockerAngle)
        *rockerAngle = wrap(angleOf(link - g.rockerPin) - mRockerRestAngle);
    if (bucketAngle)
        *bucketAngle = wrap(angleOf(bucketLink - g.bucketPin) - mBucketLinkRestAngle);
    return true;
}

bool ExcavatorKinematics::bucketCylinderLength(f32 bucketAngle, f32 minLength, f32 maxLength, f32* lengthOut) const
{
    // The bucket's angle runs one way along the stroke: bisection.
    f32 low = minLength;
    f32 high = maxLength;
    f32 angleLow = 0.0f;
    f32 angleHigh = 0.0f;
    if (!bucketLinkage(low, nullptr, &angleLow) || !bucketLinkage(high, nullptr, &angleHigh))
        return false;
    const bool rising = angleHigh > angleLow;
    if ((rising && (bucketAngle < angleLow || bucketAngle > angleHigh)) ||
        (!rising && (bucketAngle > angleLow || bucketAngle < angleHigh)))
    {
        *lengthOut = (rising ? bucketAngle < angleLow : bucketAngle > angleLow) ? low : high;
        return false;
    }
    for (u32 i = 0; i < 40; ++i)
    {
        const f32 middle = 0.5f * (low + high);
        f32 angle = 0.0f;
        if (!bucketLinkage(middle, nullptr, &angle))
            return false;
        if ((angle < bucketAngle) == rising)
            low = middle;
        else
            high = middle;
    }
    *lengthOut = 0.5f * (low + high);
    return true;
}

ArmPoint ExcavatorKinematics::bucketPinAt(f32 boomAngle, f32 stickAngle) const
{
    const ExcavatorGeometry& g = mGeometry;
    const ArmPoint stickPin = rotated(g.stickPin, g.boomFoot, boomAngle);
    return rotated(rotated(g.bucketPin, g.boomFoot, boomAngle), stickPin, stickAngle);
}

ArmPoint ExcavatorKinematics::toothTip(f32 boomAngle, f32 stickAngle, f32 bucketAngle) const
{
    const ExcavatorGeometry& g = mGeometry;
    const ArmPoint stickPin = rotated(g.stickPin, g.boomFoot, boomAngle);
    const ArmPoint pin = bucketPinAt(boomAngle, stickAngle);
    const ArmPoint tooth = rotated(rotated(restToothFrom(g), g.boomFoot, boomAngle), stickPin, stickAngle);
    return rotated(tooth, pin, bucketAngle);
}

ArmPoint ExcavatorKinematics::restToothTip() const
{
    return restToothFrom(mGeometry);
}

f32 ExcavatorKinematics::restAttitude() const
{
    return angleOf(restToothFrom(mGeometry) - mGeometry.bucketPin);
}

bool ExcavatorKinematics::armAngles(const ArmPoint& pin, f32* boomAngle, f32* stickAngle) const
{
    const ExcavatorGeometry& g = mGeometry;
    const ArmPoint toPin = pin - g.boomFoot;
    const f32 d = length(toPin);
    if (!(d > fabsf(mBoomLength - mStickLength)) || !(d < mBoomLength + mStickLength))
        return false;
    const f32 gamma = acosf(Math::clamp((mBoomLength * mBoomLength + d * d - mStickLength * mStickLength) /
                                            (2.0f * mBoomLength * d),
                                        -1.0f, 1.0f));
    // Elbow up: the boom above the line to the pin.
    const f32 boom = angleOf(toPin) + gamma;
    const ArmPoint stickPin = g.boomFoot + ArmPoint{cosf(boom) * mBoomLength, sinf(boom) * mBoomLength};
    const f32 stick = angleOf(pin - stickPin);
    *boomAngle = wrap(boom - mBoomRestAngle);
    *stickAngle = wrap(stick - mStickRestAngle - *boomAngle);
    return true;
}

ExcavatorParameters ExcavatorParameters::crawler20t()
{
    ExcavatorParameters p;
    p.engine = EngineParameters::diesel(110.0e3f, 2000.0f, 650.0f, 1400.0f, 1600.0f, 900.0f, 2200.0f);
    p.engine.inertia = 2.5f;

    p.pump.circuit = HydraulicCircuit::LoadSensing;
    p.pump.displacement = 120.0f;
    p.pump.maxPressure = 34.3e6f;
    p.pump.marginPressure = 2.0e6f;
    p.pump.standbyPressure = 2.5e6f;
    p.pump.powerLimit = 45.0e3f;

    HydraulicCylinderParameters cylinder;
    cylinder.reliefPressure = 36.0e6f;
    cylinder.cushionLength = 0.04f;
    cylinder.valveResponseTime = 0.0f;
    cylinder.valveDeadband = 0.0f;
    p.boomCylinder = cylinder;
    p.boomCylinder.boreDiameter = 0.12f;
    p.boomCylinder.rodDiameter = 0.085f;
    p.boomCylinder.stroke = 1.4f;
    p.boomCylinder.initialStroke = 1.025f;
    p.boomCylinder.pumpFlow = 220.0f;
    p.boomCylinder.count = 2;
    p.stickCylinder = cylinder;
    p.stickCylinder.boreDiameter = 0.14f;
    p.stickCylinder.rodDiameter = 0.1f;
    p.stickCylinder.stroke = 1.5f;
    p.stickCylinder.initialStroke = 0.842f;
    p.stickCylinder.pumpFlow = 220.0f;
    p.bucketCylinder = cylinder;
    p.bucketCylinder.boreDiameter = 0.12f;
    p.bucketCylinder.rodDiameter = 0.085f;
    p.bucketCylinder.stroke = 1.0f;
    p.bucketCylinder.initialStroke = 0.6f;
    p.bucketCylinder.pumpFlow = 160.0f;

    p.swingMotor.displacement = 90.0f;
    p.swingMotor.gearRatio = 170.0f;
    p.swingMotor.reliefPressure = 28.0e6f;
    p.swingMotor.maxFlow = 180.0f;
    p.swingMotor.holdingBrake = 2.0e4f;

    p.travelMotor.displacement = 115.0f;
    p.travelMotor.gearRatio = 40.0f;
    p.travelMotor.reliefPressure = 34.3e6f;
    p.travelMotor.maxFlow = 200.0f;
    p.travelMotor.holdingBrake = 4.0e4f;

    p.boomSection.ratedFlow = 220.0f;
    p.stickSection.ratedFlow = 220.0f;
    p.bucketSection.ratedFlow = 160.0f;
    p.swingSection.ratedFlow = 180.0f;
    p.swingSection.priority = 1;
    p.travelSection.ratedFlow = 200.0f;

    p.tracks.gauge = 2.39f;
    p.tracks.shoeWidth = 0.6f;
    p.tracks.contactLength = 3.65f;
    p.tracks.bottomHeight = -0.75f;
    p.tracks.sprocketRadius = 0.35f;
    p.tracks.sprocketZ = -2.0f;
    p.tracks.idlerRadius = 0.32f;
    p.tracks.idlerZ = 2.0f;
    p.tracks.roadWheels = 7;
    p.tracks.samples = 12;
    p.tracks.beltMass = 1200.0f;
    p.tracks.internalResistance = 0.03f;
    p.travelMotor.outputRadius = p.tracks.sprocketRadius;
    return p;
}

Excavator::Excavator(PhysicsWorld& world, const ExcavatorParameters& parameters, const Math::vec3& position,
                     const Math::quat& orientation)
    : mWorld(&world), mParameters(parameters), mKinematics(parameters.geometry)
{
    buildBodies(position, orientation);
    buildJoints();
    buildHydraulics();
}

Excavator::~Excavator()
{
    Joint* joints[] = {mSwing,         mBoomHinge,        mStickHinge,       mBucketHinge,    mRockerHinge,
                       mLink,          mBucketWeld,       mBoomCylinder, mStickCylinder,
                       mBucketCylinder, mTracks,          mTeeth};
    for (Joint* joint : joints)
        if (joint)
        {
            mWorld->removeJoint(joint);
            delete joint;
        }
    delete mSwingMotor;
    delete mTravelMotors[0];
    delete mTravelMotors[1];
    delete mHydraulics;
    for (u32 i = 0; i < BodyCount; ++i)
    {
        mWorld->removeBody(mIds[i]);
        delete mShapes[i];
    }
}

Math::vec3 Excavator::armWorld(const ArmPoint& point, f32 x) const
{
    return mRingCentre + mBuildOrientation * Math::vec3(x, point.y, point.z);
}

Math::quat Excavator::memberOrientation(const ArmPoint& from, const ArmPoint& to) const
{
    // The member's local +Z along it: a turn about -X by its angle in the
    // arm plane takes +Z to (0, sin a, cos a).
    const f32 angle = angleOf(to - from);
    return mBuildOrientation * Math::angleAxis(angle, Math::vec3(-1.0f, 0.0f, 0.0f));
}

void Excavator::addBody(u32 index, const Math::vec3& centre, const Math::quat& orientation,
                        const Math::vec3& halfExtents, f32 mass, const CollisionFilter& filter)
{
    RigidBody& body = mBodies[index];
    body.setBodyType(BodyType::Dynamic);
    body.setMass(mass);
    body.setInertiaTensor(Inertia::box(mass, halfExtents));
    body.setDamping(0.995f, 0.995f);
    body.setPosition(centre);
    body.setOrientation(orientation);
    body.calculateDerivedData();
    mShapes[index] = new BoxShape(halfExtents);
    BodyEntry entry;
    entry.body = &body;
    entry.shape = mShapes[index];
    entry.filter = filter;
    entry.friction = 0.6f;
    mIds[index] = mWorld->addBody(entry);
}

void Excavator::buildBodies(const Math::vec3& position, const Math::quat& orientation)
{
    const ExcavatorParameters& p = mParameters;
    const ExcavatorGeometry& g = p.geometry;
    mBuildOrientation = Math::normalize(orientation);
    mRingCentre = position + mBuildOrientation * Math::vec3(0.0f, p.ringHeight, 0.0f);
    CollisionFilter filter = p.bodyFilter;
    CollisionFilter bucketFilter = filter;
    bucketFilter.mask &= ~p.diggableGroup;

    addBody(Undercarriage, position, mBuildOrientation, p.undercarriageHalfExtents, p.undercarriageMass, filter);
    addBody(Upper, mRingCentre + mBuildOrientation * p.upperCentre, mBuildOrientation, p.upperHalfExtents,
            p.upperMass, filter);

    auto member = [&](u32 index, const ArmPoint& from, const ArmPoint& to, f32 halfWidth, f32 halfThickness, f32 mass,
                      const CollisionFilter& f)
    {
        const ArmPoint middle = scaled(from + to, 0.5f);
        addBody(index, armWorld(middle), memberOrientation(from, to),
                Math::vec3(halfWidth, halfThickness, 0.5f * length(to - from)), mass, f);
    };
    member(Boom, g.boomFoot, g.stickPin, 0.25f, 0.3f, p.boomMass, filter);
    member(Stick, g.stickRod, g.bucketPin, 0.2f, 0.22f, p.stickMass, filter);
    member(Rocker, g.rockerPin, g.linkPin, 0.12f, 0.06f, p.rockerMass, filter);
    const ArmPoint backEnd = restBackEnd(g);
    member(BucketBack, g.bucketPin, backEnd, 0.5f * g.bucketWidth, 0.05f, 0.6f * p.bucketMass, bucketFilter);
    member(BucketLip, backEnd, restToothFrom(g), 0.5f * g.bucketWidth, 0.05f, 0.4f * p.bucketMass, bucketFilter);
}

void Excavator::buildJoints()
{
    const ExcavatorParameters& p = mParameters;
    const ExcavatorGeometry& g = p.geometry;
    const Math::vec3 up = mBuildOrientation * Math::vec3(0.0f, 1.0f, 0.0f);
    const Math::vec3 armAxis = mBuildOrientation * Math::vec3(-1.0f, 0.0f, 0.0f);

    mSwing = new HingeJoint(mBodies[Undercarriage], mBodies[Upper], mRingCentre, up);
    mBoomHinge = new HingeJoint(mBodies[Upper], mBodies[Boom], armWorld(g.boomFoot), armAxis);
    mStickHinge = new HingeJoint(mBodies[Boom], mBodies[Stick], armWorld(g.stickPin), armAxis);
    mBucketHinge = new HingeJoint(mBodies[Stick], mBodies[BucketBack], armWorld(g.bucketPin), armAxis);
    mRockerHinge = new HingeJoint(mBodies[Stick], mBodies[Rocker], armWorld(g.rockerPin), armAxis);
    mLink = new DistanceJoint(mBodies[Rocker], armWorld(g.linkPin), mBodies[BucketBack], armWorld(g.bucketLinkPin));
    mBucketWeld = new FixedJoint(mBodies[BucketBack], mBodies[BucketLip], armWorld(restBackEnd(g)));
    mBoomCylinder = new HydraulicCylinder(mBodies[Upper], armWorld(g.boomBarrel), mBodies[Boom],
                                          armWorld(g.boomRod), p.boomCylinder);
    mStickCylinder = new HydraulicCylinder(mBodies[Boom], armWorld(g.stickBarrel), mBodies[Stick],
                                           armWorld(g.stickRod), p.stickCylinder);
    mBucketCylinder = new HydraulicCylinder(mBodies[Stick], armWorld(g.bucketBarrel), mBodies[Rocker],
                                            armWorld(g.linkPin), p.bucketCylinder);
    const struct
    {
        u32 a;
        u32 b;
        Math::vec3 point;
    } pins[PinCount] = {{Undercarriage, Upper, mRingCentre},
                        {Upper, Boom, armWorld(g.boomFoot)},
                        {Boom, Stick, armWorld(g.stickPin)},
                        {Stick, BucketBack, armWorld(g.bucketPin)},
                        {Stick, Rocker, armWorld(g.rockerPin)},
                        {BucketBack, BucketLip, armWorld(restBackEnd(g))}};
    for (u32 i = 0; i < PinCount; ++i)
    {
        mPins[i].bodyA = pins[i].a;
        mPins[i].bodyB = pins[i].b;
        mPins[i].localA = mBodies[pins[i].a].pointToLocal(pins[i].point);
        mPins[i].localB = mBodies[pins[i].b].pointToLocal(pins[i].point);
    }
    Joint* articulated[] = {mSwing,  mBoomHinge,  mStickHinge,       mBucketHinge,      mRockerHinge,  mLink,
                            mBucketWeld, mBoomCylinder, mStickCylinder, mBucketCylinder};
    for (Joint* joint : articulated)
    {
        joint->setArticulated(true);
        mWorld->addJoint(joint);
    }

    TrackParameters tracks = p.tracks;
    if (!(tracks.supportedMass > 0.0f))
        tracks.supportedMass = p.undercarriageMass + p.upperMass + p.boomMass + p.stickMass + p.bucketMass +
                               p.rockerMass;
    mTracks = new TrackedUndercarriage(*mWorld, mIds[Undercarriage], tracks);
    QueryFilter ground;
    ground.collision.group = 0xFFFFFFFFu;
    ground.collision.mask = ~p.bodyFilter.group;
    mTracks->setRayFilter(ground);
    mWorld->addJoint(mTracks);

    const ArmPoint tooth = restToothFrom(g);
    mTeeth = new SoilCuttingTool(mBodies[BucketLip], armWorld(tooth, 0.5f * g.bucketWidth),
                                 armWorld(tooth, -0.5f * g.bucketWidth));
    // Teeth concentrate the cut: about two thirds of a plain edge's resistance.
    mTeeth->setResistanceScale(0.65f);
    mWorld->addJoint(mTeeth);
}

void Excavator::buildHydraulics()
{
    const ExcavatorParameters& p = mParameters;
    mHydraulics = new HydraulicSystem(p.engine);
    const u32 front = mHydraulics->addPump(p.pump);
    const u32 rear = mHydraulics->addPump(p.pump);
    mSections[BoomSection] = mHydraulics->addSection(front, p.boomSection);
    mSections[BucketSection] = mHydraulics->addSection(front, p.bucketSection);
    mSections[LeftTravelSection] = mHydraulics->addSection(front, p.travelSection);
    mSections[StickSection] = mHydraulics->addSection(rear, p.stickSection);
    mSections[SwingSection] = mHydraulics->addSection(rear, p.swingSection);
    mSections[RightTravelSection] = mHydraulics->addSection(rear, p.travelSection);
    mHydraulics->addConsumer(mSections[BoomSection], *mBoomCylinder);
    mHydraulics->addConsumer(mSections[StickSection], *mStickCylinder);
    mHydraulics->addConsumer(mSections[BucketSection], *mBucketCylinder);
    mSwingMotor = new HydraulicMotor(mSwing->drive(), p.swingMotor);
    mHydraulics->addConsumer(mSections[SwingSection], *mSwingMotor);
    for (u32 side = 0; side < 2; ++side)
    {
        mTravelMotors[side] = new HydraulicMotor(mTracks->drive(side), p.travelMotor);
        mHydraulics->addConsumer(mSections[side == 0 ? LeftTravelSection : RightTravelSection], *mTravelMotors[side]);
    }
}

void Excavator::setControls(const ExcavatorControls& controls)
{
    mControls = controls;
}

void Excavator::setToothTarget(const ArmPoint& toothTip, f32 attitude)
{
    if (!isfinite(toothTip.z) || !isfinite(toothTip.y) || !isfinite(attitude))
        return;
    mAssist = true;
    mToothTarget = toothTip;
    mAttitudeTarget = attitude;
}

void Excavator::clearToothTarget()
{
    mAssist = false;
}

void Excavator::setSwingTarget(f32 angle)
{
    if (!isfinite(angle))
        return;
    mSwingAssist = true;
    mSwingTarget = angle;
}

void Excavator::clearSwingTarget()
{
    mSwingAssist = false;
}

void Excavator::setSoilBed(const SoilBed& bed)
{
    mTeeth->setSoilBed(bed);
}

void Excavator::setGroundSoil(const SoilParameters& soil)
{
    mTracks->setSoil(soil);
}

namespace
{

// Spool command for a length error: proportional (full open 0.1 m away), lifted over the
// section's overlap so the last millimetres move (electro-hydraulic deadband compensation), and
// nothing within half a millimetre.
f32 assistSpool(f32 error, f32 deadband)
{
    constexpr f32 kGain = 10.0f;
    constexpr f32 kTolerance = 5.0e-4f;
    if (fabsf(error) < kTolerance)
        return 0.0f;
    const f32 opening = Math::min(fabsf(error) * kGain, 1.0f);
    const f32 spool = deadband + (1.0f - deadband) * opening;
    return error > 0.0f ? spool : -spool;
}

}

void Excavator::runAssist()
{
    const ExcavatorKinematics& k = mKinematics;
    const ExcavatorParameters& p = mParameters;
    if (mAssist)
    {
        const ArmPoint rest = k.restToothTip() - p.geometry.bucketPin;
        const f32 reach = length(rest);
        const ArmPoint pin = mToothTarget - ArmPoint{cosf(mAttitudeTarget) * reach, sinf(mAttitudeTarget) * reach};
        f32 boomAngle = 0.0f;
        f32 stickAngle = 0.0f;
        if (k.armAngles(pin, &boomAngle, &stickAngle))
        {
            const f32 bucketAngle = wrap(mAttitudeTarget - k.restAttitude() - boomAngle - stickAngle);
            const HydraulicCylinderParameters& bucket = mBucketCylinder->parameters();
            f32 bucketLength = 0.0f;
            k.bucketCylinderLength(bucketAngle, bucket.closedLength, bucket.closedLength + bucket.stroke, &bucketLength);
            mAssistBoom = assistSpool(k.boomCylinderLength(boomAngle) - mBoomCylinder->length(), p.boomSection.deadband);
            mAssistStick =
                assistSpool(k.stickCylinderLength(stickAngle) - mStickCylinder->length(), p.stickSection.deadband);
            mAssistBucket = assistSpool(bucketLength - mBucketCylinder->length(), p.bucketSection.deadband);
        }
        else
        {
            mAssistBoom = 0.0f;
            mAssistStick = 0.0f;
            mAssistBucket = 0.0f;
        }
    }
    if (mSwingAssist)
    {
        // The slewing inertia is large: slow down over the last half radian, as an operator feathers the lever.
        const f32 error = wrap(mSwingTarget - mSwing->currentAngle());
        const f32 rate = mSwing->currentAngularVelocity();
        mAssistSwing = fabsf(error) < 2.0e-3f && fabsf(rate) < 0.02f
                           ? 0.0f
                           : Math::clamp(2.0f * error - 0.6f * rate, -1.0f, 1.0f);
    }
}

void Excavator::update(f32 step)
{
    if (!(step > 0.0f) || !isfinite(step))
        return;
    runAssist();
    HydraulicSystem& h = *mHydraulics;
    h.setEngineSpeed(mControls.engineRpm > 0.0f ? mControls.engineRpm : mParameters.engine.limiterRpm);
    h.setSpool(mSections[BoomSection], mAssist ? mAssistBoom : mControls.boom);
    h.setSpool(mSections[StickSection], mAssist ? mAssistStick : mControls.stick);
    h.setSpool(mSections[BucketSection], mAssist ? mAssistBucket : mControls.bucket);
    h.setSpool(mSections[SwingSection], mSwingAssist ? mAssistSwing : mControls.swing);
    h.setSpool(mSections[LeftTravelSection], mControls.leftTrack);
    h.setSpool(mSections[RightTravelSection], mControls.rightTrack);
    h.update(step);
}

Math::vec3 Excavator::ringCentreWorld() const
{
    return mBodies[Upper].pointToWorld(mPins[0].localB);
}

Math::vec3 Excavator::toothTipWorld() const
{
    return mTeeth->edgeMiddle();
}

Math::vec3 Excavator::bucketPinWorld() const
{
    return mBodies[BucketBack].pointToWorld(mPins[3].localB);
}

// In the upper structure's frame, from the ring (built in the build orientation, so its local axes are the arm plane's).
ArmPoint Excavator::toothTip() const
{
    const RigidBody& upper = mBodies[Upper];
    const Math::vec3 local = upper.pointToLocal(toothTipWorld()) - mPins[0].localB;
    return ArmPoint{local.z, local.y};
}

f32 Excavator::attitude() const
{
    const RigidBody& upper = mBodies[Upper];
    const Math::vec3 teeth = upper.pointToLocal(toothTipWorld());
    const Math::vec3 pin = upper.pointToLocal(bucketPinWorld());
    return atan2f(teeth.y - pin.y, teeth.z - pin.z);
}

f32 Excavator::worstJointSeparation() const
{
    f32 worst = 0.0f;
    for (const Pin& pin : mPins)
        worst = Math::max(worst, Math::length(mBodies[pin.bodyA].pointToWorld(pin.localA) -
                                              mBodies[pin.bodyB].pointToWorld(pin.localB)));
    const f32 link = Math::length(mLink->worldAnchorB() - mLink->worldAnchorA());
    return Math::max(worst, fabsf(link - mLink->minDistance()));
}

bool Excavator::finite() const
{
    for (const RigidBody& body : mBodies)
    {
        const Math::vec3& p = body.position();
        const Math::vec3& v = body.velocity();
        if (!isfinite(p.x) || !isfinite(p.y) || !isfinite(p.z) || !isfinite(v.x) || !isfinite(v.y) || !isfinite(v.z))
            return false;
    }
    return true;
}

} // namespace kinetix::Physics
