#include <kinetix/core/Log.h>
#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include "machinery/Bulldozer.h"

#include "dynamics/PhysicsWorld.h"

#include <math.h>

namespace kinetix::Physics
{

BulldozerParameters BulldozerParameters::crawler18t()
{
    BulldozerParameters p;
    p.engine = EngineParameters::diesel(150.0e3f, 1900.0f, 900.0f, 1300.0f, 1500.0f, 800.0f, 2100.0f);
    p.engine.inertia = 3.0f;

    p.implementPump.circuit = HydraulicCircuit::LoadSensing;
    p.implementPump.displacement = 80.0f;
    p.implementPump.maxPressure = 25.0e6f;
    p.implementPump.powerLimit = 35.0e3f;
    p.travelPump.circuit = HydraulicCircuit::LoadSensing;
    p.travelPump.displacement = 140.0f;
    p.travelPump.maxPressure = 40.0e6f;
    p.travelPump.powerLimit = 55.0e3f;

    HydraulicCylinderParameters cylinder;
    cylinder.reliefPressure = 25.0e6f;
    cylinder.cushionLength = 0.03f;
    cylinder.valveResponseTime = 0.0f;
    cylinder.valveDeadband = 0.0f;
    p.liftCylinder = cylinder;
    p.liftCylinder.boreDiameter = 0.11f;
    p.liftCylinder.rodDiameter = 0.065f;
    p.liftCylinder.count = 2;
    p.liftCylinder.stroke = 1.1f;
    p.liftCylinder.initialStroke = 0.8f;
    p.liftCylinder.pumpFlow = 150.0f;
    p.tiltCylinder = cylinder;
    p.tiltCylinder.boreDiameter = 0.12f;
    p.tiltCylinder.rodDiameter = 0.07f;
    p.tiltCylinder.stroke = 0.35f;
    p.tiltCylinder.initialStroke = 0.175f;
    p.tiltCylinder.pumpFlow = 60.0f;
    p.liftSection.ratedFlow = 150.0f;
    p.tiltSection.ratedFlow = 60.0f;

    p.travelMotor.displacement = 160.0f;
    p.travelMotor.gearRatio = 60.0f;
    p.travelMotor.reliefPressure = 40.0e6f;
    p.travelMotor.maxFlow = 280.0f;
    p.travelMotor.holdingBrake = 6.0e4f;
    p.travelSection.ratedFlow = 280.0f;

    p.tracks.gauge = 1.9f;
    p.tracks.shoeWidth = 0.56f;
    p.tracks.contactLength = 2.7f;
    p.tracks.bottomHeight = -p.rideHeight;
    p.tracks.sprocketRadius = 0.36f;
    p.tracks.sprocketZ = -1.6f;
    p.tracks.idlerRadius = 0.33f;
    p.tracks.idlerZ = 1.6f;
    p.tracks.roadWheels = 6;
    p.tracks.samples = 12;
    p.tracks.beltMass = 1500.0f;
    p.tracks.internalResistance = 0.04f;
    p.travelMotor.outputRadius = p.tracks.sprocketRadius;
    return p;
}

Bulldozer::Bulldozer(PhysicsWorld& world, const BulldozerParameters& parameters, const Math::vec3& position,
                     const Math::quat& orientation)
    : mWorld(&world), mParameters(parameters), mPosition(position), mOrientation(Math::normalize(orientation))
{
    const BulldozerParameters& p = mParameters;
    const CollisionFilter filter = p.bodyFilter;
    CollisionFilter bladeFilter = filter;
    bladeFilter.mask &= ~p.diggableGroup;
    const Math::vec3 lateral = mOrientation * Math::vec3(1.0f, 0.0f, 0.0f);
    const Math::vec3 forward = mOrientation * Math::vec3(0.0f, 0.0f, 1.0f);

    addBody(Tractor, position, mOrientation, p.tractorHalfExtents, p.tractorMass, filter);

    const f32 dz = p.tiltHingeZ - p.trunnionZ;
    const f32 dy = p.tiltHingeY - p.trunnionY;
    const f32 frameLength = sqrtf(dz * dz + dy * dy);
    const Math::quat frameOrientation = mOrientation * Math::angleAxis(atan2f(dy, dz), Math::vec3(-1.0f, 0.0f, 0.0f));
    addBody(PushFrame, frameWorld(0.0f, 0.5f * (p.trunnionZ + p.tiltHingeZ), 0.5f * (p.trunnionY + p.tiltHingeY)),
            frameOrientation, Math::vec3(p.pushFrameHalfWidth, p.pushFrameHalfThickness, 0.5f * frameLength),
            p.pushFrameMass, filter);

    const f32 edgeY = -p.rideHeight + p.buildEdgeHeight;
    const f32 bladeCentreY = edgeY + 0.5f * p.bladeHeight;
    const f32 bladeCentreZ = p.tiltHingeZ + p.bladeForward;
    addBody(Blade, frameWorld(0.0f, bladeCentreZ, bladeCentreY), mOrientation,
            Math::vec3(0.5f * p.bladeWidth, 0.5f * p.bladeHeight, 0.5f * p.bladeThickness), p.bladeMass, bladeFilter);

    const Math::vec3 trunnion = frameWorld(0.0f, p.trunnionZ, p.trunnionY);
    const Math::vec3 hinge = frameWorld(0.0f, p.tiltHingeZ, p.tiltHingeY);
    // + about -X raises the front; + about +Z lifts the left end.
    mTrunnion = new HingeJoint(mBodies[Tractor], mBodies[PushFrame], trunnion, -lateral);
    mTiltHinge = new HingeJoint(mBodies[PushFrame], mBodies[Blade], hinge, forward);
    mPins[0][0] = mBodies[Tractor].pointToLocal(trunnion);
    mPins[0][1] = mBodies[PushFrame].pointToLocal(trunnion);
    mPins[1][0] = mBodies[PushFrame].pointToLocal(hinge);
    mPins[1][1] = mBodies[Blade].pointToLocal(hinge);

    mLiftCylinder = new HydraulicCylinder(mBodies[Tractor], frameWorld(0.0f, p.liftBarrelZ, p.liftBarrelY),
                                          mBodies[PushFrame], frameWorld(0.0f, p.liftRodZ, p.liftRodY), p.liftCylinder);
    mTiltCylinder = new HydraulicCylinder(mBodies[PushFrame], frameWorld(p.tiltBarrel.x, p.tiltBarrel.z, p.tiltBarrel.y),
                                          mBodies[Blade], frameWorld(p.tiltRod.x, p.tiltRod.z, p.tiltRod.y),
                                          p.tiltCylinder);
    Joint* articulated[] = {mTrunnion, mTiltHinge, mLiftCylinder, mTiltCylinder};
    for (Joint* joint : articulated)
    {
        joint->setArticulated(true);
        mWorld->addJoint(joint);
    }

    TrackParameters tracks = p.tracks;
    if (!(tracks.supportedMass > 0.0f))
        tracks.supportedMass = p.tractorMass + p.pushFrameMass + p.bladeMass;
    mTracks = new TrackedUndercarriage(*mWorld, mIds[Tractor], tracks);
    QueryFilter ground;
    ground.collision.group = 0xFFFFFFFFu;
    ground.collision.mask = ~p.bodyFilter.group;
    mTracks->setRayFilter(ground);
    mWorld->addJoint(mTracks);

    const f32 edgeZ = bladeCentreZ + 0.5f * p.bladeThickness;
    mEdge = new SoilCuttingTool(mBodies[Blade], frameWorld(0.5f * p.bladeWidth, edgeZ, edgeY),
                                frameWorld(-0.5f * p.bladeWidth, edgeZ, edgeY));
    mEdge->setRakeAngle(p.cuttingAngle);
    mWorld->addJoint(mEdge);

    mHydraulics = new HydraulicSystem(p.engine);
    const u32 implement = mHydraulics->addPump(p.implementPump);
    const u32 left = mHydraulics->addPump(p.travelPump);
    const u32 right = mHydraulics->addPump(p.travelPump);
    mSections[LiftSection] = mHydraulics->addSection(implement, p.liftSection);
    mSections[TiltSection] = mHydraulics->addSection(implement, p.tiltSection);
    mSections[LeftTravelSection] = mHydraulics->addSection(left, p.travelSection);
    mSections[RightTravelSection] = mHydraulics->addSection(right, p.travelSection);
    // + lift raises: the cylinders retract.
    mHydraulics->addConsumer(mSections[LiftSection], *mLiftCylinder, -1.0f);
    mHydraulics->addConsumer(mSections[TiltSection], *mTiltCylinder);
    for (u32 side = 0; side < 2; ++side)
    {
        mTravelMotors[side] = new HydraulicMotor(mTracks->drive(side), p.travelMotor);
        mHydraulics->addConsumer(mSections[side == 0 ? LeftTravelSection : RightTravelSection], *mTravelMotors[side]);
    }
}

Bulldozer::~Bulldozer()
{
    Joint* joints[] = {mTrunnion, mTiltHinge, mLiftCylinder, mTiltCylinder, mTracks, mEdge};
    for (Joint* joint : joints)
    {
        mWorld->removeJoint(joint);
        delete joint;
    }
    delete mTravelMotors[0];
    delete mTravelMotors[1];
    delete mHydraulics;
    for (u32 i = 0; i < BodyCount; ++i)
    {
        mWorld->removeBody(mIds[i]);
        delete mShapes[i];
    }
}

Math::vec3 Bulldozer::frameWorld(f32 x, f32 z, f32 y) const
{
    return mPosition + mOrientation * Math::vec3(x, y, z);
}

void Bulldozer::addBody(u32 index, const Math::vec3& centre, const Math::quat& orientation,
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

void Bulldozer::setControls(const BulldozerControls& controls)
{
    mControls = controls;
}

void Bulldozer::setSoilBed(const SoilBed& bed)
{
    mEdge->setSoilBed(bed);
}

void Bulldozer::setGroundSoil(const SoilParameters& soil)
{
    mTracks->setSoil(soil);
}

void Bulldozer::update(f32 step)
{
    if (!(step > 0.0f) || !isfinite(step))
        return;
    HydraulicSystem& h = *mHydraulics;
    h.setEngineSpeed(mControls.engineRpm > 0.0f ? mControls.engineRpm : mParameters.engine.limiterRpm);
    h.setSpool(mSections[LiftSection], mControls.lift);
    h.setSpool(mSections[TiltSection], mControls.tilt);
    h.setSpool(mSections[LeftTravelSection], mControls.leftTrack);
    h.setSpool(mSections[RightTravelSection], mControls.rightTrack);
    h.update(step);
}

Math::vec3 Bulldozer::edgeWorld() const
{
    return mEdge->edgeMiddle();
}

f32 Bulldozer::edgeHeight() const
{
    const RigidBody& tractor = mBodies[Tractor];
    const Math::vec3 local = tractor.pointToLocal(edgeWorld());
    return local.y - mParameters.tracks.bottomHeight;
}

f32 Bulldozer::bladeTilt() const
{
    return mTiltHinge->currentAngle();
}

Math::vec3 Bulldozer::bladeForce() const
{
    const f32 step = mWorld->fixedStep();
    Math::vec3 impulse(0.0f);
    const u32 blade = mIds[Blade];
    for (const Contact& contact : mWorld->contacts())
    {
        if (contact.idA != blade && contact.idB != blade)
            continue;
        const ContactManifold& m = contact.manifold;
        Math::vec3 onB(0.0f);
        for (u32 k = 0; k < m.count; ++k)
            onB += m.normal * m.points[k].normalImpulse + m.tangent[0] * m.points[k].tangentImpulse[0] +
                   m.tangent[1] * m.points[k].tangentImpulse[1];
        impulse += contact.idB == blade ? onB : -onB;
    }
    return (step > 0.0f ? impulse / step : Math::vec3(0.0f)) + mEdge->force();
}

f32 Bulldozer::worstJointSeparation() const
{
    const f32 trunnion = Math::length(mBodies[Tractor].pointToWorld(mPins[0][0]) -
                                      mBodies[PushFrame].pointToWorld(mPins[0][1]));
    const f32 hinge = Math::length(mBodies[PushFrame].pointToWorld(mPins[1][0]) -
                                   mBodies[Blade].pointToWorld(mPins[1][1]));
    return Math::max(trunnion, hinge);
}

bool Bulldozer::finite() const
{
    for (const RigidBody& body : mBodies)
    {
        const Math::vec3& x = body.position();
        const Math::vec3& v = body.velocity();
        if (!isfinite(x.x) || !isfinite(x.y) || !isfinite(x.z) || !isfinite(v.x) || !isfinite(v.y) || !isfinite(v.z))
            return false;
    }
    return true;
}

} // namespace kinetix::Physics
