#include <kinetix/core/Log.h>
#include <kinetix/core/Math.h>
#include <kinetix/core/Types.h>

#include "machinery/MobileCrane.h"

#include "dynamics/PhysicsWorld.h"

#include <math.h>

namespace kinetix::Physics
{

namespace
{

constexpr f32 kGravity = 9.81f;
// Each telescopic section starts this far ahead of the one it slides in, retracted (m along the
// boom); the head sheave sits this far past the last one's end.
constexpr f32 kSectionStagger = 0.3f;
constexpr f32 kHeadOffset = 0.25f;
const Math::vec3 kBeamHalf(0.9f, 0.15f, 0.2f);
const Math::vec3 kJackHalf(0.3f, 0.3f, 0.3f);
// Anti-two-block switch: no hoisting with the block closer than this under the head sheave, m.
constexpr f32 kTwoBlock = 1.5f;

f32 sectionHalfWidth(u32 index)
{
    return 0.45f - 0.07f * static_cast<f32>(index);
}

f32 sectionHalfHeight(u32 index)
{
    return 0.6f - 0.09f * static_cast<f32>(index);
}

// Distance along the boom from the foot to the centre of section k (0 the base), and to the head
// sheave, with every telescopic section out by `extension`.
f32 sectionCentreAlong(const MobileCraneParameters& p, u32 index, f32 extension)
{
    if (index == 0)
        return 0.5f * p.baseLength;
    return 0.5f * p.sectionLength + kSectionStagger * static_cast<f32>(index) + extension * static_cast<f32>(index);
}

f32 headAlong(const MobileCraneParameters& p, f32 extension)
{
    const f32 n = static_cast<f32>(MobileCraneParameters::SectionCount);
    return p.sectionLength + kSectionStagger * n + extension * n + kHeadOffset;
}

}

MobileCraneParameters MobileCraneParameters::allTerrain50t()
{
    MobileCraneParameters p;
    p.engine = EngineParameters::diesel(200.0e3f, 1900.0f, 1200.0f, 1300.0f, 1500.0f, 750.0f, 2100.0f);
    p.engine.inertia = 3.0f;
    p.pump.circuit = HydraulicCircuit::LoadSensing;
    p.pump.displacement = 180.0f;
    p.pump.maxPressure = 32.0e6f;
    p.pump.powerLimit = 150.0e3f;
    p.luffCylinder.boreDiameter = 0.36f;
    p.luffCylinder.rodDiameter = 0.25f;
    p.luffCylinder.stroke = 2.1f;
    p.luffCylinder.initialStroke = 0.33f;
    p.luffCylinder.reliefPressure = 34.0e6f;
    p.luffCylinder.cushionLength = 0.05f;
    p.luffCylinder.valveResponseTime = 0.0f;
    p.luffCylinder.valveDeadband = 0.0f;
    p.luffCylinder.pumpFlow = 300.0f;
    p.slewMotor.displacement = 60.0f;
    p.slewMotor.gearRatio = 830.0f;
    p.slewMotor.reliefPressure = 28.0e6f;
    p.slewMotor.maxFlow = 100.0f;
    p.slewMotor.holdingBrake = 1.0e5f;
    p.luffSection.ratedFlow = 300.0f;
    p.slewSection.ratedFlow = 100.0f;
    p.slewSection.priority = 1;
    p.winch.drumRadius = 0.3f;
    p.winch.maxLinePull = 140.0e3f;
    p.winch.maxLineSpeed = 1.2f;
    p.winch.drumInertia = 60.0f;
    return p;
}

Math::vec3 MobileCrane::buildHeadLocal(const MobileCraneParameters& p)
{
    const f32 lift = p.buildDeployed ? p.liftHeight : 0.0f;
    const Math::vec3 slewTop(0.0f, p.chassisHalfExtents.y + lift, p.slewZ);
    const f32 extension = Math::clamp(p.buildExtension, 0.0f, p.sectionExtension);
    const f32 along = headAlong(p, extension);
    const Math::vec3 local(0.0f, p.boomFootHeight + along * sinf(p.buildElevation),
                           p.boomFootZ + along * cosf(p.buildElevation));
    return slewTop + Math::angleAxis(p.buildSlew, Math::vec3(0.0f, 1.0f, 0.0f)) * local;
}

f32 MobileCrane::restoringMoment(const MobileCraneParameters& p, f32 boomLength, f32 radius)
{
    // Slewed square over the left side (+X): the tipping line is the left pads' line at x = spread.
    const f32 line = p.outriggerSpread;
    f32 moment = 0.0f;
    auto add = [&](f32 mass, f32 x)
    {
        moment += mass * kGravity * (line - x);
    };
    add(p.chassisMass, 0.0f);
    for (u32 i = 0; i < 2; ++i)
    {
        add(p.beamMass, p.outriggerSpread - kBeamHalf.x);
        add(p.beamMass, -(p.outriggerSpread - kBeamHalf.x));
        add(p.jackMass, p.outriggerSpread);
        add(p.jackMass, -p.outriggerSpread);
    }
    add(p.superMass, p.superCentre.z);
    // Boom elevation that puts the head over this radius.
    const f32 retracted = headAlong(p, 0.0f);
    const f32 extension = Math::max(boomLength - retracted, 0.0f) / static_cast<f32>(MobileCraneParameters::SectionCount);
    const f32 cosine = Math::clamp((radius - p.boomFootZ) / headAlong(p, extension), -1.0f, 1.0f);
    add(p.baseMass, p.boomFootZ + sectionCentreAlong(p, 0, extension) * cosine);
    for (u32 i = 0; i < MobileCraneParameters::SectionCount; ++i)
        add(p.sectionMass[i], p.boomFootZ + sectionCentreAlong(p, i + 1, extension) * cosine);
    return moment;
}

LoadChart MobileCrane::computeChart(const MobileCraneParameters& p)
{
    const f32 retracted = headAlong(p, 0.0f);
    const f32 extensions[] = {0.0f, 2.0f, 4.0f, 6.0f, p.sectionExtension};
    constexpr u32 kLengths = 5;
    f32 lengths[kLengths];
    for (u32 i = 0; i < kLengths; ++i)
        lengths[i] = headAlong(p, extensions[i]);
    (void)retracted;
    f32 radii[LoadChart::MaxRadii];
    u32 radiusCount = 0;
    for (f32 r = 3.0f; r <= 34.0f && radiusCount < LoadChart::MaxRadii; r += (r < 10.0f ? 1.0f : 2.0f))
        radii[radiusCount++] = r;
    f32 ratings[kLengths * LoadChart::MaxRadii];
    for (u32 i = 0; i < kLengths; ++i)
        for (u32 j = 0; j < radiusCount; ++j)
        {
            const f32 length = lengths[i];
            const f32 r = radii[j];
            f32 rated = 0.0f;
            // Only where the boom reaches it between level and 80 deg.
            const f32 cosine = (r - p.boomFootZ) / length;
            if (cosine >= cosf(80.0f * 0.0174533f) && cosine <= 1.0f)
            {
                rated = Math::min(p.maxLoad, p.structuralMoment / r);
                const f32 lever = r - p.outriggerSpread;
                if (lever > 0.0f)
                {
                    const f32 tipping = restoringMoment(p, length, r) / (lever * kGravity);
                    rated = Math::min(rated, Math::max(p.stabilityFactor * tipping, 0.0f));
                }
            }
            ratings[i * radiusCount + j] = rated;
        }
    LoadChart chart;
    chart.set(lengths, kLengths, radii, radiusCount, ratings);
    return chart;
}

MobileCrane::MobileCrane(PhysicsWorld& world, const MobileCraneParameters& parameters, Math::vec3 position,
                         const Math::quat& orientation)
    : mWorld(&world), mParameters(parameters), mPosition(position), mOrientation(Math::normalize(orientation)),
      mWinch(parameters.winch)
{
    const MobileCraneParameters& p = mParameters;
    const CollisionFilter filter = p.bodyFilter;
    const Math::vec3 up = mOrientation * Math::vec3(0.0f, 1.0f, 0.0f);
    // Built deployed, the carrier stands lifted on its jacks: beams out, each jack down by what puts its pad on the ground.
    const f32 beamTravel = p.buildDeployed ? p.outriggerSpread - p.outriggerRetracted : 0.0f;
    const f32 jackTravel = p.buildDeployed ? Math::clamp(p.padClearance + p.liftHeight, 0.0f, p.jackStroke) : 0.0f;
    if (p.buildDeployed)
        mPosition += up * p.liftHeight;
    position = mPosition;

    addBody(Chassis, position, mOrientation, p.chassisHalfExtents, p.chassisMass, filter);
    // Outriggers: 0 front-left, 1 front-right, 2 rear-left, 3 rear-right.
    for (u32 i = 0; i < 4; ++i)
    {
        const f32 side = (i % 2 == 0) ? 1.0f : -1.0f;
        const f32 z = p.outriggerZ[i / 2];
        const f32 beamY = p.beamHeight;
        // The slides are built at their retracted positions (reading zero), then the bodies moved out by their travel.
        const Math::vec3 beamLocal(side * (p.outriggerRetracted - kBeamHalf.x), beamY, z);
        const Math::vec3 axis = mOrientation * Math::vec3(side, 0.0f, 0.0f);
        const Math::vec3 jackLocal(side * p.outriggerRetracted, p.padClearance + kJackHalf.y - p.chassisHeight, z);
        addBody(Beam0 + i, carrierWorld(beamLocal), mOrientation, kBeamHalf, p.beamMass, filter);
        addBody(Jack0 + i, carrierWorld(jackLocal), mOrientation, kJackHalf, p.jackMass, filter, true);
        mBeamSliders[i] = new SliderJoint(mBodies[Chassis], mBodies[Beam0 + i], carrierWorld(beamLocal), axis);
        mBeamSliders[i]->setLimits(0.0f, p.outriggerSpread - p.outriggerRetracted);
        mJackSliders[i] = new SliderJoint(mBodies[Beam0 + i], mBodies[Jack0 + i], carrierWorld(jackLocal), -up);
        mJackSliders[i]->setLimits(0.0f, p.jackStroke);
        if (p.buildDeployed)
        {
            mBodies[Beam0 + i].setPosition(mBodies[Beam0 + i].position() + axis * beamTravel);
            mBodies[Jack0 + i].setPosition(mBodies[Jack0 + i].position() + axis * beamTravel - up * jackTravel);
            mBodies[Beam0 + i].calculateDerivedData();
            mBodies[Jack0 + i].calculateDerivedData();
        }
        ServoSettings beam;
        beam.maxForce = p.beamForce;
        beam.maxVelocity = p.beamSpeed;
        mBeamSliders[i]->setServo(beamTravel, beam);
        // The jack holds through the beam's bending: a stiff spring (2e7 N/m beam) rather than a
        // rigid row, so four pads share the carrier's weight by compatibility, not by which one
        // the solver reached first.
        ServoSettings jack;
        jack.stiffness = 2.0e7f;
        jack.damping = 2.0f * sqrtf(2.0e7f * 15000.0f);
        jack.maxForce = p.jackForce;
        jack.maxVelocity = p.jackSpeed;
        mJackSliders[i]->setServo(jackTravel, jack);
    }
    if (p.buildDeployed)
    {
        mControls.outriggerBeams = 1.0f;
        mControls.outriggerJacks = jackTravel / p.jackStroke;
    }

    const Math::vec3 slewTop(0.0f, p.chassisHalfExtents.y, p.slewZ);
    mSlewTopLocal = slewTop;
    // Built slewed: their frame is the carrier's turned about the slewing axis.
    mSuperOrientation = mOrientation * Math::angleAxis(p.buildSlew, Math::vec3(0.0f, 1.0f, 0.0f));
    mSlewWorld = carrierWorld(slewTop);
    addBody(Superstructure, superWorld(p.superCentre), mSuperOrientation, p.superHalfExtents, p.superMass, filter);
    mSlew = new HingeJoint(mBodies[Chassis], mBodies[Superstructure], carrierWorld(slewTop), up);
    mSlewLocal[0] = mBodies[Chassis].pointToLocal(carrierWorld(slewTop));
    mSlewLocal[1] = mBodies[Superstructure].pointToLocal(carrierWorld(slewTop));

    mFootWorld = superWorld(Math::vec3(0.0f, p.boomFootHeight, p.boomFootZ));
    mBoomOrientation = mSuperOrientation * Math::angleAxis(p.buildElevation, Math::vec3(-1.0f, 0.0f, 0.0f));
    addBody(BoomBase, boomWorld(0.5f * p.baseLength, 0.0f), mBoomOrientation,
            Math::vec3(sectionHalfWidth(0), sectionHalfHeight(0), 0.5f * p.baseLength), p.baseMass, filter);
    const Math::vec3 lateral = mSuperOrientation * Math::vec3(-1.0f, 0.0f, 0.0f);
    mLuff = new HingeJoint(mBodies[Superstructure], mBodies[BoomBase], mFootWorld, lateral);
    mFootLocal[0] = mBodies[Superstructure].pointToLocal(mFootWorld);
    mFootLocal[1] = mBodies[BoomBase].pointToLocal(mFootWorld);
    const Math::vec3 along = mBoomOrientation * Math::vec3(0.0f, 0.0f, 1.0f);
    for (u32 i = 0; i < MobileCraneParameters::SectionCount; ++i)
    {
        const f32 centre = sectionCentreAlong(p, i + 1, 0.0f);
        addBody(BoomBase + 1 + i, boomWorld(centre, 0.0f), mBoomOrientation,
                Math::vec3(sectionHalfWidth(i + 1), sectionHalfHeight(i + 1), 0.5f * p.sectionLength),
                p.sectionMass[i], filter);
        mTelescopes[i] =
            new SliderJoint(mBodies[BoomBase + i], mBodies[BoomBase + 1 + i], boomWorld(centre, 0.0f), along);
        mTelescopes[i]->setLimits(0.0f, p.sectionExtension);
        ServoSettings telescope;
        telescope.maxForce = p.telescopeForce;
        const f32 extension = Math::clamp(p.buildExtension, 0.0f, p.sectionExtension);
        mTelescopes[i]->setServo(extension, telescope);
    }
    // Built extended: each section out by its share, the slides already made at the retracted pose.
    const f32 extension = Math::clamp(p.buildExtension, 0.0f, p.sectionExtension);
    for (u32 i = 0; i < MobileCraneParameters::SectionCount; ++i)
    {
        RigidBody& section = mBodies[BoomBase + 1 + i];
        section.setPosition(section.position() + along * (extension * static_cast<f32>(i + 1)));
        section.calculateDerivedData();
    }
    mTelescopeTarget = extension * static_cast<f32>(MobileCraneParameters::SectionCount);
    const Math::vec3 head = boomWorld(headAlong(p, extension), 0.0f);
    RigidBody& tip = mBodies[BoomBase + MobileCraneParameters::SectionCount];
    mHeadLocal = tip.pointToLocal(head);

    mLuffCylinder = new HydraulicCylinder(mBodies[Superstructure],
                                          superWorld(Math::vec3(0.0f, p.luffBarrelHeight, p.luffBarrelZ)),
                                          mBodies[BoomBase], boomWorld(p.luffRodAlong, -p.luffRodBelow), p.luffCylinder);

    const Math::vec3 blockHalf(0.3f, 0.4f, 0.3f);
    const Math::vec3 blockCentre = head - up * (p.hookDrop + blockHalf.y);
    addBody(HookBlock, blockCentre, mSuperOrientation, blockHalf, p.hookBlockMass, filter);
    // A load swinging a few cm/s is still swinging: sleep only when it has all but stopped (the
    // default threshold froze it at the end of a slow swing).
    mBodies[HookBlock].setSleepEpsilon(1.0e-4f);
    mRope = new Rope(mBodies[Superstructure], superWorld(p.drumPoint), mBodies[HookBlock],
                     blockCentre + up * blockHalf.y);
    mRope->addSheave(tip, head, p.sheaveRadius, p.sheaveEfficiency);
    mRope->setFalls(p.falls);
    mRope->setWinch(&mWinch);

    Joint* articulated[] = {mSlew, mLuff, mLuffCylinder};
    for (Joint* joint : articulated)
    {
        joint->setArticulated(true);
        mWorld->addJoint(joint);
    }
    for (u32 i = 0; i < 4; ++i)
    {
        mBeamSliders[i]->setArticulated(true);
        mJackSliders[i]->setArticulated(true);
        mWorld->addJoint(mBeamSliders[i]);
        mWorld->addJoint(mJackSliders[i]);
    }
    for (u32 i = 0; i < MobileCraneParameters::SectionCount; ++i)
    {
        mTelescopes[i]->setArticulated(true);
        mWorld->addJoint(mTelescopes[i]);
    }
    mRope->setArticulated(true);
    mWorld->addJoint(mRope);
    mHook = new Hook(*mWorld, mIds[HookBlock], blockCentre - up * blockHalf.y);
    mHook->setArticulated(true);

    mHydraulics = new HydraulicSystem(p.engine);
    const u32 pump = mHydraulics->addPump(p.pump);
    mLuffSection = mHydraulics->addSection(pump, p.luffSection);
    mSlewSection = mHydraulics->addSection(pump, p.slewSection);
    mHydraulics->addConsumer(mLuffSection, *mLuffCylinder);
    mSlewMotor = new HydraulicMotor(mSlew->drive(), p.slewMotor);
    mHydraulics->addConsumer(mSlewSection, *mSlewMotor);

    mLmi.setChart(computeChart(p));
}

MobileCrane::~MobileCrane()
{
    delete mHook;
    mWorld->removeJoint(mRope);
    delete mRope;
    Joint* joints[] = {mSlew, mLuff, mLuffCylinder};
    for (Joint* joint : joints)
    {
        mWorld->removeJoint(joint);
        delete joint;
    }
    for (u32 i = 0; i < 4; ++i)
    {
        mWorld->removeJoint(mBeamSliders[i]);
        mWorld->removeJoint(mJackSliders[i]);
        delete mBeamSliders[i];
        delete mJackSliders[i];
    }
    for (u32 i = 0; i < MobileCraneParameters::SectionCount; ++i)
    {
        mWorld->removeJoint(mTelescopes[i]);
        delete mTelescopes[i];
    }
    delete mSlewMotor;
    delete mHydraulics;
    for (u32 i = 0; i < BodyCount; ++i)
    {
        mWorld->removeBody(mIds[i]);
        delete mShapes[i];
    }
}

Math::vec3 MobileCrane::carrierWorld(const Math::vec3& local) const
{
    return mPosition + mOrientation * local;
}

Math::vec3 MobileCrane::superWorld(const Math::vec3& local) const
{
    return mSlewWorld + mSuperOrientation * local;
}

Math::vec3 MobileCrane::boomWorld(f32 along, f32 up) const
{
    return mFootWorld + mBoomOrientation * Math::vec3(0.0f, up, along);
}

void MobileCrane::addBody(u32 index, const Math::vec3& centre, const Math::quat& orientation,
                          const Math::vec3& halfExtents, f32 mass, const CollisionFilter& filter, bool sphere)
{
    RigidBody& body = mBodies[index];
    body.setBodyType(BodyType::Dynamic);
    body.setMass(mass);
    body.setInertiaTensor(Inertia::box(mass, halfExtents));
    body.setDamping(0.998f, 0.995f);
    body.setPosition(centre);
    body.setOrientation(orientation);
    body.calculateDerivedData();
    mHalfExtents[index] = halfExtents;
    if (sphere)
        mShapes[index] = new SphereShape(halfExtents.y);
    else
        mShapes[index] = new BoxShape(halfExtents);
    BodyEntry entry;
    entry.body = &body;
    entry.shape = mShapes[index];
    entry.filter = filter;
    entry.friction = 0.7f;
    mIds[index] = mWorld->addBody(entry);
}

void MobileCrane::setControls(const MobileCraneControls& controls)
{
    mControls = controls;
}

void MobileCrane::setAntiSway(bool enabled)
{
    mAntiSway = enabled;
    mShaper.reset();
}

void MobileCrane::update(f32 step)
{
    if (!(step > 0.0f) || !isfinite(step))
        return;
    const MobileCraneParameters& p = mParameters;
    MobileCraneControls c = mControls;

    mLmi.update(hookLoad(), radius(), boomLength(), step);
    if (mLmi.cutOut())
    {
        c.hoist = Math::min(c.hoist, 0.0f);
        c.luff = Math::max(c.luff, 0.0f);
        c.telescope = Math::min(c.telescope, 0.0f);
    }
    if (ropeLength() < kTwoBlock)
        c.hoist = Math::min(c.hoist, 0.0f);
    if (mAntiSway)
    {
        mShaper.setPeriod(InputShaper::pendulumPeriod(ropeLength()));
        c.slew = mShaper.shape(c.slew, step);
    }

    HydraulicSystem& h = *mHydraulics;
    h.setEngineSpeed(c.engineRpm > 0.0f ? c.engineRpm : p.engine.limiterRpm);
    h.setSpool(mLuffSection, c.luff);
    h.setSpool(mSlewSection, c.slew);
    h.update(step);
    mWinch.setCommand(c.hoist);

    // Telescoping: the three sections together, at the rated speed in all.
    const f32 total = p.sectionExtension * static_cast<f32>(MobileCraneParameters::SectionCount);
    mTelescopeTarget = Math::clamp(mTelescopeTarget + Math::clamp(c.telescope, -1.0f, 1.0f) * p.telescopeSpeed * step,
                                   0.0f, total);
    for (u32 i = 0; i < MobileCraneParameters::SectionCount; ++i)
        mTelescopes[i]->setServoTarget(mTelescopeTarget / static_cast<f32>(MobileCraneParameters::SectionCount));

    for (u32 i = 0; i < 4; ++i)
    {
        mBeamSliders[i]->setServoTarget(Math::clamp(c.outriggerBeams, 0.0f, 1.0f) *
                                        (p.outriggerSpread - p.outriggerRetracted));
        mJackSliders[i]->setServoTarget(Math::clamp(c.outriggerJacks, 0.0f, 1.0f) * p.jackStroke);
    }
}

Math::vec3 MobileCrane::slewAxisWorld() const
{
    return mBodies[Superstructure].pointToWorld(mSlewLocal[1]);
}

Math::vec3 MobileCrane::headSheaveWorld() const
{
    return mBodies[BoomBase + MobileCraneParameters::SectionCount].pointToWorld(mHeadLocal);
}

f32 MobileCrane::boomLength() const
{
    return Math::length(headSheaveWorld() - mBodies[BoomBase].pointToWorld(mFootLocal[1]));
}

f32 MobileCrane::boomElevation() const
{
    return mParameters.buildElevation + mLuff->currentAngle();
}

f32 MobileCrane::radius() const
{
    const Math::vec3 d = headSheaveWorld() - slewAxisWorld();
    const Math::vec3 up = mBodies[Chassis].directionToWorld(Math::vec3(0.0f, 1.0f, 0.0f));
    const Math::vec3 horizontal = d - up * Math::dot(d, up);
    return Math::length(horizontal);
}

f32 MobileCrane::hookLoad() const
{
    return mRope->hookLoad() / kGravity;
}

f32 MobileCrane::outriggerLoad(u32 index) const
{
    return index < 4 ? mJackSliders[index]->motorForce() : 0.0f;
}

f32 MobileCrane::carrierGroundLoad() const
{
    const f32 step = mWorld->fixedStep();
    f32 total = 0.0f;
    for (const Contact& contact : mWorld->contacts())
        if (contact.idA == mIds[Chassis] || contact.idB == mIds[Chassis])
            for (u32 k = 0; k < contact.manifold.count; ++k)
                total += contact.manifold.points[k].normalImpulse;
    return step > 0.0f ? total / step : 0.0f;
}

f32 MobileCrane::tilt() const
{
    const Math::vec3 up = mBodies[Chassis].directionToWorld(Math::vec3(0.0f, 1.0f, 0.0f));
    return acosf(Math::clamp(Math::dot(up, mOrientation * Math::vec3(0.0f, 1.0f, 0.0f)), -1.0f, 1.0f));
}

f32 MobileCrane::ropeLength() const
{
    return Math::length(mBodies[HookBlock].position() - headSheaveWorld());
}

f32 MobileCrane::swingAngle() const
{
    const Math::vec3 d = mBodies[HookBlock].position() - headSheaveWorld();
    const f32 length = Math::length(d);
    if (!(length > 1.0e-6f))
        return 0.0f;
    const Math::vec3 down = mOrientation * Math::vec3(0.0f, -1.0f, 0.0f);
    return acosf(Math::clamp(Math::dot(d / length, down), -1.0f, 1.0f));
}

bool MobileCrane::finite() const
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

f32 MobileCrane::worstJointSeparation() const
{
    f32 worst = Math::length(mBodies[Chassis].pointToWorld(mSlewLocal[0]) -
                             mBodies[Superstructure].pointToWorld(mSlewLocal[1]));
    worst = Math::max(worst, Math::length(mBodies[Superstructure].pointToWorld(mFootLocal[0]) -
                                          mBodies[BoomBase].pointToWorld(mFootLocal[1])));
    return worst;
}

} // namespace kinetix::Physics
