#include "check.h"
#include "machinery_rig.h"

#include <machinery/Excavator.h>

using namespace machinery_test;

namespace
{

constexpr u32 kDiggable = 1u << 7;

void settle(Scene& scene, Excavator& e, f32 seconds)
{
    const int steps = static_cast<int>(seconds / scene.step + 0.5f);
    for (int i = 0; i < steps; ++i)
    {
        e.update(scene.step);
        scene.world.step(scene.step);
    }
}

f32 distance(const ArmPoint& a, const ArmPoint& b)
{
    return sqrtf((a.z - b.z) * (a.z - b.z) + (a.y - b.y) * (a.y - b.y));
}

// Hinge angles of bucket and rocker against the four-bar's closed form from the cylinder length.
void fourBarClosure()
{
    Scene scene;
    Excavator e(scene.world, ExcavatorParameters::crawler20t(), KM::vec3(0.0f, 0.745f, 0.0f));
    e.setGroundSoil(SoilParameters::loam());
    settle(scene, e, 1.0f);
    f32 worstBucket = 0.0f;
    f32 worstRocker = 0.0f;
    f32 worstLink = 0.0f;
    f32 minimum = 1.0e9f;
    f32 maximum = -1.0e9f;
    f32 lowStroke = 1.0e9f;
    f32 highStroke = -1.0e9f;
    for (int phase = 0; phase < 2; ++phase)
    {
        ExcavatorControls c;
        c.bucket = phase == 0 ? 1.0f : -1.0f;
        e.setControls(c);
        for (int i = 0; i < 6 * 60; ++i)
        {
            e.update(scene.step);
            scene.world.step(scene.step);
            f32 rocker = 0.0f;
            f32 bucket = 0.0f;
            const bool closes = e.kinematics().bucketLinkage(e.bucketCylinder().length(), &rocker, &bucket);
            CHECK(closes);
            worstBucket = fmaxf(worstBucket, fabsf(bucket - e.bucketJoint().currentAngle()));
            worstRocker = fmaxf(worstRocker, fabsf(rocker - e.rockerJoint().currentAngle()));
            worstLink = fmaxf(worstLink, fabsf(KM::length(e.linkJoint().worldAnchorB() - e.linkJoint().worldAnchorA()) -
                                                e.linkJoint().minDistance()));
            minimum = fminf(minimum, e.bucketJoint().currentAngle());
            maximum = fmaxf(maximum, e.bucketJoint().currentAngle());
            lowStroke = fminf(lowStroke, e.bucketCylinder().stroke());
            highStroke = fmaxf(highStroke, e.bucketCylinder().stroke());
        }
    }
    printf("four-bar over the stroke %.3f..%.3f m: bucket %.1f..%.1f deg; vs closed form: bucket %.2e rad, rocker "
           "%.2e rad, H-link %.2e m\n",
           double(lowStroke), double(highStroke), double(minimum / kDeg), double(maximum / kDeg), double(worstBucket),
           double(worstRocker), double(worstLink));
    CHECK(highStroke - lowStroke > 0.95f * e.bucketCylinder().parameters().stroke);
    CHECK(maximum - minimum > 150.0f * kDeg);
    CHECK(worstBucket < 2.0e-3f);
    CHECK(worstRocker < 2.0e-3f);
    CHECK(worstLink < 1.0e-4f);
    CHECK(e.finite());
}

void reachAndDepthEnvelope()
{
    const ExcavatorParameters p = ExcavatorParameters::crawler20t();
    const ExcavatorKinematics k(p.geometry);
    f32 deepest = 1.0e9f;
    f32 furthest = 0.0f;
    f32 highest = -1.0e9f;
    const f32 boomRest = k.boomCylinderLength(0.0f) - p.boomCylinder.initialStroke;
    const f32 stickRest = k.stickCylinderLength(0.0f) - p.stickCylinder.initialStroke;
    for (int i = 0; i <= 20; ++i)
        for (int j = 0; j <= 20; ++j)
        {
            const f32 boom = k.boomAngle(boomRest + p.boomCylinder.stroke * i / 20.0f);
            const f32 stick = k.stickAngle(stickRest + p.stickCylinder.stroke * j / 20.0f);
            const ArmPoint pin = k.bucketPinAt(boom, stick);
            deepest = fminf(deepest, pin.y);
            highest = fmaxf(highest, pin.y);
            furthest = fmaxf(furthest, pin.z);
        }
    printf("bucket pin envelope: deepest %.2f m, highest %.2f m, furthest %.2f m (from the ring)\n", double(deepest),
           double(highest), double(furthest));

    // Machine at the edge of a pad, ground 8 m below in front: the arm is free to reach below grade.
    Scene scene(60.0f, false);
    scene.addStatic(new BoxShape(KM::vec3(10.0f, 1.0f, 10.0f)), KM::vec3(0.0f, -1.0f, -8.0f), KM::quat());
    scene.addStatic(new BoxShape(KM::vec3(30.0f, 1.0f, 30.0f)), KM::vec3(0.0f, -9.0f, 0.0f), KM::quat());
    Excavator e(scene.world, p, KM::vec3(0.0f, 0.745f, 0.0f));
    e.setGroundSoil(SoilParameters::loam());
    settle(scene, e, 1.0f);
    ExcavatorControls c;
    c.boom = 0.6f;
    c.stick = -0.5f;
    c.bucket = 0.7f;
    e.setControls(c);
    f32 worst = 0.0f;
    for (int i = 0; i < 3 * 60; ++i)
    {
        e.update(scene.step);
        scene.world.step(scene.step);
        const ArmPoint fk = k.toothTip(e.boomJoint().currentAngle(), e.stickJoint().currentAngle(),
                                       e.bucketJoint().currentAngle());
        worst = fmaxf(worst, distance(fk, e.toothTip()));
    }
    printf("forward kinematics vs bodies while moving: %.2e m\n", double(worst));
    CHECK(worst < 2.0e-3f);

    // Targets from the forward kinematics at stroke fractions, so inside the envelope by construction.
    struct Target
    {
        const char* name;
        f32 boom;
        f32 stick;
        f32 bucket;
    };
    const Target targets[] = {{"far", 0.55f, 0.05f, 0.35f},
                              {"high", 0.98f, 0.35f, 0.25f},
                              {"close in", 0.45f, 0.95f, 0.85f},
                              {"deep", 0.0f, 0.55f, 0.6f}};
    const f32 bucketRest = e.bucketCylinder().parameters().closedLength;
    const f32 grade = -(e.ringCentreWorld().y - 0.0f);
    for (const Target& t : targets)
    {
        const f32 boom = k.boomAngle(boomRest + p.boomCylinder.stroke * t.boom);
        const f32 stick = k.stickAngle(stickRest + p.stickCylinder.stroke * t.stick);
        f32 bucket = 0.0f;
        CHECK(k.bucketLinkage(bucketRest + p.bucketCylinder.stroke * t.bucket, nullptr, &bucket));
        const ArmPoint tooth = k.toothTip(boom, stick, bucket);
        const ArmPoint pin = k.bucketPinAt(boom, stick);
        const f32 attitude = atan2f(tooth.y - pin.y, tooth.z - pin.z);
        e.setToothTarget(tooth, attitude);
        settle(scene, e, 7.0f);
        const ArmPoint got = e.toothTip();
        const f32 error = distance(got, tooth);
        printf("tooth target %-8s (%.2f, %.2f) at %.2f rad, %.2f m from grade: reached (%.3f, %.3f) at %.3f rad, %.3f m "
               "off; joints %.1e m\n",
               t.name, double(tooth.z), double(tooth.y), double(attitude), double(tooth.y - grade), double(got.z),
               double(got.y), double(e.attitude()), double(error), double(e.worstJointSeparation()));
        CHECK(error < 0.02f);
        CHECK_NEAR(e.attitude(), attitude, 0.01f);
        CHECK(e.worstJointSeparation() < 1.0e-4f);
    }
    f32 depth = 0.0f;
    f32 reach = 0.0f;
    f32 height = 0.0f;
    for (int i = 0; i <= 24; ++i)
        for (int j = 0; j <= 24; ++j)
            for (int m = 0; m <= 8; ++m)
            {
                const f32 boom = k.boomAngle(boomRest + p.boomCylinder.stroke * i / 24.0f);
                const f32 stick = k.stickAngle(stickRest + p.stickCylinder.stroke * j / 24.0f);
                f32 bucket = 0.0f;
                if (!k.bucketLinkage(bucketRest + p.bucketCylinder.stroke * m / 8.0f, nullptr, &bucket))
                    continue;
                const ArmPoint tooth = k.toothTip(boom, stick, bucket);
                depth = fmaxf(depth, grade - tooth.y);
                height = fmaxf(height, tooth.y - grade);
                if (fabsf(tooth.y - grade) < 0.15f)
                    reach = fmaxf(reach, tooth.z);
            }
    printf("teeth envelope: digging depth %.2f m, reach at grade %.2f m (from the slew axis), height %.2f m\n",
           double(depth), double(reach), double(height));
    CHECK(depth > 3.0f);
    CHECK(reach > 8.0f);
    CHECK(height > 6.0f);
    e.clearToothTarget();
    CHECK(e.finite());
}

void slewingWithLoad()
{
    Scene scene;
    Excavator e(scene.world, ExcavatorParameters::crawler20t(), KM::vec3(0.0f, 0.745f, 0.0f));
    e.setGroundSoil(SoilParameters::loam());
    e.setToothTarget(ArmPoint{6.0f, 0.5f}, -2.6f);
    settle(scene, e, 6.0f);
    const KM::vec3 pin = e.bucketPinWorld();
    const KM::vec3 loadCentre = e.toothTipWorld() + (pin - e.toothTipWorld()) * 0.5f;
    u32 loadId = 0;
    RigidBody& load = scene.box(loadCentre, KM::vec3(0.35f), 1000.0f, false, &loadId);
    FixedJoint weld(e.bucket(), load, loadCentre);
    weld.setArticulated(true);
    scene.world.addJoint(&weld);
    settle(scene, e, 1.0f);
    const f32 start = e.swingJoint().currentAngle();
    const f32 speed = e.swingMotor().speedPerFlow() * e.parameters().swingSection.ratedFlow;
    ExcavatorControls c;
    c.swing = 1.0f;
    e.setControls(c);
    f32 fastest = 0.0f;
    f32 worstTilt = 0.0f;
    f32 worstJoint = 0.0f;
    f32 turned = 0.0f;
    f32 last = start;
    bool stopping = false;
    for (int i = 0; i < 12 * 60; ++i)
    {
        e.update(scene.step);
        scene.world.step(scene.step);
        f32 delta = e.swingJoint().currentAngle() - last;
        if (delta > kPi)
            delta -= 2.0f * kPi;
        if (delta < -kPi)
            delta += 2.0f * kPi;
        turned += delta;
        last = e.swingJoint().currentAngle();
        fastest = fmaxf(fastest, e.swingJoint().currentAngularVelocity());
        const KM::vec3 up = e.undercarriage().directionToWorld(KM::vec3(0.0f, 1.0f, 0.0f));
        worstTilt = fmaxf(worstTilt, acosf(fminf(up.y, 1.0f)));
        worstJoint = fmaxf(worstJoint, e.worstJointSeparation());
        if (!stopping && i == 7 * 60)
        {
            stopping = true;
            e.setControls(ExcavatorControls());
        }
    }
    printf("slewing with 1 t at 6 m: top speed %.3f rad/s (flow gives %.3f), turned %.1f deg, stopped at %.4f rad/s; "
           "tilt <= %.2f deg, joints <= %.1e m, engine %.0f rpm\n",
           double(fastest), double(speed), double(turned / kDeg), double(e.swingJoint().currentAngularVelocity()),
           double(worstTilt / kDeg), double(worstJoint), double(e.hydraulics().engine().rpm()));
    CHECK(fastest > 0.9f * speed);
    CHECK(fastest < 1.01f * speed);
    CHECK(turned > kPi);
    CHECK(fabsf(e.swingJoint().currentAngularVelocity()) < 1.0e-3f);
    CHECK(worstTilt < 3.0f * kDeg);
    CHECK(worstJoint < 1.0e-4f);
    CHECK(e.finite() && finiteVec(load.position()));
    scene.world.removeJoint(&weld);
}

void travel()
{
    Scene scene;
    Excavator e(scene.world, ExcavatorParameters::crawler20t(), KM::vec3(0.0f, 0.745f, 0.0f));
    e.setGroundSoil(SoilParameters::loam());
    settle(scene, e, 1.0f);
    ExcavatorControls c;
    c.leftTrack = 1.0f;
    c.rightTrack = 1.0f;
    e.setControls(c);
    settle(scene, e, 4.0f);
    const KM::vec3 forward = e.undercarriage().directionToWorld(KM::vec3(0.0f, 0.0f, 1.0f));
    const f32 speed = KM::dot(e.undercarriage().velocity(), forward);
    const f32 belt = e.tracks().side(0).beltSpeed;
    const f32 flowSpeed = e.hydraulics().section(e.section(Excavator::LeftTravelSection)).flow *
                          e.parameters().travelMotor.volumetricEfficiency / 60000.0f /
                          (e.parameters().travelMotor.displacement * 1.0e-6f * e.parameters().travelMotor.gearRatio) *
                          e.parameters().tracks.sprocketRadius * 2.0f * kPi;
    printf("travel on loam: %.3f m/s (%.1f km/h), belts %.3f m/s (flow gives %.3f), slip %.4f\n", double(speed),
           double(speed * 3.6f), double(belt), double(flowSpeed), double(e.tracks().side(0).slip));
    CHECK_NEAR(belt, flowSpeed, 0.01f * flowSpeed);
    CHECK(speed > 0.95f * belt);
    c.leftTrack = 1.0f;
    c.rightTrack = -1.0f;
    e.setControls(c);
    // The machine's centre of mass: a spin turn turns about it.
    auto centreOfMass = [&]()
    {
        KM::vec3 sum(0.0f);
        f32 mass = 0.0f;
        for (u32 i = 0; i < Excavator::BodyCount; ++i)
        {
            sum += e.body(i).position() * e.body(i).mass();
            mass += e.body(i).mass();
        }
        return sum / mass;
    };
    settle(scene, e, 1.0f);
    const KM::vec3 before = centreOfMass();
    f32 drift = 0.0f;
    for (int i = 0; i < 4 * 60; ++i)
    {
        e.update(scene.step);
        scene.world.step(scene.step);
        const KM::vec3 moved = centreOfMass() - before;
        drift = fmaxf(drift, sqrtf(moved.x * moved.x + moved.z * moved.z));
    }
    const f32 yaw = e.undercarriage().angularVelocity().y;
    printf("spin turn on loam: yaw rate %.3f rad/s, centre of mass drifted at most %.3f m\n", double(yaw), double(drift));
    CHECK(yaw < -0.2f);
    CHECK(drift < 0.5f);
    CHECK(e.finite());
}

void diggingResistance()
{
    Scene scene(60.0f, false);
    scene.addStatic(new BoxShape(KM::vec3(50.0f, 1.0f, 50.0f)), KM::vec3(0.0f, -1.0f, -50.0f + 3.0f), KM::quat());
    scene.addStatic(new BoxShape(KM::vec3(50.0f, 1.0f, 20.0f)), KM::vec3(0.0f, -1.0f, 9.0f + 20.0f), KM::quat());
    scene.addStatic(new BoxShape(KM::vec3(20.0f, 1.0f, 3.0f)), KM::vec3(23.0f, -1.0f, 6.0f), KM::quat());
    scene.addStatic(new BoxShape(KM::vec3(20.0f, 1.0f, 3.0f)), KM::vec3(-23.0f, -1.0f, 6.0f), KM::quat());
    // The pit's soil: a static body the bucket does not collide with.
    RigidBody* soil = new RigidBody();
    soil->setBodyType(BodyType::Static);
    soil->setPosition(KM::vec3(0.0f, -1.5f, 6.0f));
    soil->calculateDerivedData();
    BoxShape* soilShape = new BoxShape(KM::vec3(3.0f, 1.5f, 3.0f));
    BodyEntry entry;
    entry.body = soil;
    entry.shape = soilShape;
    entry.filter.group = kDiggable;
    scene.world.addBody(entry);
    scene.world.markStaticBroadphaseDirty();
    scene.bodies.push_back(soil);
    scene.shapes.push_back(soilShape);

    Excavator e(scene.world, ExcavatorParameters::crawler20t(), KM::vec3(0.0f, 0.745f, 0.0f));
    e.setGroundSoil(SoilParameters::loam());
    SoilBed bed;
    bed.minX = -3.0f;
    bed.maxX = 3.0f;
    bed.minZ = 3.0f;
    bed.maxZ = 9.0f;
    bed.surfaceHeight = 0.0f;
    bed.soil = SoilParameters::heavyClay();
    f32 rates[2] = {0.0f, 0.0f};
    f32 forces[2] = {0.0f, 0.0f};
    f32 stickForce[2] = {0.0f, 0.0f};
    f32 depth = 0.0f;
    f32 resistance = 0.0f;
    for (int pass = 0; pass < 2; ++pass)
    {
        SoilBed active = bed;
        if (pass == 0)
            active.surfaceHeight = -100.0f;
        e.setSoilBed(active);
        // Teeth in at 0.35 m below grade (grade is -1.1 in the arm frame),
        // 7.5 m out.
        e.setToothTarget(ArmPoint{7.5f, -1.1f + 0.3f}, -1.9f);
        settle(scene, e, 5.0f);
        e.setToothTarget(ArmPoint{7.5f, -1.1f - 0.35f}, -1.9f);
        settle(scene, e, 3.0f);
        e.clearToothTarget();
        ExcavatorControls c;
        c.stick = 0.5f;
        c.boom = 0.12f;
        e.setControls(c);
        const f32 start = e.toothTip().z;
        f32 force = 0.0f;
        f32 push = 0.0f;
        for (int i = 0; i < 60; ++i)
        {
            e.update(scene.step);
            scene.world.step(scene.step);
            force += KM::length(e.teeth().force()) / 60.0f;
            push += e.stickCylinder().force() / 60.0f;
            if (pass == 1 && i == 30)
            {
                depth = e.teeth().depth();
                resistance = e.teeth().resistance();
            }
        }
        rates[pass] = (start - e.toothTip().z) / 1.0f;
        forces[pass] = force;
        stickForce[pass] = push;
        e.setControls(ExcavatorControls());
        e.setToothTarget(ArmPoint{6.0f, 1.0f}, -1.9f);
        settle(scene, e, 4.0f);
        e.clearToothTarget();
    }
    const f32 model = 0.65f * SoilCuttingTool::cuttingResistance(bed.soil, 1.0f, depth, 0.0f, 0.785398f);
    printf("dragging the teeth at %.2f m depth: in air %.3f m/s, in heavy clay %.3f m/s; soil force %.0f N (air %.0f), "
           "resistance bound %.0f N (the model at that depth, still: %.0f); stick cylinder %.0f N vs %.0f N in air\n",
           double(depth), double(rates[0]), double(rates[1]), double(forces[1]), double(forces[0]),
           double(resistance), double(model), double(stickForce[1]), double(stickForce[0]));
    CHECK(depth > 0.2f);
    CHECK(forces[0] < 1.0f);
    CHECK(forces[1] > 0.9f * resistance);
    CHECK(forces[1] <= resistance * 1.01f);
    CHECK(rates[1] <= rates[0] + 1.0e-3f);
    // The stick cylinder (crowding in, extending) pushes harder by a good
    // part of the soil's force through the stick's lever.
    CHECK(stickForce[1] - stickForce[0] > 0.5f * forces[1]);
    CHECK(e.finite());
}

}

int main()
{
    fourBarClosure();
    reachAndDepthEnvelope();
    slewingWithLoad();
    travel();
    diggingResistance();
    return TEST_RESULT();
}
