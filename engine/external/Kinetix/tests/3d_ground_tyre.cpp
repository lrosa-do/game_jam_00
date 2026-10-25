#include "check.h"

#include <ground/Tyre.h>

#include <math.h>
#include <stdio.h>

using namespace kinetix::Physics;
using kinetix::f32;
using kinetix::u32;

namespace
{
constexpr f32 kPi = 3.14159265358979f;
constexpr f32 kDeg = kPi / 180.0f;

void peaksWhereConfigured(const TyreParameters& parameters, const char* name)
{
    const Tyre tyre(parameters);
    const SurfaceInfo asphalt;
    const f32 load = parameters.nominalLoad;
    const f32 mu = tyre.frictionCoefficient(load, asphalt);

    f32 bestKappa = 0.0f;
    f32 bestFx = 0.0f;
    for (u32 i = 0; i <= 4000; ++i)
    {
        const f32 kappa = 1.0f * static_cast<f32>(i) / 4000.0f;
        const f32 fx = tyre.evaluate(load, kappa, 0.0f, asphalt).fx;
        if (fx > bestFx)
        {
            bestFx = fx;
            bestKappa = kappa;
        }
    }
    f32 bestAlpha = 0.0f;
    f32 bestFy = 0.0f;
    for (u32 i = 0; i <= 4000; ++i)
    {
        const f32 alpha = 1.0f * static_cast<f32>(i) / 4000.0f;
        const f32 fy = tyre.evaluate(load, 0.0f, alpha, asphalt).fy;
        if (fy > bestFy)
        {
            bestFy = fy;
            bestAlpha = alpha;
        }
    }
    const f32 locked = tyre.evaluate(load, 1.0f, 0.0f, asphalt).fx;
    const f32 sliding = tyre.evaluate(load, 0.0f, 60.0f * kDeg, asphalt).fy;
    printf("%s: Fx peak %.0f N at kappa %.3f (set %.3f), locked %.0f N (%.0f%%); Fy peak %.0f N at %.2f deg "
           "(set %.2f), at 60 deg %.0f N; mu %.3f, Cx %.0f N, Calpha %.0f N/rad\n",
           name, double(bestFx), double(bestKappa), double(parameters.peakSlipRatio), double(locked),
           double(100.0f * locked / bestFx), double(bestFy), double(bestAlpha / kDeg),
           double(parameters.peakSlipAngle / kDeg), double(sliding), double(mu),
           double(tyre.longitudinalStiffness(load, asphalt)), double(tyre.corneringStiffness(load, asphalt)));
    CHECK_NEAR(bestKappa, parameters.peakSlipRatio, 0.002);
    CHECK_NEAR(bestAlpha, parameters.peakSlipAngle, 0.002);
    CHECK_NEAR(bestFx, mu * load, 1.0e-3 * mu * load);
    CHECK_NEAR(bestFy, mu * parameters.lateralFrictionRatio * load, 1.0e-3 * mu * load);
    CHECK(locked < 0.98f * bestFx);
    CHECK(locked > 0.5f * bestFx);
    CHECK(sliding < 0.99f * bestFy);
    CHECK(sliding > 0.5f * bestFy);
    f32 previous = 0.0f;
    bool monotone = true;
    for (u32 i = 1; i <= 100; ++i)
    {
        const f32 fx = tyre.evaluate(load, parameters.peakSlipRatio * static_cast<f32>(i) / 100.0f, 0.0f, asphalt).fx;
        monotone = monotone && fx > previous;
        previous = fx;
    }
    CHECK(monotone);
    const f32 h = 1.0e-4f;
    CHECK_NEAR(tyre.evaluate(load, h, 0.0f, asphalt).fx / h, tyre.longitudinalStiffness(load, asphalt),
               0.01 * tyre.longitudinalStiffness(load, asphalt));
    CHECK_NEAR(tyre.evaluate(load, 0.0f, h, asphalt).fy / h, tyre.corneringStiffness(load, asphalt),
               0.01 * tyre.corneringStiffness(load, asphalt));
}

void antisymmetric()
{
    const Tyre tyre(TyreParameters::carRoad());
    const SurfaceInfo asphalt;
    f32 worst = 0.0f;
    for (f32 kappa = -1.2f; kappa <= 1.2f; kappa += 0.05f)
    {
        for (f32 alpha = -0.6f; alpha <= 0.6f; alpha += 0.03f)
        {
            const TyreForces a = tyre.evaluate(3500.0f, kappa, alpha, asphalt);
            const TyreForces b = tyre.evaluate(3500.0f, -kappa, alpha, asphalt);
            const TyreForces c = tyre.evaluate(3500.0f, kappa, -alpha, asphalt);
            worst = fmaxf(worst, fabsf(a.fx + b.fx));
            worst = fmaxf(worst, fabsf(a.fy - b.fy));
            worst = fmaxf(worst, fabsf(a.fy + c.fy));
            worst = fmaxf(worst, fabsf(a.fx - c.fx));
            worst = fmaxf(worst, fabsf(a.mz + c.mz));
            if (kappa > 1.0e-3f)
                CHECK(a.fx > 0.0f);
            if (alpha > 1.0e-3f)
                CHECK(a.fy > 0.0f);
        }
    }
    printf("antisymmetry: worst mismatch %.3g N\n", double(worst));
    CHECK(worst < 1.0e-2f);
}

void frictionEllipse()
{
    const SurfaceInfo asphalt;
    const TyreParameters presets[3] = {TyreParameters::carRoad(), TyreParameters::truck(true),
                                       TyreParameters::tractorLug(15000.0f)};
    f32 worst = 0.0f;
    for (const TyreParameters& p : presets)
    {
        const Tyre tyre(p);
        for (f32 load = 0.2f * p.nominalLoad; load <= 2.5f * p.nominalLoad; load += 0.3f * p.nominalLoad)
        {
            const f32 mu = tyre.frictionCoefficient(load, asphalt);
            for (f32 kappa = -2.0f; kappa <= 2.0f; kappa += 0.037f)
            {
                for (f32 alpha = -1.2f; alpha <= 1.2f; alpha += 0.031f)
                {
                    const TyreForces f = tyre.evaluate(load, kappa, alpha, asphalt);
                    const f32 ex = f.fx / (mu * load);
                    const f32 ey = f.fy / (mu * p.lateralFrictionRatio * load);
                    worst = fmaxf(worst, ex * ex + ey * ey);
                    // Never more than mu Fz in total either (mu_y <= mu_x).
                    CHECK(sqrtf(f.fx * f.fx + f.fy * f.fy) <= mu * load * 1.0001f);
                }
            }
        }
    }
    printf("friction ellipse: worst (Fx/Dx)^2 + (Fy/Dy)^2 = %.6f\n", double(worst));
    CHECK(worst <= 1.0001f);

    const Tyre tyre(TyreParameters::carRoad());
    const f32 pure = tyre.evaluate(3500.0f, 0.0f, 4.0f * kDeg, asphalt).fy;
    const f32 braked = tyre.evaluate(3500.0f, -0.08f, 4.0f * kDeg, asphalt).fy;
    const f32 pureX = tyre.evaluate(3500.0f, -0.08f, 0.0f, asphalt).fx;
    const f32 turningX = tyre.evaluate(3500.0f, -0.08f, 4.0f * kDeg, asphalt).fx;
    printf("combined: Fy %.0f -> %.0f N with 8%% braking slip, Fx %.0f -> %.0f N at 4 deg\n", double(pure),
           double(braked), double(pureX), double(turningX));
    CHECK(braked < 0.9f * pure);
    CHECK(fabsf(turningX) < 0.95f * fabsf(pureX));
}

void loadAndSurfaces()
{
    const SurfaceInfo asphalt;
    const Tyre car(TyreParameters::carRoad());
    const f32 nominal = car.parameters().nominalLoad;
    CHECK(car.evaluate(0.0f, 0.1f, 0.1f, asphalt).fx == 0.0f);
    CHECK(car.evaluate(0.0f, 0.1f, 0.1f, asphalt).fy == 0.0f);
    CHECK(car.evaluate(-100.0f, 0.1f, 0.1f, asphalt).fx == 0.0f);
    CHECK(car.evaluate(NAN, 0.1f, 0.1f, asphalt).fx == 0.0f);
    CHECK(car.evaluate(nominal, NAN, 0.1f, asphalt).fy == 0.0f);
    CHECK(car.evaluate(nominal, 0.1f, INFINITY, asphalt).fx == 0.0f);
    const TyreForces huge = car.evaluate(nominal, 1.0e6f, -1.0e6f, asphalt);
    CHECK(isfinite(huge.fx) && isfinite(huge.fy) && isfinite(huge.mz));
    f32 kx = -1.0f;
    f32 ky = -1.0f;
    car.secantStiffness(0.0f, 0.1f, 0.1f, asphalt, 0.0f, kx, ky);
    CHECK(kx == 0.0f && ky == 0.0f);

    const f32 muLight = car.frictionCoefficient(0.5f * nominal, asphalt);
    const f32 muNominal = car.frictionCoefficient(nominal, asphalt);
    const f32 muHeavy = car.frictionCoefficient(2.0f * nominal, asphalt);
    printf("load sensitivity: mu %.3f / %.3f / %.3f at 0.5 / 1 / 2 Fz0\n", double(muLight), double(muNominal),
           double(muHeavy));
    CHECK(muLight > muNominal && muNominal > muHeavy);
    CHECK_NEAR(muNominal, car.parameters().friction, 1.0e-5);
    CHECK(muHeavy * 2.0f * nominal > muNominal * nominal);

    SurfaceInfo s;
    s.type = SurfaceType::Ice;
    const f32 carIce = car.frictionCoefficient(nominal, s);
    s.type = SurfaceType::WetAsphalt;
    const f32 carWet = car.frictionCoefficient(nominal, s);
    s.type = SurfaceType::Mud;
    const f32 carMud = car.frictionCoefficient(nominal, s);
    CHECK(muNominal > carWet && carWet > carMud && carMud > carIce);
    CHECK(carIce < 0.15f);
    const Tyre lug(TyreParameters::tractorLug(15000.0f));
    s.type = SurfaceType::Soil;
    const f32 lugSoil = lug.frictionCoefficient(15000.0f, s);
    const f32 lugAsphalt = lug.frictionCoefficient(15000.0f, asphalt);
    const f32 carSoil = car.frictionCoefficient(nominal, s);
    printf("surfaces: car asphalt %.2f wet %.2f mud %.2f ice %.2f soil %.2f; lug soil %.2f asphalt %.2f\n",
           double(muNominal), double(carWet), double(carMud), double(carIce), double(carSoil), double(lugSoil),
           double(lugAsphalt));
    CHECK(lugSoil > lugAsphalt);
    CHECK(lugSoil > carSoil);
    s.type = SurfaceType::Asphalt;
    s.frictionScale = 0.5f;
    CHECK_NEAR(car.frictionCoefficient(nominal, s), 0.5f * muNominal, 1.0e-5);
    // Rolling resistance: C_rr * table * scale * Fz.
    s.frictionScale = 1.0f;
    s.type = SurfaceType::Grass;
    s.rollingResistanceScale = 2.0f;
    CHECK_NEAR(car.evaluate(nominal, 0.0f, 0.0f, s).rollingResistance,
               car.parameters().rollingResistance * car.parameters().surfaceRollingResistance[3] * 2.0f * nominal,
               1.0e-3);
}

void secantAgreesWithForces()
{
    const SurfaceInfo asphalt;
    const Tyre tyre(TyreParameters::carRoad());
    f32 worst = 0.0f;
    for (f32 kappa = -1.0f; kappa <= 1.0f; kappa += 0.07f)
    {
        for (f32 alpha = -0.5f; alpha <= 0.5f; alpha += 0.04f)
        {
            f32 kx = 0.0f;
            f32 ky = 0.0f;
            tyre.secantStiffness(3000.0f, kappa, alpha, asphalt, 0.0f, kx, ky);
            CHECK(kx >= 0.0f && ky >= 0.0f);
            const TyreForces f = tyre.evaluate(3000.0f, kappa, alpha, asphalt);
            worst = fmaxf(worst, fabsf(kx * kappa - f.fx));
            worst = fmaxf(worst, fabsf(ky * alpha - f.fy));
        }
    }
    printf("secant: worst |k s - F| %.3g N\n", double(worst));
    CHECK(worst < 0.05f);
    f32 kx = 0.0f;
    f32 ky = 0.0f;
    tyre.secantStiffness(3000.0f, 0.0f, 0.0f, asphalt, 0.0f, kx, ky);
    CHECK_NEAR(kx, tyre.longitudinalStiffness(3000.0f, asphalt), 1.0);
    CHECK_NEAR(ky, tyre.corneringStiffness(3000.0f, asphalt), 1.0);
}

void camberAndAligning()
{
    const SurfaceInfo asphalt;
    const Tyre tyre(TyreParameters::carRoad());
    // Camber thrust: a wheel leaning left pushes left at zero slip angle.
    const f32 thrust = tyre.evaluate(3500.0f, 0.0f, 0.0f, asphalt, 3.0f * kDeg).fy;
    const f32 opposite = tyre.evaluate(3500.0f, 0.0f, 0.0f, asphalt, -3.0f * kDeg).fy;
    printf("camber: %.0f N at +3 deg, %.0f N at -3 deg\n", double(thrust), double(opposite));
    CHECK(thrust > 0.0f);
    CHECK_NEAR(thrust, -opposite, 1.0e-2);
    // Aligning moment: restoring (opposite to Fy) at small slip, gone by
    // twice the peak slip angle.
    const f32 alphaPeak = tyre.parameters().peakSlipAngle;
    const TyreForces small = tyre.evaluate(3500.0f, 0.0f, 0.3f * alphaPeak, asphalt);
    const TyreForces past = tyre.evaluate(3500.0f, 0.0f, 2.2f * alphaPeak, asphalt);
    printf("aligning moment: %.1f N m at %.1f deg, %.2f at %.1f deg\n", double(small.mz),
           double(0.3f * alphaPeak / kDeg), double(past.mz), double(2.2f * alphaPeak / kDeg));
    CHECK(small.mz < 0.0f && small.fy > 0.0f);
    CHECK_NEAR(small.mz, -tyre.parameters().pneumaticTrail * (1.0f - 0.15f) * small.fy, 1.0e-2);
    CHECK(past.mz == 0.0f);
    CHECK(tyre.trail(-1.0f, 2.0f * kDeg) == 0.0f);
    CHECK_NEAR(tyre.trail(0.0f, 0.0f), tyre.parameters().pneumaticTrail, 1.0e-6);
}

void parametersSanitised()
{
    TyreParameters bad = TyreParameters::carRoad();
    bad.longitudinalShape = 0.5f; // no peak below 1
    bad.lateralCurvature = 3.0f;  // folds the curve above 1
    bad.peakSlipRatio = -1.0f;
    bad.nominalLoad = NAN;
    const Tyre tyre(bad);
    CHECK(tyre.parameters().longitudinalShape > 1.0f);
    CHECK(tyre.parameters().lateralCurvature < 1.0f);
    CHECK(tyre.parameters().peakSlipRatio > 0.0f);
    CHECK(isfinite(tyre.parameters().nominalLoad) && tyre.parameters().nominalLoad > 0.0f);
    const TyreForces f = tyre.evaluate(4000.0f, 0.2f, 0.1f, SurfaceInfo());
    CHECK(isfinite(f.fx) && isfinite(f.fy) && f.fx > 0.0f && f.fy > 0.0f);
    CHECK(surfaceTypeName(SurfaceType::Soil)[0] == 's');
}
}

int main()
{
    peaksWhereConfigured(TyreParameters::carRoad(), "car road tyre");
    peaksWhereConfigured(TyreParameters::truck(false), "truck tyre");
    peaksWhereConfigured(TyreParameters::tractorLug(15000.0f), "tractor lug tyre");
    antisymmetric();
    frictionEllipse();
    loadAndSurfaces();
    secantAgreesWithForces();
    camberAndAligning();
    parametersSanitised();
    return TEST_RESULT();
}
