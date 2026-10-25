#ifndef KINETIX_PHYSICS_MACHINERY_TERRAMECHANICS_H
#define KINETIX_PHYSICS_MACHINERY_TERRAMECHANICS_H

#include <kinetix/core/Types.h>

namespace kinetix::Physics
{

// Ground under a track (or blade, or bucket) in classic terramechanics parameters, SI (Pa, m, rad).
//
//  - Bekker pressure-sinkage: p = (kc / b + kphi) z^n, plate of smaller dimension b sunk z.
//    kc (N/m^(n+1)) cohesive, kphi (N/m^(n+2)) frictional modulus of deformation.
//  - Mohr-Coulomb strength tau_max = c + sigma tan(phi); Janosi-Hanamoto shear build-up with
//    shear displacement j: tau = tau_max (1 - exp(-j / K)), K the shear deformation modulus.
//  - Hard surface (concrete, rock, asphalt): a stiff contact (`hardStiffness`, Pa per m of
//    compression - the track's pads and grousers) and Coulomb friction mu (tau_max = mu sigma),
//    still reached over the shear displacement K.
//  - For digging: bulk density and soil-on-steel friction angle.
//
// The presets are published values (Bekker's and Wong's tables).
struct SoilParameters
{
    f32 kc = 5.27e3f;
    f32 kphi = 1515.04e3f;
    f32 n = 0.7f;
    f32 cohesion = 1.72e3f;
    f32 frictionAngle = 0.506f;
    f32 shearModulus = 0.025f;
    bool hard = false;
    f32 friction = 0.7f;
    f32 hardStiffness = 4.0e7f;
    f32 density = 1700.0f;
    f32 toolFrictionAngle = 0.4f;

    static SoilParameters drySand();
    static SoilParameters loam();
    static SoilParameters clay();
    static SoilParameters heavyClay();
    // Fresh snow: soft (n > 1), little shear strength.
    static SoilParameters snow();
    static SoilParameters hardSurface(f32 mu = 0.7f);
};

namespace Terramechanics
{

// Bekker's pressure (Pa) under a plate of width b sunk z (m); zero for z <= 0. On a hard
// surface, the contact's stiffness times z.
f32 pressure(const SoilParameters& soil, f32 sinkage, f32 width);
// d pressure / d sinkage at z (Pa/m), for linearising.
f32 pressureSlope(const SoilParameters& soil, f32 sinkage, f32 width);
// z = (p / (kc / b + kphi))^(1/n).
f32 sinkage(const SoilParameters& soil, f32 pressure, f32 width);
// Mohr-Coulomb shear strength under a normal pressure sigma, Pa.
f32 shearStrength(const SoilParameters& soil, f32 normalPressure);
// Janosi-Hanamoto shear stress at shear displacement j (m, >= 0), Pa.
f32 shearStress(const SoilParameters& soil, f32 normalPressure, f32 shearDisplacement);
// Tractive effort of a track of contact area A = b L under uniform load W at slip i (0..1), the
// shear displacement growing from zero at the front to i L at the rear:
//   H = (A c + W tan phi) [1 - K / (i L) (1 - exp(-i L / K))].
// Tends to A c + W tan phi at large slip (mu W on a hard surface).
f32 tractiveEffort(const SoilParameters& soil, f32 load, f32 width, f32 length, f32 slip);
// The most such a track under W can push: A c + W tan phi.
f32 maxTractiveEffort(const SoilParameters& soil, f32 load, f32 width, f32 length);
// Bekker's motion resistance from compaction for a track of width b sunk z:
// R = b (kc / b + kphi) z^(n+1) / (n + 1), N. Zero on a hard surface.
f32 compactionResistance(const SoilParameters& soil, f32 width, f32 sinkage);

} // namespace Terramechanics

} // namespace kinetix::Physics

#endif // KINETIX_PHYSICS_MACHINERY_TERRAMECHANICS_H
