#ifndef KINETIX_PHYSICS_MACHINERY_ANTISWAY_H
#define KINETIX_PHYSICS_MACHINERY_ANTISWAY_H

#include <kinetix/core/Types.h>

#include <ct/vector.hpp>

namespace kinetix::Physics
{

// Zero-vibration input shaper for a suspended load's command (slew or trolley lever).
// A rope pendulum has period T = 2 pi sqrt(L / g); the command is split in two halves T / 2
// apart whose swings cancel. Exact for an undamped pendulum and an actuator that follows its command.
class InputShaper
{
public:
    // Seconds; <= 0 passes the command through unchanged.
    void setPeriod(f32 period);
    f32 period() const
    {
        return mPeriod;
    }
    static f32 pendulumPeriod(f32 ropeLength, f32 gravity = 9.81f);

    f32 shape(f32 command, f32 step);
    void reset();

private:
    struct Sample
    {
        f32 time = 0.0f;
        f32 value = 0.0f;
    };
    f32 mPeriod = 0.0f;
    f32 mTime = 0.0f;
    ct::Vector<Sample> mHistory;
};

} // namespace kinetix::Physics

#endif // KINETIX_PHYSICS_MACHINERY_ANTISWAY_H
