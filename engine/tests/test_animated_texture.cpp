#include "Check.h"

#include "prisma3d/render/AnimatedTexture.h"

using namespace prisma3d;

namespace
{

struct Step
{
    float delta;
    int frame;
};

void durations(AnimatedTexture& texture, int count, const float* values)
{
    CHECK(texture.setFrames(count));
    for (int i = 0; i < count; ++i) CHECK(texture.setFrameDuration(i, values[i]));
}

void uniform(AnimatedTexture& texture, int count, float duration)
{
    CHECK(texture.setFrames(count));
    for (int i = 0; i < count; ++i) CHECK(texture.setFrameDuration(i, duration));
}

template<unsigned N>
void run(AnimatedTexture& texture, const Step (&steps)[N], const char* label)
{
    for (unsigned i = 0; i < N; ++i)
    {
        texture.update(steps[i].delta);
        if (texture.getCurrentFrame() != steps[i].frame)
            printf("%s: step %u (delta %g) is on frame %d, expected %d\n", label, i,
                    static_cast<double>(steps[i].delta), texture.getCurrentFrame(), steps[i].frame);
        CHECK(texture.getCurrentFrame() == steps[i].frame);
    }
}

void testDefaults()
{
    AnimatedTexture texture(nullptr);
    CHECK(!texture.getTexture().valid());
    CHECK(texture.getFrames() == 1);
    CHECK(texture.getCurrentFrame() == 0);
    CHECK(!texture.getPause());
    CHECK(!texture.getOneShot());
    CHECK(texture.getSpeedScale() == 1.0f);
    CHECK(texture.getFrameDuration(0) == 1.0f);
    CHECK(texture.getFrameDuration(255) == 1.0f);
    CHECK(!texture.getFrameTexture(0).valid());
    CHECK(texture.getWidth() == 1 && texture.getHeight() == 1);
    CHECK(texture.getAutoUpdate());
    texture.setAutoUpdate(false);
    CHECK(!texture.getAutoUpdate());

    texture.update(0.5f);
    CHECK(texture.getCurrentFrame() == 0);
    texture.update(1.5f);
    CHECK(texture.getCurrentFrame() == 0);
}

void testRanges()
{
    AnimatedTexture texture(nullptr);
    CHECK(!texture.setFrames(0));
    CHECK(!texture.setFrames(-1));
    CHECK(!texture.setFrames(AnimatedTexture::kMaxFrames + 1));
    CHECK(texture.setFrames(AnimatedTexture::kMaxFrames));
    CHECK(texture.getFrames() == 256);
    CHECK(texture.setFrames(1));
    CHECK(texture.getFrames() == 1);

    CHECK(!texture.setCurrentFrame(-1));
    CHECK(!texture.setCurrentFrame(1));
    CHECK(texture.setCurrentFrame(0));

    CHECK(!texture.setFrameDuration(-1, 1.0f));
    CHECK(!texture.setFrameDuration(256, 1.0f));
    CHECK(texture.setFrameDuration(255, 0.5f));
    CHECK(texture.getFrameDuration(255) == 0.5f);
    CHECK(texture.getFrameDuration(256) == 0.0f);
    CHECK(texture.getFrameDuration(-1) == 0.0f);

    CHECK(!texture.setFrameTexture(-1, TextureId()));
    CHECK(!texture.setFrameTexture(256, TextureId()));
    CHECK(texture.setFrameTexture(255, TextureId()));

    CHECK(!texture.setSpeedScale(1000.0f));
    CHECK(!texture.setSpeedScale(-1001.0f));
    CHECK(texture.setSpeedScale(999.5f));
    CHECK(texture.getSpeedScale() == 999.5f);
    CHECK(texture.setSpeedScale(-1000.0f));
    CHECK(texture.getSpeedScale() == -1000.0f);
}

void testUniform()
{
    AnimatedTexture texture(nullptr);
    uniform(texture, 4, 0.25f);
    const Step steps[] = { { 0.0f, 0 }, { 0.25f, 0 }, { 0.125f, 1 }, { 0.125f, 1 }, { 0.0625f, 2 },
        { 0.5f, 0 } };
    run(texture, steps, "uniform");
}

void testNonUniform()
{
    AnimatedTexture texture(nullptr);
    const float values[3] = { 0.5f, 0.25f, 1.0f };
    durations(texture, 3, values);
    const Step steps[] = { { 0.5f, 0 }, { 0.0625f, 1 }, { 0.1875f, 1 }, { 0.0625f, 2 },
        { 0.9375f, 2 }, { 0.0625f, 0 } };
    run(texture, steps, "non-uniform");
}

void testSpeedScale()
{
    {
        AnimatedTexture texture(nullptr);
        uniform(texture, 3, 0.25f);
        CHECK(texture.setSpeedScale(0.5f));
        const Step steps[] = { { 0.5f, 0 }, { 0.0625f, 1 }, { 0.4375f, 1 }, { 0.0625f, 2 } };
        run(texture, steps, "speed 0.5");
    }
    {
        AnimatedTexture texture(nullptr);
        uniform(texture, 3, 0.25f);
        CHECK(texture.setSpeedScale(2.0f));
        const Step steps[] = { { 0.125f, 0 }, { 0.0625f, 1 }, { 0.0625f, 1 }, { 0.0625f, 2 },
            { 0.0625f, 2 }, { 0.0625f, 0 } };
        run(texture, steps, "speed 2");
    }
    {
        AnimatedTexture texture(nullptr);
        uniform(texture, 3, 0.25f);
        CHECK(texture.setSpeedScale(-1.0f));
        const Step steps[] = { { 0.25f, 0 }, { 0.0625f, 2 }, { 0.25f, 1 }, { 0.25f, 0 } };
        run(texture, steps, "speed -1");
    }
    {
        AnimatedTexture texture(nullptr);
        uniform(texture, 3, 0.25f);
        CHECK(texture.setCurrentFrame(1));
        CHECK(texture.setSpeedScale(0.0f));
        texture.update(0.0f);
        CHECK(texture.getCurrentFrame() == 1);
        texture.update(0.0625f);
        CHECK(texture.getCurrentFrame() == 1);
    }
}

void testPause()
{
    AnimatedTexture texture(nullptr);
    uniform(texture, 4, 0.25f);
    texture.setPause(true);
    CHECK(texture.getPause());
    const Step paused[] = { { 1.0f, 0 }, { 0.0f, 0 } };
    run(texture, paused, "paused");
    texture.setPause(false);
    const Step resumed[] = { { 0.0f, 3 }, { 0.0f, 3 }, { 0.0625f, 0 } };
    run(texture, resumed, "resumed");
}

void testOneShot()
{
    {
        AnimatedTexture texture(nullptr);
        uniform(texture, 3, 0.25f);
        texture.setOneShot(true);
        CHECK(texture.getOneShot());
        const Step steps[] = { { 0.3125f, 1 }, { 0.3125f, 2 }, { 0.3125f, 2 }, { 1.0f, 2 },
            { 0.0f, 2 }, { 5.0f, 2 } };
        run(texture, steps, "one shot");
    }
    {
        AnimatedTexture texture(nullptr);
        uniform(texture, 3, 0.25f);
        texture.setOneShot(true);
        CHECK(texture.setSpeedScale(-1.0f));
        const Step steps[] = { { 0.3125f, 0 }, { 0.5f, 0 } };
        run(texture, steps, "one shot backwards");
    }
}

void testWrap()
{
    AnimatedTexture texture(nullptr);
    uniform(texture, 3, 0.25f);
    const Step steps[] = { { 0.3125f, 1 }, { 0.25f, 2 }, { 0.25f, 0 }, { 0.25f, 1 } };
    run(texture, steps, "wrap");
}

void testZeroDuration()
{
    {
        AnimatedTexture texture(nullptr);
        const float values[3] = { 0.0f, 0.5f, 0.0f };
        durations(texture, 3, values);
        const Step steps[] = { { 0.0f, 0 }, { 0.0625f, 1 }, { 0.5f, 1 } };
        run(texture, steps, "zero duration");
    }
    {
        AnimatedTexture texture(nullptr);
        uniform(texture, 5, 0.0f);
        const Step steps[] = { { 0.0f, 0 }, { 0.001f, 0 }, { 0.001f, 0 } };
        run(texture, steps, "all zero from 0");
        CHECK(texture.setCurrentFrame(2));
        const Step more[] = { { 0.001f, 2 } };
        run(texture, more, "all zero from 2");
    }
    {
        AnimatedTexture texture(nullptr);
        uniform(texture, 3, 0.0f);
        CHECK(texture.setCurrentFrame(1));
        texture.setOneShot(true);
        const Step steps[] = { { 0.001f, 2 } };
        run(texture, steps, "all zero, one shot");
    }
}

void testLongStep()
{
    AnimatedTexture texture(nullptr);
    uniform(texture, 4, 0.25f);
    const Step steps[] = { { 2.5f, 0 }, { 0.0f, 0 }, { 0.0f, 1 }, { 0.0f, 1 } };
    run(texture, steps, "long step");
}

void testSetCurrentFrame()
{
    AnimatedTexture texture(nullptr);
    uniform(texture, 4, 0.25f);
    texture.update(0.1875f);
    CHECK(texture.getCurrentFrame() == 0);
    CHECK(texture.setCurrentFrame(1));
    texture.update(0.1875f);
    CHECK(texture.getCurrentFrame() == 1);
    texture.update(0.1875f);
    CHECK(texture.getCurrentFrame() == 2);
    CHECK(!texture.setCurrentFrame(4));
    CHECK(texture.getCurrentFrame() == 2);
}

} // namespace

int main()
{
    testDefaults();
    testRanges();
    testUniform();
    testNonUniform();
    testSpeedScale();
    testPause();
    testOneShot();
    testWrap();
    testZeroDuration();
    testLongStep();
    testSetCurrentFrame();
    if (failures == 0) printf("animated texture test: all passed\n");
    return failures == 0 ? 0 : 1;
}
