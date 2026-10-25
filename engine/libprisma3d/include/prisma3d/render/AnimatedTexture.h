#pragma once

#include "prisma3d/render/TextureStore.h"

#include <cstdint>

namespace prisma3d
{

class AnimatedTexture
{
public:
    static constexpr int kMaxFrames = 256;

    explicit AnimatedTexture(TextureStore* textures);
    ~AnimatedTexture();
    AnimatedTexture(const AnimatedTexture&) = delete;
    AnimatedTexture& operator=(const AnimatedTexture&) = delete;

    TextureId getTexture() const;

    bool setFrames(int frames);
    int getFrames() const;

    bool setCurrentFrame(int frame);
    int getCurrentFrame() const;

    void setPause(bool pause);
    bool getPause() const;

    void setOneShot(bool oneShot);
    bool getOneShot() const;

    bool setFrameTexture(int frame, TextureId texture);
    TextureId getFrameTexture(int frame) const;

    bool setFrameDuration(int frame, float duration);
    float getFrameDuration(int frame) const;

    bool setSpeedScale(float scale);
    float getSpeedScale() const;

    int getWidth() const;
    int getHeight() const;

    void update(float delta);

    void setAutoUpdate(bool enabled);
    bool getAutoUpdate() const;

private:
    struct Frame
    {
        TextureId texture;
        float duration = 1.0f;
    };

    void advance(float delta);
    void updateFromClock();
    void connect();
    void disconnect();

    TextureStore* mTextures;
    TextureId mProxy;
    Frame mFrames[kMaxFrames];
    int mFrameCount;
    int mCurrentFrame;
    bool mPause;
    bool mOneShot;
    float mSpeedScale;
    float mTime;
    std::uint64_t mPreviousTicks;
    bool mAutoUpdate;
    bool mConnected;
};

} // namespace prisma3d
