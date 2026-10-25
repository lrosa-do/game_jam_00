#include "prisma3d/render/AnimatedTexture.h"

#include "prisma3d/render/RenderingServer.h"

#include "platform.h"

#include <math.h>

namespace prisma3d
{

AnimatedTexture::AnimatedTexture(TextureStore* textures)
        : mTextures(textures),
          mFrameCount(1),
          mCurrentFrame(0),
          mPause(false),
          mOneShot(false),
          mSpeedScale(1.0f),
          mTime(0.0f),
          mPreviousTicks(0),
          mAutoUpdate(true),
          mConnected(false)
{
    if (mTextures) mProxy = mTextures->proxyCreate(mTextures->placeholder());
    connect();
}

AnimatedTexture::~AnimatedTexture()
{
    disconnect();
    if (mTextures && TextureStore::singleton() == mTextures) mTextures->destroy(mProxy);
}

void AnimatedTexture::connect()
{
    RenderingServer* server = RenderingServer::singleton();
    if (!server || !mAutoUpdate || mConnected) return;
    server->framePreDrawConnect(this, [this]() { updateFromClock(); });
    mConnected = true;
}

void AnimatedTexture::disconnect()
{
    if (!mConnected) return;
    RenderingServer* server = RenderingServer::singleton();
    if (server) server->framePreDrawDisconnect(this);
    mConnected = false;
}

void AnimatedTexture::setAutoUpdate(bool enabled)
{
    if (mAutoUpdate == enabled) return;
    mAutoUpdate = enabled;
    if (enabled)
    {
        mPreviousTicks = 0;
        connect();
    }
    else
        disconnect();
}

bool AnimatedTexture::getAutoUpdate() const
{
    return mAutoUpdate;
}

TextureId AnimatedTexture::getTexture() const
{
    return mProxy;
}

void AnimatedTexture::updateFromClock()
{
    float delta;
    if (mPreviousTicks == 0)
    {
        delta = 0.0f;
        mPreviousTicks = time_nanos() / 1000ull;
    }
    else
    {
        const std::uint64_t ticks = time_nanos() / 1000ull;
        delta = static_cast<float>(static_cast<double>(ticks - mPreviousTicks) / 1000000.0);
        mPreviousTicks = ticks;
    }
    update(delta);
}

void AnimatedTexture::advance(float delta)
{
    mTime += delta;

    const float speed = mSpeedScale == 0.0f ? 0.0f : fabsf(static_cast<float>(1.0 / mSpeedScale));

    int iterationsLeft = mFrameCount;
    while (iterationsLeft && !mPause)
    {
        const float frameLimit = mFrames[mCurrentFrame].duration * speed;

        if (mTime > frameLimit)
        {
            if (mSpeedScale > 0.0f) ++mCurrentFrame;
            else
                --mCurrentFrame;
            if (mCurrentFrame >= mFrameCount)
            {
                if (mOneShot) mCurrentFrame = mFrameCount - 1;
                else
                    mCurrentFrame = 0;
            }
            else if (mCurrentFrame < 0)
            {
                if (mOneShot) mCurrentFrame = 0;
                else
                    mCurrentFrame = mFrameCount - 1;
            }
            mTime -= frameLimit;
        }
        else
            break;
        --iterationsLeft;
    }
}

void AnimatedTexture::update(float delta)
{
    advance(delta);
    if (!mTextures) return;
    const TextureId texture = mFrames[mCurrentFrame].texture;
    if (mTextures->valid(texture)) mTextures->proxyUpdate(mProxy, texture);
}

bool AnimatedTexture::setFrames(int frames)
{
    if (frames < 1 || frames > kMaxFrames)
    {
        log_error("animated texture: the frame count must be from 1 to %d", kMaxFrames);
        return false;
    }
    mFrameCount = frames;
    return true;
}

int AnimatedTexture::getFrames() const
{
    return mFrameCount;
}

bool AnimatedTexture::setCurrentFrame(int frame)
{
    if (frame < 0 || frame >= mFrameCount)
    {
        log_error("animated texture: frame %d is outside the %d frames", frame, mFrameCount);
        return false;
    }
    mCurrentFrame = frame;
    mTime = 0.0f;
    return true;
}

int AnimatedTexture::getCurrentFrame() const
{
    return mCurrentFrame;
}

void AnimatedTexture::setPause(bool pause)
{
    mPause = pause;
}

bool AnimatedTexture::getPause() const
{
    return mPause;
}

void AnimatedTexture::setOneShot(bool oneShot)
{
    mOneShot = oneShot;
}

bool AnimatedTexture::getOneShot() const
{
    return mOneShot;
}

bool AnimatedTexture::setFrameTexture(int frame, TextureId texture)
{
    if (frame < 0 || frame >= kMaxFrames || (texture.valid() && texture == mProxy)) return false;
    mFrames[frame].texture = texture;
    return true;
}

TextureId AnimatedTexture::getFrameTexture(int frame) const
{
    if (frame < 0 || frame >= kMaxFrames) return TextureId();
    return mFrames[frame].texture;
}

bool AnimatedTexture::setFrameDuration(int frame, float duration)
{
    if (frame < 0 || frame >= kMaxFrames) return false;
    mFrames[frame].duration = duration;
    return true;
}

float AnimatedTexture::getFrameDuration(int frame) const
{
    if (frame < 0 || frame >= kMaxFrames) return 0.0f;
    return mFrames[frame].duration;
}

bool AnimatedTexture::setSpeedScale(float scale)
{
    if (scale < -1000.0f || scale >= 1000.0f) return false;
    mSpeedScale = scale;
    return true;
}

float AnimatedTexture::getSpeedScale() const
{
    return mSpeedScale;
}

int AnimatedTexture::getWidth() const
{
    const TextureInfo* info = mTextures ? mTextures->get(mFrames[mCurrentFrame].texture) : nullptr;
    return info ? static_cast<int>(info->width) : 1;
}

int AnimatedTexture::getHeight() const
{
    const TextureInfo* info = mTextures ? mTextures->get(mFrames[mCurrentFrame].texture) : nullptr;
    return info ? static_cast<int>(info->height) : 1;
}

} // namespace prisma3d
