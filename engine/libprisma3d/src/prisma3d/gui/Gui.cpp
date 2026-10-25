#include "prisma3d/gui/Gui.h"

#include "prisma3d/Device.h"
#include "prisma3d/render/RenderProfiler.h"

#include "platform.h"

#include <math.h>

namespace prisma3d
{

namespace
{

bool mapKey(Key key, ig::KeyCode& out)
{
    switch (key)
    {
        case Key::Backspace:
            out = ig::KeyCode::Backspace;
            return true;
        case Key::Enter:
        case Key::KpEnter:
            out = ig::KeyCode::Enter;
            return true;
        case Key::Delete:
            out = ig::KeyCode::Delete;
            return true;
        case Key::Tab:
            out = ig::KeyCode::Tab;
            return true;
        case Key::Left:
            out = ig::KeyCode::Left;
            return true;
        case Key::Right:
            out = ig::KeyCode::Right;
            return true;
        case Key::Up:
            out = ig::KeyCode::Up;
            return true;
        case Key::Down:
            out = ig::KeyCode::Down;
            return true;
        case Key::Home:
            out = ig::KeyCode::Home;
            return true;
        case Key::End:
            out = ig::KeyCode::End;
            return true;
        case Key::PageUp:
            out = ig::KeyCode::PageUp;
            return true;
        case Key::PageDown:
            out = ig::KeyCode::PageDown;
            return true;
        case Key::Escape:
            out = ig::KeyCode::Escape;
            return true;
        case Key::A:
            out = ig::KeyCode::A;
            return true;
        case Key::C:
            out = ig::KeyCode::C;
            return true;
        case Key::D:
            out = ig::KeyCode::D;
            return true;
        case Key::F:
            out = ig::KeyCode::F;
            return true;
        case Key::H:
            out = ig::KeyCode::H;
            return true;
        case Key::S:
            out = ig::KeyCode::S;
            return true;
        case Key::V:
            out = ig::KeyCode::V;
            return true;
        case Key::X:
            out = ig::KeyCode::X;
            return true;
        case Key::Y:
            out = ig::KeyCode::Y;
            return true;
        case Key::Z:
            out = ig::KeyCode::Z;
            return true;
        case Key::N:
            out = ig::KeyCode::N;
            return true;
        case Key::O:
            out = ig::KeyCode::O;
            return true;
        case Key::F4:
            out = ig::KeyCode::F4;
            return true;
        case Key::F5:
            out = ig::KeyCode::F5;
            return true;
        default:
            return false;
    }
}

bool mapButton(MouseButton button, ig::PointerButton& out)
{
    switch (button)
    {
        case MouseButton::Left:
            out = ig::PointerButton::Left;
            return true;
        case MouseButton::Middle:
            out = ig::PointerButton::Middle;
            return true;
        case MouseButton::Right:
            out = ig::PointerButton::Right;
            return true;
        default:
            return false;
    }
}

unsigned encodeUtf8(std::uint32_t codepoint, char* out)
{
    if (codepoint == 0 || codepoint > 0x10ffffu || (codepoint >= 0xd800u && codepoint <= 0xdfffu))
        return 0;
    if (codepoint < 0x80u)
    {
        out[0] = static_cast<char>(codepoint);
        return 1;
    }
    if (codepoint < 0x800u)
    {
        out[0] = static_cast<char>(0xc0u | (codepoint >> 6));
        out[1] = static_cast<char>(0x80u | (codepoint & 0x3fu));
        return 2;
    }
    if (codepoint < 0x10000u)
    {
        out[0] = static_cast<char>(0xe0u | (codepoint >> 12));
        out[1] = static_cast<char>(0x80u | ((codepoint >> 6) & 0x3fu));
        out[2] = static_cast<char>(0x80u | (codepoint & 0x3fu));
        return 3;
    }
    out[0] = static_cast<char>(0xf0u | (codepoint >> 18));
    out[1] = static_cast<char>(0x80u | ((codepoint >> 12) & 0x3fu));
    out[2] = static_cast<char>(0x80u | ((codepoint >> 6) & 0x3fu));
    out[3] = static_cast<char>(0x80u | (codepoint & 0x3fu));
    return 4;
}

} // namespace

Gui* Gui::sSingleton = nullptr;

Gui::Gui(Device& device, TextureStore& textures)
        : mDevice(device),
          mBackend(device.driver(), textures),
          mContext(nullptr),
          mFrameOpen(false),
          mFocused(true),
          mHudRect(),
          mHudRectSet(false)
{
}

Gui::~Gui()
{
    if (mFrameOpen && mContext) mContext->endFrame();
    delete mContext;
    if (sSingleton == this) sSingleton = nullptr;
}

Gui* Gui::singleton()
{
    return sSingleton;
}

bool Gui::init()
{
    if (mContext) return true;
    if (!mBackend.init()) return false;
    mContext = new ig::Context(mBackend, &mBackend.fontAtlas());
    sSingleton = this;
    return true;
}

ig::Context& Gui::context()
{
    return *mContext;
}

GuiBackend& Gui::backend()
{
    return mBackend;
}

bool Gui::translateEvent(const InputEvent& event, ig::Event& out)
{
    switch (event.type)
    {
        case InputEventType::MouseMotion:
            out = ig::Event::pointerMove(static_cast<float>(event.mouseMotion.x),
                    static_cast<float>(event.mouseMotion.y));
            return true;
        case InputEventType::MouseButton:
        {
            ig::PointerButton button;
            if (!mapButton(event.mouseButton.button, button)) return false;
            const float x = static_cast<float>(event.mouseButton.x);
            const float y = static_cast<float>(event.mouseButton.y);
            out = event.mouseButton.pressed ? ig::Event::pointerDown(button, x, y)
                                            : ig::Event::pointerUp(button, x, y);
            return true;
        }
        case InputEventType::MouseWheel:
            out = ig::Event();
            out.type = ig::EventType::PointerWheel;
            out.wheelX = event.mouseWheel.deltaX;
            out.wheelY = event.mouseWheel.deltaY;
            return true;
        case InputEventType::Key:
        {
            if (!event.key.pressed) return false;
            ig::KeyCode code;
            if (!mapKey(event.key.key, code)) return false;
            out = ig::Event::keyDown(code, (event.key.modifiers & kModControl) != 0,
                    (event.key.modifiers & kModShift) != 0);
            return true;
        }
        case InputEventType::Char:
        {
            char bytes[4];
            const unsigned length = encodeUtf8(event.character.codepoint, bytes);
            if (length == 0) return false;
            out = ig::Event::textInput(ig::StringView(bytes, length));
            return true;
        }
        default:
            return false;
    }
}

void Gui::handleEvent(const InputEvent& event)
{
    if (!mContext) return;
    if (event.type == InputEventType::Key && event.key.key == Key::F10)
    {
        if (event.key.pressed && !event.key.echo) mContext->setEnabled(!mContext->enabled());
        return;
    }
    ig::Event translated;
    if (translateEvent(event, translated)) mContext->pushEvent(translated);
}

void Gui::beginFrame(float deltaSeconds)
{
    if (!mContext) return;
    if (mFrameOpen) endFrame(false);

    const bool focused = mDevice.windowIsFocused();
    if (mFocused && !focused) mContext->pushEvent(ig::Event::focusLost());
    mFocused = focused;

    int width = 0;
    int height = 0;
    mDevice.windowGetSize(&width, &height);
    mContext->beginFrame(ig::FrameInfo(static_cast<float>(width), static_cast<float>(height),
            mDevice.windowGetScale(), deltaSeconds));
    mFrameOpen = true;
}

void Gui::endFrame(bool render)
{
    if (!mFrameOpen || !mContext) return;
    ProfileScope layoutScope("GuiLayout");
    const ig::DrawData& data = mContext->endFrame();
    mFrameOpen = false;
    ProfileScope renderScope("GuiRender");
    if (render && mContext->enabled()) mBackend.render(data);
}

void Gui::setEnabled(bool enabled)
{
    if (mContext) mContext->setEnabled(enabled);
}

bool Gui::enabled() const
{
    return mContext && mContext->enabled();
}

bool Gui::frameOpen() const
{
    return mFrameOpen;
}

bool Gui::wantsPointer() const
{
    return mContext && mContext->wantsPointer();
}

bool Gui::wantsKeyboard() const
{
    return mContext && mContext->wantsKeyboard();
}

bool Gui::wantsTextInput() const
{
    return mContext && mContext->wantsTextInput();
}

void Gui::setHudRect(const ig::Rect& rect)
{
    mHudRect = rect;
    mHudRectSet = true;
}

void Gui::clearHudRect()
{
    mHudRectSet = false;
}

ig::Rect Gui::hudRect() const
{
    if (mHudRectSet) return mHudRect;
    int width = 0;
    int height = 0;
    mDevice.windowGetSize(&width, &height);
    return ig::Rect(0.0f, 0.0f, static_cast<float>(width), static_cast<float>(height));
}

void Gui::hudText(ig::StringView text, float x, float y, const ig::Color& color, float size)
{
    if (!mContext || !mFrameOpen) return;
    const ig::Rect clip = hudRect();
    mContext->drawText(mContext->foregroundDrawList(), text, ig::Vec2(clip.x + x, clip.y + y), size,
            color, clip);
}

void Gui::hudRectFilled(const ig::Rect& rect, const ig::Color& color)
{
    if (!mContext || !mFrameOpen) return;
    const ig::Rect clip = hudRect();
    mContext->foregroundDrawList().addRectFilled(
            ig::Rect(clip.x + rect.x, clip.y + rect.y, rect.width, rect.height), color, clip);
}

void Gui::hudRectOutline(const ig::Rect& rect, const ig::Color& color, float thickness)
{
    if (!mContext || !mFrameOpen) return;
    const ig::Rect clip = hudRect();
    mContext->foregroundDrawList().addRect(
            ig::Rect(clip.x + rect.x, clip.y + rect.y, rect.width, rect.height), color, clip,
            thickness);
}

void Gui::hudLine(float x0, float y0, float x1, float y1, const ig::Color& color, float thickness)
{
    if (!mContext || !mFrameOpen) return;
    const ig::Rect clip = hudRect();
    mContext->foregroundDrawList().addLine(ig::Vec2(clip.x + x0, clip.y + y0),
            ig::Vec2(clip.x + x1, clip.y + y1), color, clip, thickness);
}

void Gui::hudCircleFilled(float x, float y, float radius, const ig::Color& color)
{
    if (!mContext || !mFrameOpen) return;
    const ig::Rect clip = hudRect();
    mContext->foregroundDrawList().addCircleFilled(ig::Vec2(clip.x + x, clip.y + y), radius, color,
            clip);
}

void Gui::hudImage(TextureId texture, const ig::Rect& rect, float degrees, const ig::Vec2& uvMin,
        const ig::Vec2& uvMax, const ig::Color& color)
{
    if (!mContext || !mFrameOpen || !texture.valid()) return;
    const ig::Rect clip = hudRect();
    const float radians = degrees * 0.017453292519943295f;
    const float c = cosf(radians);
    const float s = sinf(radians);
    const float halfWidth = rect.width * 0.5f;
    const float halfHeight = rect.height * 0.5f;
    const float centerX = clip.x + rect.x + halfWidth;
    const float centerY = clip.y + rect.y + halfHeight;
    const float offsets[4][2] = { { -halfWidth, -halfHeight }, { halfWidth, -halfHeight },
        { halfWidth, halfHeight }, { -halfWidth, halfHeight } };
    ig::Vec2 corners[4];
    for (int i = 0; i < 4; ++i)
        corners[i] = ig::Vec2(centerX + offsets[i][0] * c - offsets[i][1] * s,
                centerY + offsets[i][0] * s + offsets[i][1] * c);
    const ig::Vec2 uvs[4] = { uvMin, ig::Vec2(uvMax.x, uvMin.y), uvMax,
        ig::Vec2(uvMin.x, uvMax.y) };
    mContext->foregroundDrawList().addImageQuad(GuiBackend::guiTexture(texture), corners, uvs,
            color, clip);
}

ig::Vec2 Gui::hudTextSize(ig::StringView text, float size) const
{
    if (!mContext) return ig::Vec2();
    const ig::TextMetrics metrics = mContext->textMetrics(text, size);
    return ig::Vec2(metrics.width, metrics.height);
}

} // namespace prisma3d
