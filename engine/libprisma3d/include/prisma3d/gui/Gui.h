#pragma once

#include "prisma3d/Input.h"
#include "prisma3d/gui/GuiBackend.h"

#include <igui/Events.hpp>
#include <igui/Gui.hpp>

namespace prisma3d
{

class Device;

class Gui
{
public:
    Gui(Device& device, TextureStore& textures);
    ~Gui();
    Gui(const Gui&) = delete;
    Gui& operator=(const Gui&) = delete;

    static Gui* singleton();

    bool init();
    ig::Context& context();
    GuiBackend& backend();

    static bool translateEvent(const InputEvent& event, ig::Event& out);
    void handleEvent(const InputEvent& event);

    void beginFrame(float deltaSeconds);
    void endFrame(bool render);
    bool frameOpen() const;

    void setHudRect(const ig::Rect& rect);
    void clearHudRect();
    ig::Rect hudRect() const;
    void hudText(ig::StringView text, float x, float y, const ig::Color& color, float size);
    void hudRectFilled(const ig::Rect& rect, const ig::Color& color);
    void hudRectOutline(const ig::Rect& rect, const ig::Color& color, float thickness);
    void hudLine(float x0, float y0, float x1, float y1, const ig::Color& color, float thickness);
    void hudCircleFilled(float x, float y, float radius, const ig::Color& color);
    void hudImage(TextureId texture, const ig::Rect& rect, float degrees, const ig::Vec2& uvMin,
            const ig::Vec2& uvMax, const ig::Color& color);
    ig::Vec2 hudTextSize(ig::StringView text, float size) const;

    void setEnabled(bool enabled);
    bool enabled() const;

    bool wantsPointer() const;
    bool wantsKeyboard() const;
    bool wantsTextInput() const;

private:
    static Gui* sSingleton;

    Device& mDevice;
    GuiBackend mBackend;
    ig::Context* mContext;
    bool mFrameOpen;
    bool mFocused;
    ig::Rect mHudRect;
    bool mHudRectSet;
};

} // namespace prisma3d
