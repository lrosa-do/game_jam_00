#include "prisma3d/editor/GamePanel.h"

#include "prisma3d/Device.h"
#include "prisma3d/editor/EditorDocument.h"
#include "prisma3d/editor/WidgetHost.h"
#include "prisma3d/gui/Gui.h"
#include "prisma3d/main/Main.h"
#include "prisma3d/main/Project.h"
#include "prisma3d/scene/Camera3D.h"

#include <igui/widgets/LayoutWidgets.hpp>
#include <igui/widgets/Theme.hpp>
#include <igui/widgets/ViewWidgets.hpp>
#include <igui/widgets/WidgetApp.hpp>

#include <stdio.h>

namespace prisma3d
{

namespace rt = ig::retained;

namespace
{

const Math::Vec4 kClearColor(0.3f, 0.3f, 0.3f, 1.0f);
const char* const kClearSetting = "rendering/environment/defaults/default_clear_color";
const float kLabelInset = 8.0f;

Camera3D* firstCamera(Node* node, Camera3D*& current)
{
    Camera3D* first = nullptr;
    if (Camera3D* camera = node->castTo<Camera3D>())
    {
        first = camera;
        if (!current && camera->isCurrent()) current = camera;
    }
    for (std::size_t i = 0; i < node->getChildCount(); ++i)
    {
        Camera3D* found = firstCamera(node->getChild(i), current);
        if (!first) first = found;
    }
    return first;
}

} // namespace

class GameImage : public rt::Widget
{
public:
    IG_WIDGET_CLASS(GameImage, rt::Widget);

    void setTexture(rt::TextureHandle texture)
    {
        if (mTexture.value == texture.value) return;
        mTexture = texture;
        markDirty();
    }

    rt::TextureHandle texture() const
    {
        return mTexture;
    }

    void setMessage(const ct::String& message)
    {
        if (mMessage == message) return;
        mMessage = message;
        markDirty();
    }

    const ct::String& message() const
    {
        return mMessage;
    }

    void setCaption(const ct::String& caption)
    {
        if (mCaption == caption) return;
        mCaption = caption;
        markDirty();
    }

    Vec2f sizeHint() const override
    {
        return { 64.0f, 64.0f };
    }

    void paint(rt::PaintContext& ctx) override
    {
        if (!visible_) return;
        const rt::Rect abs = absoluteRect();
        if (ctx.isClipped(abs)) return;

        ctx.pushClip(abs);
        const rt::Theme& theme = rt::Theme::instance();
        if (overrideFont_) ctx.pushFont(overrideFont_);
        ctx.font.SetFontSize(theme.fontSize);
        ctx.font.SetBatch(&ctx.text);
        const float ascender = ctx.font.GetAscender();
        if (mMessage.empty())
        {
            if (mTexture) ctx.drawImage(mTexture, abs, rt::Rect{ 0.0f, 1.0f, 1.0f, -1.0f });
            if (!mCaption.empty())
            {
                ctx.font.SetColor(rt::Color(240, 240, 240, 255));
                ctx.font.Print(mCaption.c_str(), abs.x + kLabelInset,
                        abs.y + kLabelInset + ascender);
            }
        }
        else
        {
            ctx.fill.SetColor(theme.panelColor.r, theme.panelColor.g, theme.panelColor.b,
                    theme.panelColor.a);
            ctx.fill.Rectangle(static_cast<int>(abs.x), static_cast<int>(abs.y),
                    static_cast<int>(abs.w), static_cast<int>(abs.h), true);
            const float width = ctx.font.GetTextWidth(mMessage.c_str());
            ctx.font.SetColor(rt::Color(170, 170, 175, 255));
            ctx.font.Print(mMessage.c_str(), abs.x + (abs.w - width) * 0.5f,
                    abs.y + (abs.h - theme.fontSize) * 0.5f + ascender);
        }
        if (overrideFont_) ctx.popFont();
        ctx.popClip();
    }

private:
    rt::TextureHandle mTexture;
    ct::String mMessage;
    ct::String mCaption;
};

GamePanel::GamePanel(RenderingServer& server, EditorDocument& document)
        : mServer(server),
          mDocument(document),
          mLink(ct::make_rc<Link>(Link{ this })),
          mViewport(),
          mScenario(),
          mUpdateMode(ViewportUpdateMode::Disabled),
          mColumn(nullptr),
          mView(nullptr),
          mCamera(nullptr),
          mTexture(),
          mWidth(0),
          mHeight(0),
          mSerial(0),
          mDragButtons(0),
          mOutsideButtons(0),
          mMouseX(0),
          mMouseY(0),
          mPreDraw(false),
          mFocused(false)
{
    mViewport = mServer.viewportCreate();
    mServer.viewportSetClearColor(mViewport, kClearColor);
    mServer.viewportSetSrgbOutput(mViewport, true);
    mServer.viewportSetUpdateMode(mViewport, ViewportUpdateMode::Disabled);
}

GamePanel::~GamePanel()
{
    mLink->owner = nullptr;
    disconnectPreDraw();
    mServer.viewportFree(mViewport);
    if (Gui* gui = Main::gui()) gui->clearHudRect();
}

bool GamePanel::attach(rt::Widget* parent)
{
    if (mView || !parent) return false;
    mColumn = parent->createChild<rt::BoxLayout>(rt::LayoutDir::Vertical);
    mColumn->setPadding(0.0f);
    mColumn->setSpacing(0.0f);
    mView = mColumn->createChild<GameImage>();
    mView->setStretch(1.0f);
    mView->setMessage(kNoCameraMessage);
    return true;
}

bool GamePanel::attached() const
{
    return mView != nullptr;
}

void GamePanel::setScenario(ScenarioId scenario)
{
    mScenario = scenario;
    mServer.viewportSetScenario(mViewport, scenario);
}

ScenarioId GamePanel::scenario() const
{
    return mScenario;
}

void GamePanel::setClearColor(const Math::Vec4& color)
{
    mServer.viewportSetClearColor(mViewport, color);
}

void GamePanel::connectPreDraw()
{
    if (mPreDraw) return;
    mPreDraw = true;
    const ct::Rc<Link> link = mLink;
    mServer.framePreDrawConnect(this,
            [link]()
            {
                if (link->owner) link->owner->update(0.0f);
            });
}

void GamePanel::disconnectPreDraw()
{
    if (!mPreDraw) return;
    mPreDraw = false;
    mServer.framePreDrawDisconnect(this);
}

bool GamePanel::visibleInTree() const
{
    for (const rt::Widget* widget = mView; widget; widget = widget->parent())
        if (!widget->isVisible()) return false;
    return true;
}

void GamePanel::syncSize()
{
    const rt::Rect rect = mView->absoluteRect();
    const std::uint32_t width = static_cast<std::uint32_t>(rect.w + 0.5f);
    const std::uint32_t height = static_cast<std::uint32_t>(rect.h + 0.5f);
    if (width == 0 || height == 0 || (width == mWidth && height == mHeight)) return;
    mWidth = width;
    mHeight = height;
    mServer.viewportSetSize(mViewport, width, height);
}

Camera3D* GamePanel::findCamera(Node* scene)
{
    if (!scene) return nullptr;
    Camera3D* current = nullptr;
    Camera3D* first = firstCamera(scene, current);
    return current ? current : first;
}

void GamePanel::syncCamera()
{
    Camera3D* camera = findCamera(mDocument.scene());
    if (camera != mCamera || mDocument.sceneSerial() != mSerial)
    {
        mCamera = camera;
        mSerial = mDocument.sceneSerial();
        const Project& project = mDocument.project();
        mServer.viewportSetClearColor(mViewport,
                project.isOpen() ? project.get(kClearSetting).asColor() : kClearColor);
        mServer.viewportAttachCamera(mViewport, camera ? camera->getCameraId() : CameraId());
    }
    if (!camera)
    {
        mCaption = ct::String();
        mView->setCaption(mCaption);
        mView->setMessage(kNoCameraMessage);
        return;
    }
    char text[256];
    snprintf(text, sizeof text, "%s  |  %u x %u", camera->getName().c_str(),
            static_cast<unsigned>(mWidth), static_cast<unsigned>(mHeight));
    mCaption = ct::String(text);
    mView->setMessage(ct::String());
    mView->setCaption(mCaption);
}

void GamePanel::update(float delta)
{
    (void) delta;
    if (!mView || !mViewport.valid()) return;

    const rt::Rect rect = mView->absoluteRect();
    const bool visible = visibleInTree() && rect.w >= 1.0f && rect.h >= 1.0f;
    if (!visible)
    {
        mFocused = false;
        mDragButtons = 0;
        mOutsideButtons = 0;
        if (Gui* gui = Main::gui()) gui->clearHudRect();
        if (mUpdateMode != ViewportUpdateMode::Disabled)
        {
            mUpdateMode = ViewportUpdateMode::Disabled;
            mServer.viewportSetUpdateMode(mViewport, mUpdateMode);
        }
        return;
    }

    syncSize();
    if (Gui* gui = Main::gui()) gui->setHudRect(ig::Rect(rect.x, rect.y, rect.w, rect.h));
    if (mWidth == 0 || mHeight == 0) return;
    syncCamera();

    const ViewportUpdateMode wanted =
            mCamera ? ViewportUpdateMode::Always : ViewportUpdateMode::Disabled;
    if (mUpdateMode != wanted)
    {
        mUpdateMode = wanted;
        mServer.viewportSetUpdateMode(mViewport, mUpdateMode);
    }

    mTexture = mServer.viewportGetTexture(mViewport);
    mView->setTexture(mCamera ? WidgetHost::textureHandle(mTexture) : rt::TextureHandle());
    mView->markDirty();
}

bool GamePanel::hovered(int x, int y) const
{
    if (!mView || !visibleInTree()) return false;
    const float px = static_cast<float>(x);
    const float py = static_cast<float>(y);
    if (!mView->absoluteRect().contains(px, py)) return false;
    if (const rt::WidgetApp* app = rt::WidgetApp::existing())
    {
        if (app->popup() && app->popup()->popupContains(px, py)) return false;
        const ct::Vector<rt::FloatWindow*>& floats = app->floats();
        for (std::size_t i = 0; i < floats.size(); ++i)
            if (floats[i]->isVisible() && floats[i]->absoluteRect().contains(px, py)) return false;
    }
    return true;
}

bool GamePanel::focused() const
{
    return mFocused;
}

bool GamePanel::captured() const
{
    const Device* device = Main::device();
    return device && device->mouseGetMode() == MouseMode::Captured;
}

bool GamePanel::wantsPolling(int x, int y) const
{
    if (!mView || !visibleInTree()) return false;
    return captured() || (mOutsideButtons == 0 && (mDragButtons != 0 || hovered(x, y)));
}

void GamePanel::releaseFocus()
{
    mFocused = false;
    mDragButtons = 0;
    mOutsideButtons = 0;
}

bool GamePanel::handleEvent(const InputEvent& event)
{
    switch (event.type)
    {
        case InputEventType::MouseMotion:
            mMouseX = event.mouseMotion.x;
            mMouseY = event.mouseMotion.y;
            break;
        case InputEventType::MouseButton:
            mMouseX = event.mouseButton.x;
            mMouseY = event.mouseButton.y;
            if (event.mouseButton.pressed)
            {
                mFocused = hovered(mMouseX, mMouseY);
                if (mFocused) mDragButtons |= mouseButtonToMask(event.mouseButton.button);
                else
                    mOutsideButtons |= mouseButtonToMask(event.mouseButton.button);
            }
            else
            {
                mDragButtons &= ~mouseButtonToMask(event.mouseButton.button);
                mOutsideButtons &= ~mouseButtonToMask(event.mouseButton.button);
            }
            break;
        case InputEventType::MouseWheel:
            mMouseX = event.mouseWheel.x;
            mMouseY = event.mouseWheel.y;
            break;
        default:
            break;
    }
    return false;
}

bool GamePanel::wantsInput(const InputEvent& event) const
{
    if (!mView || !visibleInTree()) return false;
    if (captured()) return true;
    const bool outsideDrag = mOutsideButtons != 0;
    switch (event.type)
    {
        case InputEventType::MouseMotion:
            return !outsideDrag &&
                   (mDragButtons != 0 || hovered(event.mouseMotion.x, event.mouseMotion.y));
        case InputEventType::MouseButton:
            if (!event.mouseButton.pressed) return true;
            return !outsideDrag && hovered(event.mouseButton.x, event.mouseButton.y);
        case InputEventType::MouseWheel:
            return !outsideDrag && hovered(event.mouseWheel.x, event.mouseWheel.y);
        case InputEventType::Key:
            if (!event.key.pressed) return true;
            return !outsideDrag && (mDragButtons != 0 || hovered(mMouseX, mMouseY));
        default:
            return false;
    }
}

bool GamePanel::ownsInput(const InputEvent& event) const
{
    if (!wantsInput(event)) return false;
    if (event.type == InputEventType::MouseButton) return event.mouseButton.pressed;
    if (event.type == InputEventType::Key) return event.key.pressed;
    return true;
}

Camera3D* GamePanel::camera() const
{
    return mCamera;
}

bool GamePanel::showingMessage() const
{
    return mView && !mView->message().empty();
}

const ct::String& GamePanel::message() const
{
    static const ct::String empty;
    return mView ? mView->message() : empty;
}

const ct::String& GamePanel::caption() const
{
    return mCaption;
}

rt::Widget* GamePanel::view() const
{
    return mView;
}

ViewportId GamePanel::viewportId() const
{
    return mViewport;
}

CameraId GamePanel::cameraId() const
{
    return mCamera ? mCamera->getCameraId() : CameraId();
}

TextureId GamePanel::texture() const
{
    return mTexture;
}

rt::TextureHandle GamePanel::shownTexture() const
{
    return mView ? mView->texture() : rt::TextureHandle();
}

std::uint32_t GamePanel::width() const
{
    return mWidth;
}

std::uint32_t GamePanel::height() const
{
    return mHeight;
}

bool GamePanel::rendering() const
{
    return mUpdateMode == ViewportUpdateMode::Always;
}

} // namespace prisma3d
