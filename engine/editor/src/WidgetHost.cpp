#include "prisma3d/editor/WidgetHost.h"

#include "prisma3d/Device.h"
#include "prisma3d/editor/EditorIcons.h"
#include "prisma3d/editor/NumberField.h"
#include "prisma3d/gui/Gui.h"
#include "prisma3d/gui/GuiBackend.h"

#include <igui/widgets/AppWidgets.hpp>
#include <igui/widgets/BasicWidgets.hpp>
#include <igui/widgets/CodeEditor.hpp>
#include <igui/widgets/ConsoleWidget.hpp>
#include <igui/widgets/DataWidgets.hpp>
#include <igui/widgets/InputWidgets.hpp>
#include <igui/widgets/TextInputWidgets.hpp>
#include <igui/widgets/TreePropertyColorWidgets.hpp>
#include <igui/widgets/ViewWidgets.hpp>

#include <math.h>
#include <string.h>

namespace prisma3d
{

namespace rt = ig::retained;

namespace
{

static_assert(sizeof(rt::DrawVertex) == sizeof(ig::DrawVertex),
        "the two vertex layouts must match");

void setClipboard(const char* text)
{
    if (Device* device = Device::singleton()) device->clipboardSet(text);
}

rt::String getClipboard()
{
    Device* device = Device::singleton();
    if (!device || !device->clipboardHas()) return rt::String();
    const char* text = device->clipboardGet();
    return text ? rt::String(text) : rt::String();
}

rt::TextureHandle toHandle(TextureId texture)
{
    rt::TextureHandle handle;
    handle.value = static_cast<uintptr_t>(GuiBackend::guiTexture(texture).value);
    return handle;
}

TextureId toTexture(rt::TextureHandle handle)
{
    return GuiBackend::engineTexture(ig::TextureId(static_cast<std::uint64_t>(handle.value)));
}

} // namespace

WidgetHost* WidgetHost::sSingleton = nullptr;

WidgetHost::WidgetHost(Device& device, Gui& gui, TextureStore& textures)
        : mDevice(device),
          mGui(gui),
          mTextures(textures),
          mContext(nullptr),
          mFont(nullptr),
          mDrawCalls(0)
{
}

WidgetHost::~WidgetHost()
{
    shutdown();
}

WidgetHost* WidgetHost::singleton()
{
    return sSingleton;
}

bool WidgetHost::init()
{
    if (mContext) return true;
    if (sSingleton) return false;

    mContext = rt::CreateContext();
    rt::SetCurrentContext(mContext);
    rt::IO& io = rt::GetIO();
    io.setClipboardText = &setClipboard;
    io.getClipboardText = &getClipboard;

    if (mAtlas.buildDefault())
    {
        mAtlas.setTexture(createTexture(mAtlas.width(), mAtlas.height(), mAtlas.pixels()));
        mFont = &mAtlas.defaultFont();
        rt::SetWhitePixel(mAtlas.texture(), mAtlas.whitePixelUV());
    }
    if (!mFont || !mAtlas.texture())
    {
        shutdown();
        return false;
    }

    rt::WidgetApp& app = rt::WidgetApp::instance();
    if (!app.init())
    {
        shutdown();
        return false;
    }
    app.setTextureUpload([this](const unsigned char* rgba, int width, int height)
            { return createTexture(width, height, rgba); });
    app.setTextureUpdate(
            [this](rt::TextureHandle texture, const unsigned char* rgba, int width, int height)
            { return updateTexture(texture, rgba, width, height); });
    app.setTextureDestroy([this](rt::TextureHandle texture) { destroyTexture(texture); });

    sSingleton = this;
    return true;
}

void WidgetHost::shutdown()
{
    if (!mContext) return;
    rt::SetCurrentContext(mContext);
    EditorIcons::unload();
    if (rt::WidgetApp* app = rt::WidgetApp::existing())
    {
        app->shutdown();
        app->setTextureUpload(rt::WidgetApp::UploadTextureFn());
        app->setTextureUpdate(rt::WidgetApp::UpdateTextureFn());
        app->setTextureDestroy(rt::WidgetApp::DestroyTextureFn());
    }
    for (std::size_t i = 0; i < mOwned.size(); ++i) mTextures.destroy(mOwned[i]);
    mOwned.clear();
    mFont = nullptr;
    rt::DestroyContext(mContext);
    mContext = nullptr;
    if (sSingleton == this) sSingleton = nullptr;
}

bool WidgetHost::ready() const
{
    return mContext != nullptr;
}

rt::WidgetApp& WidgetHost::widgets()
{
    return rt::WidgetApp::instance();
}

const rt::Font* WidgetHost::font() const
{
    return mFont;
}

int WidgetHost::translateKey(Key key)
{
    const int value = static_cast<int>(key);
    if (value >= static_cast<int>(Key::A) && value <= static_cast<int>(Key::Z))
        return value - static_cast<int>(Key::A) + 'a';
    if (value >= static_cast<int>(Key::Space) && value <= 96) return value;
    if (value >= static_cast<int>(Key::F1) && value <= static_cast<int>(Key::F12))
        return rt::Key::F1 + (value - static_cast<int>(Key::F1));
    switch (key)
    {
        case Key::Escape:
            return rt::Key::Escape;
        case Key::Enter:
            return rt::Key::Return;
        case Key::Tab:
            return rt::Key::Tab;
        case Key::Backspace:
            return rt::Key::Backspace;
        case Key::Delete:
            return rt::Key::Delete;
        case Key::Insert:
            return 1073741897;
        case Key::Right:
            return rt::Key::Right;
        case Key::Left:
            return rt::Key::Left;
        case Key::Down:
            return rt::Key::Down;
        case Key::Up:
            return rt::Key::Up;
        case Key::PageUp:
            return rt::Key::PageUp;
        case Key::PageDown:
            return rt::Key::PageDown;
        case Key::Home:
            return rt::Key::Home;
        case Key::End:
            return rt::Key::End;
        case Key::KpEnter:
            return rt::Key::KPEnter;
        default:
            return 0;
    }
}

CursorShape WidgetHost::translateCursor(int wanted)
{
    switch (wanted)
    {
        case 1:
            return CursorShape::PointingHand;
        case 2:
            return CursorShape::IBeam;
        case 3:
            return CursorShape::Cross;
        case 4:
            return CursorShape::HSize;
        case 5:
            return CursorShape::VSize;
        case 6:
            return CursorShape::Move;
        case 7:
            return CursorShape::Forbidden;
        default:
            return CursorShape::Arrow;
    }
}

rt::TextureHandle WidgetHost::textureHandle(TextureId texture)
{
    return toHandle(texture);
}

TextureId WidgetHost::textureId(rt::TextureHandle handle)
{
    return toTexture(handle);
}

void WidgetHost::handleEvent(const InputEvent& event)
{
    if (!mContext) return;
    rt::SetCurrentContext(mContext);
    rt::IO& io = rt::GetIO();
    switch (event.type)
    {
        case InputEventType::MouseMotion:
            io.mouseX = static_cast<float>(event.mouseMotion.x);
            io.mouseY = static_cast<float>(event.mouseMotion.y);
            break;
        case InputEventType::MouseButton:
        {
            io.mouseX = static_cast<float>(event.mouseButton.x);
            io.mouseY = static_cast<float>(event.mouseButton.y);
            int slot = -1;
            if (event.mouseButton.button == MouseButton::Left) slot = 0;
            else if (event.mouseButton.button == MouseButton::Right)
                slot = 1;
            else if (event.mouseButton.button == MouseButton::Middle)
                slot = 2;
            if (slot >= 0) io.mouseDown[slot] = event.mouseButton.pressed;
            break;
        }
        case InputEventType::MouseWheel:
            io.mouseWheelX += event.mouseWheel.deltaX;
            io.mouseWheelY += event.mouseWheel.deltaY;
            break;
        case InputEventType::Char:
            if (event.character.codepoint >= 32u && event.character.codepoint != 127u)
                io.addInputCharacter(event.character.codepoint);
            break;
        case InputEventType::Key:
        {
            io.keyShift = (event.key.modifiers & kModShift) != 0;
            io.keyCtrl = (event.key.modifiers & kModControl) != 0;
            io.keyAlt = (event.key.modifiers & kModAlt) != 0;
            const int key = translateKey(event.key.key);
            if (key == 0) break;
            const int scancode = key == rt::Key::F1 ? 58 : event.key.scancode;
            io.addKeyEvent(key, scancode, io.keyShift, io.keyCtrl, io.keyAlt, event.key.pressed);
            break;
        }
        default:
            break;
    }
}

void WidgetHost::dropFiles(const ct::Vector<ct::String>& paths, int x, int y)
{
    if (!mContext || paths.empty()) return;
    rt::SetCurrentContext(mContext);
    rt::GetIO().addDropEvent(static_cast<float>(x), static_cast<float>(y), paths);
}

void WidgetHost::frame(float deltaSeconds, bool render)
{
    mDrawCalls = 0;
    if (!mContext) return;
    rt::SetCurrentContext(mContext);
    rt::IO& io = rt::GetIO();

    int width = 0;
    int height = 0;
    mDevice.windowGetSize(&width, &height);
    io.deltaTime = deltaSeconds > 0.1f ? 0.1f : deltaSeconds;
    io.displayWidth = static_cast<float>(width);
    io.displayHeight = static_cast<float>(height);

    rt::NewFrame();
    rt::WidgetApp& app = rt::WidgetApp::instance();
    app.update(io);
    // The atlas stays empty; the tree rows only draw their icon when one is passed.
    app.paint(*rt::GetDrawData(), mFont, &mIconAtlas);
    rt::Render();
    mDevice.cursorSetShape(translateCursor(io.wantedCursor));
    mDevice.setTextInput(textFocused());

    if (render) draw(*rt::GetDrawData());
}

bool WidgetHost::textFocused()
{
    rt::WidgetApp* app = rt::WidgetApp::existing();
    const rt::Widget* focus = app ? app->focused() : nullptr;
    if (!focus) return false;
    if (rt::widget_cast<rt::TextInput>(focus) || rt::widget_cast<rt::TextEdit>(focus) ||
            rt::widget_cast<rt::CodeEditor>(focus) || rt::widget_cast<rt::SpinBox>(focus) ||
            rt::widget_cast<rt::ConsoleWidget>(focus) || rt::widget_cast<rt::SearchBar>(focus) ||
            rt::widget_cast<rt::PropertyGrid>(focus) || rt::widget_cast<rt::DataGrid>(focus) ||
            rt::widget_cast<rt::TreeGrid>(focus))
        return true;
    const NumberField* number = rt::widget_cast<NumberField>(focus);
    return number && number->isEditing();
}

bool WidgetHost::modalOpen()
{
    rt::WidgetApp* app = rt::WidgetApp::existing();
    if (!app) return false;
    if (app->popup() != nullptr) return true;
    const ct::Vector<rt::FloatWindow*>& floats = app->floats();
    for (std::size_t i = 0; i < floats.size(); ++i)
        if (floats[i]->isVisible()) return true;
    return false;
}

std::uint32_t WidgetHost::drawCalls() const
{
    return mDrawCalls;
}

std::uint32_t WidgetHost::textureCount() const
{
    return static_cast<std::uint32_t>(mOwned.size());
}

rt::TextureHandle WidgetHost::createTexture(int width, int height, const unsigned char* rgba)
{
    if (width <= 0 || height <= 0 || !rgba) return rt::TextureHandle();
    const TextureId texture = mTextures.createEmpty(static_cast<std::uint32_t>(width),
            static_cast<std::uint32_t>(height), false, false, "widgets");
    if (!texture.valid()) return rt::TextureHandle();
    if (!mTextures.uploadRows(texture, 0, static_cast<std::uint32_t>(height), rgba) ||
            !mTextures.finish(texture))
    {
        mTextures.destroy(texture);
        return rt::TextureHandle();
    }
    mOwned.push_back(texture);
    return toHandle(texture);
}

bool WidgetHost::updateTexture(rt::TextureHandle texture, const unsigned char* rgba, int width,
        int height)
{
    const TextureId id = toTexture(texture);
    if (!id.valid() || !rgba || width <= 0 || height <= 0) return false;
    for (std::size_t i = 0; i < mOwned.size(); ++i)
    {
        if (mOwned[i] != id) continue;
        return mTextures.uploadRows(id, 0, static_cast<std::uint32_t>(height), rgba);
    }
    return false;
}

void WidgetHost::destroyTexture(rt::TextureHandle texture)
{
    const TextureId id = toTexture(texture);
    for (std::size_t i = 0; i < mOwned.size(); ++i)
    {
        if (mOwned[i] != id) continue;
        mTextures.destroy(id);
        mOwned[i] = mOwned.back();
        mOwned.pop_back();
        return;
    }
}

bool WidgetHost::draw(const rt::DrawData& data)
{
    mVertices.clear();
    mIndices.clear();
    mCommands.clear();
    if (data.displayWidth <= 0.0f || data.displayHeight <= 0.0f) return true;

    for (std::size_t p = 0; p < data.passes.size(); ++p)
    {
        const rt::DrawPass& pass = data.passes[p];
        if (!pass.list || pass.list->commands().empty()) continue;
        const ct::Vector<rt::DrawVertex>& vertices = pass.list->vertices();
        const ct::Vector<std::uint32_t>& indices = pass.list->indices();

        const float cosine = cosf(pass.camera.angle) * pass.camera.scale;
        const float sine = sinf(pass.camera.angle) * pass.camera.scale;
        const float pivotX = pass.camera.pivotX * data.displayWidth;
        const float pivotY = pass.camera.pivotY * data.displayHeight;
        const float offsetX = pass.camera.x - pivotX;
        const float offsetY = pass.camera.y - pivotY;
        const float translateX = cosine * offsetX - sine * offsetY + pivotX;
        const float translateY = sine * offsetX + cosine * offsetY + pivotY;

        const std::size_t vertexBase = mVertices.size();
        const std::size_t indexBase = mIndices.size();
        mVertices.resize(vertexBase + vertices.size());
        for (std::size_t v = 0; v < vertices.size(); ++v)
        {
            const rt::DrawVertex& source = vertices[v];
            ig::DrawVertex& target = mVertices[vertexBase + v];
            target.position.x = cosine * source.x - sine * source.y + translateX;
            target.position.y = sine * source.x + cosine * source.y + translateY;
            target.uv.x = source.u;
            target.uv.y = source.v;
            target.color = source.color;
        }
        mIndices.resize(indexBase + indices.size());
        if (!indices.empty())
            memcpy(mIndices.data() + indexBase, indices.data(),
                    indices.size() * sizeof(std::uint32_t));

        const ct::Vector<rt::DrawCmd>& commands = pass.list->commands();
        for (std::size_t c = 0; c < commands.size(); ++c)
        {
            const rt::DrawCmd& command = commands[c];
            if (command.indexCount == 0 ||
                    command.indexOffset + command.indexCount > indices.size())
                continue;
            ig::GeometryCommand geometry;
            geometry.clip = ig::Rect(0.0f, 0.0f, data.displayWidth, data.displayHeight);
            if (pass.camera.angle == 0.0f)
            {
                float x0 = cosine * command.clip.x + translateX;
                float y0 = cosine * command.clip.y + translateY;
                float x1 = cosine * (command.clip.x + command.clip.w) + translateX;
                float y1 = cosine * (command.clip.y + command.clip.h) + translateY;
                if (x0 > x1)
                {
                    const float swap = x0;
                    x0 = x1;
                    x1 = swap;
                }
                if (y0 > y1)
                {
                    const float swap = y0;
                    y0 = y1;
                    y1 = swap;
                }
                x0 = floorf(x0) < 0.0f ? 0.0f : floorf(x0);
                y0 = floorf(y0) < 0.0f ? 0.0f : floorf(y0);
                x1 = ceilf(x1) > data.displayWidth ? data.displayWidth : ceilf(x1);
                y1 = ceilf(y1) > data.displayHeight ? data.displayHeight : ceilf(y1);
                if (x1 <= x0 || y1 <= y0) continue;
                geometry.clip = ig::Rect(x0, y0, x1 - x0, y1 - y0);
            }
            geometry.texture = ig::TextureId(static_cast<std::uint64_t>(command.texture.value));
            geometry.firstIndex = static_cast<std::uint32_t>(indexBase) + command.indexOffset;
            geometry.indexCount = command.indexCount;
            geometry.vertexOffset = static_cast<std::int32_t>(vertexBase);
            mCommands.push_back(ig::DrawCommand(geometry));
        }
    }

    if (mCommands.empty()) return true;
    ig::DrawData out;
    out.displaySize = ig::Vec2(data.displayWidth, data.displayHeight);
    out.dpiScale = 1.0f;
    out.vertices = ig::Span<const ig::DrawVertex>(mVertices.data(), mVertices.size());
    out.indices = ig::Span<const ig::DrawIndex>(mIndices.data(), mIndices.size());
    out.textBytes = ig::Span<const char>();
    out.commands = ig::Span<const ig::DrawCommand>(mCommands.data(), mCommands.size());
    const bool drawn = mGui.backend().render(out);
    mDrawCalls = static_cast<std::uint32_t>(mCommands.size());
    return drawn;
}

} // namespace prisma3d
