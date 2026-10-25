#pragma once

#include "prisma3d/Device.h"
#include "prisma3d/Input.h"
#include "prisma3d/render/TextureStore.h"

#include <igui/DrawData.hpp>
#include <igui/widgets/IconAtlas.hpp>
#include <igui/widgets/Retained.hpp>
#include <igui/widgets/WidgetApp.hpp>

#include <ct/string.hpp>
#include <ct/vector.hpp>

#include <cstdint>

namespace prisma3d
{

class Gui;

class WidgetHost
{
public:
    WidgetHost(Device& device, Gui& gui, TextureStore& textures);
    ~WidgetHost();
    WidgetHost(const WidgetHost&) = delete;
    WidgetHost& operator=(const WidgetHost&) = delete;

    static WidgetHost* singleton();

    bool init();
    void shutdown();
    bool ready() const;

    ig::retained::WidgetApp& widgets();
    const ig::retained::Font* font() const;

    static int translateKey(Key key);
    static CursorShape translateCursor(int wanted);
    static ig::retained::TextureHandle textureHandle(TextureId texture);
    static TextureId textureId(ig::retained::TextureHandle handle);
    void handleEvent(const InputEvent& event);
    void dropFiles(const ct::Vector<ct::String>& paths, int x, int y);
    void frame(float deltaSeconds, bool render);
    static bool textFocused();
    static bool modalOpen();

    std::uint32_t drawCalls() const;
    std::uint32_t textureCount() const;

private:
    ig::retained::TextureHandle createTexture(int width, int height, const unsigned char* rgba);
    bool updateTexture(ig::retained::TextureHandle texture, const unsigned char* rgba, int width,
            int height);
    void destroyTexture(ig::retained::TextureHandle texture);
    bool draw(const ig::retained::DrawData& data);

    static WidgetHost* sSingleton;

    Device& mDevice;
    Gui& mGui;
    TextureStore& mTextures;
    ig::retained::Context* mContext;
    ig::retained::FontAtlas mAtlas;
    ig::retained::IconAtlas mIconAtlas;
    const ig::retained::Font* mFont;
    ct::Vector<TextureId> mOwned;
    ct::Vector<ig::DrawVertex> mVertices;
    ct::Vector<ig::DrawIndex> mIndices;
    ct::Vector<ig::DrawCommand> mCommands;
    std::uint32_t mDrawCalls;
};

} // namespace prisma3d
