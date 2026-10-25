#include "prisma3d/editor/EditorAbout.h"

#include "prisma3d/editor/EditorIcons.h"
#include "prisma3d/editor/EditorVersion.h"

#include <igui/widgets/BasicWidgets.hpp>
#include <igui/widgets/LayoutWidgets.hpp>
#include <igui/widgets/Theme.hpp>
#include <igui/widgets/WidgetApp.hpp>

namespace prisma3d
{

namespace rt = ig::retained;

namespace
{

const float kDialogWidth = 460.0f;
const float kDialogHeight = 140.0f;
const float kHeaderHeight = 64.0f;
const float kHeaderIcon = 40.0f;

} // namespace

class AboutHeader : public rt::Widget
{
public:
    IG_WIDGET_CLASS(AboutHeader, rt::Widget);

    Vec2f sizeHint() const override
    {
        return { 0.0f, kHeaderHeight };
    }

    void paint(rt::PaintContext& ctx) override
    {
        if (!visible_) return;
        const rt::Rect abs = absoluteRect();
        if (ctx.isClipped(abs)) return;

        const rt::Theme& theme = rt::Theme::instance();
        ctx.drawIcon(EditorIcons::get("Node3D"), abs.x + 4.0f, abs.y + (abs.h - kHeaderIcon) * 0.5f,
                kHeaderIcon);

        if (overrideFont_) ctx.pushFont(overrideFont_);
        ctx.pushClip(abs);
        ctx.font.SetFontSize(theme.fontSize);
        ctx.font.SetBatch(&ctx.text);
        const float ascender = ctx.font.GetAscender();
        const float left = abs.x + kHeaderIcon + 18.0f;
        const float line = theme.fontSize * 1.5f;
        float y = abs.y + (abs.h - line * 2.0f) * 0.5f + ascender;
        ctx.font.SetColor(theme.textColor);
        ctx.font.Print(EditorAbout::name(), left, y);
        y += line;
        ctx.font.SetColor(theme.textDisabled);
        ct::String second = "Version ";
        second += EditorAbout::version();
        second += " \xC2\xB7 ";
        second += EditorAbout::vendor();
        ctx.font.Print(second.c_str(), left, y);
        ctx.popClip();
        if (overrideFont_) ctx.popFont();
    }
};

class AboutDialog : public rt::FloatWindow
{
public:
    IG_WIDGET_CLASS(AboutDialog, rt::FloatWindow);

    AboutDialog()
            : rt::FloatWindow("About")
    {
        setResizable(false);
        setMinimizable(false);

        rt::BoxLayout* main = setContent<rt::BoxLayout>(rt::LayoutDir::Vertical);
        main->setSpacing(8.0f);
        main->setPadding(16.0f);
        main->createChild<AboutHeader>();

        setFloatSize(kDialogWidth, kDialogHeight);
        const rt::IO& io = rt::GetIO();
        setFloatPos((io.displayWidth - kDialogWidth) * 0.5f,
                (io.displayHeight - kDialogHeight) * 0.5f);
    }
};

const char* EditorAbout::name()
{
    return PRISMA3D_EDITOR_NAME;
}

const char* EditorAbout::version()
{
    return PRISMA3D_EDITOR_VERSION;
}

const char* EditorAbout::vendor()
{
    return PRISMA3D_EDITOR_VENDOR;
}

ct::String EditorAbout::signature()
{
    ct::String text = name();
    text += " ";
    text += version();
    text += " \xC2\xB7 ";
    text += vendor();
    return text;
}

ct::String EditorAbout::windowTitle(const ct::String& document)
{
    ct::String title = document;
    if (!title.empty()) title += " - ";
    title += signature();
    return title;
}

rt::FloatWindow* EditorAbout::dialog()
{
    rt::WidgetApp* app = rt::WidgetApp::existing();
    if (!app) return nullptr;
    const ct::Vector<rt::FloatWindow*>& floats = app->floats();
    for (std::size_t i = floats.size(); i > 0; --i)
        if (AboutDialog* found = rt::widget_cast<AboutDialog>(floats[i - 1])) return found;
    return nullptr;
}

rt::FloatWindow* EditorAbout::open()
{
    rt::WidgetApp* app = rt::WidgetApp::existing();
    if (!app) return nullptr;
    if (rt::FloatWindow* existing = dialog())
    {
        app->bringToFront(existing);
        return existing;
    }
    return app->addFloat<AboutDialog>();
}

} // namespace prisma3d
