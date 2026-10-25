#include "prisma3d/editor/EditorStatusBar.h"

#include "prisma3d/editor/EditorAbout.h"
#include "prisma3d/scene/Node.h"

#include "platform.h"

#include <igui/widgets/Theme.hpp>

#include <stdio.h>
#include <string.h>
#include <time.h>

namespace prisma3d
{

namespace rt = ig::retained;

namespace
{

const float kFieldShare[EditorStatusBar::kFieldCount] = { 0.10f, 0.60f, 0.10f, 0.05f, 0.05f, 0.10f,
    0.0f };
const float kFieldMinimum[EditorStatusBar::kFieldCount] = { 150.0f, 110.0f, 190.0f, 200.0f, 160.0f,
    110.0f, 262.0f };
const float kTextInset = 8.0f;
const float kSpacing = 6.0f;

} // namespace

class StatusField : public rt::Widget
{
public:
    IG_WIDGET_CLASS(StatusField, rt::Widget);

    explicit StatusField(float width = 100.0f)
            : mWidth(width),
              mColor(200, 200, 205, 255)
    {
    }

    void setWidth(float width)
    {
        if (width == mWidth) return;
        mWidth = width;
        markDirty();
    }

    float width() const
    {
        return mWidth;
    }

    void setText(const ct::String& text)
    {
        if (mText == text) return;
        mText = text;
        markDirty();
    }

    const ct::String& text() const
    {
        return mText;
    }

    void setColor(const rt::Color& color)
    {
        mColor = color;
        markDirty();
    }

    Vec2f sizeHint() const override
    {
        return { mWidth, 0.0f };
    }

    void paint(rt::PaintContext& ctx) override
    {
        if (!visible_) return;
        const rt::Rect abs = absoluteRect();
        if (ctx.isClipped(abs)) return;

        const rt::Theme& theme = rt::Theme::instance();
        ctx.pushClip(abs);
        ctx.font.SetFontSize(theme.fontSize * 0.9f);
        ctx.font.SetBatch(&ctx.text);
        const float ascender = ctx.font.GetAscender();
        ctx.font.SetColor(mColor);
        const float top = abs.y + (abs.h - theme.fontSize * 0.9f) * 0.5f;
        ctx.font.Print(mText.c_str(), abs.x + kTextInset, top + ascender);
        ctx.popClip();
    }

private:
    ct::String mText;
    float mWidth;
    rt::Color mColor;
};

EditorStatusBar::EditorStatusBar(EditorDocument& document, EditorSelection& selection,
        RenderingServer& server)
        : mDocument(document),
          mSelection(selection),
          mServer(server),
          mGizmo(nullptr),
          mBar(nullptr),
          mFields{},
          mLogCursor(EditorLog::instance().total()),
          mPendingLevel(0),
          mCapturing(false),
          mBarWidth(0.0f),
          mAccumulated(0.0f),
          mFrames(0),
          mFps(0.0f),
          mFrameMilliseconds(0.0f),
          mUpdates(0)
{
}

EditorStatusBar::~EditorStatusBar()
{
    captureLog(false);
}

bool EditorStatusBar::attach(rt::StatusBar* bar)
{
    if (mBar || !bar) return false;
    mBar = bar;
    mBar->setSpacing(kSpacing);
    for (int i = 0; i < kFieldCount; ++i)
        mFields[i] = mBar->createChild<StatusField>(kFieldMinimum[i]);
    mFields[kScene]->clicked.connect(
            [this]
            {
                if (mSaveHandler) mSaveHandler();
            });
    mFields[kScene]->setTooltip("Click to save the scene (Ctrl+S)");
    mFields[kLog]->setTooltip("Last engine message");
    mFields[kSelection]->setTooltip("Selected nodes");
    mFields[kTool]->setTooltip("Gizmo mode, space and snap");
    mFields[kDraw]->setTooltip("Draw calls and triangles of the last frame");
    mFields[kPerformance]->setTooltip("Frames per second and frame time (0.5 s average)");
    mFields[kVersion]->setTooltip("Editor version");
    mFields[kVersion]->setColor(rt::Color(150, 150, 158, 255));
    mFields[kVersion]->clicked.connect([] { EditorAbout::open(); });
    set(kVersion, EditorAbout::signature());
    fitWidths();
    update(0.0f);
    return true;
}

bool EditorStatusBar::attached() const
{
    return mBar != nullptr;
}

void EditorStatusBar::setGizmo(const EditorGizmo* gizmo)
{
    mGizmo = gizmo;
}

void EditorStatusBar::setSaveHandler(const ct::Function<void()>& handler)
{
    mSaveHandler = handler;
}

void EditorStatusBar::captureLog(bool capture)
{
    if (capture == mCapturing) return;
    mCapturing = capture;
    EditorLog::instance().capture(capture);
}

bool EditorStatusBar::capturingLog() const
{
    return mCapturing;
}

void EditorStatusBar::postMessage(int level, const char* text)
{
    EditorLog::instance().post(level, text);
}

ct::String EditorStatusBar::formatTime(std::int64_t seconds)
{
    if (seconds <= 0) return ct::String();
    const time_t stamp = static_cast<time_t>(seconds);
    struct tm local;
#if defined(_WIN32)
    localtime_s(&local, &stamp);
#else
    localtime_r(&stamp, &local);
#endif
    char buffer[16];
    snprintf(buffer, sizeof buffer, "%02d:%02d", local.tm_hour, local.tm_min);
    return ct::String(buffer);
}

const char* EditorStatusBar::toolName(EditorGizmo::Tool tool)
{
    switch (tool)
    {
        case EditorGizmo::Tool::Select:
            return "Select";
        case EditorGizmo::Tool::Transform:
            return "Transform";
        case EditorGizmo::Tool::Move:
            return "Move";
        case EditorGizmo::Tool::Rotate:
            return "Rotate";
        case EditorGizmo::Tool::Scale:
            return "Scale";
    }
    return "";
}

void EditorStatusBar::set(Field field, const ct::String& value)
{
    if (mTexts[field] == value) return;
    mTexts[field] = value;
    if (mFields[field]) mFields[field]->setText(value);
}

void EditorStatusBar::refreshScene()
{
    ct::String text = mDocument.dirty() ? "Unsaved" : "Saved";
    text += " · ";
    text += mDocument.sceneName();
    const ct::String time = formatTime(mDocument.savedAt());
    if (!time.empty())
    {
        text += " · ";
        text += time;
    }
    set(kScene, text);
    if (mFields[kScene])
        mFields[kScene]->setColor(
                mDocument.dirty() ? rt::Color(235, 190, 90, 255) : rt::Color(150, 205, 150, 255));
}

void EditorStatusBar::refreshLog()
{
    ct::Vector<EditorLog::Entry> entries;
    EditorLog::instance().read(mLogCursor, entries);
    while (!entries.empty() && entries.back().script) entries.pop_back();
    if (entries.empty()) return;
    mPendingLevel = entries.back().level;
    ct::String text = entries.back().text;
    const std::size_t newline = text.find('\n');
    if (newline != ct::String::npos) text = text.substr(0, newline);
    set(kLog, text);
    rt::Color color(190, 190, 195, 255);
    if (mPendingLevel == static_cast<int>(LOGLEVEL_WARN)) color = rt::Color(235, 190, 90, 255);
    else if (mPendingLevel >= static_cast<int>(LOGLEVEL_ERROR))
        color = rt::Color(240, 100, 100, 255);
    if (mFields[kLog]) mFields[kLog]->setColor(color);
}

void EditorStatusBar::refreshSelection()
{
    const ct::Vector<Node*>& selected = mSelection.selectedNodes();
    if (selected.empty())
    {
        set(kSelection, "No selection");
        return;
    }
    char buffer[32];
    snprintf(buffer, sizeof buffer, "%u", static_cast<unsigned>(selected.size()));
    ct::String text(buffer);
    text += selected.size() == 1 ? " node · " : " nodes · ";
    text += selected[0]->getName();
    text += " (";
    text += selected[0]->getClassName();
    text += ")";
    set(kSelection, text);
}

void EditorStatusBar::refreshTool()
{
    if (!mGizmo)
    {
        set(kTool, ct::String());
        return;
    }
    ct::String text = toolName(mGizmo->tool());
    text += mGizmo->localSpace() ? " · Local" : " · Global";
    if (mGizmo->snapEnabled())
    {
        char buffer[48];
        const EditorGizmo::Tool tool = mGizmo->tool();
        if (tool == EditorGizmo::Tool::Rotate)
            snprintf(buffer, sizeof buffer, " · Snap %g°",
                    static_cast<double>(mGizmo->settings().rotateSnap));
        else if (tool == EditorGizmo::Tool::Scale)
            snprintf(buffer, sizeof buffer, " · Snap %g%%",
                    static_cast<double>(mGizmo->settings().scaleSnap));
        else
            snprintf(buffer, sizeof buffer, " · Snap %g m",
                    static_cast<double>(mGizmo->settings().translateSnap));
        text += buffer;
    }
    set(kTool, text);
}

void EditorStatusBar::refreshDraw()
{
    const RenderStats& stats = mServer.getStats();
    char buffer[64];
    snprintf(buffer, sizeof buffer, "Draws %u · Triangles %u", stats.drawCalls, stats.triangles);
    set(kDraw, ct::String(buffer));
}

void EditorStatusBar::refreshPerformance(float delta)
{
    if (delta > 0.0f)
    {
        mAccumulated += delta;
        ++mFrames;
    }
    if (mAccumulated >= kAverageSeconds && mFrames > 0)
    {
        mFrameMilliseconds = mAccumulated / static_cast<float>(mFrames) * 1000.0f;
        mFps = static_cast<float>(mFrames) / mAccumulated;
        mAccumulated = 0.0f;
        mFrames = 0;
    }
    char buffer[48];
    snprintf(buffer, sizeof buffer, "%.0f FPS · %.1f ms", static_cast<double>(mFps),
            static_cast<double>(mFrameMilliseconds));
    set(kPerformance, ct::String(buffer));
}

void EditorStatusBar::fitWidths()
{
    if (!mBar) return;
    const float width = mBar->rect().w;
    if (width <= 0.0f || width == mBarWidth) return;
    mBarWidth = width;
    float used = 16.0f + kSpacing * static_cast<float>(kFieldCount);
    for (int i = 0; i < kFieldCount; ++i) used += kFieldMinimum[i];
    const float extra = width > used ? width - used : 0.0f;
    for (int i = 0; i < kFieldCount; ++i)
        mFields[i]->setWidth(kFieldMinimum[i] + extra * kFieldShare[i]);
    mBar->layout();
    mBar->markDirty();
}

void EditorStatusBar::update(float delta)
{
    if (!mBar) return;
    ++mUpdates;
    fitWidths();
    refreshScene();
    refreshLog();
    refreshSelection();
    refreshTool();
    refreshDraw();
    refreshPerformance(delta);
}

const ct::String& EditorStatusBar::text(Field field) const
{
    return mTexts[field];
}

rt::Widget* EditorStatusBar::field(Field field) const
{
    return mFields[field];
}

rt::StatusBar* EditorStatusBar::bar() const
{
    return mBar;
}

float EditorStatusBar::fps() const
{
    return mFps;
}

float EditorStatusBar::frameMilliseconds() const
{
    return mFrameMilliseconds;
}

unsigned int EditorStatusBar::updateCount() const
{
    return mUpdates;
}

} // namespace prisma3d
