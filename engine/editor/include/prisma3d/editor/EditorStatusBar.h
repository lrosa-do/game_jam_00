#pragma once

#include "prisma3d/editor/EditorDocument.h"
#include "prisma3d/editor/EditorGizmo.h"
#include "prisma3d/editor/EditorLog.h"
#include "prisma3d/editor/EditorSelection.h"
#include "prisma3d/render/RenderingServer.h"

#include <igui/widgets/LayoutWidgets.hpp>

#include <ct/function.hpp>
#include <ct/string.hpp>

#include <cstdint>

namespace prisma3d
{

class StatusField;

class EditorStatusBar
{
public:
    enum Field : int
    {
        kScene,
        kLog,
        kSelection,
        kTool,
        kDraw,
        kPerformance,
        kVersion,
        kFieldCount
    };

    static constexpr float kAverageSeconds = 0.5f;

    EditorStatusBar(EditorDocument& document, EditorSelection& selection, RenderingServer& server);
    ~EditorStatusBar();
    EditorStatusBar(const EditorStatusBar&) = delete;
    EditorStatusBar& operator=(const EditorStatusBar&) = delete;

    bool attach(ig::retained::StatusBar* bar);
    bool attached() const;

    void setGizmo(const EditorGizmo* gizmo);
    void setSaveHandler(const ct::Function<void()>& handler);
    void captureLog(bool capture);
    bool capturingLog() const;
    void postMessage(int level, const char* text);

    void update(float delta);

    const ct::String& text(Field field) const;
    ig::retained::Widget* field(Field field) const;
    ig::retained::StatusBar* bar() const;
    float fps() const;
    float frameMilliseconds() const;
    unsigned int updateCount() const;

    static ct::String formatTime(std::int64_t seconds);
    static const char* toolName(EditorGizmo::Tool tool);

private:
    void refreshScene();
    void refreshLog();
    void refreshSelection();
    void refreshTool();
    void refreshDraw();
    void refreshPerformance(float delta);
    void fitWidths();
    void set(Field field, const ct::String& value);

    EditorDocument& mDocument;
    EditorSelection& mSelection;
    RenderingServer& mServer;
    const EditorGizmo* mGizmo;
    ig::retained::StatusBar* mBar;
    StatusField* mFields[kFieldCount];
    ct::String mTexts[kFieldCount];
    ct::Function<void()> mSaveHandler;
    std::size_t mLogCursor;
    int mPendingLevel;
    bool mCapturing;
    float mBarWidth;
    float mAccumulated;
    unsigned int mFrames;
    float mFps;
    float mFrameMilliseconds;
    unsigned int mUpdates;
};

} // namespace prisma3d
