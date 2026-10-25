#pragma once

#include "prisma3d/Input.h"

#include <igui/widgets/ToolBar.hpp>

#include <ct/function.hpp>
#include <ct/ptr.hpp>
#include <ct/string.hpp>

#include <cstdint>

namespace prisma3d
{

class EditorDocument;
class SceneTree;

class EditorPlay
{
public:
    enum State : int
    {
        kStopped,
        kPlaying,
        kPaused
    };

    enum Button : int
    {
        kPlay,
        kPause,
        kStop,
        kRun,
        kButtonCount
    };

    static constexpr const char* kRunnerName = "runner";

    EditorPlay(SceneTree& tree, EditorDocument& document);
    ~EditorPlay();
    EditorPlay(const EditorPlay&) = delete;
    EditorPlay& operator=(const EditorPlay&) = delete;

    bool attach(ig::retained::ToolBar* bar);
    bool attached() const;
    int buttonId(Button button) const;
    static const char* buttonTooltip(Button button);
    static ig::retained::IconId buttonIcon(Button button);

    using ViewHandler = ct::Function<void(bool)>;

    void setViewHandler(const ViewHandler& handler);

    State state() const;
    bool playing() const;
    bool paused() const;

    bool play();
    void pause();
    void setPaused(bool paused);
    bool stop();

    bool canPlay() const;
    bool canPause() const;
    bool canStop() const;
    bool canRun() const;

    void setRunnerPath(const ct::String& path);
    ct::String runnerPath() const;
    ct::String runTarget() const;
    bool run();
    void stopRunner();
    bool runnerRunning() const;
    int runnerExitCode() const;
    unsigned int runnerLaunches() const;

    void update();
    bool handleEvent(const InputEvent& event);

private:
    struct Link
    {
        EditorPlay* owner;
    };

    void syncButtons();
    void clickedButton(int id);
    void toggledButton(int id, bool checked);
    void finishRunner();

    SceneTree& mTree;
    EditorDocument& mDocument;
    ct::Rc<Link> mLink;
    ig::retained::ToolBar* mBar;
    int mButtons[kButtonCount];
    ViewHandler mViewHandler;
    ct::String mRunnerPath;
    std::int64_t mRunner;
    unsigned int mLaunches;
    int mExitCode;
    State mState;
    bool mSyncing;
    bool mPreviousHint;
    bool mPreviousPhysics;
};

} // namespace prisma3d
