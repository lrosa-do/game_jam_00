#pragma once

#include <igui/widgets/BasicWidgets.hpp>
#include <igui/widgets/InputWidgets.hpp>
#include <igui/widgets/LayoutWidgets.hpp>
#include <igui/widgets/Signal.hpp>

#include <ct/string.hpp>
#include <ct/vector.hpp>

namespace prisma3d
{

class AnimationLibraryEditor : public ig::retained::BoxLayout
{
public:
    IG_WIDGET_CLASS(AnimationLibraryEditor, ig::retained::BoxLayout);

    struct Clip
    {
        ct::String name;
        float duration = 0.0f;
        bool playing = false;
        bool current = false;
        float position = 0.0f;
    };

    static constexpr float kFrameStep = 1.0f / 30.0f;

    static constexpr const char* kNoClipsText = "No animations";
    static constexpr const char* kImportText = "Add from file...";
    static constexpr const char* kImportTooltip =
            "Import every animation of a glTF, FBX, B3D, MS3D or pmesh file; bones are matched by "
            "name to the player's skeleton. Dropping a model file here does the same.";

    AnimationLibraryEditor();

    void setClips(const ct::Vector<Clip>& clips);
    std::size_t clipCount() const;
    const Clip& clip(std::size_t index) const;
    static ct::String durationText(float seconds);

    ig::retained::Label* label(std::size_t index) const;
    ig::retained::Button* playButton(std::size_t index) const;
    ig::retained::Button* removeButton(std::size_t index) const;
    ig::retained::Button* importButton() const;

    ig::retained::Signal<> importRequested;
    ig::retained::Signal<const ct::String&> removeRequested;
    ig::retained::Signal<const ct::String&> playRequested;
    ig::retained::Signal<> stopRequested;
    ig::retained::Signal<const ct::String&, float> seekRequested;
    ig::retained::Signal<> pauseRequested;

private:
    struct Row
    {
        ig::retained::BoxLayout* row = nullptr;
        ig::retained::Label* label = nullptr;
        ig::retained::Button* play = nullptr;
        ig::retained::Button* remove = nullptr;
    };

    void rebuild();
    void buildScrub(std::size_t index);
    void syncPosition();
    float steppedTime(float delta) const;

    ct::Vector<Clip> mClips;
    ct::Vector<Row> mRows;
    ig::retained::BoxLayout* mList;
    ig::retained::Label* mEmpty;
    ig::retained::Button* mImport;
    ig::retained::BoxLayout* mScrub;
    ig::retained::Slider* mSlider;
    ig::retained::Label* mTime;
    ct::String mTimeText;
    bool mSyncing;
};

} // namespace prisma3d
