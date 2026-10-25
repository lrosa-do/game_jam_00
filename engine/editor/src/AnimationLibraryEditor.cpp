#include "prisma3d/editor/AnimationLibraryEditor.h"

#include "prisma3d/editor/EditorIcons.h"

#include <stdio.h>

namespace prisma3d
{

namespace rt = ig::retained;

namespace
{

const float kSpacing = 4.0f;
const float kButtonWidth = 30.0f;

ct::String timeText(float position, float duration)
{
    char text[48];
    snprintf(text, sizeof(text), "%.2f / %.2f s", static_cast<double>(position),
            static_cast<double>(duration));
    return ct::String(text);
}

} // namespace

AnimationLibraryEditor::AnimationLibraryEditor()
        : rt::BoxLayout(rt::LayoutDir::Vertical),
          mList(nullptr),
          mEmpty(nullptr),
          mImport(nullptr),
          mScrub(nullptr),
          mSlider(nullptr),
          mTime(nullptr),
          mTimeText(),
          mSyncing(false)
{
    setPadding(0.0f);
    setSpacing(kSpacing);
    setStretch(1.0f);
    mList = createChild<rt::BoxLayout>(rt::LayoutDir::Vertical);
    mList->setPadding(0.0f);
    mList->setSpacing(kSpacing);
    mEmpty = mList->createChild<rt::Label>(ct::String(kNoClipsText));
    rt::BoxLayout* footer = createChild<rt::BoxLayout>(rt::LayoutDir::Horizontal);
    footer->setPadding(0.0f);
    footer->setSpacing(kSpacing);
    mImport = footer->createChild<rt::Button>(kImportText);
    mImport->setIconId(EditorIcons::get("Load"));
    mImport->setTooltip(kImportTooltip);
    mImport->setStretch(1.0f);
    mImport->clicked.connect([this] { importRequested.emit(); });
}

ct::String AnimationLibraryEditor::durationText(float seconds)
{
    char text[32];
    snprintf(text, sizeof(text), "%.2f s", static_cast<double>(seconds));
    return ct::String(text);
}

void AnimationLibraryEditor::setClips(const ct::Vector<Clip>& clips)
{
    bool same = clips.size() == mClips.size();
    for (std::size_t i = 0; same && i < clips.size(); ++i)
        same = clips[i].name == mClips[i].name && clips[i].duration == mClips[i].duration &&
               clips[i].playing == mClips[i].playing && clips[i].current == mClips[i].current;
    if (same)
    {
        for (std::size_t i = 0; i < clips.size(); ++i) mClips[i].position = clips[i].position;
        syncPosition();
        return;
    }
    mClips = clips;
    rebuild();
}

void AnimationLibraryEditor::rebuild()
{
    for (std::size_t i = 0; i < mRows.size(); ++i) mList->removeChild(mRows[i].row);
    mRows.clear();
    if (mScrub) mList->removeChild(mScrub);
    mScrub = nullptr;
    mSlider = nullptr;
    mTime = nullptr;
    mEmpty->setVisible(mClips.empty());
    for (std::size_t i = 0; i < mClips.size(); ++i)
    {
        const Clip& clip = mClips[i];
        Row row;
        row.row = mList->createChild<rt::BoxLayout>(rt::LayoutDir::Horizontal);
        row.row->setPadding(0.0f);
        row.row->setSpacing(kSpacing);
        row.label = row.row->createChild<rt::Label>(
                clip.name + "  (" + durationText(clip.duration) + ")");
        row.label->setStretch(1.0f);
        row.play = row.row->createChild<rt::Button>();
        row.play->setIconId(EditorIcons::get(clip.playing ? "Stop" : "Play"));
        row.play->setSize(kButtonWidth, 0.0f);
        row.play->setTooltip(clip.playing ? "Stop and reset the pose" : "Play in the viewport");
        const ct::String name = clip.name;
        const bool playing = clip.playing;
        row.play->clicked.connect(
                [this, name, playing]
                {
                    if (playing) stopRequested.emit();
                    else
                        playRequested.emit(name);
                });
        row.remove = row.row->createChild<rt::Button>();
        row.remove->setIconId(EditorIcons::get("Remove"));
        row.remove->setSize(kButtonWidth, 0.0f);
        row.remove->setTooltip("Remove this animation from the player");
        row.remove->clicked.connect([this, name] { removeRequested.emit(name); });
        mRows.push_back(row);
        if (clip.current) buildScrub(i);
    }
    markDirty();
}

void AnimationLibraryEditor::buildScrub(std::size_t index)
{
    const Clip& clip = mClips[index];
    const ct::String name = clip.name;
    mScrub = mList->createChild<rt::BoxLayout>(rt::LayoutDir::Horizontal);
    mScrub->setPadding(0.0f);
    mScrub->setSpacing(kSpacing);
    rt::Button* back = mScrub->createChild<rt::Button>();
    back->setIconId(EditorIcons::get("Back"));
    back->setSize(kButtonWidth, 0.0f);
    back->setTooltip("One frame back (1/30 s)");
    back->clicked.connect([this, name] { seekRequested.emit(name, steppedTime(-kFrameStep)); });
    mSlider = mScrub->createChild<rt::Slider>(0.0f, clip.duration > 0.0f ? clip.duration : 1.0f,
            clip.position);
    mSlider->setStretch(1.0f);
    mSlider->onValueChanged.connect(
            [this, name](float value)
            {
                if (!mSyncing) seekRequested.emit(name, value);
            });
    rt::Button* forward = mScrub->createChild<rt::Button>();
    forward->setIconId(EditorIcons::get("Forward"));
    forward->setSize(kButtonWidth, 0.0f);
    forward->setTooltip("One frame forward (1/30 s)");
    forward->clicked.connect([this, name] { seekRequested.emit(name, steppedTime(kFrameStep)); });
    rt::Button* pause = mScrub->createChild<rt::Button>();
    pause->setIconId(EditorIcons::get("Pause"));
    pause->setSize(kButtonWidth, 0.0f);
    pause->setTooltip("Hold the pose at this time");
    pause->clicked.connect([this] { pauseRequested.emit(); });
    mTimeText = timeText(clip.position, clip.duration);
    mTime = mScrub->createChild<rt::Label>(mTimeText);
}

float AnimationLibraryEditor::steppedTime(float delta) const
{
    if (!mSlider) return 0.0f;
    float time = mSlider->value() + delta;
    if (time < mSlider->minValue()) time = mSlider->minValue();
    if (time > mSlider->maxValue()) time = mSlider->maxValue();
    return time;
}

void AnimationLibraryEditor::syncPosition()
{
    if (!mSlider) return;
    for (std::size_t i = 0; i < mClips.size(); ++i)
    {
        if (!mClips[i].current) continue;
        const float position = mClips[i].position;
        if (position < mSlider->value() - 0.0005f || position > mSlider->value() + 0.0005f)
        {
            mSyncing = true;
            mSlider->setValue(position);
            mSyncing = false;
        }
        const ct::String text = timeText(position, mClips[i].duration);
        if (mTime && text != mTimeText)
        {
            mTimeText = text;
            mTime->setText(text);
        }
        return;
    }
}

std::size_t AnimationLibraryEditor::clipCount() const
{
    return mClips.size();
}

const AnimationLibraryEditor::Clip& AnimationLibraryEditor::clip(std::size_t index) const
{
    return mClips[index];
}

rt::Label* AnimationLibraryEditor::label(std::size_t index) const
{
    return index < mRows.size() ? mRows[index].label : nullptr;
}

rt::Button* AnimationLibraryEditor::playButton(std::size_t index) const
{
    return index < mRows.size() ? mRows[index].play : nullptr;
}

rt::Button* AnimationLibraryEditor::removeButton(std::size_t index) const
{
    return index < mRows.size() ? mRows[index].remove : nullptr;
}

rt::Button* AnimationLibraryEditor::importButton() const
{
    return mImport;
}

} // namespace prisma3d
