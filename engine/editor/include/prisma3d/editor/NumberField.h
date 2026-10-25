#pragma once

#include <igui/widgets/BasicWidgets.hpp>
#include <igui/widgets/Signal.hpp>
#include <igui/widgets/TextInputWidgets.hpp>

#include <ct/string.hpp>

namespace prisma3d
{

class NumberField : public ig::retained::Widget
{
public:
    IG_WIDGET_CLASS(NumberField, ig::retained::Widget);

    explicit NumberField(const char* tag = "");
    ~NumberField() override;

    float value() const;
    void setValue(float value);

    void setDragSensitivity(float unitsPerPixel);
    float dragSensitivity() const;

    void setRange(double minimum, double maximum, bool orLess = false, bool orGreater = false);
    void clearRange();
    bool ranged() const;
    double rangeMin() const;
    double rangeMax() const;
    float ratio() const;
    void setRatio(float ratio);
    bool isGrabbing() const;
    bool grabberVisible() const;
    ig::retained::Rect grabberRect() const;

    void setTag(const char* tag);
    const ct::String& tag() const;
    void setTagColor(const ig::retained::Color& color);

    const ct::String& text() const;
    ct::String format(float value) const;

    bool isEditing() const;
    bool isDragging() const;
    bool isActive() const;
    ig::retained::TextInput* editor() const;

    void beginEdit();
    void commitEdit();
    void cancelEdit();

    ig::retained::Signal<float> valueChanged;
    ig::retained::Signal<> editFinished;

    Vec2f sizeHint() const override;
    void layout() override;
    void paint(ig::retained::PaintContext& ctx) override;
    void onMousePress(ig::retained::MouseEvent& event) override;
    void onMouseRelease(ig::retained::MouseEvent& event) override;
    void onMouseMove(ig::retained::MouseEvent& event) override;
    void onKeyPress(ig::retained::KeyEvent& event) override;

private:
    class Editor;

    void finishEdit(bool commit);
    void cancelDrag();
    float trackWidth() const;
    float trackOffset() const;
    bool overGrabber(float localX, float localY) const;
    void setValueInternal(float value, bool notify);

    ct::String mTag;
    ct::String mText;
    ct::String mEditStart;
    ig::retained::Color mTagColor;
    Editor* mEditor;
    float mValue;
    float mSensitivity;
    float mPressX;
    float mPressValue;
    double mMin;
    double mMax;
    float mGrabFrom;
    float mGrabRatio;
    bool mRanged;
    bool mOrLess;
    bool mOrGreater;
    bool mGrabbing;
    bool mOverGrabber;
    bool mPressed;
    bool mDragged;
    bool mEditing;
};

} // namespace prisma3d
