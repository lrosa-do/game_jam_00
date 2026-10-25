#pragma once

#include "prisma3d/scene/Control.h"

#include <ct/string.hpp>

namespace prisma3d
{

class LineEdit : public Control
{
public:
    PRISMA3D_NODE_CLASS(LineEdit, Control);
    static void bindProperties();

    LineEdit();
    ~LineEdit() override;

    void setText(const ct::String& text);
    const ct::String& getText() const;
    void setPlaceholder(const ct::String& text);
    const ct::String& getPlaceholder() const;
    void setFontSize(float size);
    float getFontSize() const;
    void setFontColor(const Math::Vec4& color);
    Math::Vec4 getFontColor() const;
    void setBackgroundColor(const Math::Vec4& color);
    Math::Vec4 getBackgroundColor() const;
    void setFocusColor(const Math::Vec4& color);
    Math::Vec4 getFocusColor() const;
    void setMaxLength(int length);
    int getMaxLength() const;
    void setEditable(bool editable);
    bool isEditable() const;

    void grabFocus();
    void releaseFocus();
    bool hasFocus() const;
    void clear();
    void submit();

    Math::Vec2 getMinimumSize() const override;

protected:
    void notificationHandler(int what) override;
    void inputHandler(const InputEvent& event) override;
    void draw(Gui& gui) override;

private:
    bool containsPoint(const Math::Vec2& point) const;
    void insertCodepoint(std::uint32_t codepoint);
    void eraseBefore();
    void eraseAfter();
    std::size_t previousBoundary(std::size_t index) const;
    std::size_t nextBoundary(std::size_t index) const;
    int codepointCount() const;
    void changed();
    void fireHook(bool submitted);

    ct::String mText;
    ct::String mPlaceholder;
    float mFontSize;
    Math::Vec4 mFontColor;
    Math::Vec4 mBackgroundColor;
    Math::Vec4 mFocusColor;
    int mMaxLength;
    bool mEditable;
    bool mFocused;
    std::size_t mCaret;
    double mBlink;
};

} // namespace prisma3d
