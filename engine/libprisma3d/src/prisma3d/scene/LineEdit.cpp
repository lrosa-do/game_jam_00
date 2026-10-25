#include "prisma3d/scene/LineEdit.h"

#include "prisma3d/Input.h"
#include "prisma3d/gui/Gui.h"
#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/ScriptInstance.h"

namespace prisma3d
{

namespace
{

const float kPaddingX = 8.0f;
const float kPaddingY = 6.0f;
const double kBlinkPeriod = 1.0;

bool continuation(char byte)
{
    return (static_cast<unsigned char>(byte) & 0xc0u) == 0x80u;
}

} // namespace

LineEdit::LineEdit()
        : mText(),
          mPlaceholder(),
          mFontSize(18.0f),
          mFontColor(1.0f, 1.0f, 1.0f, 1.0f),
          mBackgroundColor(0.08f, 0.09f, 0.12f, 1.0f),
          mFocusColor(0.4f, 0.65f, 1.0f, 1.0f),
          mMaxLength(0),
          mEditable(true),
          mFocused(false),
          mCaret(0),
          mBlink(0.0)
{
    setProcess(true);
    setProcessInput(true);
    updateMinimumSize();
}

LineEdit::~LineEdit()
{
}

void LineEdit::setText(const ct::String& text)
{
    mText = text;
    mCaret = mText.size();
}

const ct::String& LineEdit::getText() const
{
    return mText;
}

void LineEdit::setPlaceholder(const ct::String& text)
{
    mPlaceholder = text;
}

const ct::String& LineEdit::getPlaceholder() const
{
    return mPlaceholder;
}

void LineEdit::setFontSize(float size)
{
    mFontSize = size < 1.0f ? 1.0f : size;
    updateMinimumSize();
}

float LineEdit::getFontSize() const
{
    return mFontSize;
}

void LineEdit::setFontColor(const Math::Vec4& color)
{
    mFontColor = color;
}

Math::Vec4 LineEdit::getFontColor() const
{
    return mFontColor;
}

void LineEdit::setBackgroundColor(const Math::Vec4& color)
{
    mBackgroundColor = color;
}

Math::Vec4 LineEdit::getBackgroundColor() const
{
    return mBackgroundColor;
}

void LineEdit::setFocusColor(const Math::Vec4& color)
{
    mFocusColor = color;
}

Math::Vec4 LineEdit::getFocusColor() const
{
    return mFocusColor;
}

void LineEdit::setMaxLength(int length)
{
    mMaxLength = length < 0 ? 0 : length;
}

int LineEdit::getMaxLength() const
{
    return mMaxLength;
}

void LineEdit::setEditable(bool editable)
{
    mEditable = editable;
    if (!editable) mFocused = false;
}

bool LineEdit::isEditable() const
{
    return mEditable;
}

void LineEdit::grabFocus()
{
    if (!mEditable) return;
    mFocused = true;
    mBlink = 0.0;
    mCaret = mText.size();
}

void LineEdit::releaseFocus()
{
    mFocused = false;
}

bool LineEdit::hasFocus() const
{
    return mFocused;
}

void LineEdit::clear()
{
    if (mText.empty()) return;
    mText = ct::String();
    mCaret = 0;
    changed();
}

void LineEdit::submit()
{
    fireHook(true);
}

Math::Vec2 LineEdit::getMinimumSize() const
{
    return Math::Vec2(kPaddingX * 2.0f + mFontSize * 4.0f, mFontSize + kPaddingY * 2.0f);
}

bool LineEdit::containsPoint(const Math::Vec2& point) const
{
    const Math::Vec2 origin = getGlobalPosition();
    const Math::Vec2 size = getDrawSize();
    return point.x >= origin.x && point.y >= origin.y && point.x < origin.x + size.x &&
           point.y < origin.y + size.y;
}

std::size_t LineEdit::previousBoundary(std::size_t index) const
{
    if (index == 0) return 0;
    --index;
    while (index > 0 && continuation(mText[index])) --index;
    return index;
}

std::size_t LineEdit::nextBoundary(std::size_t index) const
{
    if (index >= mText.size()) return mText.size();
    ++index;
    while (index < mText.size() && continuation(mText[index])) ++index;
    return index;
}

int LineEdit::codepointCount() const
{
    int count = 0;
    for (std::size_t i = 0; i < mText.size(); ++i)
        if (!continuation(mText[i])) ++count;
    return count;
}

void LineEdit::insertCodepoint(std::uint32_t codepoint)
{
    if (codepoint < 32 || codepoint == 127) return;
    if (mMaxLength > 0 && codepointCount() >= mMaxLength) return;
    char bytes[4];
    int length = 0;
    if (codepoint < 0x80)
    {
        bytes[length++] = static_cast<char>(codepoint);
    }
    else if (codepoint < 0x800)
    {
        bytes[length++] = static_cast<char>(0xc0 | (codepoint >> 6));
        bytes[length++] = static_cast<char>(0x80 | (codepoint & 0x3f));
    }
    else if (codepoint < 0x10000)
    {
        bytes[length++] = static_cast<char>(0xe0 | (codepoint >> 12));
        bytes[length++] = static_cast<char>(0x80 | ((codepoint >> 6) & 0x3f));
        bytes[length++] = static_cast<char>(0x80 | (codepoint & 0x3f));
    }
    else
    {
        bytes[length++] = static_cast<char>(0xf0 | (codepoint >> 18));
        bytes[length++] = static_cast<char>(0x80 | ((codepoint >> 12) & 0x3f));
        bytes[length++] = static_cast<char>(0x80 | ((codepoint >> 6) & 0x3f));
        bytes[length++] = static_cast<char>(0x80 | (codepoint & 0x3f));
    }
    if (mCaret > mText.size()) mCaret = mText.size();
    ct::String result = mText.substr(0, mCaret);
    result.append(bytes, static_cast<std::size_t>(length));
    result += mText.substr(mCaret);
    mText = result;
    mCaret += static_cast<std::size_t>(length);
    changed();
}

void LineEdit::eraseBefore()
{
    if (mCaret == 0 || mText.empty()) return;
    const std::size_t start = previousBoundary(mCaret);
    mText = mText.substr(0, start) + mText.substr(mCaret);
    mCaret = start;
    changed();
}

void LineEdit::eraseAfter()
{
    if (mCaret >= mText.size()) return;
    const std::size_t end = nextBoundary(mCaret);
    mText = mText.substr(0, mCaret) + mText.substr(end);
    changed();
}

void LineEdit::changed()
{
    mBlink = 0.0;
    fireHook(false);
}

void LineEdit::fireHook(bool submitted)
{
    for (Node* node = this; node; node = node->getParent())
    {
        ScriptInstance* script = node->getScriptInstance();
        if (!script || !script->hasScript()) continue;
        if (submitted) script->textSubmitted(this, mText);
        else
            script->textChanged(this, mText);
        return;
    }
}

void LineEdit::inputHandler(const InputEvent& event)
{
    if (event.type == InputEventType::MouseButton && event.mouseButton.pressed &&
            event.mouseButton.button == MouseButton::Left)
    {
        const bool inside = mEditable && isVisibleInTree() &&
                            containsPoint(Math::Vec2(static_cast<float>(event.mouseButton.x),
                                    static_cast<float>(event.mouseButton.y)));
        if (inside && !mFocused) grabFocus();
        else if (!inside && mFocused)
            releaseFocus();
        return;
    }
    if (!mFocused || !mEditable) return;
    SceneTree* tree = getTree();
    if (event.type == InputEventType::Char)
    {
        insertCodepoint(event.character.codepoint);
        if (tree) tree->setInputAsHandled();
        return;
    }
    if (event.type != InputEventType::Key || !event.key.pressed) return;
    switch (event.key.key)
    {
        case Key::Backspace:
            eraseBefore();
            break;
        case Key::Delete:
            eraseAfter();
            break;
        case Key::Left:
            mCaret = previousBoundary(mCaret);
            mBlink = 0.0;
            break;
        case Key::Right:
            mCaret = nextBoundary(mCaret);
            mBlink = 0.0;
            break;
        case Key::Home:
            mCaret = 0;
            break;
        case Key::End:
            mCaret = mText.size();
            break;
        case Key::Enter:
        case Key::KpEnter:
            submit();
            break;
        case Key::Escape:
            releaseFocus();
            break;
        default:
            break;
    }
    if (tree) tree->setInputAsHandled();
}

void LineEdit::notificationHandler(int what)
{
    Control::notificationHandler(what);
    if (what == kNotificationProcess && mFocused) mBlink += getProcessDeltaTime();
}

void LineEdit::draw(Gui& gui)
{
    const Math::Vec2 origin = getGlobalPosition();
    const Math::Vec2 size = getDrawSize();
    const float fontSize = mFontSize * drawScale();
    if (size.x <= 0.0f || size.y <= 0.0f) return;
    const Math::Vec4 modulate = getEffectiveModulate();
    const auto tint = [&](const Math::Vec4& color, float alpha)
    {
        return Control::guiColor(Math::Vec4(color.x * modulate.x, color.y * modulate.y,
                color.z * modulate.z, color.w * modulate.w * alpha));
    };
    const ig::Rect rect(origin.x, origin.y, size.x, size.y);
    gui.hudRectFilled(rect, tint(mBackgroundColor, 1.0f));
    gui.hudRectOutline(rect, mFocused ? tint(mFocusColor, 1.0f) : tint(mFontColor, 0.35f), 1.0f);

    const float padding = kPaddingX * drawScale();
    const float innerWidth = size.x - padding * 2.0f;
    const float textY = origin.y + (size.y - fontSize) * 0.5f;
    if (mText.empty())
    {
        if (!mPlaceholder.empty())
            gui.hudText(ig::StringView(mPlaceholder.c_str(), mPlaceholder.size()),
                    origin.x + padding, textY, tint(mFontColor, 0.4f), fontSize);
    }
    std::size_t first = 0;
    if (!mText.empty())
    {
        const std::size_t caret = mCaret > mText.size() ? mText.size() : mCaret;
        while (first < caret)
        {
            const ig::Vec2 measured =
                    gui.hudTextSize(ig::StringView(mText.c_str() + first, caret - first), fontSize);
            if (measured.x <= innerWidth) break;
            first = nextBoundary(first);
        }
        gui.hudText(ig::StringView(mText.c_str() + first, mText.size() - first), origin.x + padding,
                textY, tint(mFontColor, 1.0f), fontSize);
    }
    if (mFocused && mEditable)
    {
        const double phase =
                mBlink -
                static_cast<double>(static_cast<long long>(mBlink / kBlinkPeriod)) * kBlinkPeriod;
        if (phase < kBlinkPeriod * 0.5)
        {
            const std::size_t caret = mCaret > mText.size() ? mText.size() : mCaret;
            const float caretX =
                    caret > first
                            ? gui.hudTextSize(ig::StringView(mText.c_str() + first, caret - first),
                                         fontSize)
                                      .x
                            : 0.0f;
            gui.hudRectFilled(ig::Rect(origin.x + padding + caretX, textY, 2.0f, fontSize),
                    tint(mFontColor, 1.0f));
        }
    }
}

void LineEdit::bindProperties()
{
    using DB = NodeClassDB;
    using P = LineEdit;
    DB::addProperty<P>(PropertyInfo(PropertyType::String, "text"), &P::setText, &P::getText);
    DB::addProperty<P>(PropertyInfo(PropertyType::String, "placeholder"), &P::setPlaceholder,
            &P::getPlaceholder);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Float, "font_size", 1.0, 512.0, 1.0),
            &P::setFontSize, &P::getFontSize);
    DB::addProperty<P>(PropertyInfo(PropertyType::Color, "font_color"), &P::setFontColor,
            &P::getFontColor);
    DB::addProperty<P>(PropertyInfo(PropertyType::Color, "background_color"),
            &P::setBackgroundColor, &P::getBackgroundColor);
    DB::addProperty<P>(PropertyInfo(PropertyType::Color, "focus_color"), &P::setFocusColor,
            &P::getFocusColor);
    DB::addProperty<P>(PropertyInfo::range(PropertyType::Int, "max_length", 0.0, 100000.0, 1.0),
            &P::setMaxLength, &P::getMaxLength);
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "editable"), &P::setEditable,
            &P::isEditable);
}

} // namespace prisma3d
