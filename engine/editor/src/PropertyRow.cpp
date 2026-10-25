#include "prisma3d/editor/PropertyRow.h"

#include "prisma3d/editor/AssetGrid.h"
#include "prisma3d/editor/EditorIcons.h"
#include "prisma3d/editor/PropertyWidgets.h"

#include <igui/widgets/Theme.hpp>
#include <igui/widgets/WidgetApp.hpp>

#include <math.h>
#include <string.h>

namespace prisma3d
{

namespace rt = ig::retained;

namespace
{

const float kDegrees = 57.295779513082320876798f;
const float kRadians = 0.017453292519943295769f;
const float kEditorSpacing = 4.0f;
const float kBrowseWidth = 34.0f;
const char* const kNoChoice = "(none)";

rt::Color axisColor(int axis)
{
    switch (axis)
    {
        case 0:
            return rt::Color(226, 86, 86, 255);
        case 1:
            return rt::Color(124, 200, 92, 255);
        case 2:
            return rt::Color(88, 142, 236, 255);
        default:
            return rt::Color(200, 200, 200, 255);
    }
}

const char* const kAxisNames[4] = { "X", "Y", "Z", "W" };

struct ResourceFilter
{
    const char* type;
    const char* extensions;
};

const ResourceFilter kResourceFilters[] = {
    { "Mesh", "*.glb;*.gltf;*.fbx;*.obj;*.b3d;*.3ds;*.ms3d;*.mesh;*.pmesh" },
    { "Texture2D", "*.png;*.jpg;*.jpeg;*.tga;*.bmp;*.dds;*.hdr" },
    { "Material", "*.mat" },
    { "AudioStream", "*.wav;*.mp3;*.flac;*.ogg" },
    { "AnimationLibrary", "*.glb;*.gltf;*.fbx;*.b3d;*.ms3d;*.pmesh" },
};

double snapped(double value, double step)
{
    return step != 0.0 ? floor(value / step + 0.5) * step : value;
}

bool sameText(const ct::String& a, const char* b)
{
    return a == b;
}

void splitItem(const ct::String& item, ct::String& name, bool& hasValue, std::int64_t& value)
{
    const std::size_t colon = item.find(':');
    if (colon == ct::String::npos)
    {
        name = item;
        hasValue = false;
        value = 0;
        return;
    }
    name = item.substr(0, colon);
    hasValue = true;
    value = static_cast<std::int64_t>(item.substr(colon + 1).to_int());
}

ct::Vector<ct::String> splitList(const ct::String& text, char separator)
{
    ct::Vector<ct::String> parts;
    std::size_t start = 0;
    for (std::size_t i = 0; i <= text.size(); ++i)
    {
        if (i != text.size() && text[i] != separator) continue;
        parts.push_back(text.substr(start, i - start));
        start = i + 1;
    }
    return parts;
}

} // namespace

PropertyRow::Kind PropertyRow::kindOf(const PropertyInfo& info)
{
    switch (info.type)
    {
        case PropertyType::Bool:
            return Kind::Bool;
        case PropertyType::Int:
            if (info.hint == PropertyHint::Enum) return Kind::Enum;
            if (info.hint == PropertyHint::Flags) return Kind::Flags;
            if (info.hint == PropertyHint::Layers3DRender ||
                    info.hint == PropertyHint::Layers3DPhysics)
                return Kind::Layers;
            return Kind::Int;
        case PropertyType::Float:
            return Kind::Float;
        case PropertyType::Vec2:
            return Kind::Vec2;
        case PropertyType::Vec3:
            return Kind::Vec3;
        case PropertyType::Vec4:
            return Kind::Vec4;
        case PropertyType::Quat:
            return Kind::Quat;
        case PropertyType::Color:
            return Kind::Color;
        case PropertyType::String:
            if (info.hint == PropertyHint::Enum && info.enumCount() > 0) return Kind::TextEnum;
            return Kind::String;
        case PropertyType::NodePath:
            return Kind::NodePath;
        case PropertyType::ResourcePath:
            if (hasResourceType(info, "Mesh")) return Kind::Mesh;
            if (hasResourceType(info, "Material")) return Kind::Material;
            if (hasResourceType(info, "AnimationLibrary")) return Kind::AnimationLibrary;
            return Kind::ResourcePath;
        default:
            return Kind::None;
    }
}

bool PropertyRow::isWide(const PropertyInfo& info)
{
    const Kind kind = kindOf(info);
    switch (kind)
    {
        case Kind::Int:
        case Kind::Float:
            return info.hint == PropertyHint::Range && info.rangeMax > info.rangeMin;
        case Kind::Flags:
        case Kind::Layers:
        case Kind::Vec2:
        case Kind::Vec3:
        case Kind::Vec4:
        case Kind::Quat:
        case Kind::ResourcePath:
        case Kind::Mesh:
        case Kind::AnimationLibrary:
            return true;
        default:
            return false;
    }
}

ct::String PropertyRow::fileFilter(const PropertyInfo& info)
{
    ct::String filter;
    const auto add = [&filter](const ct::String& pattern)
    {
        if (pattern.empty()) return;
        if (!filter.empty()) filter += ';';
        filter += pattern;
    };

    if (info.hint == PropertyHint::File)
    {
        const ct::Vector<ct::String> items = splitList(info.hintString, ',');
        for (std::size_t i = 0; i < items.size(); ++i)
        {
            ct::String item = items[i].trimmed();
            if (item.empty()) continue;
            if (!item.contains('*'))
            {
                if (item[0] == '.') item = item.substr(1);
                item = ct::String("*.") + item;
            }
            add(item);
        }
    }
    else if (info.hint == PropertyHint::ResourceType)
    {
        const ct::Vector<ct::String> types = splitList(info.hintString, ',');
        for (std::size_t i = 0; i < types.size(); ++i)
        {
            const ct::String type = types[i].trimmed();
            for (const ResourceFilter& known: kResourceFilters)
                if (sameText(type, known.type)) add(known.extensions);
        }
    }
    return filter;
}

bool PropertyRow::filterMatches(const ct::String& filter, const ct::String& path)
{
    if (filter.empty()) return true;
    const std::size_t dot = path.rfind('.');
    if (dot == ct::String::npos) return false;
    ct::String extension = path.substr(dot + 1);
    for (std::size_t i = 0; i < extension.size(); ++i)
        if (extension[i] >= 'A' && extension[i] <= 'Z')
            extension.data()[i] = static_cast<char>(extension[i] + ('a' - 'A'));
    const ct::Vector<ct::String> patterns = splitList(filter, ';');
    for (std::size_t i = 0; i < patterns.size(); ++i)
    {
        ct::String pattern = patterns[i].trimmed();
        if (pattern.starts_with("*.")) pattern = pattern.substr(2);
        for (std::size_t k = 0; k < pattern.size(); ++k)
            if (pattern[k] >= 'A' && pattern[k] <= 'Z')
                pattern.data()[k] = static_cast<char>(pattern[k] + ('a' - 'A'));
        if (pattern == extension) return true;
    }
    return false;
}

bool PropertyRow::acceptsDrop(const rt::DragPayload& payload)
{
    if (!isEnabledInTree()) return false;
    if (mKind != Kind::ResourcePath && mKind != Kind::AnimationLibrary) return false;
    ct::String path;
    if (!AssetGrid::payloadPath(payload, path)) return false;
    return filterMatches(fileFilter(mInfo), path);
}

void PropertyRow::onDropReceive(const rt::DragPayload& payload)
{
    ct::String path;
    if (!AssetGrid::payloadPath(payload, path)) return;
    if (mKind == Kind::AnimationLibrary)
    {
        animationDropped.emit(path);
        return;
    }
    const PropertyValue value = PropertyValue::fromResourcePath(path);
    commit(value);
    setValue(value);
}

bool PropertyRow::hasResourceType(const PropertyInfo& info, const char* type)
{
    if (info.hint != PropertyHint::ResourceType) return false;
    const ct::Vector<ct::String> types = splitList(info.hintString, ',');
    for (std::size_t i = 0; i < types.size(); ++i)
        if (sameText(types[i].trimmed(), type)) return true;
    return false;
}

ct::String PropertyRow::materialDisplayName(const ct::String& path)
{
    if (path.empty()) return ct::String(kEmptyMaterial);
    const std::size_t at = path.find("::");
    if (at == ct::String::npos) return path;
    const ct::String name = path.substr(at + 2);
    return name.empty() ? path : name;
}

PropertyRow::PropertyRow(const PropertyInfo& info, const ct::String& label)
        : rt::BoxLayout(isWide(info) ? rt::LayoutDir::Vertical : rt::LayoutDir::Horizontal),
          mInfo(info),
          mLabel(label),
          mKind(kindOf(info)),
          mRevert(nullptr),
          mFields{},
          mFieldCount(0),
          mCheck(nullptr),
          mCombo(nullptr),
          mText(nullptr),
          mSwatch(nullptr),
          mBrowse(nullptr),
          mNodePick(nullptr),
          mGrid(nullptr),
          mMesh(nullptr),
          mAnimations(nullptr),
          mMaterialName(nullptr),
          mMaterialButton(nullptr),
          mEuler(0.0f, 0.0f, 0.0f),
          mSyncing(false)
{
    setPadding(0.0f);
    setSpacing(kEditorSpacing);
    build();
}

PropertyRow::~PropertyRow()
{
}

const PropertyInfo& PropertyRow::info() const
{
    return mInfo;
}

PropertyRow::Kind PropertyRow::kind() const
{
    return mKind;
}

const ct::String& PropertyRow::label() const
{
    return mLabel;
}

const PropertyValue& PropertyRow::value() const
{
    return mValue;
}

bool PropertyRow::isIntegral() const
{
    return mKind == Kind::Int;
}

float PropertyRow::toShown(double raw) const
{
    return static_cast<float>(mInfo.radiansAsDegrees ? raw * kDegrees : raw);
}

double PropertyRow::toRaw(float shown) const
{
    return mInfo.radiansAsDegrees ? static_cast<double>(shown) * kRadians
                                  : static_cast<double>(shown);
}

double PropertyRow::snap(double value) const
{
    const bool ranged = mInfo.hint == PropertyHint::Range;
    double step =
            ranged && mInfo.rangeStep > 0.0 ? mInfo.rangeStep : (isIntegral() ? 1.0 : kDefaultStep);
    if (isIntegral()) step = step < 1.0 ? 1.0 : floor(step + 0.5);
    const double minimum = ranged ? mInfo.rangeMin : 0.0;
    if (fabs(minimum) > step * 1.0e14) return snapped(value, step);
    return snapped(value - minimum, step) + minimum;
}

double PropertyRow::constrain(double value) const
{
    if (isnan(value)) return value;
    value = snap(value);
    if (isIntegral()) value = floor(value + 0.5);
    const bool ranged = mInfo.hint == PropertyHint::Range && mInfo.rangeMax > mInfo.rangeMin;
    if (ranged)
    {
        if (!mInfo.rangeOrGreater && value > mInfo.rangeMax) value = mInfo.rangeMax;
        if (!mInfo.rangeOrLess && value < mInfo.rangeMin) value = mInfo.rangeMin;
    }
    return value;
}

float PropertyRow::dragSpeed() const
{
    if (isIntegral()) return kIntegerDragSpeed;
    double step = mInfo.hint == PropertyHint::Range && mInfo.rangeStep > 0.0 ? mInfo.rangeStep
                                                                             : kDefaultStep;
    if (step < kDefaultStep) step = kDefaultStep;
    return static_cast<float>(step) * kFloatDragSpeed;
}

PropertyValue PropertyRow::constrained(const PropertyValue& value) const
{
    switch (mKind)
    {
        case Kind::Int:
            return PropertyValue::fromInt(static_cast<std::int64_t>(constrain(value.asFloat())));
        case Kind::Float:
        {
            const double shown =
                    mInfo.radiansAsDegrees ? value.asFloat() * kDegrees : value.asFloat();
            const double result = constrain(shown);
            return PropertyValue::fromFloat(mInfo.radiansAsDegrees ? result * kRadians : result);
        }
        default:
            return value;
    }
}

void PropertyRow::build()
{
    const bool wide = isWide(mInfo);
    NameLabel* name = nullptr;
    rt::Widget* editors = this;

    if (wide)
    {
        rt::BoxLayout* header = createChild<rt::BoxLayout>(rt::LayoutDir::Horizontal);
        header->setPadding(0.0f);
        header->setSpacing(kEditorSpacing);
        name = header->createChild<NameLabel>(mLabel);
        name->setStretch(1.0f);
        mRevert = header->createChild<RevertButton>();
    }
    else
    {
        name = createChild<NameLabel>(mLabel);
        name->setStretch(4.0f);
        mRevert = createChild<RevertButton>();
        rt::BoxLayout* box = createChild<rt::BoxLayout>(rt::LayoutDir::Horizontal);
        box->setPadding(0.0f);
        box->setSpacing(kEditorSpacing);
        box->setStretch(6.0f);
        editors = box;
    }
    name->setTooltip(mInfo.name);
    mRevert->reverted.connect([this] { revertRequested.emit(); });

    switch (mKind)
    {
        case Kind::Bool:
            mCheck = editors->createChild<rt::CheckBox>("On");
            mCheck->setStretch(1.0f);
            mCheck->toggled.connect([this](bool checked) { boxToggled(-1, checked); });
            break;
        case Kind::Int:
        case Kind::Float:
            buildNumbers(editors, 1);
            break;
        case Kind::Enum:
            buildEnum(editors);
            break;
        case Kind::Flags:
            buildFlags(editors);
            break;
        case Kind::Layers:
            buildLayers(editors);
            break;
        case Kind::Vec2:
        case Kind::Vec3:
        case Kind::Vec4:
        case Kind::Quat:
        {
            rt::BoxLayout* box = editors->createChild<rt::BoxLayout>(rt::LayoutDir::Horizontal);
            box->setPadding(0.0f);
            box->setSpacing(kEditorSpacing);
            const int count = mKind == Kind::Vec2 ? 2 : (mKind == Kind::Vec4 ? 4 : 3);
            buildNumbers(box, count);
            break;
        }
        case Kind::Color:
            mSwatch = editors->createChild<ColorSwatch>();
            mSwatch->setStretch(1.0f);
            mSwatch->setAlphaEnabled(mInfo.hint != PropertyHint::ColorNoAlpha);
            mSwatch->colorEdited.connect([this](const Math::Vec4& color) { colorEdited(color); });
            break;
        case Kind::String:
        case Kind::NodePath:
        case Kind::ResourcePath:
            buildText(editors);
            break;
        case Kind::TextEnum:
            buildTextEnum(editors);
            break;
        case Kind::Mesh:
            buildMesh(editors);
            break;
        case Kind::Material:
            buildMaterial(editors);
            break;
        case Kind::AnimationLibrary:
            buildAnimations(editors);
            break;
        case Kind::None:
            break;
    }
}

void PropertyRow::buildNumbers(rt::Widget* parent, int count)
{
    const bool single = count == 1;
    const bool ranged =
            single && mInfo.hint == PropertyHint::Range && mInfo.rangeMax > mInfo.rangeMin;
    mFieldCount = count;
    for (int i = 0; i < count; ++i)
    {
        NumberField* field = parent->createChild<NumberField>(single ? "" : kAxisNames[i]);
        field->setStretch(1.0f);
        if (ranged)
            field->setRange(mInfo.rangeMin, mInfo.rangeMax, mInfo.rangeOrLess,
                    mInfo.rangeOrGreater);
        if (!single) field->setTagColor(axisColor(i));
        field->setDragSensitivity(mKind == Kind::Quat ? 0.5f : dragSpeed());
        field->valueChanged.connect([this, i](float shown) { numberEdited(i, shown); });
        mFields[i] = field;
    }
}

void PropertyRow::buildEnum(rt::Widget* parent)
{
    mCombo = parent->createChild<rt::ComboBox>();
    mCombo->setStretch(1.0f);
    const ct::Vector<ct::String> items = splitList(mInfo.hintString, ',');
    std::int64_t next = 0;
    for (std::size_t i = 0; i < items.size(); ++i)
    {
        ct::String name;
        bool hasValue = false;
        std::int64_t value = 0;
        splitItem(items[i], name, hasValue, value);
        if (hasValue) next = value;
        mCombo->addItem(name);
        mChoices.push_back(next);
        ++next;
    }
    mCombo->selectionChanged.connect([this](int index) { choiceSelected(index); });
}

void PropertyRow::buildFlags(rt::Widget* parent)
{
    const ct::Vector<ct::String> items = splitList(mInfo.hintString, ',');
    for (std::size_t i = 0; i < items.size(); ++i)
    {
        ct::String name;
        bool hasValue = false;
        std::int64_t value = 0;
        splitItem(items[i].trimmed(), name, hasValue, value);
        if (name.empty()) continue;
        if (hasValue)
        {
            if (value == 0) continue;
        }
        else
            value = static_cast<std::int64_t>(1) << i;
        rt::CheckBox* box = parent->createChild<rt::CheckBox>(name);
        const int index = static_cast<int>(mBoxes.size());
        box->toggled.connect([this, index](bool checked) { boxToggled(index, checked); });
        mBoxes.push_back(box);
        mBoxValues.push_back(value);
    }
}

void PropertyRow::buildLayers(rt::Widget* parent)
{
    const bool physics = mInfo.hint == PropertyHint::Layers3DPhysics;
    mGrid = parent->createChild<LayerGrid>(physics ? kPhysicsLayers : kRenderLayers,
            physics ? kPhysicsGroupSize : kRenderGroupSize);
    mGrid->changed.connect([this](std::int64_t bits) { layerChanged(bits); });
}

void PropertyRow::buildTextEnum(rt::Widget* parent)
{
    mCombo = parent->createChild<rt::ComboBox>();
    mCombo->setStretch(1.0f);
    const std::size_t count = mInfo.enumCount();
    for (std::size_t i = 0; i < count; ++i)
    {
        const ct::String name = mInfo.enumName(i);
        mTextChoices.push_back(name);
        mCombo->addItem(name.empty() ? ct::String(kNoChoice) : name);
    }
    mCombo->selectionChanged.connect([this](int index) { textChoiceSelected(index); });
}

void PropertyRow::buildMesh(rt::Widget* parent)
{
    mMesh = parent->createChild<MeshPropertyEditor>();
    mMesh->saveRequested.connect([this] { meshSaveRequested.emit(); });
    mMesh->removeRequested.connect([this](int surface) { meshRemoveRequested.emit(surface); });
    mMesh->extractRequested.connect([this](int surface) { meshExtractRequested.emit(surface); });
}

void PropertyRow::buildAnimations(rt::Widget* parent)
{
    mAnimations = parent->createChild<AnimationLibraryEditor>();
    mAnimations->importRequested.connect([this] { animationImportRequested.emit(); });
    mAnimations->removeRequested.connect(
            [this](const ct::String& name) { animationRemoveRequested.emit(name); });
    mAnimations->playRequested.connect(
            [this](const ct::String& name) { animationPlayRequested.emit(name); });
    mAnimations->stopRequested.connect([this] { animationStopRequested.emit(); });
    mAnimations->seekRequested.connect([this](const ct::String& name, float time)
            { animationSeekRequested.emit(name, time); });
    mAnimations->pauseRequested.connect([this] { animationPauseRequested.emit(); });
}

void PropertyRow::buildMaterial(rt::Widget* parent)
{
    mMaterialName = parent->createChild<rt::Label>(ct::String(kEmptyMaterial));
    mMaterialName->setStretch(1.0f);
    mMaterialButton = parent->createChild<rt::Button>();
    mMaterialButton->setIconId(EditorIcons::get("MoveDown"));
    mMaterialButton->setSize(kBrowseWidth, 0.0f);
    mMaterialButton->setTooltip("Choose a material");
    mMaterialButton->clicked.connect([this] { materialPickRequested.emit(); });
}

void PropertyRow::buildText(rt::Widget* parent)
{
    rt::Widget* container = parent;
    if (mKind == Kind::ResourcePath || mKind == Kind::NodePath)
    {
        rt::BoxLayout* box = parent->createChild<rt::BoxLayout>(rt::LayoutDir::Horizontal);
        box->setPadding(0.0f);
        box->setSpacing(kEditorSpacing);
        box->setStretch(1.0f);
        container = box;
    }
    mText = container->createChild<CommitInput>();
    mText->setStretch(1.0f);
    if (mKind == Kind::String)
        mText->textChanged.connect([this](const ct::String& text) { textEdited(text); });
    else
        mText->committed.connect([this](const ct::String& text) { textEdited(text); });

    if (mKind == Kind::ResourcePath)
    {
        mBrowse = container->createChild<rt::Button>();
        mBrowse->setIconId(EditorIcons::get("Load"));
        mBrowse->setSize(kBrowseWidth, 0.0f);
        mBrowse->setTooltip("Choose a file");
        mBrowse->clicked.connect([this] { browseRequested.emit(); });
    }
    else if (mKind == Kind::NodePath)
    {
        mNodePick = container->createChild<rt::Button>();
        mNodePick->setIconId(EditorIcons::get("ListSelect"));
        mNodePick->setSize(kBrowseWidth, 0.0f);
        mNodePick->setTooltip("Choose a node");
        mNodePick->clicked.connect([this] { nodePickRequested.emit(); });
    }
}

void PropertyRow::setValue(const PropertyValue& value)
{
    mValue = value;
    const bool wasSyncing = mSyncing;
    mSyncing = true;
    switch (mKind)
    {
        case Kind::Bool:
            mCheck->setChecked(value.asBool());
            break;
        case Kind::Int:
        {
            const float shown = static_cast<float>(value.asInt());
            mFields[0]->setValue(shown);
            break;
        }
        case Kind::Float:
        {
            const float shown = toShown(value.asFloat());
            mFields[0]->setValue(shown);
            break;
        }
        case Kind::Enum:
        {
            const std::int64_t wanted = value.asInt();
            int selected = -1;
            for (std::size_t i = 0; i < mChoices.size(); ++i)
                if (mChoices[i] == wanted)
                {
                    selected = static_cast<int>(i);
                    break;
                }
            mCombo->setSelectedIndex(selected);
            break;
        }
        case Kind::Flags:
        {
            const std::int64_t bits = value.asInt();
            for (std::size_t i = 0; i < mBoxes.size(); ++i)
                mBoxes[i]->setChecked((bits & mBoxValues[i]) == mBoxValues[i]);
            break;
        }
        case Kind::Layers:
            mGrid->setBits(value.asInt());
            break;
        case Kind::Vec2:
        {
            const Math::Vec2 v = value.asVec2();
            mFields[0]->setValue(v.x);
            mFields[1]->setValue(v.y);
            break;
        }
        case Kind::Vec3:
        {
            const Math::Vec3 v = value.asVec3();
            mFields[0]->setValue(v.x);
            mFields[1]->setValue(v.y);
            mFields[2]->setValue(v.z);
            break;
        }
        case Kind::Vec4:
        {
            const Math::Vec4 v = value.asVec4();
            mFields[0]->setValue(v.x);
            mFields[1]->setValue(v.y);
            mFields[2]->setValue(v.z);
            mFields[3]->setValue(v.w);
            break;
        }
        case Kind::Quat:
        {
            const Math::Quaternion q = value.asQuat();
            const Math::Quaternion current = Math::Quaternion::FromEulerAngles(mEuler.x * kRadians,
                    mEuler.y * kRadians, mEuler.z * kRadians);
            if (fabsf(q.Dot(current)) < 1.0f - 1.0e-6f)
            {
                const Math::Vec3 euler = q.ToEulerAngles();
                mEuler = Math::Vec3(euler.x * kDegrees, euler.y * kDegrees, euler.z * kDegrees);
            }
            mFields[0]->setValue(mEuler.x);
            mFields[1]->setValue(mEuler.y);
            mFields[2]->setValue(mEuler.z);
            break;
        }
        case Kind::Color:
            mSwatch->setColor(value.asColor());
            break;
        case Kind::String:
        case Kind::NodePath:
        case Kind::ResourcePath:
            if (mKind == Kind::String) mText->setText(value.asString());
            else
                mText->setCommittedText(value.asString());
            break;
        case Kind::TextEnum:
        {
            const ct::String wanted = value.asString();
            int selected = -1;
            for (std::size_t i = 0; i < mTextChoices.size(); ++i)
                if (mTextChoices[i] == wanted)
                {
                    selected = static_cast<int>(i);
                    break;
                }
            mCombo->setSelectedIndex(selected);
            break;
        }
        case Kind::Mesh:
            mMesh->setPath(value.asString());
            break;
        case Kind::AnimationLibrary:
            break;
        case Kind::Material:
        {
            const ct::String path = value.asString();
            mMaterialName->setText(materialDisplayName(path));
            mMaterialName->setTooltip(path);
            break;
        }
        case Kind::None:
            break;
    }
    mSyncing = wasSyncing;
}

void PropertyRow::commit(const PropertyValue& value)
{
    if (value.type() == PropertyType::Float && mValue.type() == PropertyType::Float &&
            static_cast<float>(value.asFloat()) == static_cast<float>(mValue.asFloat()))
        return;
    if (value == mValue) return;
    mValue = value;
    edited.emit(mValue);
}

void PropertyRow::numberEdited(int index, float shown)
{
    if (mSyncing) return;
    const bool wasSyncing = mSyncing;
    switch (mKind)
    {
        case Kind::Int:
        {
            const double result = constrain(static_cast<double>(shown));
            mSyncing = true;
            mFields[0]->setValue(static_cast<float>(result));
            mSyncing = wasSyncing;
            commit(PropertyValue::fromInt(static_cast<std::int64_t>(result)));
            break;
        }
        case Kind::Float:
        {
            const double result = constrain(static_cast<double>(shown));
            mSyncing = true;
            mFields[0]->setValue(static_cast<float>(result));
            mSyncing = wasSyncing;
            commit(PropertyValue::fromFloat(toRaw(static_cast<float>(result))));
            break;
        }
        case Kind::Vec2:
        case Kind::Vec3:
        case Kind::Vec4:
        {
            const float result = static_cast<float>(constrain(static_cast<double>(shown)));
            mSyncing = true;
            mFields[index]->setValue(result);
            mSyncing = wasSyncing;
            float v[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
            for (int i = 0; i < mFieldCount; ++i) v[i] = mFields[i]->value();
            if (mKind == Kind::Vec2) commit(PropertyValue::fromVec2(Math::Vec2(v[0], v[1])));
            else if (mKind == Kind::Vec3)
                commit(PropertyValue::fromVec3(Math::Vec3(v[0], v[1], v[2])));
            else
                commit(PropertyValue::fromVec4(Math::Vec4(v[0], v[1], v[2], v[3])));
            break;
        }
        case Kind::Quat:
        {
            float* axes[3] = { &mEuler.x, &mEuler.y, &mEuler.z };
            *axes[index] = shown;
            const Math::Quaternion q = Math::Quaternion::FromEulerAngles(mEuler.x * kRadians,
                    mEuler.y * kRadians, mEuler.z * kRadians);
            commit(PropertyValue::fromQuat(q));
            break;
        }
        default:
            break;
    }
}

void PropertyRow::boxToggled(int index, bool checked)
{
    if (mSyncing) return;
    if (mKind == Kind::Bool)
    {
        commit(PropertyValue::fromBool(checked));
        return;
    }
    if (index < 0 || index >= static_cast<int>(mBoxes.size())) return;
    std::int64_t bits = mValue.asInt();
    if (checked) bits |= mBoxValues[static_cast<std::size_t>(index)];
    else
        bits &= ~mBoxValues[static_cast<std::size_t>(index)];
    commit(PropertyValue::fromInt(bits));
}

void PropertyRow::layerChanged(std::int64_t bits)
{
    if (mSyncing) return;
    commit(PropertyValue::fromInt(bits));
}

void PropertyRow::textChoiceSelected(int index)
{
    if (mSyncing || index < 0 || index >= static_cast<int>(mTextChoices.size())) return;
    commit(PropertyValue::fromString(mTextChoices[static_cast<std::size_t>(index)]));
}

void PropertyRow::choiceSelected(int index)
{
    if (mSyncing || index < 0 || index >= static_cast<int>(mChoices.size())) return;
    commit(PropertyValue::fromInt(mChoices[static_cast<std::size_t>(index)]));
}

void PropertyRow::colorEdited(const Math::Vec4& color)
{
    if (mSyncing) return;
    commit(PropertyValue::fromColor(color));
}

void PropertyRow::textEdited(const ct::String& text)
{
    if (mSyncing) return;
    if (mKind == Kind::NodePath) commit(PropertyValue::fromNodePath(text));
    else if (mKind == Kind::ResourcePath)
        commit(PropertyValue::fromResourcePath(text));
    else
        commit(PropertyValue::fromString(text));
}

bool PropertyRow::isActive() const
{
    for (int i = 0; i < mFieldCount; ++i)
        if (mFields[i]->isActive()) return true;
    if (mText && mText->isFocused()) return true;
    if (mSwatch && mSwatch->isOpen()) return true;
    return false;
}

void PropertyRow::setRevertible(bool revertible)
{
    mRevert->setActive(revertible);
}

bool PropertyRow::revertible() const
{
    return mRevert->active();
}

int PropertyRow::componentCount() const
{
    return mFieldCount;
}

NumberField* PropertyRow::field(int index) const
{
    return index >= 0 && index < mFieldCount ? mFields[index] : nullptr;
}

rt::CheckBox* PropertyRow::check() const
{
    return mCheck;
}

rt::ComboBox* PropertyRow::combo() const
{
    return mCombo;
}

rt::TextInput* PropertyRow::text() const
{
    return mText;
}

ColorSwatch* PropertyRow::swatch() const
{
    return mSwatch;
}

rt::Button* PropertyRow::browseButton() const
{
    return mBrowse;
}

rt::Button* PropertyRow::nodeButton() const
{
    return mNodePick;
}

LayerGrid* PropertyRow::layerGrid() const
{
    return mGrid;
}

AnimationLibraryEditor* PropertyRow::animationEditor() const
{
    return mAnimations;
}

MeshPropertyEditor* PropertyRow::meshEditor() const
{
    return mMesh;
}

rt::Label* PropertyRow::materialLabel() const
{
    return mMaterialName;
}

rt::Button* PropertyRow::materialButton() const
{
    return mMaterialButton;
}

int PropertyRow::textChoiceCount() const
{
    return static_cast<int>(mTextChoices.size());
}

const ct::String& PropertyRow::textChoice(int index) const
{
    static const ct::String empty;
    return index >= 0 && index < textChoiceCount() ? mTextChoices[static_cast<std::size_t>(index)]
                                                   : empty;
}

RevertButton* PropertyRow::revertButton() const
{
    return mRevert;
}

int PropertyRow::boxCount() const
{
    return static_cast<int>(mBoxes.size());
}

rt::CheckBox* PropertyRow::box(int index) const
{
    return index >= 0 && index < boxCount() ? mBoxes[static_cast<std::size_t>(index)] : nullptr;
}

int PropertyRow::choiceCount() const
{
    return static_cast<int>(mChoices.size());
}

std::int64_t PropertyRow::choiceValue(int index) const
{
    return index >= 0 && index < choiceCount() ? mChoices[static_cast<std::size_t>(index)] : 0;
}

} // namespace prisma3d
