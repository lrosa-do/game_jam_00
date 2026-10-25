#include "prisma3d/editor/PrimitiveParameterForm.h"

#include "prisma3d/editor/EditorIcons.h"
#include "prisma3d/editor/PropertyNames.h"

#include <igui/widgets/WidgetApp.hpp>

#include <math.h>

namespace prisma3d
{

namespace rt = ig::retained;

namespace
{

const float kFloatDrag = 0.01f;
const float kCountDrag = 0.5f;
const char* const kAxisTags[3] = { "X", "Y", "Z" };

rt::Color axisTagColor(int axis)
{
    switch (axis)
    {
        case 0:
            return rt::Color(226, 86, 86, 255);
        case 1:
            return rt::Color(124, 200, 92, 255);
        default:
            return rt::Color(88, 142, 236, 255);
    }
}

} // namespace

PrimitiveParameterForm::PrimitiveParameterForm()
        : rt::BoxLayout(rt::LayoutDir::Vertical),
          mHeightmapRow(nullptr),
          mHeightmap(nullptr),
          mHeightmapBrowse(nullptr),
          mRebuildPending(false),
          mSyncing(false),
          mEmitting(0)
{
    setPadding(0.0f);
    setSpacing(kRowSpacing);
    primitiveDefaults(PrimitiveKind::Box, mRecipe);
    buildContent();
}

PrimitiveParameterForm::Row PrimitiveParameterForm::addRow(rt::Widget* parent,
        const ct::String& label, const ct::String& tooltip)
{
    Row parts;
    parts.row = parent->createChild<rt::BoxLayout>(rt::LayoutDir::Horizontal);
    parts.row->setPadding(0.0f);
    parts.row->setSpacing(kRowSpacing);
    NameLabel* name = parts.row->createChild<NameLabel>(label);
    name->setStretch(4.0f);
    name->setTooltip(tooltip);
    parts.revert = parts.row->createChild<RevertButton>();
    parts.editors = parts.row->createChild<rt::BoxLayout>(rt::LayoutDir::Horizontal);
    parts.editors->setPadding(0.0f);
    parts.editors->setSpacing(kRowSpacing);
    parts.editors->setStretch(6.0f);
    return parts;
}

ct::String PrimitiveParameterForm::parameterLabel(const char* key)
{
    return capitalizePropertyName(ct::String(key));
}

void PrimitiveParameterForm::setRecipe(const PrimitiveRecipe& recipe)
{
    const bool restructure = recipe.kind != mRecipe.kind;
    mRecipe = recipe;
    const bool wasSyncing = mSyncing;
    mSyncing = true;
    if (restructure)
    {
        if (mEmitting > 0)
        {
            mRebuildPending = true;
            if (rt::WidgetApp* app = rt::WidgetApp::existing()) app->requestLayout();
        }
        else
            buildContent();
    }
    else if (!mRebuildPending)
        refreshValues();
    mSyncing = wasSyncing;
}

void PrimitiveParameterForm::setKind(PrimitiveKind kind)
{
    PrimitiveRecipe fresh;
    primitiveDefaults(kind, fresh);
    setRecipe(fresh);
}

PrimitiveKind PrimitiveParameterForm::kind() const
{
    return mRecipe.kind;
}

const PrimitiveRecipe& PrimitiveParameterForm::recipe() const
{
    return mRecipe;
}

bool PrimitiveParameterForm::complete() const
{
    return !primitiveKindHasHeightmap(mRecipe.kind) || !mRecipe.heightmap.empty();
}

ct::String PrimitiveParameterForm::path() const
{
    return primitiveRecipePath(mRecipe);
}

void PrimitiveParameterForm::layout()
{
    if (mRebuildPending)
    {
        mRebuildPending = false;
        const bool wasSyncing = mSyncing;
        mSyncing = true;
        buildContent();
        mSyncing = wasSyncing;
    }
    rt::BoxLayout::layout();
}

void PrimitiveParameterForm::clearContent()
{
    const ct::Vector<rt::Widget*> children = this->children();
    for (std::size_t i = 0; i < children.size(); ++i) removeChild(children[i]);
    mParameters.clear();
    mHeightmapRow = nullptr;
    mHeightmap = nullptr;
    mHeightmapBrowse = nullptr;
}

void PrimitiveParameterForm::buildContent()
{
    mRebuildPending = false;
    cancelEditing();
    clearContent();

    const PrimitiveKind kind = mRecipe.kind;
    const std::size_t count = primitiveParamCount(kind);
    const int slot = primitiveHeightmapSlot(kind);
    int valueIndex = 0;
    for (std::size_t p = 0; p < count; ++p)
    {
        if (slot >= 0 && static_cast<int>(p) == slot) buildHeightmapRow();
        const PrimitiveParamDef& def = primitiveParam(kind, p);
        Parameter parameter;
        parameter.def = &def;
        parameter.valueIndex = valueIndex;
        parameter.count = def.type == PrimitiveParamType::Vec3 ? 3 : 1;
        valueIndex += parameter.count;

        Row parts = addRow(this, parameterLabel(def.key), ct::String(def.key));
        parameter.row = parts.row;
        parameter.revert = parts.revert;
        const int index = static_cast<int>(mParameters.size());
        for (int c = 0; c < parameter.count; ++c)
        {
            NumberField* field = parts.editors->createChild<NumberField>(
                    parameter.count > 1 ? kAxisTags[c] : "");
            field->setStretch(1.0f);
            if (parameter.count > 1) field->setTagColor(axisTagColor(c));
            field->setDragSensitivity(
                    def.type == PrimitiveParamType::Count ? kCountDrag : kFloatDrag);
            field->valueChanged.connect(
                    [this, index, c](float shown) { parameterEdited(index, c, shown); });
            field->editFinished.connect([this] { editFinished.emit(); });
            parameter.fields[c] = field;
        }
        parameter.revert->reverted.connect([this, index] { parameterReverted(index); });
        mParameters.push_back(parameter);
    }
    refreshValues();
    if (rt::WidgetApp* app = rt::WidgetApp::existing()) app->requestLayout();
}

void PrimitiveParameterForm::buildHeightmapRow()
{
    Row parts = addRow(this, "Heightmap", "Image used as the height field");
    mHeightmapRow = parts.row;
    mHeightmap = parts.editors->createChild<CommitInput>();
    mHeightmap->setStretch(1.0f);
    mHeightmap->committed.connect([this](const ct::String& text) { heightmapCommitted(text); });
    mHeightmapBrowse = parts.editors->createChild<rt::Button>();
    mHeightmapBrowse->setIconId(EditorIcons::get("Load"));
    mHeightmapBrowse->setSize(kBrowseWidth, 0.0f);
    mHeightmapBrowse->setTooltip("Choose an image");
    mHeightmapBrowse->clicked.connect([this] { browseHeightmapRequested.emit(); });
}

void PrimitiveParameterForm::refreshRevert(int parameter)
{
    const Parameter& item = mParameters[static_cast<std::size_t>(parameter)];
    bool different = false;
    for (int c = 0; c < item.count; ++c)
        if (mRecipe.values[item.valueIndex + c] != item.def->fallback) different = true;
    item.revert->setActive(different);
}

void PrimitiveParameterForm::refreshValues()
{
    const bool wasSyncing = mSyncing;
    mSyncing = true;
    for (std::size_t p = 0; p < mParameters.size(); ++p)
    {
        const Parameter& item = mParameters[p];
        for (int c = 0; c < item.count; ++c)
            if (!item.fields[c]->isActive())
                item.fields[c]->setValue(mRecipe.values[item.valueIndex + c]);
        refreshRevert(static_cast<int>(p));
    }
    if (mHeightmap && !(mHeightmap->isFocused() && mHeightmap->pending()))
        mHeightmap->setCommittedText(mRecipe.heightmap);
    mSyncing = wasSyncing;
}

double PrimitiveParameterForm::constrain(const PrimitiveParamDef& def, double value) const
{
    if (isnan(value)) return static_cast<double>(def.fallback);
    if (def.type == PrimitiveParamType::Count) value = floor(value + 0.5);
    if (value < static_cast<double>(def.minimum)) value = static_cast<double>(def.minimum);
    if (value > static_cast<double>(def.maximum)) value = static_cast<double>(def.maximum);
    return value;
}

void PrimitiveParameterForm::emitChanged()
{
    ++mEmitting;
    changed.emit();
    --mEmitting;
}

void PrimitiveParameterForm::parameterEdited(int parameter, int component, float shown)
{
    if (mSyncing) return;
    const Parameter& item = mParameters[static_cast<std::size_t>(parameter)];
    const float value = static_cast<float>(constrain(*item.def, static_cast<double>(shown)));
    mSyncing = true;
    item.fields[component]->setValue(value);
    mSyncing = false;
    mRecipe.values[item.valueIndex + component] = value;
    refreshRevert(parameter);
    emitChanged();
}

void PrimitiveParameterForm::parameterReverted(int parameter)
{
    const Parameter& item = mParameters[static_cast<std::size_t>(parameter)];
    mSyncing = true;
    for (int c = 0; c < item.count; ++c)
    {
        mRecipe.values[item.valueIndex + c] = item.def->fallback;
        item.fields[c]->setValue(item.def->fallback);
    }
    mSyncing = false;
    refreshRevert(parameter);
    emitChanged();
}

void PrimitiveParameterForm::heightmapCommitted(const ct::String& text)
{
    if (mSyncing) return;
    mRecipe.heightmap = text.trimmed();
    emitChanged();
}

void PrimitiveParameterForm::setHeightmapFile(const ct::String& file)
{
    if (!mHeightmap) return;
    mRecipe.heightmap = file;
    mHeightmap->setCommittedText(file);
    emitChanged();
}

void PrimitiveParameterForm::cancelEditing()
{
    for (std::size_t p = 0; p < mParameters.size(); ++p)
        for (int c = 0; c < mParameters[p].count; ++c)
            if (mParameters[p].fields[c]->isEditing()) mParameters[p].fields[c]->cancelEdit();
    rt::WidgetApp* app = rt::WidgetApp::existing();
    if (!app) return;
    rt::Widget* focused = app->focused();
    if (focused && focused == mHeightmap) app->setFocused(nullptr);
}

bool PrimitiveParameterForm::isActive() const
{
    for (std::size_t p = 0; p < mParameters.size(); ++p)
        for (int c = 0; c < mParameters[p].count; ++c)
            if (mParameters[p].fields[c]->isActive()) return true;
    if (mHeightmap && mHeightmap->isFocused() && mHeightmap->pending()) return true;
    return false;
}

int PrimitiveParameterForm::parameterCount() const
{
    return static_cast<int>(mParameters.size());
}

ct::String PrimitiveParameterForm::parameterKey(int parameter) const
{
    if (parameter < 0 || parameter >= parameterCount()) return ct::String();
    return ct::String(mParameters[static_cast<std::size_t>(parameter)].def->key);
}

int PrimitiveParameterForm::fieldCount(int parameter) const
{
    if (parameter < 0 || parameter >= parameterCount()) return 0;
    return mParameters[static_cast<std::size_t>(parameter)].count;
}

NumberField* PrimitiveParameterForm::field(int parameter, int component) const
{
    if (component < 0 || component >= fieldCount(parameter)) return nullptr;
    return mParameters[static_cast<std::size_t>(parameter)].fields[component];
}

RevertButton* PrimitiveParameterForm::parameterRevert(int parameter) const
{
    if (parameter < 0 || parameter >= parameterCount()) return nullptr;
    return mParameters[static_cast<std::size_t>(parameter)].revert;
}

CommitInput* PrimitiveParameterForm::heightmapInput() const
{
    return mHeightmap;
}

rt::Button* PrimitiveParameterForm::heightmapBrowseButton() const
{
    return mHeightmapBrowse;
}

} // namespace prisma3d
