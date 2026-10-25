#pragma once

#include "prisma3d/editor/NumberField.h"
#include "prisma3d/editor/PropertyWidgets.h"
#include "prisma3d/scene/PrimitiveMeshes.h"

#include <igui/widgets/BasicWidgets.hpp>
#include <igui/widgets/LayoutWidgets.hpp>
#include <igui/widgets/Signal.hpp>

#include <ct/string.hpp>
#include <ct/vector.hpp>

namespace prisma3d
{

class PrimitiveParameterForm : public ig::retained::BoxLayout
{
public:
    IG_WIDGET_CLASS(PrimitiveParameterForm, ig::retained::BoxLayout);

    struct Row
    {
        ig::retained::BoxLayout* row = nullptr;
        ig::retained::BoxLayout* editors = nullptr;
        RevertButton* revert = nullptr;
    };

    static constexpr float kRowSpacing = 4.0f;
    static constexpr float kBrowseWidth = 34.0f;

    PrimitiveParameterForm();

    static Row addRow(ig::retained::Widget* parent, const ct::String& label,
            const ct::String& tooltip);
    static ct::String parameterLabel(const char* key);

    void setRecipe(const PrimitiveRecipe& recipe);
    void setKind(PrimitiveKind kind);
    PrimitiveKind kind() const;
    const PrimitiveRecipe& recipe() const;
    bool complete() const;
    ct::String path() const;

    void setHeightmapFile(const ct::String& file);
    void cancelEditing();
    bool isActive() const;

    int parameterCount() const;
    ct::String parameterKey(int parameter) const;
    int fieldCount(int parameter) const;
    NumberField* field(int parameter, int component) const;
    RevertButton* parameterRevert(int parameter) const;
    CommitInput* heightmapInput() const;
    ig::retained::Button* heightmapBrowseButton() const;

    ig::retained::Signal<> changed;
    ig::retained::Signal<> editFinished;
    ig::retained::Signal<> browseHeightmapRequested;

    void layout() override;

private:
    struct Parameter
    {
        const PrimitiveParamDef* def = nullptr;
        ig::retained::BoxLayout* row = nullptr;
        RevertButton* revert = nullptr;
        NumberField* fields[3] = {};
        int count = 0;
        int valueIndex = 0;
    };

    void parameterEdited(int parameter, int component, float shown);
    void parameterReverted(int parameter);
    void heightmapCommitted(const ct::String& text);
    void emitChanged();
    void clearContent();
    void buildContent();
    void buildHeightmapRow();
    void refreshValues();
    void refreshRevert(int parameter);
    double constrain(const PrimitiveParamDef& def, double value) const;

    ig::retained::BoxLayout* mHeightmapRow;
    CommitInput* mHeightmap;
    ig::retained::Button* mHeightmapBrowse;
    ct::Vector<Parameter> mParameters;
    PrimitiveRecipe mRecipe;
    bool mRebuildPending;
    bool mSyncing;
    int mEmitting;
};

} // namespace prisma3d
