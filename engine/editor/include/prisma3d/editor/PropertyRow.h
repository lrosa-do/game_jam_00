#pragma once

#include "prisma3d/editor/AnimationLibraryEditor.h"
#include "prisma3d/editor/MeshPropertyEditor.h"
#include "prisma3d/editor/NumberField.h"
#include "prisma3d/editor/PropertyWidgets.h"
#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/PropertyValue.h"

#include <igui/widgets/BasicWidgets.hpp>
#include <igui/widgets/ComboBox.hpp>
#include <igui/widgets/InputWidgets.hpp>
#include <igui/widgets/LayoutWidgets.hpp>
#include <igui/widgets/Signal.hpp>
#include <igui/widgets/TextInputWidgets.hpp>
#include <igui/widgets/TreePropertyColorWidgets.hpp>

#include <ct/string.hpp>
#include <ct/vector.hpp>

#include <cstdint>

namespace prisma3d
{

class PropertyRow : public ig::retained::BoxLayout
{
public:
    IG_WIDGET_CLASS(PropertyRow, ig::retained::BoxLayout);

    enum class Kind : std::uint8_t
    {
        None,
        Bool,
        Int,
        Float,
        Enum,
        Flags,
        Layers,
        Vec2,
        Vec3,
        Vec4,
        Quat,
        Color,
        String,
        NodePath,
        ResourcePath,
        TextEnum,
        Mesh,
        Material,
        AnimationLibrary
    };

    static constexpr float kRowHeight = 26.0f;
    static constexpr double kDefaultStep = 0.001;
    static constexpr float kFloatDragSpeed = 5.0f;
    static constexpr float kIntegerDragSpeed = 0.5f;
    static constexpr int kRenderLayers = 20;
    static constexpr int kPhysicsLayers = 32;
    static constexpr int kRenderGroupSize = 5;
    static constexpr int kPhysicsGroupSize = 4;

    PropertyRow(const PropertyInfo& info, const ct::String& label);
    ~PropertyRow() override;

    static bool filterMatches(const ct::String& filter, const ct::String& path);
    bool acceptsDrop(const ig::retained::DragPayload& payload) override;
    void onDropReceive(const ig::retained::DragPayload& payload) override;

    static Kind kindOf(const PropertyInfo& info);
    static bool isWide(const PropertyInfo& info);
    static ct::String fileFilter(const PropertyInfo& info);
    static bool hasResourceType(const PropertyInfo& info, const char* type);
    static ct::String materialDisplayName(const ct::String& path);
    static constexpr const char* kEmptyMaterial = "<empty>";

    const PropertyInfo& info() const;
    Kind kind() const;
    const ct::String& label() const;

    const PropertyValue& value() const;
    void setValue(const PropertyValue& value);
    PropertyValue constrained(const PropertyValue& value) const;

    bool isActive() const;
    void setRevertible(bool revertible);
    bool revertible() const;

    int componentCount() const;
    NumberField* field(int index) const;
    ig::retained::CheckBox* check() const;
    ig::retained::ComboBox* combo() const;
    ig::retained::TextInput* text() const;
    ColorSwatch* swatch() const;
    ig::retained::Button* browseButton() const;
    ig::retained::Button* nodeButton() const;
    LayerGrid* layerGrid() const;
    MeshPropertyEditor* meshEditor() const;
    AnimationLibraryEditor* animationEditor() const;
    ig::retained::Label* materialLabel() const;
    ig::retained::Button* materialButton() const;
    int textChoiceCount() const;
    const ct::String& textChoice(int index) const;
    RevertButton* revertButton() const;
    int boxCount() const;
    ig::retained::CheckBox* box(int index) const;
    int choiceCount() const;
    std::int64_t choiceValue(int index) const;

    ig::retained::Signal<const PropertyValue&> edited;
    ig::retained::Signal<> revertRequested;
    ig::retained::Signal<> browseRequested;
    ig::retained::Signal<> materialPickRequested;
    ig::retained::Signal<> nodePickRequested;
    ig::retained::Signal<> animationImportRequested;
    ig::retained::Signal<const ct::String&> animationDropped;
    ig::retained::Signal<const ct::String&> animationRemoveRequested;
    ig::retained::Signal<const ct::String&> animationPlayRequested;
    ig::retained::Signal<> animationStopRequested;
    ig::retained::Signal<const ct::String&, float> animationSeekRequested;
    ig::retained::Signal<> animationPauseRequested;
    ig::retained::Signal<> meshSaveRequested;
    ig::retained::Signal<int> meshRemoveRequested;
    ig::retained::Signal<int> meshExtractRequested;

private:
    void build();
    void buildNumbers(ig::retained::Widget* parent, int count);
    void buildEnum(ig::retained::Widget* parent);
    void buildFlags(ig::retained::Widget* parent);
    void buildLayers(ig::retained::Widget* parent);
    void buildText(ig::retained::Widget* parent);
    void buildTextEnum(ig::retained::Widget* parent);
    void buildMesh(ig::retained::Widget* parent);
    void buildMaterial(ig::retained::Widget* parent);
    void buildAnimations(ig::retained::Widget* parent);

    void numberEdited(int index, float shown);
    void boxToggled(int index, bool checked);
    void layerChanged(std::int64_t bits);
    void choiceSelected(int index);
    void textChoiceSelected(int index);
    void colorEdited(const Math::Vec4& color);
    void textEdited(const ct::String& text);
    void commit(const PropertyValue& value);

    float toShown(double raw) const;
    double toRaw(float shown) const;
    double snap(double value) const;
    double constrain(double value) const;
    float dragSpeed() const;
    bool isIntegral() const;

    PropertyInfo mInfo;
    ct::String mLabel;
    Kind mKind;
    PropertyValue mValue;
    RevertButton* mRevert;
    NumberField* mFields[4];
    int mFieldCount;
    ig::retained::CheckBox* mCheck;
    ig::retained::ComboBox* mCombo;
    CommitInput* mText;
    ColorSwatch* mSwatch;
    ig::retained::Button* mBrowse;
    ig::retained::Button* mNodePick;
    LayerGrid* mGrid;
    MeshPropertyEditor* mMesh;
    AnimationLibraryEditor* mAnimations;
    ig::retained::Label* mMaterialName;
    ig::retained::Button* mMaterialButton;
    ct::Vector<ct::String> mTextChoices;
    ct::Vector<ig::retained::CheckBox*> mBoxes;
    ct::Vector<std::int64_t> mBoxValues;
    ct::Vector<std::int64_t> mChoices;
    Math::Vec3 mEuler;
    bool mSyncing;
};

} // namespace prisma3d
