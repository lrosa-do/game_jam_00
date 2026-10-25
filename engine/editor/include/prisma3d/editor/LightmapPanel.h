#pragma once

#include "prisma3d/editor/LightmapBaker.h"
#include "prisma3d/editor/NumberField.h"

#include <igui/widgets/BasicWidgets.hpp>
#include <igui/widgets/InputWidgets.hpp>
#include <igui/widgets/LayoutWidgets.hpp>
#include <igui/widgets/ScrollWidgets.hpp>

#include <ct/function.hpp>
#include <ct/ptr.hpp>
#include <ct/string.hpp>
#include <ct/vector.hpp>

#include <cstdint>

namespace prisma3d
{

class EditorDocument;
class EditorSelection;

class LightmapPanel
{
public:
    static constexpr const char* kTitle = "Lightmap";
    static constexpr const char* kNoNodeText = "No LightmapGI node in the scene.";
    static constexpr const char* kNoSunText = "No DirectionalLight3D in the scene.";

    enum Field : int
    {
        kResolution,
        kPadding,
        kShadowResolution,
        kSamples,
        kSunRadius,
        kFilterRadius,
        kBiasTexels,
        kAmbientGround,
        kFieldCount
    };

    LightmapPanel(LightmapBaker& baker, EditorDocument& document, EditorSelection& selection);
    ~LightmapPanel();
    LightmapPanel(const LightmapPanel&) = delete;
    LightmapPanel& operator=(const LightmapPanel&) = delete;

    bool attach(ig::retained::Widget* parent);
    bool attached() const;
    void update();

    void reveal();
    using RevealHandler = ct::Function<void()>;
    void setRevealHandler(const RevealHandler& handler);

    LightmapBaker& baker() const;
    bool createNode();
    bool setSunStatic();
    bool markSelectedStatic();
    bool unwrapMissing();
    bool unwrapRow(int row);
    bool bake();
    void cancel();
    void draftPreset();
    void finalPreset();

    int rowCount() const;
    ct::String rowName(int row) const;
    bool rowHasUv2(int row) const;
    ig::retained::Button* rowButton(int row) const;

    ig::retained::Button* nodeButton() const;
    ig::retained::Button* sunButton() const;
    ig::retained::Button* staticButton() const;
    ig::retained::Button* draftButton() const;
    ig::retained::Button* finalButton() const;
    ig::retained::Button* unwrapButton() const;
    ig::retained::Button* bakeButton() const;
    ig::retained::Button* cancelButton() const;
    ig::retained::Label* nodeLabel() const;
    ig::retained::Label* dataLabel() const;
    ig::retained::Label* sunLabel() const;
    ig::retained::Label* ambientLabel() const;
    ig::retained::Label* statusLabel() const;
    ig::retained::Label* summaryLabel() const;
    ig::retained::ProgressBar* progressBar() const;
    NumberField* field(Field which) const;
    ig::retained::ScrollView* scrollView() const;
    const ct::String& error() const;

private:
    struct Link
    {
        LightmapPanel* owner;
    };

    struct Row
    {
        ct::String path;
        ct::String name;
        bool uv2 = false;
        bool bakeable = true;
        ig::retained::Label* label = nullptr;
        ig::retained::Label* state = nullptr;
        ig::retained::Button* button = nullptr;
    };

    std::uint64_t rowSignature(const ct::Vector<MeshInstance3D*>& meshes) const;
    void rebuildRows(const ct::Vector<MeshInstance3D*>& meshes);
    void syncFields();
    void fieldChanged(Field which, float value);
    void fail(const ct::String& error);
    void clearError();

    LightmapBaker& mBaker;
    EditorDocument& mDocument;
    EditorSelection& mSelection;
    ct::Rc<Link> mLink;
    RevealHandler mReveal;
    ig::retained::ScrollView* mScroll;
    ig::retained::BoxLayout* mRoot;
    ig::retained::Label* mNodeLabel;
    ig::retained::Label* mDataLabel;
    ig::retained::Button* mNodeButton;
    ig::retained::Label* mSunLabel;
    ig::retained::Button* mSunButton;
    ig::retained::Label* mAmbientLabel;
    ig::retained::Label* mMeshesLabel;
    ig::retained::BoxLayout* mRows;
    ig::retained::Button* mStaticButton;
    ig::retained::Button* mDraft;
    ig::retained::Button* mFinal;
    NumberField* mFields[kFieldCount];
    ig::retained::Button* mUnwrap;
    ig::retained::Button* mBake;
    ig::retained::Button* mCancel;
    ig::retained::ProgressBar* mProgress;
    ig::retained::Label* mStatus;
    ig::retained::Label* mSummary;
    ct::Vector<Row> mRowList;
    std::uint64_t mSignature;
    ct::String mError;
    bool mSyncing;
};

} // namespace prisma3d
