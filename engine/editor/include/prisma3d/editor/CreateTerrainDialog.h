#pragma once

#include "prisma3d/editor/NumberField.h"
#include "prisma3d/editor/PropertyWidgets.h"

#include <igui/widgets/BasicWidgets.hpp>
#include <igui/widgets/ComboBox.hpp>
#include <igui/widgets/Signal.hpp>
#include <igui/widgets/TextInputWidgets.hpp>
#include <igui/widgets/ViewWidgets.hpp>

#include <ct/ptr.hpp>
#include <ct/string.hpp>

#include <cstdint>

namespace prisma3d
{

class EditorSettings;
class Project;

struct TerrainCreateSettings
{
    std::uint16_t size = 513;
    float worldSize = 1000.0f;
    ct::String heightmap;
    float heightScale = 100.0f;
    float flatHeight = 0.0f;
};

class CreateTerrainDialog : public ig::retained::FloatWindow
{
public:
    IG_WIDGET_CLASS(CreateTerrainDialog, ig::retained::FloatWindow);

    static constexpr const char* kTitle = "Create Terrain";
    static constexpr const char* kDefaultName = "Terrain";
    static constexpr const char* kCreateText = "Create";
    static constexpr const char* kCancelText = "Cancel";
    static constexpr const char* kHeightmapFilter = "*.png;*.jpg;*.jpeg;*.tga;*.bmp;*.raw;*.r16";
    static constexpr int kSizeCount = 4;

    CreateTerrainDialog(const TerrainCreateSettings& initial, const Project* project,
            EditorSettings* settings);
    ~CreateTerrainDialog() override;

    static std::uint16_t sizeOption(int index);
    static int sizeIndex(std::uint16_t size);

    ct::String nameText() const;
    TerrainCreateSettings settings() const;
    void setHeightmap(const ct::String& path);
    const ct::String& error() const;

    void accept();
    void cancel();

    ig::retained::TextInput* nameInput() const;
    ig::retained::ComboBox* sizeBox() const;
    NumberField* worldSizeField() const;
    CommitInput* heightmapInput() const;
    NumberField* heightScaleField() const;
    NumberField* flatHeightField() const;
    ig::retained::Button* createButton() const;
    ig::retained::Button* cancelButton() const;

    ig::retained::Signal<const ct::String&, const TerrainCreateSettings&> created;

    void onKeyPress(ig::retained::KeyEvent& event) override;
    ig::retained::Widget* focusOnOpen() override;

private:
    struct Link
    {
        CreateTerrainDialog* owner;
    };

    NumberField* addNumberRow(ig::retained::BoxLayout* parent, const char* label,
            const char* tooltip, float value, double minimum, double maximum);
    void browse();
    void heightmapChosen(const ct::String& path);
    void syncRows();
    void close();

    ct::Rc<Link> mLink;
    const Project* mProject;
    EditorSettings* mSettings;
    ig::retained::TextInput* mName;
    ig::retained::ComboBox* mSize;
    NumberField* mWorldSize;
    CommitInput* mHeightmap;
    ig::retained::Button* mBrowse;
    NumberField* mHeightScale;
    NumberField* mFlatHeight;
    ig::retained::Label* mError;
    ig::retained::Button* mCreate;
    ig::retained::Button* mCancel;
    ct::String mErrorText;
    bool mClosed;
};

} // namespace prisma3d
