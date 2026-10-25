#pragma once

#include "prisma3d/editor/NumberField.h"
#include "prisma3d/scene/PrimitiveMeshes.h"

#include <igui/widgets/BasicWidgets.hpp>
#include <igui/widgets/LayoutWidgets.hpp>
#include <igui/widgets/Signal.hpp>

#include <ct/string.hpp>
#include <ct/vector.hpp>

#include <cstdint>

namespace prisma3d
{

class MeshPropertyEditor : public ig::retained::BoxLayout
{
public:
    IG_WIDGET_CLASS(MeshPropertyEditor, ig::retained::BoxLayout);

    enum class Mode : std::uint8_t
    {
        None,
        Model,
        Primitive
    };

    static constexpr const char* kPartSeparator = "::";
    static constexpr const char* kNoMeshText = "No mesh";
    static constexpr const char* kSaveText = "Save as .pmesh";

    struct Surface
    {
        ct::String material;
        std::uint32_t triangles = 0;
    };

    MeshPropertyEditor();

    static int typeCount();
    static ct::String typeName(int index);
    static ct::String partOf(const ct::String& path);
    static ct::String fileOf(const ct::String& path);
    static ct::String summaryOf(const ct::String& path);
    static ct::String statsOf(std::uint32_t surfaces, std::uint32_t triangles);

    void setPath(const ct::String& path);
    const ct::String& path() const;
    Mode mode() const;
    void setStats(std::uint32_t surfaces, std::uint32_t triangles);
    void clearStats();
    void setSurfaces(const ct::Vector<Surface>& surfaces, bool editable);
    void clearSurfaces();
    std::size_t surfaceCount() const;

    ig::retained::Label* summaryLabel() const;
    ig::retained::Label* statsLabel() const;
    ig::retained::Label* surfaceLabel(std::size_t index) const;
    ig::retained::Button* saveButton() const;

    ig::retained::Signal<> saveRequested;
    ig::retained::Signal<int> removeRequested;
    ig::retained::Signal<int> extractRequested;

private:
    void rebuildSurfaces();

    ig::retained::Label* mSummary;
    ig::retained::Label* mStats;
    ig::retained::BoxLayout* mSurfaceList;
    ig::retained::Button* mSave;
    ct::Vector<Surface> mSurfaces;
    ct::Vector<ig::retained::Label*> mSurfaceLabels;
    ct::Vector<ig::retained::Widget*> mSurfaceRows;
    ct::String mPath;
    bool mEditable;
};

} // namespace prisma3d
