#include "prisma3d/editor/MeshPropertyEditor.h"

#include "prisma3d/editor/EditorIcons.h"
#include "prisma3d/editor/PrimitiveParameterForm.h"
#include "prisma3d/editor/PropertyNames.h"

#include <stdio.h>

namespace prisma3d
{

namespace rt = ig::retained;

namespace
{

const char* const kTimes = " \xC3\x97 ";
const float kButtonWidth = 30.0f;

ct::String number(float value)
{
    char buffer[32];
    snprintf(buffer, sizeof buffer, "%.6g", static_cast<double>(value));
    return ct::String(buffer);
}

ct::String lastComponent(const ct::String& path)
{
    const std::size_t slash = path.rfind('/');
    return slash == ct::String::npos ? path : path.substr(slash + 1);
}

ct::String describe(const PrimitiveRecipe& recipe)
{
    const float* v = recipe.values;
    ct::String text = MeshPropertyEditor::typeName(static_cast<int>(recipe.kind));
    text += ' ';
    switch (recipe.kind)
    {
        case PrimitiveKind::Box:
            text += number(v[0]) + kTimes + number(v[1]) + kTimes + number(v[2]);
            break;
        case PrimitiveKind::Plane:
            text += number(v[0]) + kTimes + number(v[1]) + ", " + number(v[2]) + kTimes +
                    number(v[3]) + " segments";
            break;
        case PrimitiveKind::Sphere:
            text += "r " + number(v[0]) + ", " + number(v[1]) + " rings, " + number(v[2]) +
                    " slices";
            break;
        case PrimitiveKind::Cylinder:
        case PrimitiveKind::Cone:
            text += "r " + number(v[0]) + ", h " + number(v[1]) + ", " + number(v[2]) + " slices";
            break;
        case PrimitiveKind::Capsule:
            text += "r " + number(v[0]) + ", h " + number(v[1]) + ", " + number(v[2]) + " rings, " +
                    number(v[3]) + " slices";
            break;
        case PrimitiveKind::Torus:
            text += "R " + number(v[0]) + ", r " + number(v[1]) + ", " + number(v[2]) + kTimes +
                    number(v[3]) + " segments";
            break;
        case PrimitiveKind::HillsPlane:
            text += number(v[0]) + kTimes + number(v[1]) + ", " + number(v[2]) + kTimes +
                    number(v[3]) + " segments, height " + number(v[4]) + ", " +
                    lastComponent(recipe.heightmap);
            break;
        case PrimitiveKind::Heightfield:
            text += "cell " + number(v[0]) + ", height " + number(v[1]) + ", " +
                    lastComponent(recipe.heightmap);
            break;
    }
    return text;
}

} // namespace

MeshPropertyEditor::MeshPropertyEditor()
        : rt::BoxLayout(rt::LayoutDir::Vertical),
          mSummary(nullptr),
          mStats(nullptr),
          mSurfaceList(nullptr),
          mSave(nullptr),
          mEditable(false)
{
    setPadding(0.0f);
    setSpacing(PrimitiveParameterForm::kRowSpacing);
    mSummary = createChild<rt::Label>(ct::String(kNoMeshText));
    mStats = createChild<rt::Label>(ct::String());
    mStats->setVisible(false);
    mSurfaceList = createChild<rt::BoxLayout>(rt::LayoutDir::Vertical);
    mSurfaceList->setPadding(0.0f);
    mSurfaceList->setSpacing(PrimitiveParameterForm::kRowSpacing);
    mSurfaceList->setVisible(false);
    mSave = createChild<rt::Button>(kSaveText);
    mSave->setIconId(EditorIcons::get("Save"));
    mSave->setTooltip("Write this mesh to a new .pmesh file and use it on this node");
    mSave->setVisible(false);
    mSave->clicked.connect([this] { saveRequested.emit(); });
}

int MeshPropertyEditor::typeCount()
{
    return static_cast<int>(primitiveKindCount());
}

ct::String MeshPropertyEditor::typeName(int index)
{
    return capitalizePropertyName(ct::String(primitiveKindName(static_cast<PrimitiveKind>(index))));
}

ct::String MeshPropertyEditor::partOf(const ct::String& path)
{
    const std::size_t at = path.find(kPartSeparator);
    return at == ct::String::npos ? ct::String() : path.substr(at + 2);
}

ct::String MeshPropertyEditor::fileOf(const ct::String& path)
{
    const std::size_t at = path.find(kPartSeparator);
    return at == ct::String::npos ? path : path.substr(0, at);
}

ct::String MeshPropertyEditor::summaryOf(const ct::String& path)
{
    if (path.empty()) return ct::String(kNoMeshText);
    PrimitiveRecipe recipe;
    ct::String error;
    if (isPrimitivePath(path) && parsePrimitiveRecipe(path, recipe, error)) return describe(recipe);
    const ct::String part = partOf(path);
    ct::String text = part.empty() ? ct::String("Mesh") : part;
    text += " (";
    text += lastComponent(fileOf(path));
    text += ')';
    return text;
}

ct::String MeshPropertyEditor::statsOf(std::uint32_t surfaces, std::uint32_t triangles)
{
    ct::String text;
    text.append_number(static_cast<unsigned long long>(surfaces));
    text += surfaces == 1 ? " surface, " : " surfaces, ";
    text.append_number(static_cast<unsigned long long>(triangles));
    text += triangles == 1 ? " triangle" : " triangles";
    return text;
}

const ct::String& MeshPropertyEditor::path() const
{
    return mPath;
}

MeshPropertyEditor::Mode MeshPropertyEditor::mode() const
{
    if (mPath.empty()) return Mode::None;
    PrimitiveRecipe recipe;
    ct::String error;
    if (isPrimitivePath(mPath) && parsePrimitiveRecipe(mPath, recipe, error))
        return Mode::Primitive;
    return Mode::Model;
}

void MeshPropertyEditor::setPath(const ct::String& path)
{
    mPath = path;
    mSummary->setText(summaryOf(mPath));
    mSummary->setTooltip(mPath);
    if (mPath.empty()) clearStats();
}

void MeshPropertyEditor::setStats(std::uint32_t surfaces, std::uint32_t triangles)
{
    mStats->setText(statsOf(surfaces, triangles));
    if (!mStats->isVisible())
    {
        mStats->setVisible(true);
        markDirty();
    }
}

void MeshPropertyEditor::clearStats()
{
    if (mStats->isVisible())
    {
        mStats->setVisible(false);
        markDirty();
    }
}

void MeshPropertyEditor::setSurfaces(const ct::Vector<Surface>& surfaces, bool editable)
{
    bool same = editable == mEditable && surfaces.size() == mSurfaces.size();
    for (std::size_t i = 0; same && i < surfaces.size(); ++i)
        same = surfaces[i].material == mSurfaces[i].material &&
               surfaces[i].triangles == mSurfaces[i].triangles;
    if (same && mSurfaceList->isVisible()) return;
    mSurfaces = surfaces;
    mEditable = editable;
    rebuildSurfaces();
}

void MeshPropertyEditor::clearSurfaces()
{
    if (mSurfaces.empty() && !mSurfaceList->isVisible() && !mSave->isVisible()) return;
    mSurfaces.clear();
    mEditable = false;
    rebuildSurfaces();
}

void MeshPropertyEditor::rebuildSurfaces()
{
    for (std::size_t i = 0; i < mSurfaceRows.size(); ++i)
        mSurfaceList->removeChild(mSurfaceRows[i]);
    mSurfaceRows.clear();
    mSurfaceLabels.clear();
    for (std::size_t i = 0; i < mSurfaces.size(); ++i)
    {
        ct::String text;
        text.append_number(static_cast<unsigned long long>(i));
        text += ": ";
        text += mSurfaces[i].material.empty() ? ct::String("no material") : mSurfaces[i].material;
        text += "  (";
        text.append_number(static_cast<unsigned long long>(mSurfaces[i].triangles));
        text += mSurfaces[i].triangles == 1 ? " triangle)" : " triangles)";
        rt::BoxLayout* row = mSurfaceList->createChild<rt::BoxLayout>(rt::LayoutDir::Horizontal);
        row->setPadding(0.0f);
        row->setSpacing(PrimitiveParameterForm::kRowSpacing);
        rt::Label* label = row->createChild<rt::Label>(text);
        label->setStretch(1.0f);
        if (mEditable)
        {
            const int index = static_cast<int>(i);
            rt::Button* extract = row->createChild<rt::Button>();
            extract->setIconId(EditorIcons::get("ReparentToNewNode"));
            extract->setSize(kButtonWidth, 0.0f);
            extract->setTooltip("Move this surface to a new node");
            extract->setEnabled(mSurfaces.size() > 1);
            extract->clicked.connect([this, index] { extractRequested.emit(index); });
            rt::Button* remove = row->createChild<rt::Button>();
            remove->setIconId(EditorIcons::get("Remove"));
            remove->setSize(kButtonWidth, 0.0f);
            remove->setTooltip("Delete this surface from the mesh");
            remove->setEnabled(mSurfaces.size() > 1);
            remove->clicked.connect([this, index] { removeRequested.emit(index); });
        }
        mSurfaceRows.push_back(row);
        mSurfaceLabels.push_back(label);
    }
    mSurfaceList->setVisible(!mSurfaces.empty());
    mSave->setVisible(mEditable);
    markDirty();
}

std::size_t MeshPropertyEditor::surfaceCount() const
{
    return mSurfaces.size();
}

rt::Label* MeshPropertyEditor::surfaceLabel(std::size_t index) const
{
    return index < mSurfaceLabels.size() ? mSurfaceLabels[index] : nullptr;
}

rt::Button* MeshPropertyEditor::saveButton() const
{
    return mSave;
}

rt::Label* MeshPropertyEditor::summaryLabel() const
{
    return mSummary;
}

rt::Label* MeshPropertyEditor::statsLabel() const
{
    return mStats;
}

} // namespace prisma3d
