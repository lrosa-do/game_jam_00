#include "prisma3d/editor/LightmapBaker.h"

#include "prisma3d/editor/AssetImport.h"
#include "prisma3d/editor/EditorDocument.h"
#include "prisma3d/editor/EditorSelection.h"
#include "prisma3d/editor/EditorTreeEdits.h"
#include "prisma3d/editor/HierarchyActions.h"
#include "prisma3d/editor/LightmapMeshIO.h"
#include "prisma3d/main/Project.h"
#include "prisma3d/render/MeshStore.h"
#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/SceneResources.h"
#include "prisma3d/scene/WorldEnvironment.h"

#include "platform.h"

#include <math.h>
#include <string.h>

namespace prisma3d
{

namespace
{

const char* const kClearColorSetting = "rendering/environment/defaults/default_clear_color";
const char* const kDraftName = "Draft";
const char* const kFinalName = "Final";

float srgbToLinear(float value)
{
    if (value <= 0.04045f) return value / 12.92f;
    return powf((value + 0.055f) / 1.055f, 2.4f);
}

Math::Vec3 srgbToLinear(const Math::Vec3& color)
{
    return Math::Vec3(srgbToLinear(color.x), srgbToLinear(color.y), srgbToLinear(color.z));
}

Math::Vec3 scaleOf(const Math::Mat4& transform)
{
    return Math::Vec3(Math::Vec3(transform.col0.x, transform.col0.y, transform.col0.z).Length(),
            Math::Vec3(transform.col1.x, transform.col1.y, transform.col1.z).Length(),
            Math::Vec3(transform.col2.x, transform.col2.y, transform.col2.z).Length());
}

Node* findEnvironmentNode(Node* node)
{
    if (!node) return nullptr;
    if (node->castTo<WorldEnvironment>()) return node;
    for (std::size_t i = 0; i < node->getChildCount(); ++i)
        if (Node* found = findEnvironmentNode(node->getChild(i))) return found;
    return nullptr;
}

} // namespace

LightmapBaker::LightmapBaker(EditorDocument& document, EditorSelection& selection,
        HierarchyActions& actions)
        : mDocument(document),
          mSelection(selection),
          mActions(actions),
          mState(State::Idle),
          mCancelRequested(false),
          mThenBake(false),
          mBakeIndex(0),
          mLayerResolution(0),
          mSunDirection(0.0f, -1.0f, 0.0f),
          mSunColor(1.0f, 1.0f, 1.0f),
          mUnwrapTotal(0),
          mUnwrapDone(0),
          mStartNanos(0)
{
    mSettings.bake.shadowResolution = 2048;
    mSettings.bake.sampleCount = 4;
    mSettings.bake.sunAngularRadius = 2.0f;
}

LightmapBaker::~LightmapBaker()
{
    mJob.cancel();
}

LightmapBaker::Settings& LightmapBaker::settings()
{
    return mSettings;
}

const LightmapBaker::Settings& LightmapBaker::settings() const
{
    return mSettings;
}

LightmapBaker::State LightmapBaker::state() const
{
    return mState;
}

bool LightmapBaker::busy() const
{
    return mState != State::Idle;
}

const ct::String& LightmapBaker::message() const
{
    return mMessage;
}

const LightmapBaker::Summary& LightmapBaker::summary() const
{
    return mSummary;
}

const LightmapUnwrapResult& LightmapBaker::lastAtlas() const
{
    return mAtlas;
}

Node* LightmapBaker::resolve(const ct::String& path) const
{
    Node* scene = mDocument.scene();
    return scene ? scene->getNode(path) : nullptr;
}

ct::String LightmapBaker::relativePath(const ct::String& absolute) const
{
    ct::String relative;
    if (mDocument.project().relativePath(absolute, relative)) return relative;
    return absolute;
}

ct::String LightmapBaker::sceneFolder() const
{
    if (!mDocument.hasScenePath()) return ct::String();
    return AssetImport::parentOf(mDocument.sceneAbsolutePath(mDocument.scenePath()));
}

ct::String LightmapBaker::sceneStem() const
{
    ct::String name = AssetImport::nameOf(mDocument.scenePath());
    if (name.ends_with(EditorDocument::kSceneSuffix))
        name = name.substr(0, name.size() - strlen(EditorDocument::kSceneSuffix));
    return name;
}

ct::String LightmapBaker::dataPath() const
{
    const ct::String folder = sceneFolder();
    if (folder.empty()) return ct::String();
    return AssetImport::joinPath(folder, sceneStem() + kDataExtension);
}

ct::String LightmapBaker::uniqueMeshPath(const ct::String& name) const
{
    const ct::String folder = AssetImport::joinPath(sceneFolder(), sceneStem() + kFolderSuffix);
    fs_create_directory(folder.c_str());
    ct::String clean;
    for (std::size_t i = 0; i < name.size(); ++i)
    {
        const char c = name[i];
        const bool plain = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
                           (c >= '0' && c <= '9') || c == '_' || c == '-';
        clean += plain ? c : '_';
    }
    if (clean.empty()) clean = "mesh";
    for (unsigned int n = 1;; ++n)
    {
        ct::String stem = clean;
        if (n > 1)
        {
            stem += "_";
            stem.append_number(n);
        }
        const ct::String path = AssetImport::joinPath(folder, stem + ".pmesh");
        if (!file_exists(path.c_str())) return path;
    }
}

LightmapGI* LightmapBaker::findLightmap(Node* node) const
{
    if (!node) return nullptr;
    if (LightmapGI* gi = node->castTo<LightmapGI>()) return gi;
    for (std::size_t i = 0; i < node->getChildCount(); ++i)
        if (LightmapGI* found = findLightmap(node->getChild(i))) return found;
    return nullptr;
}

LightmapGI* LightmapBaker::lightmapNode() const
{
    return findLightmap(mDocument.scene());
}

bool LightmapBaker::ensureLightmapNode(ct::String& error)
{
    error.clear();
    if (mDocument.playing())
    {
        error = "Stop the game before baking lightmaps.";
        return false;
    }
    if (LightmapGI* gi = lightmapNode())
    {
        mSelection.clear();
        mSelection.add(gi);
        return true;
    }
    Node* scene = mDocument.scene();
    if (!scene)
    {
        error = "There is no open scene.";
        return false;
    }
    if (!mActions.createNodeUnder(kLightmapNodeType, scene, error))
    {
        if (error.empty()) error = "The LightmapGI node could not be created.";
        return false;
    }
    return true;
}

DirectionalLight3D* LightmapBaker::findSun(Node* node, bool onlyStatic) const
{
    if (!node) return nullptr;
    if (DirectionalLight3D* light = node->castTo<DirectionalLight3D>())
        if (light->isVisibleInTree() &&
                (!onlyStatic || light->getBakeMode() == LightBakeMode::Static))
            return light;
    for (std::size_t i = 0; i < node->getChildCount(); ++i)
        if (DirectionalLight3D* found = findSun(node->getChild(i), onlyStatic)) return found;
    return nullptr;
}

DirectionalLight3D* LightmapBaker::sun() const
{
    if (DirectionalLight3D* light = findSun(mDocument.scene(), true)) return light;
    return findSun(mDocument.scene(), false);
}

bool LightmapBaker::sunIsStatic() const
{
    DirectionalLight3D* light = sun();
    return light && light->getBakeMode() == LightBakeMode::Static;
}

bool LightmapBaker::setSunStatic(ct::String& error)
{
    error.clear();
    DirectionalLight3D* light = sun();
    if (!light)
    {
        error = "There is no DirectionalLight3D in the scene.";
        return false;
    }
    PropertyValue before;
    NodeClassDB::getProperty(light, "light_bake_mode", before);
    EditorTreeEdits::setProperty(&mDocument.undoRedo(), kStaticAction, light, "light_bake_mode",
            before, PropertyValue::fromInt(static_cast<int>(LightBakeMode::Static)));
    return true;
}

void LightmapBaker::collectMeshes(Node* node, ct::Vector<MeshInstance3D*>& out) const
{
    if (!node) return;
    if (MeshInstance3D* mesh = node->castTo<MeshInstance3D>())
        if (mesh->getGiMode() == GiMode::Static && mesh->isVisibleInTree() &&
                mesh->getMesh().valid())
            out.push_back(mesh);
    for (std::size_t i = 0; i < node->getChildCount(); ++i) collectMeshes(node->getChild(i), out);
}

ct::Vector<MeshInstance3D*> LightmapBaker::meshes() const
{
    ct::Vector<MeshInstance3D*> out;
    collectMeshes(mDocument.scene(), out);
    return out;
}

bool LightmapBaker::markStatic(const ct::Vector<Node*>& nodes, ct::String& error)
{
    error.clear();
    EditorUndoRedoManager& undo = mDocument.undoRedo();
    bool any = false;
    for (std::size_t i = 0; i < nodes.size(); ++i)
    {
        VisualInstance3D* visual = nodes[i] ? nodes[i]->castTo<VisualInstance3D>() : nullptr;
        if (!visual || visual->getGiMode() == GiMode::Static) continue;
        if (!any) undo.createAction("Set GI Mode");
        any = true;
        PropertyValue before;
        NodeClassDB::getProperty(visual, "gi_mode", before);
        undo.addDoProperty(visual, "gi_mode",
                PropertyValue::fromInt(static_cast<int>(GiMode::Static)));
        undo.addUndoProperty(visual, "gi_mode", before);
    }
    if (any) undo.commitAction();
    else
        error = "Select a mesh that is not Static yet.";
    return any;
}

Math::Vec3 LightmapBaker::ambient() const
{
    LightmapGI* gi = lightmapNode();
    if (!gi) return Math::Vec3(0.0f, 0.0f, 0.0f);
    if (gi->getEnvironmentMode() == LightmapGI::EnvironmentMode::Disabled)
        return Math::Vec3(0.0f, 0.0f, 0.0f);
    if (gi->getEnvironmentMode() == LightmapGI::EnvironmentMode::CustomColor)
        return srgbToLinear(gi->getEnvironmentCustomColor()) * gi->getEnvironmentCustomEnergy();

    Math::Vec4 clear(0.3f, 0.3f, 0.3f, 1.0f);
    if (mDocument.project().isOpen()) clear = mDocument.project().get(kClearColorSetting).asColor();
    Math::Vec3 clearLinear = srgbToLinear(Math::Vec3(clear.x, clear.y, clear.z));

    Node* node = findEnvironmentNode(mDocument.scene());
    WorldEnvironment* environment = node ? node->castTo<WorldEnvironment>() : nullptr;
    if (!environment) return clearLinear;

    const EnvironmentSettings& env = environment->getEnvironment();
    if (env.background == BackgroundMode::Color) clearLinear = srgbToLinear(env.backgroundColor);
    const bool skyBackground = env.background == BackgroundMode::Sky;
    if (env.ambientSource == AmbientSource::Disabled) return Math::Vec3(0.0f, 0.0f, 0.0f);
    if (env.ambientSource == AmbientSource::Color)
        return srgbToLinear(env.ambientColor) * env.ambientEnergy;
    if (env.ambientSource == AmbientSource::Background && !skyBackground)
        return clearLinear * env.backgroundEnergy;
    const Math::Vec3 sky = (env.sky.skyTopColor + env.sky.skyHorizonColor) * 0.5f;
    return srgbToLinear(sky) * env.sky.skyEnergy * env.ambientSkyContribution * env.ambientEnergy;
}

void LightmapBaker::applyPreset(bool draft)
{
    LightmapBakeSettings& bake = mSettings.bake;
    if (draft)
    {
        mSettings.resolution = 1024;
        bake.shadowResolution = 2048;
        bake.sampleCount = 4;
        bake.filterRadius = 1.0f;
    }
    else
    {
        mSettings.resolution = 4096;
        bake.shadowResolution = 8192;
        bake.sampleCount = 16;
        bake.filterRadius = 2.0f;
    }
    bake.sunAngularRadius = 2.0f;
    bake.ambientGround = 0.35f;
    bake.biasTexels = 3.0f;
    bake.bias = 0.0f;
    mSettings.padding = 8;

    LightmapGI* gi = lightmapNode();
    if (!gi) return;
    EditorUndoRedoManager& undo = mDocument.undoRedo();
    PropertyValue quality;
    PropertyValue size;
    PropertyValue denoiser;
    PropertyValue scale;
    NodeClassDB::getProperty(gi, "quality", quality);
    NodeClassDB::getProperty(gi, "max_texture_size", size);
    NodeClassDB::getProperty(gi, "use_denoiser", denoiser);
    NodeClassDB::getProperty(gi, "texel_scale", scale);
    undo.createAction(draft ? kDraftName : kFinalName);
    undo.addDoProperty(gi, "quality",
            PropertyValue::fromInt(static_cast<int>(
                    draft ? LightmapGI::BakeQuality::Low : LightmapGI::BakeQuality::High)));
    undo.addUndoProperty(gi, "quality", quality);
    undo.addDoProperty(gi, "max_texture_size", PropertyValue::fromInt(draft ? 2048 : 4096));
    undo.addUndoProperty(gi, "max_texture_size", size);
    undo.addDoProperty(gi, "use_denoiser", PropertyValue::fromBool(false));
    undo.addUndoProperty(gi, "use_denoiser", denoiser);
    undo.addDoProperty(gi, "texel_scale", PropertyValue::fromSingle(1.0f));
    undo.addUndoProperty(gi, "texel_scale", scale);
    undo.commitAction();
}

LightmapBakeSettings LightmapBaker::effectiveSettings() const
{
    LightmapBakeSettings bake = mSettings.bake;
    bake.ambient = ambient();
    if (DirectionalLight3D* light = sun())
    {
        const float half = light->getAngularDistance() * 0.5f;
        if (half > bake.sunAngularRadius) bake.sunAngularRadius = half;
    }
    if (bake.sunAngularRadius > 0.0f && bake.sampleCount < 2) bake.sampleCount = 2;
    return bake;
}

bool LightmapBaker::prepare(ct::String& error) const
{
    if (mDocument.playing())
    {
        error = "Stop the game before baking lightmaps.";
        return false;
    }
    if (!mDocument.scene())
    {
        error = "There is no open scene.";
        return false;
    }
    if (!mDocument.project().isOpen() || !mDocument.hasScenePath())
    {
        error = "Save the scene inside a project first: the lightmap is stored next to it.";
        return false;
    }
    if (!lightmapNode())
    {
        error = "Add a LightmapGI node first (Scene > Bake Lightmaps...).";
        return false;
    }
    if (!sun())
    {
        error = "The scene has no DirectionalLight3D to bake.";
        return false;
    }
    if (!sunIsStatic())
    {
        error = "Set the Bake Mode of the sun to Static first.";
        return false;
    }
    return true;
}

bool LightmapBaker::beginUnwrap(const ct::String& path, ct::String& error)
{
    Node* node = resolve(path);
    MeshInstance3D* mesh = node ? node->castTo<MeshInstance3D>() : nullptr;
    if (!mesh || !mesh->getMesh().valid())
    {
        error = ct::String("The mesh '") + path + "' is gone.";
        return false;
    }
    if (!LightmapMeshIO::isStaticTriangleMesh(mesh->getMesh()))
    {
        error = ct::String("'") + mesh->getName() + "' is not a static triangle mesh.";
        return false;
    }
    if (sceneres::pathOf(mesh->getMesh()).empty())
    {
        error = ct::String("'") + mesh->getName() +
                "' has no resource path: save its mesh as a .pmesh first.";
        return false;
    }
    ct::Vector<SurfaceData> surfaces;
    if (!LightmapMeshIO::readSurfaces(mesh->getMesh(), surfaces, error)) return false;

    LightmapUnwrapSettings unwrap;
    unwrap.scale = scaleOf(mesh->getGlobalTransform());
    unwrap.resolution = 0;
    unwrap.padding = mSettings.padding;
    const float area = LightmapUnwrapper::worldSurfaceArea(surfaces, unwrap.scale);
    float texelScale = mesh->getLightmapTexelScale();
    if (LightmapGI* gi = lightmapNode()) texelScale *= gi->getTexelScale();
    unwrap.texelsPerUnit =
            LightmapUnwrapper::fittedTexelsPerUnit(area, mSettings.resolution) * texelScale;
    if (unwrap.texelsPerUnit <= 0.0f)
    {
        error = ct::String("'") + mesh->getName() + "' has no surface area.";
        return false;
    }
    if (!mJob.start(surfaces, unwrap))
    {
        error = "The previous unwrap is still stopping.";
        return false;
    }
    mUnwrapping = path;
    mMessage = ct::String("Unwrapping ") + mesh->getName();
    return true;
}

bool LightmapBaker::applyUnwrap(const ct::String& path, const ct::Vector<SurfaceData>& surfaces,
        ct::String& error)
{
    Node* node = resolve(path);
    MeshInstance3D* mesh = node ? node->castTo<MeshInstance3D>() : nullptr;
    if (!mesh)
    {
        error = ct::String("The mesh '") + path + "' is gone.";
        return false;
    }
    const ct::String absolute = uniqueMeshPath(mesh->getName());
    if (!LightmapMeshIO::saveMesh(mesh->getMesh(), surfaces, mesh->getName(), absolute, error))
        return false;
    PropertyValue before;
    NodeClassDB::getProperty(mesh, "mesh", before);
    const ct::String relative = relativePath(absolute);
    EditorTreeEdits::setProperty(&mDocument.undoRedo(), kUnwrapAction, mesh, "mesh", before,
            PropertyValue::fromResourcePath(relative));
    if (!LightmapMeshIO::hasUv2(mesh->getMesh()))
    {
        error = ct::String("The unwrapped mesh '") + relative + "' could not be loaded back.";
        return false;
    }
    log_info("lightmap: unwrapped '%s' into %s", mesh->getName().c_str(), relative.c_str());
    return true;
}

bool LightmapBaker::startUnwrap(const ct::Vector<MeshInstance3D*>& nodes, ct::String& error)
{
    error.clear();
    if (busy())
    {
        error = "A lightmap job is already running.";
        return false;
    }
    if (mDocument.playing())
    {
        error = "Stop the game before unwrapping.";
        return false;
    }
    if (!mDocument.project().isOpen() || !mDocument.hasScenePath())
    {
        error = "Save the scene inside a project first: the unwrapped meshes are stored next to "
                "it.";
        return false;
    }
    Node* scene = mDocument.scene();
    if (!scene) return false;
    reset();
    for (std::size_t i = 0; i < nodes.size(); ++i)
        if (nodes[i]) mUnwrapQueue.push_back(scene->getPathTo(nodes[i]));
    if (mUnwrapQueue.empty())
    {
        error = "There is nothing to unwrap.";
        return false;
    }
    mUnwrapTotal = static_cast<std::uint32_t>(mUnwrapQueue.size());
    mUnwrapDone = 0;
    mThenBake = false;
    mStartNanos = time_nanos();
    mState = State::Unwrapping;
    if (!beginUnwrap(mUnwrapQueue[0], error))
    {
        finish(false, error);
        return false;
    }
    return true;
}

bool LightmapBaker::startBake(ct::String& error)
{
    error.clear();
    if (busy())
    {
        error = "A lightmap job is already running.";
        return false;
    }
    if (!lightmapNode() && !ensureLightmapNode(error)) return false;
    if (sun() && !sunIsStatic())
    {
        if (!setSunStatic(error)) return false;
        log_info("lightmap: Bake Mode of '%s' set to Static for the bake",
                sun()->getName().c_str());
    }
    if (!prepare(error)) return false;
    const ct::Vector<MeshInstance3D*> list = meshes();
    if (list.empty())
    {
        error = "There are no visible MeshInstance3D nodes with GI Mode Static.";
        return false;
    }
    Node* scene = mDocument.scene();
    reset();
    for (std::size_t i = 0; i < list.size(); ++i)
    {
        if (!LightmapMeshIO::isStaticTriangleMesh(list[i]->getMesh())) continue;
        BakeItem item;
        item.path = scene->getPathTo(list[i]);
        mBakeItems.push_back(item);
        if (!LightmapMeshIO::hasUv2(list[i]->getMesh())) mUnwrapQueue.push_back(item.path);
    }
    if (mBakeItems.empty())
    {
        error = "None of the Static meshes can be baked (skinned or not triangles).";
        return false;
    }
    mThenBake = true;
    mStartNanos = time_nanos();
    mUnwrapTotal = static_cast<std::uint32_t>(mUnwrapQueue.size());
    mUnwrapDone = 0;
    if (!mUnwrapQueue.empty())
    {
        mState = State::Unwrapping;
        if (!beginUnwrap(mUnwrapQueue[0], error))
        {
            finish(false, error);
            return false;
        }
        return true;
    }
    if (!beginBakeStage(error))
    {
        finish(false, error);
        return false;
    }
    return true;
}

bool LightmapBaker::beginBakeStage(ct::String& error)
{
    Node* scene = mDocument.scene();
    LightmapGI* gi = lightmapNode();
    DirectionalLight3D* light = sun();
    if (!scene || !gi || !light)
    {
        error = "The scene changed: the LightmapGI node or the sun is gone.";
        return false;
    }
    for (std::size_t i = 0; i < mBakeItems.size(); ++i)
    {
        Node* node = resolve(mBakeItems[i].path);
        MeshInstance3D* mesh = node ? node->castTo<MeshInstance3D>() : nullptr;
        const MeshInfo* info = mesh ? MeshStore::singleton()->get(mesh->getMesh()) : nullptr;
        if (!info || !LightmapMeshIO::hasUv2(mesh->getMesh()))
        {
            error = ct::String("The mesh '") + mBakeItems[i].path + "' has no second UV set.";
            return false;
        }
        mBakeItems[i].mesh = mesh->getMesh();
        mBakeItems[i].casts = mesh->getCastShadows();
        mBakeItems[i].transform = mesh->getGlobalTransform();
        mBakeItems[i].bounds = info->aabb.Transformed(mBakeItems[i].transform);
    }
    mCasters.clear();
    for (std::size_t i = 0; i < mBakeItems.size(); ++i)
    {
        if (!mBakeItems[i].casts) continue;
        LightmapBakePass::Caster caster;
        caster.mesh = mBakeItems[i].mesh;
        caster.transform = mBakeItems[i].transform;
        mCasters.push_back(caster);
    }
    const Math::Mat4 sunTransform = light->getGlobalTransform();
    mSunDirection = sunTransform.TransformVector(Math::Vec3(0.0f, 0.0f, -1.0f)).Normalized();
    mSunColor = srgbToLinear(light->getColor()) * light->getEnergy();
    mBakeSettings = effectiveSettings();

    std::uint32_t resolution = mSettings.resolution;
    const std::uint32_t limit = static_cast<std::uint32_t>(gi->getMaxTextureSize());
    if (resolution > limit) resolution = limit;
    if (resolution < 64) resolution = 64;
    if (resolution > 4096) resolution = 4096;
    mLayerResolution = resolution;
    mLayers.clear();
    mBakeIndex = 0;
    mMessage = "Baking";
    mState = State::Baking;
    return true;
}

void LightmapBaker::tickUnwrap()
{
    if (mCancelRequested)
    {
        mJob.cancel();
        if (mJob.running()) return;
        ct::Vector<SurfaceData> discard;
        bool ignored = false;
        mJob.collect(discard, ignored);
        finish(false, "Cancelled");
        return;
    }
    if (mUnwrapping.empty())
    {
        finish(false, "Unwrap state lost");
        return;
    }
    ct::Vector<SurfaceData> surfaces;
    bool succeeded = false;
    if (!mJob.collect(surfaces, succeeded))
    {
        Node* node = resolve(mUnwrapping);
        mMessage = ct::String("Unwrapping ") + (node ? node->getName() : mUnwrapping) + " ";
        mMessage.append_number(static_cast<unsigned long long>(mJob.percent()));
        mMessage += "%";
        return;
    }

    ct::String error;
    if (!succeeded)
    {
        error = mJob.error().empty() ? ct::String("The unwrap failed.") : mJob.error();
        finish(false, error);
        return;
    }
    mAtlas = mJob.result();
    if (!applyUnwrap(mUnwrapping, surfaces, error))
    {
        finish(false, error);
        return;
    }
    ++mUnwrapDone;
    mUnwrapQueue.erase(mUnwrapQueue.begin());
    mUnwrapping.clear();
    if (!mUnwrapQueue.empty())
    {
        if (!beginUnwrap(mUnwrapQueue[0], error)) finish(false, error);
        return;
    }
    if (!mThenBake)
    {
        ct::String text = "Unwrapped ";
        text.append_number(static_cast<unsigned long long>(mUnwrapDone));
        text += mUnwrapDone == 1 ? " mesh" : " meshes";
        finish(true, text);
        return;
    }
    if (!beginBakeStage(error)) finish(false, error);
}

void LightmapBaker::tickBake()
{
    if (mCancelRequested)
    {
        mPass.cancel();
        mLayers.clear();
        finish(false, "Cancelled");
        return;
    }
    if (mBakeIndex >= mBakeItems.size())
    {
        ct::String error;
        if (!finishBake(error)) finish(false, error);
        return;
    }
    const BakeItem& item = mBakeItems[mBakeIndex];
    if (!mPass.active() && !mPass.done())
    {
        Node* node = resolve(item.path);
        MeshInstance3D* mesh = node ? node->castTo<MeshInstance3D>() : nullptr;
        if (!mesh || !mPass.begin(item.mesh, item.transform, mCasters, mSunDirection, mSunColor,
                             mLayerResolution, mBakeSettings))
        {
            finish(false, ct::String("The mesh '") + item.path + "' could not be baked.");
            return;
        }
        return;
    }
    if (!mPass.step())
    {
        finish(false, ct::String("The GPU bake of '") + item.path + "' failed.");
        return;
    }
    if (!mPass.done())
    {
        Node* node = resolve(item.path);
        mMessage = ct::String("Baking ") + (node ? node->getName() : item.path) + " ";
        mMessage.append_number(static_cast<unsigned long long>(mPass.samplesDone()));
        mMessage += "/";
        mMessage.append_number(static_cast<unsigned long long>(mPass.samplesTotal() + 1));
        return;
    }

    const ct::Vector<std::uint16_t>& pixels = mPass.pixels();
    const std::size_t offset = mLayers.size();
    mLayers.resize(offset + pixels.size());
    memcpy(mLayers.data() + offset, pixels.data(), pixels.size() * sizeof(std::uint16_t));
    mPass.cancel();
    ++mBakeIndex;
}

bool LightmapBaker::finishBake(ct::String& error)
{
    LightmapGI* gi = lightmapNode();
    if (!gi)
    {
        error = "The LightmapGI node is gone.";
        return false;
    }
    ct::Unique<LightmapGIData> data = ct::make_unique<LightmapGIData>();
    if (!data->setLightImages(mLayerResolution, mLayerResolution,
                static_cast<std::uint32_t>(mBakeItems.size()), mLayers.data()))
    {
        error = "The lightmap texture could not be created.";
        return false;
    }
    const Math::Mat4 inverse = gi->getGlobalTransform().Inverse();
    Math::Box bounds = Math::Box::Empty;
    for (std::size_t i = 0; i < mBakeItems.size(); ++i)
    {
        data->addUser(gi->getPathTo(resolve(mBakeItems[i].path)),
                Math::Vec4(0.0f, 0.0f, 1.0f, 1.0f), static_cast<int>(i));
        bounds = bounds.Union(mBakeItems[i].bounds.Transformed(inverse));
    }
    data->setBounds(bounds);

    const ct::String absolute = dataPath();
    ct::String saveError;
    if (!data->save(absolute, &saveError))
    {
        error = saveError.empty() ? ct::String("The lightmap could not be saved.") : saveError;
        return false;
    }
    data = ct::Unique<LightmapGIData>();

    PropertyValue before;
    NodeClassDB::getProperty(gi, "light_data", before);
    EditorTreeEdits::setProperty(&mDocument.undoRedo(), kBakeAction, gi, "light_data", before,
            PropertyValue::fromResourcePath(relativePath(absolute)));
    if (!gi->getLightData())
    {
        error = "The baked lightmap could not be loaded back.";
        return false;
    }

    mSummary.meshes = static_cast<std::uint32_t>(mBakeItems.size());
    mSummary.resolution = mLayerResolution;
    mSummary.samples = mBakeSettings.sampleCount;
    ct::String text = "Baked ";
    text.append_number(static_cast<unsigned long long>(mSummary.meshes));
    text += mSummary.meshes == 1 ? " mesh at " : " meshes at ";
    text.append_number(static_cast<unsigned long long>(mLayerResolution));
    text += ", ";
    text.append_number(static_cast<unsigned long long>(mSummary.samples));
    text += mSummary.samples == 1 ? " sample" : " samples";
    finish(true, text);
    return true;
}

void LightmapBaker::finish(bool ok, const ct::String& text)
{
    mSummary.ok = ok;
    mSummary.seconds = static_cast<double>(time_nanos() - mStartNanos) * 1e-9;
    mSummary.text = text;
    mMessage = text;
    if (ok) log_info("lightmap: %s in %.2f s", text.c_str(), mSummary.seconds);
    else
        log_warn("lightmap: %s", text.c_str());
    mState = State::Idle;
    mCancelRequested = false;
    mThenBake = false;
    mUnwrapQueue.clear();
    mUnwrapping.clear();
    mBakeItems.clear();
    mCasters.clear();
    mLayers.clear();
    mBakeIndex = 0;
    mPass.cancel();
}

void LightmapBaker::reset()
{
    mCancelRequested = false;
    mThenBake = false;
    mUnwrapQueue.clear();
    mUnwrapping.clear();
    mBakeItems.clear();
    mLayers.clear();
    mBakeIndex = 0;
    mSummary = Summary();
}

void LightmapBaker::cancel()
{
    if (mState == State::Idle) return;
    mCancelRequested = true;
    mJob.cancel();
}

void LightmapBaker::update()
{
    if (mState == State::Unwrapping) tickUnwrap();
    else if (mState == State::Baking)
        tickBake();
}

float LightmapBaker::progress() const
{
    if (mState == State::Unwrapping)
    {
        if (mUnwrapTotal == 0) return 0.0f;
        const float part = static_cast<float>(mJob.percent()) * 0.01f;
        const float total = static_cast<float>(mUnwrapTotal) * (mThenBake ? 2.0f : 1.0f);
        return (static_cast<float>(mUnwrapDone) + part) / total;
    }
    if (mState == State::Baking)
    {
        const float meshes = static_cast<float>(mBakeItems.size());
        if (meshes <= 0.0f) return 0.0f;
        float part = 0.0f;
        if (mPass.active())
            part = static_cast<float>(mPass.samplesDone()) /
                   static_cast<float>(mPass.samplesTotal() + 1);
        const float done = (static_cast<float>(mBakeIndex) + part) / meshes;
        if (!mThenBake || mUnwrapTotal == 0) return done;
        return 0.5f + done * 0.5f;
    }
    return 0.0f;
}

} // namespace prisma3d
