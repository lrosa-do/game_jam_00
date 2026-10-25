#pragma once

#include "prisma3d/Device.h"
#include "prisma3d/Input.h"
#include "prisma3d/InputMap.h"
#include "prisma3d/scene/PropertyValue.h"

#include <ct/json.hpp>
#include <ct/slotmap32.hpp>
#include <ct/string.hpp>
#include <ct/vector.hpp>

#include <cstddef>
#include <cstdint>

namespace prisma3d
{

class Engine;
class Node;
class PhysicsServer3D;
class RenderingServer;
class Viewport;
struct SceneResult;
struct ViewportTag;
using ViewportId = ct::Handle32<ViewportTag>;

struct ProjectSettingInfo
{
    ct::String name;
    PropertyType type = PropertyType::Nil;
    PropertyValue defaultValue;
    double minimum = 0.0;
    double maximum = 0.0;
    bool limited = false;
    std::uint64_t allowedMask = 0;
};

struct ProjectInputAction
{
    ct::String name;
    float deadzone = InputMap::kDefaultDeadzone;
    ct::Vector<InputEvent> events;
};

class Project
{
public:
    static constexpr const char* kFormatName = "prisma3d-project";
    static constexpr std::uint32_t kFormatVersion = 1;
    static constexpr const char* kManifestName = "projecto.json";
    static constexpr const char* kScenesFolder = "Scenes";
    static constexpr const char* kDefaultAssetRoot = "Assets";

    Project();
    Project(const Project&) = delete;
    Project& operator=(const Project&) = delete;
    Project(Project&&) = default;
    Project& operator=(Project&&) = default;

    bool create(const ct::String& parentDirectory, const ct::String& name);
    bool open(const ct::String& manifestPath);
    bool openPacked(const ct::String& manifestName);
    bool save();
    void close();

    bool isOpen() const;
    const ct::String& error() const;
    const ct::Vector<ct::String>& warnings() const;

    const ct::String& name() const;
    bool setName(const ct::String& name);
    const ct::String& root() const;
    const ct::String& manifestPath() const;
    const ct::String& assetRoot() const;
    bool setAssetRoot(const ct::String& assetRoot);
    ct::String assetPath() const;
    ct::String absolutePath(const ct::String& relative) const;
    bool relativePath(const ct::String& path, ct::String& out) const;

    const ct::Vector<ct::String>& scenes() const;
    bool hasScene(const ct::String& scene) const;
    bool addScene(const ct::String& scene);
    bool removeScene(const ct::String& scene);
    bool renameScene(const ct::String& from, const ct::String& to);
    const ct::String& mainScene() const;
    bool setMainScene(const ct::String& scene);

    const ct::Vector<ct::String>& searchPaths() const;
    bool addSearchPath(const ct::String& path);
    bool removeSearchPath(const ct::String& path);
    ct::Vector<ct::String> resolvedSearchPaths() const;

    static std::size_t settingCount();
    static const ProjectSettingInfo& settingInfo(std::size_t index);
    static const ProjectSettingInfo* findSetting(const ct::String& name);
    static PropertyValue getDefault(const ct::String& name);
    bool has(const ct::String& name) const;
    PropertyValue get(const ct::String& name) const;
    bool set(const ct::String& name, const PropertyValue& value);
    bool isDefault(const ct::String& name) const;
    void reset(const ct::String& name);

    const ct::Vector<ProjectInputAction>& inputActions() const;
    const ProjectInputAction* findInputAction(const ct::String& name) const;
    bool setInputAction(const ProjectInputAction& action);
    bool removeInputAction(const ct::String& name);

    static bool normalizeRelativePath(const ct::String& path, ct::String& out, ct::String& reason);

    void fillDeviceDesc(DeviceDesc& desc) const;
    void applyToEngine(Engine& engine) const;
    void applyToPhysics(PhysicsServer3D& physics) const;
    void applyToInputMap(InputMap& map) const;
    void applyToRendering(RenderingServer& server) const;
    void applyToViewport(Viewport& viewport) const;
    void applyToViewport(RenderingServer& server, ViewportId viewport,
            bool clearColor = true) const;
    void registerSearchPaths();
    void unregisterSearchPaths();

    Node* loadMainScene(Node* parent, SceneResult& result) const;

private:
    bool fail(const ct::String& message);
    bool openManifest(const ct::String& path);
    bool readManifest(const ct::Json& document, const ct::String& manifestPath);
    bool readSettings(const ct::Json& settings);
    bool readInputAction(const ct::Json& json, const ct::String& path, ProjectInputAction& out);
    ct::Json writeInputAction(const ProjectInputAction& action) const;
    ct::String buildManifest() const;
    std::size_t findScene(const ct::String& scene) const;
    std::int64_t integer(const char* name) const;
    double number(const char* name) const;
    bool flag(const char* name) const;

    bool mOpen;
    ct::String mName;
    ct::String mRoot;
    ct::String mManifestPath;
    ct::String mAssetRoot;
    ct::Vector<ct::String> mScenes;
    ct::String mMainScene;
    ct::Vector<ct::String> mSearchPaths;
    ct::Vector<PropertyValue> mValues;
    ct::Vector<ProjectInputAction> mInputActions;
    ct::Json mExtraSettings;
    ct::Vector<ct::String> mRegisteredPaths;
    ct::String mError;
    ct::Vector<ct::String> mWarnings;
};

} // namespace prisma3d
