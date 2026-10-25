#pragma once

#include <ct/string.hpp>

namespace prisma3d
{

class EditorDocument;

class DefaultProject
{
public:
    static constexpr const char* kFolder = "projects";
    static constexpr const char* kName = "Default";
    static constexpr const char* kScriptPath = "Assets/Scripts/rotate.py";
    static constexpr const char* kMaterialFolder = "Assets/Materials";
    static constexpr const char* kMaterialName = "Orange";
    static constexpr const char* kFloorMaterialName = "Ground";
    static constexpr const char* kScriptText =
            "class Rotate(MeshInstance3D):\n"
            "    speed = 90.0\n"
            "\n"
            "    def _ready(self):\n"
            "        print(\"rotate.py ready on \" + self.name)\n"
            "\n"
            "    def _process(self, delta):\n"
            "        self.rotation_degrees.y += self.speed * delta\n";

    static ct::String parentFolder(const ct::String& appFolder);
    static ct::String manifestPath(const ct::String& appFolder);
    static bool exists(const ct::String& appFolder);
    static bool wanted(const char* projectArgument, const ct::String& lastProject);

    static bool open(EditorDocument& document, const ct::String& appFolder);
    static bool create(EditorDocument& document, const ct::String& appFolder);

private:
    static bool writeAssets(EditorDocument& document, ct::String& error);
    static bool buildScene(EditorDocument& document, ct::String& error);
};

} // namespace prisma3d
