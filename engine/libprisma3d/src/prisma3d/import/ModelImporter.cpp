#include "prisma3d/import/ModelImporter.h"

#include "platform.h"

namespace prisma3d
{

namespace
{

Math::Mat4 composeLocal(const ModelNodeData& node)
{
    const Math::Mat3 rotation = node.rotation.ToMat3();
    const Math::Mat3 basis(rotation.col0 * node.scale.x, rotation.col1 * node.scale.y,
            rotation.col2 * node.scale.z);
    return Math::Mat4(basis, node.translation);
}

bool reject(ModelData& out, const ct::String& path, const char* what)
{
    ct::String message = "'";
    message += path;
    message += "' ";
    message += what;
    log_error("ModelLoader: %s", message.c_str());
    out.error = message;
    return false;
}

ModelImporter* select(const ct::Vector<ModelImporter*>& importers, const ct::String& extension)
{
    for (std::size_t i = 0; i < importers.size(); ++i)
    {
        if (importers[i]->supports(extension.c_str())) return importers[i];
    }
    return nullptr;
}

} // namespace

Math::Mat4 ModelData::nodeLocalTransform(int node) const
{
    return composeLocal(nodes[static_cast<std::size_t>(node)]);
}

Math::Mat4 ModelData::nodeGlobalTransform(int node) const
{
    Math::Mat4 result = nodeLocalTransform(node);
    int current = nodes[static_cast<std::size_t>(node)].parent;
    std::size_t steps = 0;
    while (current >= 0 && steps++ < nodes.size())
    {
        result = nodeLocalTransform(current) * result;
        current = nodes[static_cast<std::size_t>(current)].parent;
    }
    return result;
}

ModelLoader* ModelLoader::sSingleton = nullptr;

ModelLoader::ModelLoader()
        : mImporters()
{
    sSingleton = this;
}

ModelLoader::~ModelLoader()
{
    for (std::size_t i = 0; i < mImporters.size(); ++i) delete mImporters[i];
    if (sSingleton == this) sSingleton = nullptr;
}

ModelLoader* ModelLoader::singleton()
{
    return sSingleton;
}

void ModelLoader::addImporter(ModelImporter* importer)
{
    if (!importer) return;
    mImporters.push_back(importer);
}

std::uint32_t ModelLoader::importerCount() const
{
    return static_cast<std::uint32_t>(mImporters.size());
}

ModelImporter* ModelLoader::importer(std::uint32_t index) const
{
    return index < mImporters.size() ? mImporters[index] : nullptr;
}

ModelImporter* ModelLoader::importerFor(const ct::String& path) const
{
    const ct::String extension = extensionOf(path);
    if (extension.empty()) return nullptr;
    return select(mImporters, extension);
}

bool ModelLoader::load(const ct::String& path, ModelData& out) const
{
    const ct::String extension = extensionOf(path);
    if (extension.empty()) return reject(out, path, "has no file extension");

    ModelImporter* selected = select(mImporters, extension);
    if (!selected)
    {
        ct::String what = "has no importer registered for '.";
        what += extension;
        what += "'";
        return reject(out, path, what.c_str());
    }

    ModelData loaded;
    if (!selected->import(path, loaded))
    {
        if (loaded.error.empty()) return reject(out, path, "failed to import");
        log_error("ModelLoader: '%s' failed to import: %s", path.c_str(), loaded.error.c_str());
        out.error = loaded.error;
        return false;
    }
    out = static_cast<ModelData&&>(loaded);
    return true;
}

bool ModelLoader::loadFromMemory(const std::uint8_t* data, std::size_t size,
        const ct::String& virtualName, ModelData& out) const
{
    const ct::String extension = extensionOf(virtualName);
    if (extension.empty()) return reject(out, virtualName, "has no file extension");

    ModelImporter* selected = select(mImporters, extension);
    if (!selected)
    {
        ct::String what = "has no importer registered for '.";
        what += extension;
        what += "'";
        return reject(out, virtualName, what.c_str());
    }
    if (!data || size == 0) return reject(out, virtualName, "has no data");

    ModelData loaded;
    if (!selected->importMemory(data, size, virtualName, loaded))
    {
        if (loaded.error.empty()) return reject(out, virtualName, "failed to import");
        log_error("ModelLoader: '%s' failed to import: %s", virtualName.c_str(),
                loaded.error.c_str());
        out.error = loaded.error;
        return false;
    }
    out = static_cast<ModelData&&>(loaded);
    return true;
}

ct::String ModelLoader::extensionOf(const ct::String& path)
{
    std::size_t dot = ct::String::npos;
    std::size_t slash = ct::String::npos;
    for (std::size_t i = 0; i < path.size(); ++i)
    {
        if (path[i] == '/' || path[i] == '\\') slash = i;
        else if (path[i] == '.')
            dot = i;
    }
    ct::String extension;
    if (dot == ct::String::npos || (slash != ct::String::npos && dot < slash) ||
            dot + 1 >= path.size())
        return extension;
    for (std::size_t i = dot + 1; i < path.size(); ++i)
    {
        char c = path[i];
        if (c >= 'A' && c <= 'Z') c = static_cast<char>(c - 'A' + 'a');
        extension.push_back(c);
    }
    return extension;
}

} // namespace prisma3d
