#pragma once

#include "prisma3d/scene/ScriptServer.h"

#include <ct/string.hpp>
#include <ct/vector.hpp>

namespace prisma3d
{

class ZenScripting : public ScriptServer
{
public:
    struct Impl;

    typedef void (*PrintHandler)(const char* line, void* user);

    static constexpr double kDefaultReloadInterval = 0.5;
    static constexpr const char* kScriptList = "scripts.json";

    ZenScripting();
    ~ZenScripting() override;

    static ZenScripting* instance();
    Impl* getImpl() const;

    void attachScript(Node* node, const ct::String& path) override;
    void update(double delta) override;

    void setPrintHandler(PrintHandler handler, void* user);

    unsigned long long getBindingsSignature() const;
    int preload(const ct::Vector<ct::String>& paths);
    int preloadList(const ct::String& listName);
    enum ExportResult
    {
        kExported,
        kNotNodeScript,
        kExportFailed
    };
    ExportResult exportBytecode(const ct::String& path, const ct::String& outputFile,
            bool stripDebug, ct::String& error);

    int reloadChanged();
    void setHotReload(bool enabled, double interval = kDefaultReloadInterval);
    bool isHotReload() const;

    int getCompileCount() const;
    int getErrorCount() const;
    const ct::String& getLastError() const;
    void clearErrors();

    bool isFaulted(const Node* node) const;
    bool isRunnable(const Node* node) const;

private:
    Impl* mImpl;
};

} // namespace prisma3d
