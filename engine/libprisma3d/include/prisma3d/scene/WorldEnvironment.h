#pragma once

#include "prisma3d/render/RenderingServer.h"
#include "prisma3d/scene/Node.h"

namespace prisma3d
{

class WorldEnvironment : public Node
{
public:
    PRISMA3D_NODE_CLASS(WorldEnvironment, Node);
    static void bindProperties();

    WorldEnvironment();
    ~WorldEnvironment() override;

    void setEnvironment(const EnvironmentSettings& environment);
    const EnvironmentSettings& getEnvironment() const;

protected:
    void notificationHandler(int what) override;
    void validateProperty(PropertyInfo& info) const override;

private:
    ct::String groupName() const;
    void updateCurrentEnvironment();

    EnvironmentSettings mEnvironment;
    ScenarioId mScenario;
};

} // namespace prisma3d
