#pragma once

#include "prisma3d/import/ModelImporter.h"
#include "prisma3d/render/AssetLoader.h"
#include "prisma3d/scene/Node3D.h"

namespace prisma3d
{

Node3D* instantiateModel(Node* parent, const ModelData& document, const ct::Vector<MeshId>& meshes,
        int collisionType = 0);
Node3D* instantiateModel(Node* parent, ModelId model);
Node3D* buildModel(ModelId model);

} // namespace prisma3d
