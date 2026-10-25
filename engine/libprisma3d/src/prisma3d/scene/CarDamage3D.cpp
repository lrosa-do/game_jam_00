#include "prisma3d/scene/CarDamage3D.h"

#include "NodeFactory.h"
#include "prisma3d/core/FileSystem.h"
#include "prisma3d/main/Engine.h"
#include "prisma3d/main/Main.h"
#include "prisma3d/physics/PhysicsServer3D.h"
#include "prisma3d/scene/GroundVehicle3D.h"
#include "prisma3d/scene/ImpactFx3D.h"
#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/VisualInstance3D.h"
#include "vehicle/CarDamage.h"

#include "platform.h"

#include <math.h>

namespace prisma3d
{

CarDamage3D::CarDamage3D()
        : Node3D(),
          mDamage(nullptr),
          mShown(),
          mMeshes(),
          mAnchors(),
          mModel(),
          mSkipPrefix("wheel"),
          mImpactFxPath(),
          mMinImpulse(300.0f),
          mStepImpulse(0.0f),
          mTotalImpulse(0.0f),
          mEatenNose(0.0f),
          mEatenTail(0.0f),
          mSmokeTick(0),
          mLastSpeed(0.0f),
          mSoundWait(0.0f),
          mFailed(false)
{
    setPhysicsProcess(true);
}

CarDamage3D::~CarDamage3D()
{
    release();
}

void CarDamage3D::setModel(const ct::String& path)
{
    if (path == mModel) return;
    mModel = path;
    release();
    mFailed = false;
}

const ct::String& CarDamage3D::getModel() const
{
    return mModel;
}

void CarDamage3D::setSkipPrefix(const ct::String& prefix)
{
    mSkipPrefix = prefix;
}

const ct::String& CarDamage3D::getSkipPrefix() const
{
    return mSkipPrefix;
}

void CarDamage3D::setMinImpulse(float impulse)
{
    mMinImpulse = impulse < 0.0f ? 0.0f : impulse;
}

float CarDamage3D::getMinImpulse() const
{
    return mMinImpulse;
}

void CarDamage3D::setImpactFxPath(const ct::String& path)
{
    mImpactFxPath = path;
}

const ct::String& CarDamage3D::getImpactFxPath() const
{
    return mImpactFxPath;
}

ImpactFx3D* CarDamage3D::impactFx() const
{
    if (mImpactFxPath.empty()) return nullptr;
    Node* node = getNode(mImpactFxPath);
    return node ? node->castTo<ImpactFx3D>() : nullptr;
}

bool CarDamage3D::isBuilt() const
{
    return mDamage != nullptr;
}

float CarDamage3D::getPeak() const
{
    return mDamage ? mDamage->peak() : 0.0f;
}

float CarDamage3D::getNoseCrush() const
{
    return mDamage ? mDamage->noseCrush() : 0.0f;
}

float CarDamage3D::getTailCrush() const
{
    return mDamage ? mDamage->tailCrush() : 0.0f;
}

float CarDamage3D::getStepImpulse() const
{
    return mStepImpulse;
}

float CarDamage3D::getTotalImpulse() const
{
    return mTotalImpulse;
}

float CarDamage3D::damageNear(const Math::Vec3& point, float radius) const
{
    if (!mDamage) return 0.0f;
    return mDamage->damageNear(chassis().InverseAffine().TransformPoint(point), radius);
}

void CarDamage3D::hit(const Math::Vec3& point, float impulse)
{
    if (mDamage) mDamage->hit(chassis().InverseAffine().TransformPoint(point), impulse);
}

void CarDamage3D::reset()
{
    mTotalImpulse = 0.0f;
    mEatenNose = 0.0f;
    mEatenTail = 0.0f;
    if (mDamage)
    {
        mDamage->reset();
        mDamage->update(1.0f);
        followShape();
        moveAnchors();
    }
}

CarDamage* CarDamage3D::getDamage() const
{
    return mDamage;
}

GroundVehicle3D* CarDamage3D::getVehicle() const
{
    Node* parent = getParent();
    return parent ? parent->castTo<GroundVehicle3D>() : nullptr;
}

Math::Mat4 CarDamage3D::chassis() const
{
    GroundVehicle3D* vehicle = getVehicle();
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (!vehicle) return getGlobalTransform();
    if (server && vehicle->isSimulated()) return server->bodyGetTransform(vehicle->getBody());
    return vehicle->getGlobalTransform();
}

bool CarDamage3D::build()
{
    AssetLoader* loader = Main::assets();
    MaterialStore* materials = Main::materials();
    if (!loader || !materials || mModel.empty()) return false;
    const ct::String file = AssetLoader::modelFile(mModel);
    ModelId id = loader->findModelByPath(mModel);
    if (!id.valid())
    {
        if (!FileSystem::getSingleton().exists(file))
        {
            log_error("car damage: '%s' was not found", mModel.c_str());
            return false;
        }
        id = loader->requestModel(mModel);
    }
    if (!id.valid()) return false;
    if (loader->modelStatus(id) == LoadStatus::InProgress) loader->waitAll();
    const ModelInfo* info = loader->model(id);
    if (!info || info->status != LoadStatus::Loaded || !info->importer)
    {
        log_error("car damage: '%s' did not load", mModel.c_str());
        return false;
    }

    const ModelData& document = info->document;
    const Math::Mat4 bake =
            getTransform() * Math::Mat4::Scale(Math::Vec3(info->scale, info->scale, info->scale));
    ct::Vector<CarPieceSurface> body;
    ct::Vector<MaterialId> bodyMaterials;
    ct::Vector<CarPieceSurface> glass;
    ct::Vector<MaterialId> glassMaterials;
    for (std::size_t n = 0; n < document.nodes.size(); ++n)
    {
        const ModelNodeData& node = document.nodes[n];
        if (node.mesh < 0 || static_cast<std::size_t>(node.mesh) >= document.meshes.size())
            continue;
        if (!mSkipPrefix.empty() && node.name.starts_with(mSkipPrefix)) continue;
        const Math::Mat4 place = bake * document.nodeGlobalTransform(static_cast<int>(n));
        const ModelMeshData& mesh = document.meshes[static_cast<std::size_t>(node.mesh)];
        for (std::size_t p = 0; p < mesh.primitives.size(); ++p)
        {
            const ModelPrimitiveData& primitive = mesh.primitives[p];
            if (primitive.topology != prisma::Topology::Triangles) continue;
            CarPieceSurface piece;
            if (!info->importer->readPrimitive(document, primitive, piece.surface)) continue;
            if (piece.surface.indices.empty() || piece.surface.positions.empty()) continue;
            piece.surface.joints.clear();
            piece.surface.weights.clear();
            for (std::size_t i = 0; i < piece.surface.positions.size(); ++i)
                piece.surface.positions[i] = place.TransformPoint(piece.surface.positions[i]);
            for (std::size_t i = 0; i < piece.surface.normals.size(); ++i)
                piece.surface.normals[i] =
                        place.TransformVector(piece.surface.normals[i]).NormalizedSafe();
            for (std::size_t i = 0; i < piece.surface.tangents.size(); ++i)
            {
                const Math::Vec4& t = piece.surface.tangents[i];
                const Math::Vec3 turned =
                        place.TransformVector(Math::Vec3(t.x, t.y, t.z)).NormalizedSafe();
                piece.surface.tangents[i] = Math::Vec4(turned.x, turned.y, turned.z, t.w);
            }
            MaterialId material;
            bool transparent = false;
            if (primitive.material >= 0 &&
                    static_cast<std::size_t>(primitive.material) < document.materials.size())
            {
                const ModelMaterialData& data =
                        document.materials[static_cast<std::size_t>(primitive.material)];
                piece.material = data.name;
                transparent = data.alphaMode == ModelAlphaMode::Blend;
                if (static_cast<std::size_t>(primitive.material) < info->materials.size())
                    material = info->materials[static_cast<std::size_t>(primitive.material)];
            }
            if (transparent)
            {
                glass.push_back(piece);
                glassMaterials.push_back(material);
            }
            else
            {
                body.push_back(piece);
                bodyMaterials.push_back(material);
            }
        }
    }
    if (body.empty())
    {
        log_error("car damage: '%s' has no body to crumple", mModel.c_str());
        return false;
    }

    mDamage = new CarDamage();
    mMeshes.push_back(mDamage->addPiece(getName() + "_body", CarDamage::PieceKind::Body, body,
            bodyMaterials, Math::Mat4::Identity()));
    if (!glass.empty())
        mMeshes.push_back(mDamage->addPiece(getName() + "_glass", CarDamage::PieceKind::Glass,
                glass, glassMaterials, Math::Mat4::Identity()));
    mDamage->finish();

    const Math::Mat4 back = getTransform().InverseAffine();
    for (std::size_t i = 0; i < mMeshes.size(); ++i)
    {
        MeshInstance3D* shown = createChild<MeshInstance3D>();
        shown->setInternal(true);
        shown->setMesh(mMeshes[i]);
        shown->setTransform(back);
        mShown.push_back(shown);
    }

    anchorChildren();
    GroundVehicle3D* vehicle = getVehicle();
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (vehicle && server) server->bodySetImpulseMonitor(vehicle->getBody(), true);
    return true;
}

void CarDamage3D::anchorChildren()
{
    mAnchors.clear();
    if (!mDamage) return;
    const Math::Mat4 place = getTransform();
    for (std::size_t i = 0; i < getChildCount(); ++i)
    {
        Node3D* child = getChild(i)->castTo<Node3D>();
        if (!child || child->isInternal() || child->getName().empty()) continue;
        Anchor anchor;
        anchor.name = child->getName();
        anchor.rest = child->getTransform();
        const Math::Vec4 origin = anchor.rest.col3;
        CarDamage::VertexRef ref;
        if (!mDamage->nearestBodyVertex(
                    place.TransformPoint(Math::Vec3(origin.x, origin.y, origin.z)), ref))
            continue;
        anchor.piece = ref.piece;
        anchor.part = ref.part;
        anchor.vertex = ref.vertex;
        mAnchors.push_back(anchor);
    }
}

void CarDamage3D::moveAnchors()
{
    if (!mDamage || mAnchors.empty()) return;
    const Math::Mat4 back = getTransform().InverseAffine();
    for (std::size_t i = 0; i < mAnchors.size(); ++i)
    {
        const Anchor& anchor = mAnchors[i];
        Node* node = getNode(anchor.name);
        Node3D* child = node ? node->castTo<Node3D>() : nullptr;
        if (!child || child->getParent() != this) continue;
        CarDamage::VertexRef ref;
        ref.piece = anchor.piece;
        ref.part = anchor.part;
        ref.vertex = anchor.vertex;
        const Math::Vec3 moved = back.TransformVector(mDamage->vertexMoved(ref));
        Math::Mat4 frame = anchor.rest;
        frame.col3 = frame.col3 + Math::Vec4(moved.x, moved.y, moved.z, 0.0f);
        child->setTransform(frame);
    }
}

void CarDamage3D::release()
{
    for (std::size_t i = 0; i < mShown.size(); ++i) NodeFactory::destroy(mShown[i]);
    mShown.clear();
    MeshStore* meshes = Main::meshes();
    for (std::size_t i = 0; i < mMeshes.size(); ++i)
        if (meshes && meshes->valid(mMeshes[i])) meshes->destroy(mMeshes[i]);
    mMeshes.clear();
    delete mDamage;
    mDamage = nullptr;
    mStepImpulse = 0.0f;
    mEatenNose = 0.0f;
    mEatenTail = 0.0f;
}

void CarDamage3D::listen()
{
    mStepImpulse = 0.0f;
    GroundVehicle3D* vehicle = getVehicle();
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (!vehicle || !server || !mDamage) return;
    const float before = mLastSpeed;
    mLastSpeed = server->bodyGetLinearVelocity(vehicle->getBody()).Length();
    if (mSoundWait > 0.0f) mSoundWait -= static_cast<float>(getPhysicsProcessDeltaTime());
    const ct::Vector<PhysicsContactImpulse>* impulses =
            server->bodyGetContactImpulses(vehicle->getBody());
    if (!impulses || impulses->empty()) return;
    const Math::Mat4 inverse = chassis().InverseAffine();
    ImpactFx3D* fx = impactFx();
    float strongest = 0.0f;
    Math::Vec3 where(0.0f, 0.0f, 0.0f);
    bool heard = false;
    for (std::size_t i = 0; i < impulses->size(); ++i)
    {
        const PhysicsContactImpulse& contact = (*impulses)[i];
        if (contact.impulse < mMinImpulse) continue;
        mStepImpulse += contact.impulse;
        mDamage->hit(inverse.TransformPoint(contact.point), contact.impulse);
        const bool otherShows = server->bodyGetContactImpulses(contact.other) &&
                                contact.other.bits() < vehicle->getBody().bits();
        if (fx && !otherShows) fx->impact(contact.point, contact.impulse);
        if (contact.impulse > strongest)
        {
            strongest = contact.impulse;
            where = contact.point;
            heard = !otherShows;
        }
    }
    mTotalImpulse += mStepImpulse;
    if (!fx || !heard) return;
    const Engine* engine = Engine::singleton();
    const float pace = engine ? static_cast<float>(engine->getTimeScale()) : 1.0f;
    const float mass = server->bodyGetMass(vehicle->getBody());
    const float lost = mass > 0.0f ? mStepImpulse / mass : 0.0f;
    if (lost > 3.5f * pace && before > 4.0f)
    {
        fx->crash(where, 0.9f);
        mSoundWait = 0.45f;
    }
    else if (mSoundWait <= 0.0f)
    {
        fx->crash(where, fminf(0.35f + lost * 0.08f, 1.0f));
        mSoundWait = 0.45f;
    }
}

void CarDamage3D::smokeCrushed()
{
    if ((++mSmokeTick & 3) != 0) return;
    ImpactFx3D* fx = impactFx();
    if (!fx) return;
    const Math::Mat4 body = chassis();
    if (mDamage->noseCrush() > 0.25f)
        fx->smoke(body.TransformPoint(
                          Math::Vec3(0.0f, 0.55f, mDamage->high().z - mDamage->noseCrush() - 0.4f)),
                1);
    if (mDamage->tailCrush() > 0.25f)
        fx->smoke(body.TransformPoint(
                          Math::Vec3(0.0f, 0.55f, mDamage->low().z + mDamage->tailCrush() + 0.4f)),
                1);
}

void CarDamage3D::followShape()
{
    GroundVehicle3D* vehicle = getVehicle();
    PhysicsServer3D* server = PhysicsServer3D::singleton();
    if (!vehicle || !server || !mDamage || !vehicle->isSimulated()) return;
    const float more = (mDamage->noseCrush() - mEatenNose) - (mDamage->tailCrush() - mEatenTail);
    mEatenNose = mDamage->noseCrush();
    mEatenTail = mDamage->tailCrush();
    if (more != 0.0f)
    {
        Math::Mat4 frame = server->bodyGetTransform(vehicle->getBody());
        const Math::Vec3 forward =
                Math::Vec3(frame.col2.x, frame.col2.y, frame.col2.z).NormalizedSafe();
        frame.col3 =
                frame.col3 + Math::Vec4(forward.x * more, forward.y * more, forward.z * more, 0.0f);
        server->bodySetTransform(vehicle->getBody(), frame);
    }
    float rear = mDamage->low().z + mDamage->tailCrush();
    float front = mDamage->high().z - mDamage->noseCrush();
    if (rear > -0.3f) rear = -0.3f;
    if (front < 0.3f) front = 0.3f;
    vehicle->setLengthwiseExtent(rear, front);
}

void CarDamage3D::notificationHandler(int what)
{
    Node3D::notificationHandler(what);
    switch (what)
    {
        case kNotificationPhysicsProcess:
        {
            if (!mDamage && !mFailed && !mModel.empty() && isInsideTree())
            {
                if (!build())
                {
                    release();
                    mFailed = true;
                }
            }
            if (!mDamage) break;
            listen();
            if (mDamage->update(static_cast<float>(getPhysicsProcessDeltaTime())))
            {
                followShape();
                moveAnchors();
            }
            smokeCrushed();
            break;
        }
        case kNotificationExitTree:
        {
            GroundVehicle3D* vehicle = getVehicle();
            PhysicsServer3D* server = PhysicsServer3D::singleton();
            if (vehicle && server) server->bodySetImpulseMonitor(vehicle->getBody(), false);
            break;
        }
        default:
            break;
    }
}

void CarDamage3D::bindProperties()
{
    using DB = NodeClassDB;
    using P = CarDamage3D;
    DB::addProperty<P>(PropertyInfo::file("model", "*.glb,*.gltf,*.b3d,*.fbx,*.obj"), &P::setModel,
            &P::getModel);
    DB::addProperty<P>(PropertyInfo(PropertyType::String, "skip_prefix"), &P::setSkipPrefix,
            &P::getSkipPrefix);
    DB::addProperty<P>(
            PropertyInfo::range(PropertyType::Float, "min_impulse", 0.0, 100000.0, 1.0, true),
            &P::setMinImpulse, &P::getMinImpulse);
    DB::addProperty<P>(PropertyInfo::nodePath("impact_fx", "ImpactFx3D"), &P::setImpactFxPath,
            &P::getImpactFxPath);
}

} // namespace prisma3d
