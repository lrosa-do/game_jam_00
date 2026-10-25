#include "prisma3d/scene/BoostJet3D.h"

#include "NodeFactory.h"
#include "prisma3d/core/Pixmap.h"
#include "prisma3d/main/Main.h"
#include "prisma3d/scene/NodeClassDB.h"
#include "prisma3d/scene/PrimitiveMeshes.h"
#include "prisma3d/scene/SceneResources.h"
#include "prisma3d/scene/VisualInstance3D.h"

#include <math.h>

namespace prisma3d
{

namespace
{

const float kGlowAlong[BoostJet3D::kGlows] = { 0.0f, 0.3f, 0.7f, 1.2f };
const float kGlowWide[BoostJet3D::kGlows] = { 1.4f, 1.1f, 0.8f, 0.5f };
const Math::Vec4 kGlowTint[BoostJet3D::kGlows] = { Math::Vec4(0.75f, 0.85f, 1.0f, 1.0f),
    Math::Vec4(0.35f, 0.55f, 1.0f, 1.0f), Math::Vec4(0.6f, 0.45f, 0.6f, 1.0f),
    Math::Vec4(1.0f, 0.45f, 0.12f, 1.0f) };

TextureId flameTexture()
{
    TextureStore* textures = Main::textures();
    if (!textures) return TextureId();
    const int width = 64;
    const int height = 256;
    Pixmap pixels(width, height, 4);
    for (int y = 0; y < height; ++y)
    {
        const float along = (static_cast<float>(y) + 0.5f) / static_cast<float>(height);
        const float reach = 1.0f - 0.75f * along;
        for (int x = 0; x < width; ++x)
        {
            const float across =
                    fabsf((static_cast<float>(x) + 0.5f) / static_cast<float>(width) * 2.0f - 1.0f);
            float edge = 1.0f - across / reach;
            edge = edge < 0.0f ? 0.0f : edge * edge;
            const float strength = edge * powf(1.0f - along, 1.3f) * fminf(along * 14.0f, 1.0f);
            const float heat = edge * (1.0f - along);
            const float tip = along > 0.45f ? (along - 0.45f) / 0.55f : 0.0f;
            const float r = strength * (0.25f + 0.75f * tip + 0.75f * heat);
            const float g = strength * (0.50f - 0.10f * tip + 0.50f * heat);
            const float b = strength * (1.00f - 0.90f * tip);
            pixels.set_pixel(static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y),
                    static_cast<std::uint8_t>(fminf(r, 1.0f) * 255.0f),
                    static_cast<std::uint8_t>(fminf(g, 1.0f) * 255.0f),
                    static_cast<std::uint8_t>(fminf(b, 1.0f) * 255.0f), 255);
        }
    }
    return textures->createFromPixmap(pixels, true, true, "flame");
}

void flameSurface(SurfaceData& surface)
{
    for (int sheet = 0; sheet < 6; ++sheet)
    {
        const float angle = static_cast<float>(sheet) * 3.14159265f / 6.0f;
        const Math::Vec3 side(cosf(angle) * 0.5f, sinf(angle) * 0.5f, 0.0f);
        const std::uint32_t first = static_cast<std::uint32_t>(surface.positions.size());
        const Math::Vec3 corners[4] = { side * -1.0f, side, side + Math::Vec3(0.0f, 0.0f, 1.0f),
            side * -1.0f + Math::Vec3(0.0f, 0.0f, 1.0f) };
        const Math::Vec2 uvs[4] = { Math::Vec2(0.0f, 0.0f), Math::Vec2(1.0f, 0.0f),
            Math::Vec2(1.0f, 1.0f), Math::Vec2(0.0f, 1.0f) };
        for (int i = 0; i < 4; ++i)
        {
            surface.positions.push_back(corners[i]);
            surface.normals.push_back(Math::Vec3(0.0f, 1.0f, 0.0f));
            surface.uvs.push_back(uvs[i]);
        }
        const std::uint32_t order[6] = { 0, 1, 2, 0, 2, 3 };
        for (std::uint32_t index: order) surface.indices.push_back(first + index);
    }
    MeshStore::generateTangents(surface);
}

void glowSurface(SurfaceData& surface)
{
    planeMeshArrays(surface, 1.0f, 1.0f, 1, 1, 1.0f);
    for (std::size_t i = 0; i < surface.positions.size(); ++i)
    {
        const Math::Vec3 p = surface.positions[i];
        surface.positions[i] = Math::Vec3(p.x, -p.z, 0.0f);
        surface.normals[i] = Math::Vec3(0.0f, 0.0f, 1.0f);
    }
}

} // namespace

BoostJet3D::BoostJet3D()
        : Node3D(),
          mFlameTexture(),
          mFlameMaterial(),
          mFlameMesh(),
          mGlowMaterials(),
          mGlowMeshes(),
          mJets(),
          mGlows(),
          mGlowTexture(),
          mPhase(0.0f),
          mGlowColor(0.0f, 0.0f, 0.0f, 0.0f),
          mJetsShown(true),
          mFlicker(0.0f),
          mBurning(false),
          mBuilt(false)
{
    setProcess(true);
}

BoostJet3D::~BoostJet3D()
{
    release();
}

void BoostJet3D::setBurning(bool burning)
{
    mBurning = burning;
    for (MeshInstance3D* jet: mJets)
        if (jet) jet->setVisible(burning && mJetsShown);
    for (MeshInstance3D* glow: mGlows)
        if (glow) glow->setVisible(burning);
}

bool BoostJet3D::isBurning() const
{
    return mBurning;
}

void BoostJet3D::setGlowTexture(const ct::String& path)
{
    mGlowTexture = path;
    if (!mBuilt) return;
    MaterialStore* materials = Main::materials();
    const TextureId texture = path.empty() ? TextureId() : sceneres::loadTexture(path, true);
    for (int g = 0; g < kGlows; ++g)
        if (materials && mGlowMaterials[g].valid())
            materials->setTexture(mGlowMaterials[g], MaterialTexture::Albedo, texture);
}

const ct::String& BoostJet3D::getGlowTexture() const
{
    return mGlowTexture;
}

void BoostJet3D::setPhase(float phase)
{
    mPhase = phase;
}

float BoostJet3D::getPhase() const
{
    return mPhase;
}

void BoostJet3D::setJets(bool jets)
{
    mJetsShown = jets;
    setBurning(mBurning);
}

bool BoostJet3D::hasJets() const
{
    return mJetsShown;
}

void BoostJet3D::setGlowColor(const Math::Vec4& color)
{
    mGlowColor = color;
    tintGlows();
}

const Math::Vec4& BoostJet3D::getGlowColor() const
{
    return mGlowColor;
}

void BoostJet3D::tintGlows()
{
    MaterialStore* materials = Main::materials();
    if (!materials) return;
    for (int g = 0; g < kGlows; ++g)
        if (mGlowMaterials[g].valid())
            materials->setColor(mGlowMaterials[g], MaterialParam::Albedo,
                    mGlowColor.w > 0.0f ? mGlowColor : kGlowTint[g]);
}

void BoostJet3D::build()
{
    MaterialStore* materials = Main::materials();
    MeshStore* meshes = Main::meshes();
    if (mBuilt || !materials || !meshes) return;
    mBuilt = true;

    SurfaceData flame;
    flameSurface(flame);
    mFlameTexture = flameTexture();
    mFlameMaterial = materials->create("boost flame");
    materials->setShading(mFlameMaterial, MaterialShading::Unshaded);
    materials->setBlendMode(mFlameMaterial, MaterialBlend::Add);
    materials->setCull(mFlameMaterial, MaterialCull::Disabled);
    materials->setTexture(mFlameMaterial, MaterialTexture::Albedo, mFlameTexture);
    mFlameMesh = meshes->create("boost flame");
    meshes->addSurface(mFlameMesh, flame, "boost flame");
    meshes->setSurfaceMaterial(mFlameMesh, 0, mFlameMaterial);
    for (int layer = 0; layer < 2; ++layer)
    {
        MeshInstance3D* jet = createChild<MeshInstance3D>();
        jet->setInternal(true);
        jet->setMesh(mFlameMesh);
        jet->setCastShadows(false);
        jet->setVisible(mBurning && mJetsShown);
        mJets[layer] = jet;
    }

    const TextureId glowTexture =
            mGlowTexture.empty() ? TextureId() : sceneres::loadTexture(mGlowTexture, true);
    for (int g = 0; g < kGlows; ++g)
    {
        SurfaceData glow;
        glowSurface(glow);
        mGlowMaterials[g] = materials->create("glow");
        materials->setShading(mGlowMaterials[g], MaterialShading::Unshaded);
        materials->setBlendMode(mGlowMaterials[g], MaterialBlend::Add);
        materials->setCull(mGlowMaterials[g], MaterialCull::Disabled);
        materials->setBillboard(mGlowMaterials[g], MaterialBillboard::FixedY, true);
        materials->setColor(mGlowMaterials[g], MaterialParam::Albedo, kGlowTint[g]);
        if (glowTexture.valid())
            materials->setTexture(mGlowMaterials[g], MaterialTexture::Albedo, glowTexture);
        mGlowMeshes[g] = meshes->create("glow");
        meshes->addSurface(mGlowMeshes[g], glow, "glow");
        meshes->setSurfaceMaterial(mGlowMeshes[g], 0, mGlowMaterials[g]);
        MeshInstance3D* node = createChild<MeshInstance3D>();
        node->setInternal(true);
        node->setMesh(mGlowMeshes[g]);
        node->setCastShadows(false);
        node->setPosition(Math::Vec3(0.0f, 0.0f, kGlowAlong[g]));
        node->setScale(Math::Vec3(kGlowWide[g], kGlowWide[g], kGlowWide[g]));
        node->setVisible(mBurning);
        mGlows[g] = node;
    }
    tintGlows();
}

void BoostJet3D::release()
{
    for (MeshInstance3D*& jet: mJets)
    {
        if (jet) NodeFactory::destroy(jet);
        jet = nullptr;
    }
    for (MeshInstance3D*& glow: mGlows)
    {
        if (glow) NodeFactory::destroy(glow);
        glow = nullptr;
    }
    MeshStore* meshes = Main::meshes();
    MaterialStore* materials = Main::materials();
    TextureStore* textures = Main::textures();
    for (int g = 0; g < kGlows; ++g)
    {
        if (meshes && mGlowMeshes[g].valid()) meshes->destroy(mGlowMeshes[g]);
        if (materials && mGlowMaterials[g].valid()) materials->destroy(mGlowMaterials[g]);
        mGlowMeshes[g] = MeshId();
        mGlowMaterials[g] = MaterialId();
    }
    if (meshes && mFlameMesh.valid()) meshes->destroy(mFlameMesh);
    if (materials && mFlameMaterial.valid()) materials->destroy(mFlameMaterial);
    if (textures && mFlameTexture.valid()) textures->destroy(mFlameTexture);
    mFlameMesh = MeshId();
    mFlameMaterial = MaterialId();
    mFlameTexture = TextureId();
    mBuilt = false;
}

void BoostJet3D::flicker(float step)
{
    if (!mBurning) return;
    mFlicker += step;
    for (int layer = 0; layer < 2; ++layer)
    {
        if (!mJets[layer]) continue;
        const float phase = mFlicker * (layer == 0 ? 47.0f : 71.0f) + mPhase;
        const float waver =
                0.5f * sinf(phase) + 0.3f * sinf(phase * 2.3f + 1.0f) + 0.2f * sinf(phase * 5.1f);
        const float length = (layer == 0 ? 2.0f : 1.1f) * (1.0f + 0.16f * waver);
        const float thick = (layer == 0 ? 0.50f : 0.30f) * (1.0f - 0.08f * waver);
        mJets[layer]->setScale(Math::Vec3(thick, thick, length));
    }
}

void BoostJet3D::notificationHandler(int what)
{
    Node3D::notificationHandler(what);
    switch (what)
    {
        case kNotificationEnterTree:
            build();
            break;
        case kNotificationProcess:
            flicker(static_cast<float>(getProcessDeltaTime()));
            break;
        default:
            break;
    }
}

void BoostJet3D::bindProperties()
{
    using DB = NodeClassDB;
    using P = BoostJet3D;
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "burning"), &P::setBurning, &P::isBurning);
    DB::addProperty<P>(PropertyInfo::file("glow_texture", "*.png,*.jpg,*.tga,*.dds"),
            &P::setGlowTexture, &P::getGlowTexture);
    DB::addProperty<P>(PropertyInfo(PropertyType::Float, "phase"), &P::setPhase, &P::getPhase);
    DB::addProperty<P>(PropertyInfo(PropertyType::Bool, "jets"), &P::setJets, &P::hasJets);
    DB::addProperty<P>(PropertyInfo::color("glow_color"), &P::setGlowColor, &P::getGlowColor);
}

} // namespace prisma3d
