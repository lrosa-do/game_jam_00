#include "prisma3d/editor/EditorGizmo.h"
#include "prisma3d/editor/EditorNodeIcons.h"
#include "prisma3d/editor/EditorUndoRedoManager.h"

#include "prisma3d/render/Batch.h"
#include "prisma3d/render/MeshStore.h"
#include "prisma3d/scene/Collision.h"
#include "prisma3d/scene/Node3D.h"
#include "prisma3d/scene/SceneTree.h"
#include "prisma3d/scene/VisualInstance3D.h"

#include <math.h>

namespace prisma3d
{

namespace
{

using Math::Mat3;
using Math::Mat4;
using Math::Vec3;
using Math::Vec4;

const float kPi = 3.14159265358979323846f;
const float kEpsilon = 0.00001f;
const float kDegreesToRadians = kPi / 180.0f;
const float kRadiansToDegrees = 180.0f / kPi;
const float kViewBaseHeight = 400.0f;
const float kRingThickness = 0.02f;
const float kRingOrientation = 0.005f;
const float kPickInflate = 0.001f;
const float kLineLength = 1048576.0f;

struct PlaneEquation
{
    Vec3 normal;
    float d;

    PlaneEquation(const Vec3& n, const Vec3& point)
            : normal(n),
              d(n.Dot(point))
    {
    }

    bool intersectsRay(const Vec3& from, const Vec3& dir, Vec3& out) const
    {
        const float den = normal.Dot(dir);
        if (fabsf(den) < kEpsilon) return false;
        float dist = (normal.Dot(from) - d) / den;
        if (dist > kEpsilon) return false;
        dist = -dist;
        out = from + dir * dist;
        return true;
    }
};

Vec3 normalized(const Vec3& v)
{
    const float length = v.Length();
    return length > 0.0f ? v / length : Vec3(0.0f, 0.0f, 0.0f);
}

Vec3 absolute(const Vec3& v)
{
    return Vec3(fabsf(v.x), fabsf(v.y), fabsf(v.z));
}

int minAxisIndex(const Vec3& v)
{
    return v.x < v.y ? (v.x < v.z ? 0 : 2) : (v.y < v.z ? 1 : 2);
}

Vec3 directionTo(const Vec3& from, const Vec3& to)
{
    return normalized(to - from);
}

double snapped(double value, double step)
{
    if (step != 0.0) value = floor(value / step + 0.5) * step;
    return value;
}

Vec3 snapped(const Vec3& value, double step)
{
    return Vec3(static_cast<float>(snapped(value.x, step)),
            static_cast<float>(snapped(value.y, step)), static_cast<float>(snapped(value.z, step)));
}

Mat3 axisAngle(const Vec3& axis, float angle)
{
    const float c = cosf(angle);
    const float s = sinf(angle);
    const float t = 1.0f - c;
    const float x = axis.x;
    const float y = axis.y;
    const float z = axis.z;
    return Mat3(Vec3(t * x * x + c, t * x * y + s * z, t * x * z - s * y),
            Vec3(t * x * y - s * z, t * y * y + c, t * y * z + s * x),
            Vec3(t * x * z + s * y, t * y * z - s * x, t * z * z + c));
}

Vec3 xformInverse(const Mat3& basis, const Vec3& v)
{
    return Vec3(basis.col0.Dot(v), basis.col1.Dot(v), basis.col2.Dot(v));
}

Mat3 scaleLocal(const Mat3& basis, const Vec3& scale)
{
    return Mat3(basis.col0 * scale.x, basis.col1 * scale.y, basis.col2 * scale.z);
}

Mat3 orthonormalized(const Mat3& basis)
{
    Vec3 x = basis.col0.Normalized();
    Vec3 y = basis.col1 - x * x.Dot(basis.col1);
    y = y.Normalized();
    Vec3 z = basis.col2 - x * x.Dot(basis.col2) - y * y.Dot(basis.col2);
    z = z.Normalized();
    return Mat3(x, y, z);
}

Mat3 scaledOrthogonal(const Mat3& basis, const Vec3& scale)
{
    Vec3 s = Vec3(-1.0f, -1.0f, -1.0f) + scale;
    const bool sign = s.x + s.y + s.z < 0.0f;
    const Mat3 b = orthonormalized(basis);
    s = xformInverse(b, s);
    Vec3 dots(0.0f, 0.0f, 0.0f);
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) dots[j] += s[i] * fabsf(basis[i].Normalized().Dot(b[j]));
    if (sign != (dots.x + dots.y + dots.z < 0.0f)) dots = -dots;
    return scaleLocal(basis, Vec3(1.0f, 1.0f, 1.0f) + dots);
}

Mat3 lookingAt(const Vec3& target, const Vec3& up)
{
    const Vec3 z = -target.Normalized();
    Vec3 x = up.Cross(z);
    if (x.Length() < kEpsilon)
    {
        const Vec3 helper = fabsf(up.x) < 0.9f ? Vec3(1.0f, 0.0f, 0.0f) : Vec3(0.0f, 1.0f, 0.0f);
        x = up.Cross(helper);
    }
    x = x.Normalized();
    const Vec3 y = z.Cross(x);
    return Mat3(x, y, z);
}

float signedAngleTo(const Vec3& from, const Vec3& to, const Vec3& axis)
{
    const Vec3 cross = from.Cross(to);
    const float angle = atan2f(cross.Length(), from.Dot(to));
    return cross.Dot(axis) < 0.0f ? -angle : angle;
}

bool segmentIntersectsSphere(const Vec3& from, const Vec3& to, const Vec3& center, float radius,
        Vec3* hit, Vec3* normal)
{
    const Vec3 spherePos = center - from;
    const Vec3 rel = to - from;
    const float relLength = rel.Length();
    if (relLength < kEpsilon) return false;
    const Vec3 direction = rel / relLength;
    const float sphereDistance = direction.Dot(spherePos);
    const float rayDistance = Vec3::Distance(spherePos, direction * sphereDistance);
    if (rayDistance >= radius) return false;
    const float squared = radius * radius - rayDistance * rayDistance;
    float distance = sphereDistance;
    if (squared >= kEpsilon) distance -= sqrtf(squared);
    if (distance < 0.0f || distance > relLength) return false;
    const Vec3 result = from + direction * distance;
    if (hit) *hit = result;
    if (normal) *normal = normalized(result - center);
    return true;
}

struct Color
{
    float r;
    float g;
    float b;
    float a;
};

float hue(const Color& c)
{
    float low = c.r < c.g ? c.r : c.g;
    low = low < c.b ? low : c.b;
    float high = c.r > c.g ? c.r : c.g;
    high = high > c.b ? high : c.b;
    const float delta = high - low;
    if (delta == 0.0f) return 0.0f;
    float h = 0.0f;
    if (c.r == high) h = (c.g - c.b) / delta;
    else if (c.g == high)
        h = 2.0f + (c.b - c.r) / delta;
    else
        h = 4.0f + (c.r - c.g) / delta;
    h /= 6.0f;
    if (h < 0.0f) h += 1.0f;
    return h;
}

float saturation(const Color& c)
{
    float low = c.r < c.g ? c.r : c.g;
    low = low < c.b ? low : c.b;
    float high = c.r > c.g ? c.r : c.g;
    high = high > c.b ? high : c.b;
    return high != 0.0f ? (high - low) / high : 0.0f;
}

Color fromHsv(float h, float s, float v, float alpha)
{
    Color c = { v, v, v, alpha };
    if (s == 0.0f) return c;
    h *= 6.0f;
    h = fmodf(h, 6.0f);
    const int i = static_cast<int>(floorf(h));
    const float f = h - static_cast<float>(i);
    const float p = v * (1.0f - s);
    const float q = v * (1.0f - s * f);
    const float t = v * (1.0f - s * (1.0f - f));
    switch (i)
    {
        case 0:
            c.r = v;
            c.g = t;
            c.b = p;
            break;
        case 1:
            c.r = q;
            c.g = v;
            c.b = p;
            break;
        case 2:
            c.r = p;
            c.g = v;
            c.b = t;
            break;
        case 3:
            c.r = p;
            c.g = q;
            c.b = v;
            break;
        case 4:
            c.r = t;
            c.g = p;
            c.b = v;
            break;
        default:
            c.r = v;
            c.g = p;
            c.b = q;
            break;
    }
    return c;
}

bool finite(const Mat4& m)
{
    const float* data = m.Data();
    for (int i = 0; i < 16; ++i)
        if (!isfinite(data[i])) return false;
    return true;
}

const Vec3 kIVec(0.0f, 0.0f, -1.0f);
const Vec3 kNIVec(-1.0f, -1.0f, 0.0f);
const Vec3 kIVec2(-1.0f, 0.0f, 0.0f);
const Vec3 kIVec3(0.0f, -1.0f, 0.0f);

void revolve(const Vec3* profile, int count, int sides, ct::Vector<Vec3>& out)
{
    const float step = 2.0f * kPi / static_cast<float>(sides);
    for (int k = 0; k < sides; ++k)
    {
        const Mat3 ma = axisAngle(kIVec, static_cast<float>(k) * step);
        const Mat3 mb = axisAngle(kIVec, static_cast<float>(k + 1) * step);
        for (int j = 0; j < count - 1; ++j)
        {
            const Vec3 p0 = ma * profile[j];
            const Vec3 p1 = mb * profile[j];
            const Vec3 p2 = mb * profile[j + 1];
            const Vec3 p3 = ma * profile[j + 1];
            out.push_back(p0);
            out.push_back(p1);
            out.push_back(p2);
            out.push_back(p0);
            out.push_back(p2);
            out.push_back(p3);
        }
    }
}

} // namespace

EditorGizmo::EditorGizmo(SceneTree& tree, EditorSelection& selection)
        : mTree(tree),
          mSelection(selection),
          mListener(nullptr),
          mTool(Tool::Transform),
          mLocal(false),
          mSnap(false),
          mShown(true),
          mModifiers(0),
          mVisible(false),
          mTransform(Mat4::Identity()),
          mScale(1.0f),
          mHighlight(-1),
          mClicked(nullptr),
          mSelectionInProgress(false),
          mClickedWantsAppend(false),
          mEmptyClick(false),
          mLeftDown(false),
          mEdits(0)
{
    mTree.addListener(this);
    buildMeshes();
}

EditorGizmo::~EditorGizmo()
{
    mTree.removeListener(this);
}

void EditorGizmo::setChangeListener(EditorChangeListener* listener)
{
    mListener = listener;
}

Node* EditorGizmo::sceneRoot() const
{
    return mRootProvider ? mRootProvider() : mTree.getRoot();
}

const EditorSelection& EditorGizmo::selection() const
{
    return mSelection;
}

void EditorGizmo::setRootProvider(const ct::Function<Node*()>& provider)
{
    mRootProvider = provider;
}

EditorGizmo::Settings& EditorGizmo::settings()
{
    return mSettings;
}

const EditorGizmo::Settings& EditorGizmo::settings() const
{
    return mSettings;
}

void EditorGizmo::setTool(Tool tool)
{
    if (tool == mTool) return;
    if (mEdit.mode != Mode::None) commit();
    mTool = tool;
    mHighlight = -1;
}

EditorGizmo::Tool EditorGizmo::tool() const
{
    return mTool;
}

void EditorGizmo::setLocalSpace(bool local)
{
    mLocal = local;
    update();
}

bool EditorGizmo::localSpace() const
{
    return mLocal;
}

void EditorGizmo::setSnapEnabled(bool enabled)
{
    mSnap = enabled;
}

bool EditorGizmo::snapEnabled() const
{
    return mSnap;
}

void EditorGizmo::setShown(bool shown)
{
    mShown = shown;
}

bool EditorGizmo::shown() const
{
    return mShown;
}

bool EditorGizmo::snapActive() const
{
    const bool keyDown = (mModifiers & kModControl) != 0;
    return mSnap != keyDown;
}

float EditorGizmo::translateSnap() const
{
    float value = mSettings.translateSnap;
    if (mModifiers & kModShift) value /= 10.0f;
    return value;
}

float EditorGizmo::rotateSnap() const
{
    float value = mSettings.rotateSnap;
    if (mModifiers & kModShift) value /= 3.0f;
    return value;
}

float EditorGizmo::scaleSnap() const
{
    float value = mSettings.scaleSnap;
    if (mModifiers & kModShift) value /= 2.0f;
    return value;
}

void EditorGizmo::setModifiers(unsigned modifiers)
{
    mModifiers = modifiers;
}

unsigned EditorGizmo::modifiers() const
{
    return mModifiers;
}

void EditorGizmo::setView(const EditorView& view)
{
    mView = view;
}

const EditorView& EditorGizmo::view() const
{
    return mView;
}

bool EditorGizmo::visible() const
{
    return mVisible && mShown;
}

const Math::Mat4& EditorGizmo::transform() const
{
    return mTransform;
}

float EditorGizmo::scale() const
{
    return mScale;
}

int EditorGizmo::highlight() const
{
    return mHighlight;
}

EditorGizmo::Mode EditorGizmo::mode() const
{
    return mEdit.mode;
}

EditorGizmo::Plane EditorGizmo::plane() const
{
    return mEdit.plane;
}

bool EditorGizmo::editing() const
{
    return mEdit.mode != Mode::None;
}

unsigned int EditorGizmo::editCount() const
{
    return mEdits;
}

void EditorGizmo::nodeRemoved(Node* node)
{
    if (node == mClicked) mClicked = nullptr;
    bool used = false;
    for (std::size_t i = 0; i < mTargets.size(); ++i)
        if (mTargets[i].node == node) used = true;
    if (used)
    {
        mTargets.clear();
        mEdit.mode = Mode::None;
        mLeftDown = false;
    }
}

bool EditorGizmo::hasSelection() const
{
    return mVisible;
}

void EditorGizmo::notifyEdited()
{
    if (mListener) mListener->sceneEdited();
}

Math::Vec3 EditorGizmo::cameraNormal() const
{
    return mView.forward();
}

void EditorGizmo::buildMeshes()
{
    {
        const Vec3 arrow[5] = { kNIVec * 0.0f + kIVec * 0.0f, kNIVec * 0.01f + kIVec * 0.0f,
            kNIVec * 0.01f + kIVec * kArrowOffset, kNIVec * kArrowRadius + kIVec * kArrowOffset,
            kNIVec * 0.0f + kIVec * (kArrowOffset + kArrowSize) };
        revolve(arrow, 5, 16, mMove.positions);
    }
    {
        const Vec3 arrow[6] = { kNIVec * 0.0f + kIVec * 0.0f, kNIVec * 0.01f + kIVec * 0.0f,
            kNIVec * 0.01f + kIVec * kScaleOffset,
            kNIVec * 0.5f * kScaleSize + kIVec * kScaleOffset,
            kNIVec * 0.5f * kScaleSize + kIVec * (kScaleOffset + kScaleSize),
            kNIVec * 0.0f + kIVec * (kScaleOffset + kScaleSize) };
        revolve(arrow, 6, 4, mScaleBox.positions);
    }
    {
        const Vec3 vec = kIVec2 - kIVec3;
        const Vec3 plane[4] = { vec * kPlaneDst, vec * kPlaneDst + kIVec2 * kPlaneSize,
            vec * (kPlaneDst + kPlaneSize), vec * kPlaneDst - kIVec3 * kPlaneSize };
        const Mat3 ma = axisAngle(kIVec, kPi / 2.0f);
        const Vec3 points[4] = { ma * plane[0], ma * plane[1], ma * plane[2], ma * plane[3] };
        mPlane.positions.push_back(points[0]);
        mPlane.positions.push_back(points[1]);
        mPlane.positions.push_back(points[2]);
        mPlane.positions.push_back(points[0]);
        mPlane.positions.push_back(points[2]);
        mPlane.positions.push_back(points[3]);
    }
    {
        const int n = 128;
        const int m = 3;
        const float step = 2.0f * kPi / static_cast<float>(n);
        for (int j = 0; j < n; ++j)
        {
            const Mat3 basis = axisAngle(kIVec, static_cast<float>(j) * step);
            const Vec3 vertex = basis * (kIVec2 * kCircleSize);
            for (int k = 0; k < m; ++k)
            {
                const float angle = (2.0f * kPi * static_cast<float>(k)) / static_cast<float>(m);
                const Vec3 normal = kIVec * cosf(angle) + kIVec2 * sinf(angle);
                mRing.positions.push_back(vertex);
                mRing.normals.push_back(basis * normal);
            }
        }
        for (int j = 0; j < n; ++j)
        {
            for (int k = 0; k < m; ++k)
            {
                const int currentRing = j * m;
                const int nextRing = ((j + 1) % n) * m;
                const int currentSegment = k;
                const int nextSegment = (k + 1) % m;
                mRing.indices.push_back(static_cast<std::uint32_t>(currentRing + nextSegment));
                mRing.indices.push_back(static_cast<std::uint32_t>(currentRing + currentSegment));
                mRing.indices.push_back(static_cast<std::uint32_t>(nextRing + currentSegment));
                mRing.indices.push_back(static_cast<std::uint32_t>(nextRing + currentSegment));
                mRing.indices.push_back(static_cast<std::uint32_t>(nextRing + nextSegment));
                mRing.indices.push_back(static_cast<std::uint32_t>(currentRing + nextSegment));
            }
        }
    }
}

void EditorGizmo::update()
{
    const ct::Vector<Node*> top = mSelection.topSelectedNodes();
    Vec3 sum(0.0f, 0.0f, 0.0f);
    Mat3 basis = Mat3::Identity();
    int count = 0;
    for (std::size_t i = 0; i < top.size(); ++i)
    {
        Node3D* node = top[i]->castTo<Node3D>();
        if (!node || !node->isVisibleInTree()) continue;
        const Mat4 xf = node->getGlobalTransform();
        if (!finite(xf)) continue;
        sum += xf.GetTranslation();
        if (mLocal) basis = xf.UpperLeft3x3();
        ++count;
    }
    mVisible = count > 0;
    mTransform = Mat4(basis, count > 0 ? sum / static_cast<float>(count) : Vec3(0.0f, 0.0f, 0.0f));
    if (!mVisible)
    {
        mHighlight = -1;
        return;
    }

    const Vec3 origin = mTransform.GetTranslation();
    const Vec3 cameraPosition = mView.position();
    if ((origin - cameraPosition).Length() < kEpsilon) return;
    const Vec3 camz = mView.forward();
    const Vec3 camy = -mView.up();
    const float planeDistance = camz.Dot(origin - cameraPosition);
    const float gizmoDistance = fabsf(planeDistance) > kEpsilon ? fabsf(planeDistance) : kEpsilon;
    const float d0 = mView.project(cameraPosition + camz * gizmoDistance).y;
    const float d1 = mView.project(cameraPosition + camz * gizmoDistance + camy).y;
    const float dd = fabsf(d0 - d1) > kEpsilon ? fabsf(d0 - d1) : kEpsilon;
    const float baseHeight = kViewBaseHeight;
    const float height = mView.height < baseHeight ? mView.height : baseHeight;
    mScale = (mSettings.size / dd) * height / baseHeight;
}

Math::Vec3 EditorGizmo::gizmoColumn(int axis) const
{
    return mTransform[axis].xyz().Normalized();
}

Math::Mat4 EditorGizmo::axisTransform(int axis) const
{
    Mat3 basis = Mat3::Identity();
    const Vec3 a = mTransform[axis].xyz().Normalized();
    const Vec3 b = mTransform[(axis + 1) % 3].xyz().Normalized();
    if (a.Dot(b) < 1.0f) basis = lookingAt(a, b);
    basis = scaleLocal(basis, Vec3(mScale, mScale, mScale));
    return Mat4(basis, mTransform.GetTranslation());
}

Math::Mat4 EditorGizmo::planeTransform(int axis) const
{
    Mat4 plane = axisTransform(axis);
    if (!mView.orthogonal())
    {
        const Vec3 target =
                mEdit.mode == Mode::None ? plane.GetTranslation() : mEdit.original.GetTranslation();
        const Vec3 dir = directionTo(mView.position(), target);
        for (int j = 0; j < 3; ++j)
        {
            const float sign = plane[j].xyz().Normalized().Dot(dir) > kEpsilon ? -1.0f : 1.0f;
            plane[j] = Vec4(plane[j].xyz() * sign, 0.0f);
        }
    }
    return plane;
}

Math::Vec4 EditorGizmo::highlightColor(const Math::Vec4& color)
{
    const Color base = { color.x, color.y, color.z, color.w };
    const Color c = fromHsv(hue(base), saturation(base) * 0.25f, 1.0f, 1.0f);
    return Vec4(c.r, c.g, c.b, c.a);
}

void EditorGizmo::drawTriangles(BatchRenderer& batch, const MeshData& mesh, const Math::Mat4& model,
        const Math::Vec4& color, bool cull)
{
    batch.setColor(color.x, color.y, color.z, color.w);
    const Vec3 eye = mView.position();
    const Vec3 forward = mView.forward();
    batch.begin(BatchRenderer::ModeTriangles);
    for (std::size_t i = 0; i + 2 < mesh.positions.size(); i += 3)
    {
        const Vec3 a = model.TransformPoint(mesh.positions[i]);
        const Vec3 b = model.TransformPoint(mesh.positions[i + 1]);
        const Vec3 c = model.TransformPoint(mesh.positions[i + 2]);
        if (cull)
        {
            const Vec3 normal = (b - a).Cross(c - a);
            const Vec3 toTriangle = mView.orthogonal() ? forward : a - eye;
            if (normal.Dot(toTriangle) <= 0.0f) continue;
        }
        batch.vertex3(a.x, a.y, a.z);
        batch.vertex3(c.x, c.y, c.z);
        batch.vertex3(b.x, b.y, b.z);
    }
    batch.end();
}

void EditorGizmo::drawRing(BatchRenderer& batch, int axis, const Math::Vec4& color)
{
    const Mat4 model = axisTransform(axis);
    const Mat3 viewRotation = mView.camera.UpperLeft3x3().Transposed();
    const Mat3 modelView = orthonormalized(viewRotation * model.UpperLeft3x3());
    mScratch.clear();
    for (std::size_t i = 0; i < mRing.positions.size(); ++i)
    {
        Vec3 vertex = mRing.positions[i];
        const Vec3 n = modelView * vertex;
        const float orientation = Vec3(0.0f, 0.0f, -1.0f).Dot(n);
        if (orientation <= kRingOrientation) vertex += mRing.normals[i] * kRingThickness;
        mScratch.push_back(model.TransformPoint(vertex));
    }

    batch.setColor(color.x, color.y, color.z, color.w);
    const Vec3 eye = mView.position();
    const Vec3 forward = mView.forward();
    batch.begin(BatchRenderer::ModeTriangles);
    for (std::size_t i = 0; i + 2 < mRing.indices.size(); i += 3)
    {
        const Vec3 a = mScratch[mRing.indices[i]];
        const Vec3 b = mScratch[mRing.indices[i + 1]];
        const Vec3 c = mScratch[mRing.indices[i + 2]];
        const Vec3 normal = (b - a).Cross(c - a);
        if (normal.LengthSquared() < 1e-14f) continue;
        const Vec3 toTriangle = mView.orthogonal() ? forward : a - eye;
        if (normal.Dot(toTriangle) <= 0.0f) continue;
        batch.vertex3(a.x, a.y, a.z);
        batch.vertex3(c.x, c.y, c.z);
        batch.vertex3(b.x, b.y, b.z);
    }
    batch.end();
}

void EditorGizmo::draw(BatchRenderer& batch)
{
    if (!visible() || mTool == Tool::Select) return;
    const Vec3 origin = mTransform.GetTranslation();
    if ((origin - mView.position()).Length() < kEpsilon) return;
    if (fabsf(mTransform.UpperLeft3x3().Determinant()) < kEpsilon * kEpsilon) return;

    const bool showMove = mTool == Tool::Transform || mTool == Tool::Move;
    const bool showRotate = mTool == Tool::Transform || mTool == Tool::Rotate;
    const bool showScale = mTool == Tool::Scale;

    const BatchRenderer::Space space = batch.space();
    batch.setSpace(BatchRenderer::Space::World);
    batch.pushMatrix();
    batch.loadIdentity();
    batch.setTexture(batch.whiteTexture());
    batch.setDepthTest(false);
    batch.setDepthWrite(false);
    batch.setBlend(true);
    batch.setBlendMode(BatchRenderer::BlendMode::Alpha);
    batch.setCullFace(false);

    Vec4 colors[3];
    Vec4 highlights[3];
    for (int i = 0; i < 3; ++i)
    {
        colors[i] = mSettings.axisColor[i];
        colors[i].w *= mSettings.opacity;
        highlights[i] = highlightColor(colors[i]);
    }

    for (int i = 0; i < 3; ++i)
    {
        if (showMove)
        {
            const bool hl = mHighlight == kHighlightMovePlane + i;
            drawTriangles(batch, mPlane, planeTransform(i), hl ? highlights[i] : colors[i], false);
        }
        if (showScale)
        {
            const bool hl = mHighlight == kHighlightScalePlane + i;
            drawTriangles(batch, mPlane, planeTransform(i), hl ? highlights[i] : colors[i], false);
        }
    }
    for (int i = 0; i < 3; ++i)
    {
        if (showMove)
        {
            const bool hl = mHighlight == kHighlightMove + i;
            drawTriangles(batch, mMove, axisTransform(i), hl ? highlights[i] : colors[i], true);
        }
        if (showScale)
        {
            const bool hl = mHighlight == kHighlightScale + i;
            drawTriangles(batch, mScaleBox, axisTransform(i), hl ? highlights[i] : colors[i], true);
        }
    }
    if (showRotate)
    {
        for (int i = 0; i < 3; ++i)
        {
            const bool hl = mHighlight == kHighlightRotate + i;
            drawRing(batch, i, hl ? highlights[i] : colors[i]);
        }
    }

    if (mEdit.mode != Mode::None)
    {
        const bool axes[3] = { mEdit.plane == Plane::XAxis || mEdit.plane == Plane::XY ||
                                       mEdit.plane == Plane::XZ,
            mEdit.plane == Plane::YAxis || mEdit.plane == Plane::XY || mEdit.plane == Plane::YZ,
            mEdit.plane == Plane::ZAxis || mEdit.plane == Plane::XZ || mEdit.plane == Plane::YZ };
        for (int i = 0; i < 3; ++i)
        {
            if (!axes[i]) continue;
            const Vec3 direction = gizmoColumn(i);
            const Vec3 a = origin - direction * kLineLength;
            const Vec3 b = origin + direction * kLineLength;
            batch.setColor(highlights[i].x, highlights[i].y, highlights[i].z, highlights[i].w);
            batch.begin(BatchRenderer::ModeLines);
            batch.vertex3(a.x, a.y, a.z);
            batch.vertex3(b.x, b.y, b.z);
            batch.end();
        }
    }

    batch.popMatrix();
    batch.setDefault3DState();
    batch.setBlendMode(BatchRenderer::BlendMode::Alpha);
    batch.setColor(1.0f, 1.0f, 1.0f, 1.0f);
    batch.setSpace(space);
}

float EditorGizmo::screenCost(const Math::Vec3& center, float radius, const Math::Vec3& pos) const
{
    const Vec3 up = mView.up();
    const Math::Vec2 screenPos = mView.project(center);
    const float screenSize = Math::Vec2::Distance(screenPos, mView.project(center + up * radius));
    const float screenDistance = Math::Vec2::Distance(screenPos, mView.project(pos));
    const float ratio = screenDistance / (screenSize > 1.0f ? screenSize : 1.0f);
    return ratio * ratio;
}

bool EditorGizmo::pickPlane(const Math::Mat4& gt, const Math::Vec3& rayPos, const Math::Vec3& ray,
        float& cost, int& axis)
{
    Mat4 pt = gt;
    if (!mView.orthogonal())
    {
        const Vec3 dir = directionTo(mView.position(), pt.GetTranslation());
        for (int j = 0; j < 3; ++j)
        {
            const float sign = pt[j].xyz().Normalized().Dot(dir) > kEpsilon ? -1.0f : 1.0f;
            pt[j] = Vec4(pt[j].xyz() * sign, 0.0f);
        }
    }
    bool found = false;
    for (int i = 0; i < 3; ++i)
    {
        const Vec3 ivec2 = pt[(i + 1) % 3].xyz().Normalized();
        const Vec3 ivec3 = pt[(i + 2) % 3].xyz().Normalized();
        const Vec3 grabber =
                pt.GetTranslation() + (ivec2 + ivec3) * mScale * (kPlaneDst + kPlaneSize * 0.5f);
        const float grabberSize = mScale * kPlaneSize * 0.5f;
        const PlaneEquation plane(pt[i].xyz().Normalized(), pt.GetTranslation());
        Vec3 r;
        if (!plane.intersectsRay(rayPos, ray, r)) continue;
        const float dist = Vec3::Distance(r, grabber);
        const float effective = grabberSize * 1.5f;
        if (dist >= effective) continue;
        const float c = screenCost(grabber, effective, r);
        if (c < cost)
        {
            cost = c;
            axis = i;
            found = true;
        }
    }
    return found;
}

bool EditorGizmo::selectMove(const Math::Vec3& rayPos, const Math::Vec3& ray, bool highlightOnly,
        float x, float y)
{
    const Vec3 origin = mTransform.GetTranslation();
    int axis = -1;
    float cost = 1e20f;
    for (int i = 0; i < 3; ++i)
    {
        const Vec3 grabber = origin + gizmoColumn(i) * mScale * (kArrowOffset + kArrowSize * 0.5f);
        const float radius = mScale * kArrowSize * 0.5f;
        const float effective = radius * 1.5f;
        Vec3 r;
        if (!segmentIntersectsSphere(rayPos, rayPos + ray * kMaxZ, grabber, effective, &r, nullptr))
            continue;
        const float c = screenCost(grabber, effective, r);
        if (c < cost)
        {
            cost = c;
            axis = i;
        }
    }

    bool isPlane = false;
    int planeAxis = -1;
    if (pickPlane(mTransform, rayPos, ray, cost, planeAxis))
    {
        axis = planeAxis;
        isPlane = true;
    }

    if (axis == -1) return false;
    if (highlightOnly)
    {
        mHighlight = axis + (isPlane ? kHighlightMovePlane : kHighlightMove);
        return true;
    }
    mEdit.mode = Mode::Translate;
    computeEdit(x, y);
    mEdit.plane = static_cast<Plane>(static_cast<int>(Plane::XAxis) + axis + (isPlane ? 3 : 0));
    return true;
}

bool EditorGizmo::selectRotate(const Math::Vec3& rayPos, const Math::Vec3& ray, bool highlightOnly,
        float x, float y)
{
    const Vec3 origin = mTransform.GetTranslation();
    const Vec3 camera = cameraNormal();
    int axis = -1;

    const float rayLength = Vec3::Distance(origin, rayPos) + (kCircleSize * mScale) * 4.0f;
    Vec3 hit;
    Vec3 hitNormal;
    if (segmentIntersectsSphere(rayPos, rayPos + ray * rayLength, origin, mScale * kCircleSize,
                &hit, &hitNormal))
    {
        if (hitNormal.Dot(camera) < 0.05f)
        {
            const Vec3 local = absolute(xformInverse(mTransform.UpperLeft3x3(), hit - origin));
            const int minAxis = minAxisIndex(local);
            if (local[minAxis] < mScale * kRingHalfWidth) axis = minAxis;
        }
    }

    if (axis == -1)
    {
        float best = 1e20f;
        for (int i = 0; i < 3; ++i)
        {
            const PlaneEquation plane(gizmoColumn(i), origin);
            Vec3 r;
            if (!plane.intersectsRay(rayPos, ray, r)) continue;
            const float dist = Vec3::Distance(r, origin);
            const Vec3 direction = normalized(r - origin);
            if (camera.Dot(direction) > kRingOrientation) continue;
            if (dist > mScale * (kCircleSize - kRingHalfWidth) &&
                    dist < mScale * (kCircleSize + kRingHalfWidth))
            {
                const float d = Vec3::Distance(rayPos, r);
                if (d < best)
                {
                    best = d;
                    axis = i;
                }
            }
        }
    }

    if (axis == -1) return false;
    if (highlightOnly)
    {
        mHighlight = axis + kHighlightRotate;
        return true;
    }
    mEdit.mode = Mode::Rotate;
    computeEdit(x, y);
    mEdit.plane = static_cast<Plane>(static_cast<int>(Plane::XAxis) + axis);
    mEdit.accumulatedRotationAngle = 0.0;
    mEdit.rotationAngle = 0.0;
    return true;
}

bool EditorGizmo::selectScale(const Math::Vec3& rayPos, const Math::Vec3& ray, bool highlightOnly,
        float x, float y)
{
    const Vec3 origin = mTransform.GetTranslation();
    int axis = -1;
    float cost = 1e20f;
    for (int i = 0; i < 3; ++i)
    {
        const Vec3 grabber = origin + gizmoColumn(i) * mScale * (kScaleOffset + kScaleSize * 0.5f);
        const float radius = mScale * kScaleSize * 0.5f;
        const float effective = radius * 1.5f;
        Vec3 r;
        if (!segmentIntersectsSphere(rayPos, rayPos + ray * kMaxZ, grabber, effective, &r, nullptr))
            continue;
        const float c = screenCost(grabber, effective, r);
        if (c < cost)
        {
            cost = c;
            axis = i;
        }
    }

    bool isPlane = false;
    int planeAxis = -1;
    if (pickPlane(mTransform, rayPos, ray, cost, planeAxis))
    {
        axis = planeAxis;
        isPlane = true;
    }

    if (axis == -1) return false;
    if (highlightOnly)
    {
        mHighlight = axis + (isPlane ? kHighlightScalePlane : kHighlightScale);
        return true;
    }
    mEdit.mode = Mode::Scale;
    computeEdit(x, y);
    mEdit.plane = static_cast<Plane>(static_cast<int>(Plane::XAxis) + axis + (isPlane ? 3 : 0));
    return true;
}

bool EditorGizmo::selectGizmo(float x, float y, bool highlightOnly)
{
    if (!visible()) return false;
    if (highlightOnly) mHighlight = -1;
    const Vec3 rayPos = mView.rayOrigin(x, y);
    const Vec3 ray = mView.rayDirection(x, y);

    if (mTool == Tool::Transform || mTool == Tool::Move)
        if (selectMove(rayPos, ray, highlightOnly, x, y)) return true;
    if (mTool == Tool::Transform || mTool == Tool::Rotate)
        if (selectRotate(rayPos, ray, highlightOnly, x, y)) return true;
    if (mTool == Tool::Scale)
        if (selectScale(rayPos, ray, highlightOnly, x, y)) return true;
    return false;
}

void EditorGizmo::computeEdit(float x, float y)
{
    mEdit.originalLocal = mLocal;
    mEdit.clickRay = mView.rayDirection(x, y);
    mEdit.clickRayPos = mView.rayOrigin(x, y);
    mEdit.plane = Plane::View;
    update();
    mEdit.center = mTransform.GetTranslation();

    mTargets.clear();
    const ct::Vector<Node*> top = mSelection.topSelectedNodes();
    for (std::size_t i = 0; i < top.size(); ++i)
    {
        Node3D* node = top[i]->castTo<Node3D>();
        if (!node) continue;
        Target target;
        target.node = node;
        target.originalLocal = node->getTransform();
        target.original = node->getGlobalTransform();
        target.position = node->getPosition();
        target.rotation = node->getRotation();
        target.scale = node->getScale();
        mTargets.push_back(target);
    }
}

void EditorGizmo::beginTransform(Mode mode, float x, float y)
{
    if (!hasSelection()) return;
    mEdit.mode = mode;
    computeEdit(x, y);
    mEdit.initialClickVector = Vec3(0.0f, 0.0f, 0.0f);
    mEdit.previousRotationVector = Vec3(0.0f, 0.0f, 0.0f);
    mEdit.accumulatedRotationAngle = 0.0;
    mEdit.rotationAngle = 0.0;
    mEdit.viewAxisLocal = normalized(xformInverse(mTransform.UpperLeft3x3(), cameraNormal()));
}

void EditorGizmo::finishTransform()
{
    mEdit.mode = Mode::None;
    mEdit.initialClickVector = Vec3(0.0f, 0.0f, 0.0f);
    mEdit.previousRotationVector = Vec3(0.0f, 0.0f, 0.0f);
    mEdit.accumulatedRotationAngle = 0.0;
    mEdit.rotationAngle = 0.0;
    mLocal = mEdit.originalLocal;
    mTargets.clear();
    update();
    mClicked = nullptr;
}

void EditorGizmo::cancel()
{
    if (mEdit.mode == Mode::None) return;
    for (std::size_t i = 0; i < mTargets.size(); ++i)
    {
        Node3D* node = mTargets[i].node;
        node->setPosition(mTargets[i].position);
        node->setRotation(mTargets[i].rotation);
        node->setScale(mTargets[i].scale);
    }
    mLeftDown = false;
    finishTransform();
}

void EditorGizmo::commit()
{
    if (mEdit.mode == Mode::None) return;
    ++mEdits;
    recordTransform();
    finishTransform();
}

void EditorGizmo::recordTransform()
{
    EditorUndoRedoManager* undo = mListener ? mListener->undoRedoManager() : nullptr;
    if (!undo)
    {
        notifyEdited();
        return;
    }

    struct Trs
    {
        Node3D* node;
        Math::Vec3 position;
        Math::Vec3 rotation;
        Math::Vec3 scale;
    };
    ct::Vector<Trs> before;
    ct::Vector<Trs> after;
    for (std::size_t i = 0; i < mTargets.size(); ++i)
    {
        Node3D* node = mTargets[i].node;
        const Math::Vec3 position = node->getPosition();
        const Math::Vec3 rotation = node->getRotation();
        const Math::Vec3 scale = node->getScale();
        const Target& target = mTargets[i];
        if (position == target.position && rotation == target.rotation && scale == target.scale)
            continue;
        before.push_back(Trs{ node, target.position, target.rotation, target.scale });
        after.push_back(Trs{ node, position, rotation, scale });
    }
    if (before.empty()) return;

    const char* name = mEdit.mode == Mode::Translate ? kTranslateAction
                       : mEdit.mode == Mode::Rotate  ? kRotateAction
                                                     : kScaleAction;
    const auto apply = [](const ct::Vector<Trs>& list)
    {
        for (std::size_t i = 0; i < list.size(); ++i)
        {
            list[i].node->setPosition(list[i].position);
            list[i].node->setRotation(list[i].rotation);
            list[i].node->setScale(list[i].scale);
        }
    };
    undo->createAction(name);
    undo->addDoMethod([apply, after] { apply(after); });
    undo->addUndoMethod([apply, before] { apply(before); });
    undo->commitAction(false);
}

void EditorGizmo::applyTo(Node3D* node, const Math::Mat4& transform, bool local)
{
    if (fabsf(transform.UpperLeft3x3().Determinant()) < 1e-12f) return;
    if (local) node->setTransform(transform);
    else
        node->setGlobalTransform(transform);
}

Math::Mat4 EditorGizmo::computeTransform(Mode mode, const Math::Mat4& original,
        const Math::Mat4& originalLocal, Math::Vec3 motion, double extra, bool local,
        bool orthogonal, bool viewAxis) const
{
    switch (mode)
    {
        case Mode::Scale:
        {
            if (snapActive()) motion = snapped(motion, extra);
            const Vec3 factor = motion + Vec3(1.0f, 1.0f, 1.0f);
            if (local)
            {
                return Mat4(scaleLocal(originalLocal.UpperLeft3x3(), factor),
                        originalLocal.GetTranslation());
            }
            const Mat3 diagonal(Vec3(factor.x, 0.0f, 0.0f), Vec3(0.0f, factor.y, 0.0f),
                    Vec3(0.0f, 0.0f, factor.z));
            const Mat4 base = Mat4::Translation(mEdit.center);
            const Mat4 scaled(diagonal, Vec3(0.0f, 0.0f, 0.0f));
            Mat4 s = base * (scaled * (base.InverseAffine() * original));
            if (orthogonal)
            {
                const Mat3 basis = scaledOrthogonal(original.UpperLeft3x3(), factor);
                s = Mat4(basis, s.GetTranslation());
            }
            return s;
        }
        case Mode::Translate:
        {
            if (snapActive()) motion = snapped(motion, extra);
            if (local)
            {
                const Mat3 basis = originalLocal.UpperLeft3x3();
                return Mat4(basis, originalLocal.GetTranslation() + basis * motion);
            }
            return Mat4(original.UpperLeft3x3(), original.GetTranslation() + motion);
        }
        case Mode::Rotate:
        {
            const Mat3 parentGlobal =
                    original.UpperLeft3x3() * originalLocal.UpperLeft3x3().Inverse();
            Vec3 axis;
            if (local && !viewAxis) axis = originalLocal.UpperLeft3x3() * motion;
            else
                axis = xformInverse(parentGlobal, motion);
            const float angle = static_cast<float>(extra);
            if (local)
            {
                const Mat3 basis =
                        axisAngle(normalized(axis), angle) * originalLocal.UpperLeft3x3();
                return Mat4(basis, originalLocal.GetTranslation());
            }
            const Mat3 basis = parentGlobal * axisAngle(normalized(axis), angle) *
                               originalLocal.UpperLeft3x3();
            const Vec3 origin =
                    axisAngle(motion, angle) * (original.GetTranslation() - mEdit.center) +
                    mEdit.center;
            return Mat4(basis, origin);
        }
        default:
            break;
    }
    return original;
}

void EditorGizmo::applyTransform(const Math::Vec3& motion, double snap)
{
    const bool local = mLocal && !(mEdit.plane == Plane::View && mEdit.mode != Mode::Rotate);
    const bool globalViewPlane =
            mEdit.plane == Plane::View && (mEdit.mode != Mode::Rotate || !mLocal);
    for (std::size_t i = 0; i < mTargets.size(); ++i)
    {
        const Target& target = mTargets[i];
        const Mat4 xform = computeTransform(mEdit.mode, target.original, target.originalLocal,
                motion, snap, local, mEdit.plane != Plane::View, globalViewPlane);
        applyTo(target.node, xform, local);
    }
    update();
}

void EditorGizmo::updateTransform(bool shift)
{
    const Vec3 rayPos = mView.rayOrigin(mEdit.mouseX, mEdit.mouseY);
    const Vec3 ray = mView.rayDirection(mEdit.mouseX, mEdit.mouseY);
    const Vec3 camera = cameraNormal();
    const bool local = mLocal && !(mEdit.plane == Plane::View && mEdit.mode != Mode::Rotate);
    const Mat3 gizmoBasis = mTransform.UpperLeft3x3();
    double snap = 0.001;

    switch (mEdit.mode)
    {
        case Mode::Scale:
        case Mode::Translate:
        {
            const bool scaling = mEdit.mode == Mode::Scale;
            Vec3 motionMask(0.0f, 0.0f, 0.0f);
            Vec3 planeNormal = camera;
            bool planeMove = false;
            const int axis = static_cast<int>(mEdit.plane) - static_cast<int>(Plane::XAxis);
            switch (mEdit.plane)
            {
                case Plane::View:
                    planeNormal = camera;
                    break;
                case Plane::XAxis:
                case Plane::YAxis:
                case Plane::ZAxis:
                    motionMask = gizmoBasis[axis].Normalized();
                    planeNormal = normalized(motionMask.Cross(motionMask.Cross(camera)));
                    break;
                case Plane::YZ:
                    motionMask = scaling ? gizmoBasis[2].Normalized() + gizmoBasis[1].Normalized()
                                         : Vec3(0.0f, 0.0f, 0.0f);
                    planeNormal = gizmoBasis[0].Normalized();
                    planeMove = true;
                    break;
                case Plane::XZ:
                    motionMask = scaling ? gizmoBasis[2].Normalized() + gizmoBasis[0].Normalized()
                                         : Vec3(0.0f, 0.0f, 0.0f);
                    planeNormal = gizmoBasis[1].Normalized();
                    planeMove = true;
                    break;
                case Plane::XY:
                    motionMask = scaling ? gizmoBasis[0].Normalized() + gizmoBasis[1].Normalized()
                                         : Vec3(0.0f, 0.0f, 0.0f);
                    planeNormal = gizmoBasis[2].Normalized();
                    planeMove = true;
                    break;
            }
            const PlaneEquation plane(planeNormal, mEdit.center);
            Vec3 intersection;
            if (!plane.intersectsRay(rayPos, ray, intersection)) break;
            Vec3 click;
            if (!plane.intersectsRay(mEdit.clickRayPos, mEdit.clickRay, click)) break;

            Vec3 motion = intersection - click;
            if (scaling)
            {
                if (mEdit.plane != Plane::View)
                {
                    if (!planeMove) motion = motionMask * motionMask.Dot(motion);
                    else if (shift)
                        motion = motionMask * motionMask.Dot(motion);
                }
                else
                {
                    const float clickDistance = Vec3::Distance(click, mEdit.center);
                    const float intersectionDistance = Vec3::Distance(intersection, mEdit.center);
                    if (clickDistance == 0.0f) break;
                    const float scale = intersectionDistance - clickDistance;
                    motion = Vec3(scale, scale, scale);
                }
                motion = motion / Vec3::Distance(click, mEdit.center);
                if (snapActive()) snap = scaleSnap() / 100.0;
                if (local) motion = mEdit.original.UpperLeft3x3().Inverse() * motion;
            }
            else
            {
                if (mEdit.plane != Plane::View && !planeMove)
                    motion = motionMask * motionMask.Dot(motion);
                if (snapActive()) snap = translateSnap();
                if (local) motion = mTransform.UpperLeft3x3().Inverse() * motion;
            }
            applyTransform(motion, snap);
            break;
        }
        case Mode::Rotate:
        {
            PlaneEquation plane(camera, mEdit.center);
            if (!mView.orthogonal())
            {
                const Vec3 toObject = mEdit.center - mView.position();
                if (toObject.Length() > kEpsilon)
                    plane = PlaneEquation(normalized(toObject), mEdit.center);
            }

            Vec3 localAxis(0.0f, 0.0f, 0.0f);
            Vec3 globalAxis(0.0f, 0.0f, 0.0f);
            switch (mEdit.plane)
            {
                case Plane::View:
                    localAxis = mEdit.viewAxisLocal;
                    globalAxis = camera;
                    break;
                case Plane::XAxis:
                    localAxis = Vec3(1.0f, 0.0f, 0.0f);
                    break;
                case Plane::YAxis:
                    localAxis = Vec3(0.0f, 1.0f, 0.0f);
                    break;
                case Plane::ZAxis:
                    localAxis = Vec3(0.0f, 0.0f, 1.0f);
                    break;
                default:
                    break;
            }
            if (mEdit.plane != Plane::View) globalAxis = normalized(gizmoBasis * localAxis);

            Vec3 intersection;
            if (!plane.intersectsRay(rayPos, ray, intersection)) break;
            Vec3 click;
            if (!plane.intersectsRay(mEdit.clickRayPos, mEdit.clickRay, click)) break;

            const Vec3 currentRotationVector = normalized(intersection - mEdit.center);
            if (mEdit.initialClickVector.Length() == 0.0f)
            {
                const PlaneEquation rotationPlane(globalAxis, mEdit.center);
                Vec3 clickOnPlane;
                if (rotationPlane.intersectsRay(mEdit.clickRayPos, mEdit.clickRay, clickOnPlane))
                    mEdit.initialClickVector = normalized(clickOnPlane - mEdit.center);
                else
                    mEdit.initialClickVector = normalized(click - mEdit.center);
                mEdit.previousRotationVector = currentRotationVector;
                mEdit.accumulatedRotationAngle = 0.0;
                mEdit.rotationAngle = 0.0;
            }

            const float orthogonalThreshold = cosf(85.0f * kDegreesToRadians);
            const bool axisIsOrthogonal = fabsf(plane.normal.Dot(globalAxis)) < orthogonalThreshold;

            if (mEdit.previousRotationVector.Length() != 0.0f)
            {
                mEdit.accumulatedRotationAngle += static_cast<double>(signedAngleTo(
                        mEdit.previousRotationVector, currentRotationVector, globalAxis));
            }
            mEdit.previousRotationVector = currentRotationVector;

            if (snapActive()) snap = rotateSnap();

            if (axisIsOrthogonal)
            {
                const Vec3 projectionAxis = plane.normal.Cross(globalAxis);
                const Vec3 delta = intersection - click;
                const float projection = delta.Dot(projectionAxis);
                const double orthAngle = (projection * (kPi / 2.0)) / (mScale * kCircleSize);
                mEdit.rotationAngle = snapActive() ? snapped(orthAngle * kRadiansToDegrees, snap) *
                                                             kDegreesToRadians
                                                   : orthAngle;
            }
            else
            {
                mEdit.rotationAngle =
                        snapActive() ? snapped(mEdit.accumulatedRotationAngle * kRadiansToDegrees,
                                               snap) *
                                               kDegreesToRadians
                                     : mEdit.accumulatedRotationAngle;
            }

            const Vec3 computeAxis = local ? localAxis : globalAxis;
            applyTransform(computeAxis, mEdit.rotationAngle);
            break;
        }
        default:
            break;
    }
}

EditorNodeIcons::Settings& EditorGizmo::iconSettings()
{
    return mIconSettings;
}

Node3D* EditorGizmo::pick(float x, float y) const
{
    Node* root = mRootProvider ? mRootProvider() : mTree.getRoot();
    if (!root) return nullptr;
    const Vec3 origin = mView.rayOrigin(x, y);
    const Vec3 direction = mView.rayDirection(x, y);
    MeshStore* meshes = MeshStore::singleton();

    Node3D* best = nullptr;
    float bestDistance = 1e20f;
    ct::Vector<Node*> stack;
    stack.push_back(root);
    while (!stack.empty())
    {
        Node* node = stack.back();
        stack.pop_back();
        for (std::size_t i = 0; i < node->getChildCount(); ++i) stack.push_back(node->getChild(i));

        Node3D* spatial = node->castTo<Node3D>();
        if (!spatial || !spatial->isVisibleInTree()) continue;

        const float iconDistance =
                EditorNodeIcons::pickDistance(mView, *spatial, mIconSettings, x, y);
        if (iconDistance >= 0.0f && iconDistance < bestDistance)
        {
            bestDistance = iconDistance;
            best = spatial;
        }

        VisualInstance3D* visual = node->castTo<VisualInstance3D>();
        if (!visual) continue;

        const Mat4 inverse = visual->getGlobalTransform().InverseAffine();
        const Vec3 localOrigin = inverse.TransformPoint(origin);
        const Vec3 localDirection = inverse.TransformVector(direction);
        const float scale = localDirection.Length();
        if (scale < kEpsilon) continue;

        float distance = -1.0f;
        const CollisionMesh* collision = nullptr;
        if (MeshInstance3D* instance = visual->castTo<MeshInstance3D>())
            if (meshes) collision = meshes->collision(instance->getMesh());
        if (collision && collision->valid())
        {
            SweepHit hit;
            if (collision->raycast(Math::Ray(localOrigin, localDirection / scale), kMaxZ, hit))
                distance = hit.t / scale;
        }
        else
        {
            const Math::Box box = visual->getAabb().Expanded(kPickInflate);
            float tMin = 0.0f;
            float tMax = 0.0f;
            if (box.IntersectRay(Math::Ray(localOrigin, localDirection), tMin, tMax) &&
                    tMax >= 0.0f)
                distance = tMin > 0.0f ? tMin : 0.0f;
        }
        if (distance < 0.0f || distance >= bestDistance) continue;
        bestDistance = distance;
        best = visual;
    }
    return best;
}

void EditorGizmo::selectClicked()
{
    Node3D* node = mClicked;
    mClicked = nullptr;
    if (!node) return;

    if (mClickedWantsAppend)
    {
        const ct::Vector<Node*> top = mSelection.topSelectedNodes();
        const Node* active = top.empty() ? nullptr : top.back();
        if (mSelection.isSelected(node))
        {
            mSelection.remove(node);
            if (node != active) mSelection.add(node);
        }
        else
            mSelection.add(node);
    }
    else if (!mSelection.isSelected(node))
    {
        mSelection.clear();
        mSelection.add(node);
    }
    update();
}

void EditorGizmo::clearSelected()
{
    mSelection.clear();
    update();
}

bool EditorGizmo::onLeftPress(float x, float y)
{
    mLeftDown = true;
    mClickedWantsAppend = (mModifiers & kModShift) != 0;
    mEdit.mouseX = x;
    mEdit.mouseY = y;
    mEdit.originalMouseX = x;
    mEdit.originalMouseY = y;
    mEdit.mode = Mode::None;
    mEdit.original = mTransform;
    mClicked = nullptr;
    mSelectionInProgress = false;
    mEmptyClick = false;
    update();
    mEdit.original = mTransform;

    if (visible() && selectGizmo(x, y, false)) return true;

    mClicked = pick(x, y);
    const bool nodeSelected = hasSelection();
    const bool alt = (mModifiers & kModAlt) != 0;
    const bool control = (mModifiers & kModControl) != 0;

    if (mClicked && !mSelection.isSelected(mClicked))
    {
        if (!nodeSelected || (!alt && !(mTool == Tool::Transform && control)))
        {
            mSelectionInProgress = true;
            return true;
        }
    }

    if (!mClicked)
    {
        if (nodeSelected)
        {
            Mode mode = Mode::None;
            if (mTool == Tool::Transform)
            {
                if (control) mode = Mode::Rotate;
                else if (alt)
                    mode = Mode::Translate;
            }
            else if (alt)
            {
                switch (mTool)
                {
                    case Tool::Rotate:
                        mode = Mode::Rotate;
                        break;
                    case Tool::Move:
                        mode = Mode::Translate;
                        break;
                    case Tool::Scale:
                        mode = Mode::Scale;
                        break;
                    default:
                        break;
                }
            }
            if (mode != Mode::None)
            {
                beginTransform(mode, x, y);
                return true;
            }
        }
        mEmptyClick = true;
        return true;
    }

    if (!mClickedWantsAppend)
    {
        const bool clickedSelected = mSelection.isSelected(mClicked);
        Mode mode = Mode::None;
        switch (mTool)
        {
            case Tool::Transform:
                if (control && nodeSelected) mode = Mode::Rotate;
                else if (alt && nodeSelected)
                    mode = Mode::Translate;
                else if (clickedSelected)
                    mode = Mode::Translate;
                break;
            case Tool::Rotate:
                if (clickedSelected || (alt && nodeSelected)) mode = Mode::Rotate;
                break;
            case Tool::Move:
                if (clickedSelected || (alt && nodeSelected)) mode = Mode::Translate;
                break;
            case Tool::Scale:
                if (clickedSelected || (alt && nodeSelected)) mode = Mode::Scale;
                break;
            default:
                break;
        }
        if (mode != Mode::None) beginTransform(mode, x, y);
    }
    return true;
}

bool EditorGizmo::onLeftRelease(float x, float y)
{
    if (!mLeftDown) return false;
    mLeftDown = false;
    mEdit.mouseX = x;
    mEdit.mouseY = y;

    const bool moved = Math::Vec2::Distance(Math::Vec2(mEdit.originalMouseX, mEdit.originalMouseY),
                               Math::Vec2(x, y)) > kMovementThreshold;
    mSelectionInProgress = false;
    if (mClicked && mEdit.mode == Mode::None) selectClicked();
    else if (mEmptyClick && !moved)
        clearSelected();
    mEmptyClick = false;
    mClicked = nullptr;

    if (mEdit.mode != Mode::None)
    {
        if (mEdit.originalMouseX != x || mEdit.originalMouseY != y) commit();
        else
            finishTransform();
    }
    return true;
}

bool EditorGizmo::onMotion(float x, float y, unsigned buttonMask)
{
    mEdit.mouseX = x;
    mEdit.mouseY = y;
    const bool left = (buttonMask & mouseButtonToMask(MouseButton::Left)) != 0;

    if (!left && mEdit.mode == Mode::None)
    {
        if (visible()) selectGizmo(x, y, true);
        else
            mHighlight = -1;
        return false;
    }
    if (!left || !mLeftDown) return false;

    const bool thresholdPassed =
            Math::Vec2::Distance(Math::Vec2(mEdit.originalMouseX, mEdit.originalMouseY),
                    Math::Vec2(x, y)) > kMovementThreshold;
    if (mSelectionInProgress && thresholdPassed)
    {
        mClicked = nullptr;
        mSelectionInProgress = false;
    }
    if (mClicked && thresholdPassed && (mTool == Tool::Transform || mTool == Tool::Move))
    {
        const bool selectMode = mTool == Tool::Transform;
        const bool clickedSelected = mSelection.isSelected(mClicked);
        if (mEdit.mode == Mode::None && (selectMode || clickedSelected) && hasSelection())
        {
            beginTransform(Mode::Translate, mEdit.originalMouseX, mEdit.originalMouseY);
            mClicked = nullptr;
        }
    }
    if (mEdit.mode == Mode::None) return true;
    updateTransform((mModifiers & kModShift) != 0);
    return true;
}

bool EditorGizmo::onKey(const InputEvent& event)
{
    if (!event.key.pressed) return false;
    if (event.key.echo) return false;
    const Key key = event.key.key;
    if (key == Key::Escape)
    {
        if (mEdit.mode != Mode::None)
        {
            cancel();
            return true;
        }
        if (!mSelection.selectedNodes().empty())
        {
            clearSelected();
            return true;
        }
        return false;
    }
    if (event.key.modifiers & (kModControl | kModAlt | kModSuper)) return false;
    switch (key)
    {
        case Key::Q:
            setTool(Tool::Transform);
            return true;
        case Key::W:
            setTool(Tool::Move);
            return true;
        case Key::E:
            setTool(Tool::Rotate);
            return true;
        case Key::R:
            setTool(Tool::Scale);
            return true;
        case Key::V:
            setTool(Tool::Select);
            return true;
        case Key::T:
            if (mEdit.mode == Mode::None) setLocalSpace(!mLocal);
            return true;
        case Key::Y:
            setSnapEnabled(!mSnap);
            return true;
        default:
            return false;
    }
}

bool EditorGizmo::handleEvent(const InputEvent& event, float x, float y)
{
    switch (event.type)
    {
        case InputEventType::Key:
        {
            const Key key = event.key.key;
            const bool pressed = event.key.pressed;
            unsigned bit = 0;
            if (key == Key::LeftControl || key == Key::RightControl) bit = kModControl;
            else if (key == Key::LeftShift || key == Key::RightShift)
                bit = kModShift;
            else if (key == Key::LeftAlt || key == Key::RightAlt)
                bit = kModAlt;
            if (bit)
            {
                mModifiers = pressed ? (mModifiers | bit) : (mModifiers & ~bit);
                return false;
            }
            return onKey(event);
        }
        case InputEventType::MouseMotion:
            return onMotion(x, y, event.mouseMotion.buttonMask);
        case InputEventType::MouseButton:
        {
            mModifiers = (mModifiers & ~(kModShift | kModControl | kModAlt)) |
                         (event.mouseButton.modifiers & (kModShift | kModControl | kModAlt));
            if (event.mouseButton.button == MouseButton::Left)
            {
                if (event.mouseButton.pressed) return onLeftPress(x, y);
                return onLeftRelease(x, y);
            }
            if (event.mouseButton.button == MouseButton::Right && event.mouseButton.pressed &&
                    mEdit.mode != Mode::None)
            {
                cancel();
                return true;
            }
            return false;
        }
        default:
            return false;
    }
}

} // namespace prisma3d
