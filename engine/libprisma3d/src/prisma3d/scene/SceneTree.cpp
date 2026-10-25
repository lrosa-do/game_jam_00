#include "prisma3d/scene/SceneTree.h"

#include "prisma3d/scene/Skeleton3D.h"
#include "prisma3d/scene/Tween.h"

#include "NodeFactory.h"
#include "prisma3d/Device.h"
#include "prisma3d/Input.h"
#include "prisma3d/audio/AudioServer.h"
#include "prisma3d/physics/PhysicsServer3D.h"
#include "prisma3d/render/RenderProfiler.h"
#include "prisma3d/scene/AudioListener3D.h"
#include "prisma3d/scene/Camera3D.h"
#include "prisma3d/scene/Collision.h"
#include "prisma3d/scene/Node.h"
#include "prisma3d/scene/Node3D.h"
#include "prisma3d/scene/SceneSerializer.h"
#include "prisma3d/scene/SceneTreeListener.h"
#include "prisma3d/scene/ScriptServer.h"
#include "prisma3d/scene/Viewport.h"

#include "platform.h"

#include <ct/sort.hpp>

namespace prisma3d
{

namespace
{

bool inTreeOrder(const Node* a, const Node* b)
{
    if (a == b) return false;
    ct::Vector<std::ptrdiff_t> first;
    ct::Vector<std::ptrdiff_t> second;
    for (const Node* node = a; node; node = node->getParent()) first.push_back(node->getIndex());
    for (const Node* node = b; node; node = node->getParent()) second.push_back(node->getIndex());

    std::size_t i = first.size();
    std::size_t j = second.size();
    while (i > 0 && j > 0)
    {
        const std::ptrdiff_t left = first[--i];
        const std::ptrdiff_t right = second[--j];
        if (left != right) return left < right;
    }
    return i == 0 && j > 0;
}

} // namespace

SceneTree* SceneTree::sSingleton = nullptr;

SceneTree::SceneTree()
        : mRoot(nullptr),
          mTransformChanged(),
          mDeleteQueue(),
          mTweens(),
          mListeners(),
          mNotifyDepth(0),
          mListenersDirty(false),
          mFrame(0),
          mNodeCount(0),
          mProcessDelta(0.0),
          mPhysicsDelta(0.0),
          mCloseRequestSent(false),
          mExitCode(0),
          mQuit(false),
          mAutoAcceptQuit(true),
          mPaused(false),
          mInitialized(false),
          mInputHandled(false),
          mScenePending(false),
          mCurrentScene(nullptr),
          mCurrentScenePath(),
          mPendingScenePath(),
          mPendingTransition(Transition::None),
          mPendingTransitionSeconds(0.0),
          mTransition(Transition::None),
          mTransitionSeconds(0.0),
          mTransitionElapsed(0.0),
          mTransitionPixels(),
          mTransitionWidth(0),
          mTransitionHeight(0),
          mTransitionFresh(false),
          mProcessHook(),
          mCollisionWorld(new CollisionWorld())
{
    if (!sSingleton) sSingleton = this;
}

SceneTree::~SceneTree()
{
    if (mInitialized) finalize();
    if (sSingleton == this) sSingleton = nullptr;
    delete mCollisionWorld;
}

SceneTree* SceneTree::singleton()
{
    return sSingleton;
}

Node* SceneTree::getRoot() const
{
    return mRoot;
}

void SceneTree::initialize()
{
    if (mInitialized) return;

    MainLoop::initialize();

    mRoot = NodeFactory::make<Window>();
    mRoot->setName("root");
    mRoot->setProcessMode(Node::ProcessMode::Pausable);
    mInitialized = true;
    mCloseRequestSent = false;
    mRoot->setTree(this);
}

bool SceneTree::physicsProcess(double time)
{
    flushTransformNotifications();

    if (MainLoop::physicsProcess(time)) mQuit = true;

    mPhysicsDelta = time;

    if (mRoot)
    {
        bool quit = mQuit;
        processNodes(mRoot, Node::kNotificationPhysicsProcess, quit);
    }

    processTweens(time, true);

    flushSkeletonUpdates();
    flushTransformNotifications();

    if (!mPaused)
    {
        PhysicsServer3D* physics = PhysicsServer3D::singleton();
        if (physics)
        {
            ProfileScope scope("PhysicsStep");
            physics->step(time);
        }
        flushTransformNotifications();
    }

    flushDeleteQueue();

    return mQuit;
}

bool SceneTree::process(double time)
{
    Device* device = Device::singleton();
    if (device && device->closeRequested() && !mCloseRequestSent)
    {
        mCloseRequestSent = true;
        notification(Node::kNotificationWmCloseRequest);
        // A game that asks before closing (auto accept quit off) keeps its window open and hears
        // every later close too.
        if (!mAutoAcceptQuit && !mQuit)
        {
            device->cancelClose();
            mCloseRequestSent = false;
        }
    }

    if (MainLoop::process(time)) mQuit = true;

    mProcessDelta = time;

    if (ScriptServer* scripts = ScriptServer::singleton()) scripts->update(time);
    if (mTransition != Transition::None)
    {
        mTransitionElapsed += time;
        if (mTransitionElapsed >= mTransitionSeconds) mTransition = Transition::None;
    }
    flushSceneChange();

    flushTransformNotifications();

    if (mRoot)
    {
        bool quit = mQuit;
        processNodes(mRoot, Node::kNotificationProcess, quit);
        drawNodes(mRoot);
        updateDefaultListener();
        if (mProcessHook) mProcessHook();
    }

    processTweens(time, false);

    flushSkeletonUpdates();

    {
        ProfileScope scope("Collisions");
        mCollisionWorld->update();
    }

    flushTransformNotifications();
    flushDeleteQueue();

    ++mFrame;

    return mQuit;
}

void SceneTree::finalize()
{
    flushDeleteQueue();

    if (mRoot)
    {
        Node* root = mRoot;
        root->setTree(nullptr);
        root->propagateAfterExitTree();
        mRoot = nullptr;
        delete root;

        flushDeleteQueue();
    }

    mTransformChanged.clear();

    for (std::size_t i = 0; i < mTweens.size(); ++i) delete mTweens[i];
    mTweens.clear();

    mInitialized = false;

    MainLoop::finalize();
}

void SceneTree::notification(int what)
{
    switch (what)
    {
        case Node::kNotificationWmCloseRequest:
            if (mRoot) mRoot->propagateNotification(what);
            if (mAutoAcceptQuit) mQuit = true;
            break;
        case MainLoop::kNotificationOsMemoryWarning:
        case MainLoop::kNotificationWmAbout:
        case MainLoop::kNotificationCrash:
        case MainLoop::kNotificationApplicationResumed:
        case MainLoop::kNotificationApplicationPaused:
        case MainLoop::kNotificationApplicationFocusIn:
        case MainLoop::kNotificationApplicationFocusOut:
            if (mRoot) mRoot->propagateNotification(what);
            break;
        default:
            break;
    }
}

void SceneTree::quit(int exitCode)
{
    mExitCode = exitCode;
    mQuit = true;
}

int SceneTree::getExitCode() const
{
    return mExitCode;
}

void SceneTree::setAutoAcceptQuit(bool enabled)
{
    mAutoAcceptQuit = enabled;
}

bool SceneTree::isAutoAcceptQuit() const
{
    return mAutoAcceptQuit;
}

void SceneTree::setPaused(bool paused)
{
    if (paused == mPaused) return;

    mPaused = paused;

    if (mRoot) mRoot->propagatePauseNotification(paused);
}

bool SceneTree::isPaused() const
{
    return mPaused;
}

std::uint64_t SceneTree::getFrame() const
{
    return mFrame;
}

std::uint32_t SceneTree::getPlanarReflectorCount() const
{
    return mPlanarReflectors;
}

std::size_t SceneTree::getNodeCount() const
{
    return mNodeCount;
}

double SceneTree::getProcessDeltaTime() const
{
    return mProcessDelta;
}

double SceneTree::getPhysicsProcessDeltaTime() const
{
    return mPhysicsDelta;
}

void SceneTree::flushTransformNotifications()
{
    const std::size_t count = mTransformChanged.size();
    for (std::size_t i = 0; i < count; ++i)
    {
        Node3D* node = mTransformChanged[i];
        if (!node) continue;
        node->mTransformChangedQueued = false;
        node->notification(Node::kNotificationTransformChanged);
    }
    mTransformChanged.erase(mTransformChanged.begin(), mTransformChanged.begin() + count);
}

CollisionWorld& SceneTree::getCollisionWorld()
{
    return *mCollisionWorld;
}

void SceneTree::queueDelete(Node* node)
{
    if (!node || node->mQueuedForDeletion) return;
    node->mQueuedForDeletion = true;
    mDeleteQueue.push_back(node);
}

Tween* SceneTree::createTween()
{
    Tween* tween = new Tween(TweenKey());
    tween->mPersistent = true;
    mTweens.push_back(tween);
    return tween;
}

void SceneTree::tweenOnce(const ct::Function<void(Tween&)>& build)
{
    if (!build) return;
    Tween* tween = new Tween(TweenKey());
    mTweens.push_back(tween);
    build(*tween);
}

void SceneTree::addParticleCollider(ParticleCollision3D* collider)
{
    for (std::size_t i = 0; i < mParticleColliders.size(); ++i)
        if (mParticleColliders[i] == collider) return;
    mParticleColliders.push_back(collider);
}

void SceneTree::removeParticleCollider(ParticleCollision3D* collider)
{
    for (std::size_t i = 0; i < mParticleColliders.size(); ++i)
    {
        if (mParticleColliders[i] != collider) continue;
        mParticleColliders.erase(mParticleColliders.begin() + i);
        return;
    }
}

const ct::Vector<ParticleCollision3D*>& SceneTree::getParticleColliders() const
{
    return mParticleColliders;
}

bool SceneTree::hasTween(const Tween* tween) const
{
    for (std::size_t i = 0; i < mTweens.size(); ++i)
        if (mTweens[i] == tween) return true;
    return false;
}

std::size_t SceneTree::getTweenCount() const
{
    return mTweens.size();
}

void SceneTree::dropTweenNode(Node* node)
{
    for (std::size_t i = 0; i < mTweens.size(); ++i) mTweens[i]->dropNode(node);
}

void SceneTree::processTweens(double delta, bool physics)
{
    const std::size_t count = mTweens.size();
    std::size_t index = 0;
    for (std::size_t visited = 0; visited < count; ++visited)
    {
        Tween* tween = mTweens[index];

        const bool skip = !tween->canProcess(mPaused) ||
                          (physics == (tween->getProcessMode() == TweenProcessMode::Idle));
        if (!skip && !tween->step(delta))
        {
            mTweens.erase(mTweens.begin() + index);
            delete tween;
            continue;
        }
        ++index;
    }
}

void SceneTree::addListener(SceneTreeListener* listener)
{
    if (!listener) return;
    for (std::size_t i = 0; i < mListeners.size(); ++i)
        if (mListeners[i] == listener) return;
    mListeners.push_back(listener);
}

void SceneTree::removeListener(SceneTreeListener* listener)
{
    for (std::size_t i = 0; i < mListeners.size(); ++i)
    {
        if (mListeners[i] != listener) continue;
        if (mNotifyDepth > 0)
        {
            mListeners[i] = nullptr;
            mListenersDirty = true;
        }
        else
            mListeners.erase(mListeners.begin() + i);
        return;
    }
}

void SceneTree::endNotify()
{
    if (--mNotifyDepth > 0 || !mListenersDirty) return;
    mListenersDirty = false;
    std::size_t kept = 0;
    for (std::size_t i = 0; i < mListeners.size(); ++i)
        if (mListeners[i]) mListeners[kept++] = mListeners[i];
    mListeners.resize(kept);
}

void SceneTree::treeChanged()
{
    const std::size_t count = mListeners.size();
    ++mNotifyDepth;
    for (std::size_t i = 0; i < count; ++i)
        if (SceneTreeListener* listener = mListeners[i]) listener->treeChanged();
    endNotify();
}

void SceneTree::nodeAdded(Node* node)
{
    ++mNodeCount;
    const std::size_t count = mListeners.size();
    ++mNotifyDepth;
    for (std::size_t i = 0; i < count; ++i)
        if (SceneTreeListener* listener = mListeners[i]) listener->nodeAdded(node);
    endNotify();
}

void SceneTree::nodeRemoved(Node* node)
{
    if (mNodeCount > 0) --mNodeCount;
    if (node == mCurrentScene) mCurrentScene = nullptr;
    const std::size_t count = mListeners.size();
    ++mNotifyDepth;
    for (std::size_t i = 0; i < count; ++i)
        if (SceneTreeListener* listener = mListeners[i]) listener->nodeRemoved(node);
    endNotify();
}

void SceneTree::nodeRenamed(Node* node)
{
    const std::size_t count = mListeners.size();
    ++mNotifyDepth;
    for (std::size_t i = 0; i < count; ++i)
        if (SceneTreeListener* listener = mListeners[i]) listener->nodeRenamed(node);
    endNotify();
}

void SceneTree::flushSkeletonUpdates()
{
    ProfileScope scope("Skeletons");
    const std::size_t count = mSkeletonUpdates.size();
    for (std::size_t i = 0; i < count; ++i)
    {
        Skeleton3D* skeleton = mSkeletonUpdates[i];
        if (skeleton) skeleton->updateSkeleton();
    }
    mSkeletonUpdates.erase(mSkeletonUpdates.begin(), mSkeletonUpdates.begin() + count);
}

void SceneTree::queueSkeletonUpdate(Skeleton3D* skeleton)
{
    for (std::size_t i = 0; i < mSkeletonUpdates.size(); ++i)
        if (mSkeletonUpdates[i] == skeleton) return;
    mSkeletonUpdates.push_back(skeleton);
}

void SceneTree::dropSkeletonUpdate(Skeleton3D* skeleton)
{
    for (std::size_t i = 0; i < mSkeletonUpdates.size(); ++i)
        if (mSkeletonUpdates[i] == skeleton) mSkeletonUpdates[i] = nullptr;
}

void SceneTree::queueTransformChanged(Node3D* node)
{
    if (node->mTransformChangedQueued) return;
    node->mTransformChangedQueued = true;
    mTransformChanged.push_back(node);
}

void SceneTree::addToGroup(const ct::String& group, Node* node)
{
    ct::Vector<Node*>* nodes = mGroups.find(group);
    if (!nodes)
    {
        mGroups.put(group, ct::Vector<Node*>());
        nodes = mGroups.find(group);
    }
    for (std::size_t i = 0; i < nodes->size(); ++i)
        if ((*nodes)[i] == node) return;
    nodes->push_back(node);
}

void SceneTree::removeFromGroup(const ct::String& group, Node* node)
{
    ct::Vector<Node*>* nodes = mGroups.find(group);
    if (!nodes) return;
    for (std::size_t i = 0; i < nodes->size(); ++i)
    {
        if ((*nodes)[i] != node) continue;
        nodes->erase(nodes->begin() + i);
        break;
    }
    if (nodes->empty()) mGroups.erase(group);
}

bool SceneTree::hasGroup(const ct::String& group) const
{
    return mGroups.contains(group);
}

std::size_t SceneTree::getNodeCountInGroup(const ct::String& group) const
{
    const ct::Vector<Node*>* nodes = mGroups.find(group);
    return nodes ? nodes->size() : 0;
}

Node* SceneTree::getFirstNodeInGroup(const ct::String& group) const
{
    ct::Vector<Node*> nodes;
    getNodesInGroup(group, nodes);
    return nodes.empty() ? nullptr : nodes[0];
}

void SceneTree::getNodesInGroup(const ct::String& group, ct::Vector<Node*>& nodes) const
{
    nodes.clear();
    const ct::Vector<Node*>* found = mGroups.find(group);
    if (!found) return;
    for (std::size_t i = 0; i < found->size(); ++i) nodes.push_back((*found)[i]);
    ct::sort(nodes.begin(), nodes.end(), [](Node* a, Node* b) { return inTreeOrder(a, b); });
}

void SceneTree::flushDeleteQueue()
{
    while (!mDeleteQueue.empty())
    {
        Node* node = mDeleteQueue.front();
        mDeleteQueue.erase(mDeleteQueue.begin());
        node->mQueuedForDeletion = false;

        if (node == mRoot) mRoot = nullptr;
        if (node->mParent) node->mParent->removeChild(node);

        delete node;
    }
}

void SceneTree::inputEvent(const InputEvent& event)
{
    if (!mRoot || Node::countProcessingInput() == 0) return;

    ct::Vector<Node*> nodes;
    collectInputNodes(mRoot, nodes);

    mInputHandled = false;
    for (std::size_t i = nodes.size(); i > 0; --i)
    {
        if (mInputHandled) break;
        Node* node = nodes[i - 1];
        if (!node->isInsideTree() || node->isQueuedForDeletion() || !node->isProcessingInput() ||
                !node->canProcess())
            continue;
        node->input(event);
    }
}

void SceneTree::setInputAsHandled()
{
    mInputHandled = true;
}

bool SceneTree::isInputHandled() const
{
    return mInputHandled;
}

void SceneTree::collectInputNodes(Node* node, ct::Vector<Node*>& nodes) const
{
    if (node->isProcessingInput()) nodes.push_back(node);
    for (std::size_t i = 0; i < node->mChildren.size(); ++i)
        collectInputNodes(node->mChildren[i], nodes);
}

void SceneTree::setCurrentScene(Node* scene, const ct::String& path)
{
    mCurrentScene = scene;
    if (!path.empty()) mCurrentScenePath = path;
    else if (scene && !scene->getSceneFilePath().empty())
        mCurrentScenePath = scene->getSceneFilePath();
}

Node* SceneTree::getCurrentScene() const
{
    return mCurrentScene;
}

const ct::String& SceneTree::getCurrentScenePath() const
{
    return mCurrentScenePath;
}

SceneTree::Transition SceneTree::transitionFromName(const ct::String& name)
{
    struct Named
    {
        const char* name;
        Transition transition;
    };
    static const Named kNames[] = { { "fade", Transition::Fade },
        { "slide_left", Transition::SlideLeft }, { "slide_right", Transition::SlideRight },
        { "slide_up", Transition::SlideUp }, { "slide_down", Transition::SlideDown },
        { "zoom_in", Transition::ZoomIn }, { "zoom_out", Transition::ZoomOut },
        { "wipe_left", Transition::WipeLeft }, { "wipe_right", Transition::WipeRight } };
    for (const Named& named: kNames)
        if (name == named.name) return named.transition;
    return Transition::None;
}

void SceneTree::changeSceneToFile(const ct::String& path, Transition transition, double seconds)
{
    mPendingScenePath = path;
    mScenePending = true;
    mPendingTransition = seconds > 0.0 ? transition : Transition::None;
    mPendingTransitionSeconds = seconds;
    if (mPendingTransition == Transition::None) return;
    if (Device* device = Device::singleton()) device->requestFrameCapture();
}

SceneTree::Transition SceneTree::getTransition() const
{
    return mTransition;
}

double SceneTree::getTransitionProgress() const
{
    if (mTransition == Transition::None || mTransitionSeconds <= 0.0) return -1.0;
    const double progress = mTransitionElapsed / mTransitionSeconds;
    return progress > 1.0 ? 1.0 : progress;
}

bool SceneTree::takeTransitionPixels(ct::Vector<std::uint8_t>& pixels, int& width, int& height)
{
    if (!mTransitionFresh) return false;
    mTransitionFresh = false;
    pixels = static_cast<ct::Vector<std::uint8_t>&&>(mTransitionPixels);
    width = mTransitionWidth;
    height = mTransitionHeight;
    mTransitionPixels = ct::Vector<std::uint8_t>();
    return true;
}

void SceneTree::setProcessHook(const ct::Function<void()>& hook)
{
    mProcessHook = hook;
}

bool SceneTree::reloadCurrentScene()
{
    if (mCurrentScenePath.empty()) return false;
    changeSceneToFile(mCurrentScenePath);
    return true;
}

bool SceneTree::isSceneChangePending() const
{
    return mScenePending;
}

void SceneTree::flushSceneChange()
{
    if (!mScenePending || !mRoot) return;
    mScenePending = false;
    const ct::String path = mPendingScenePath;
    mPendingScenePath = ct::String();

    Node holder;
    SceneResult result;
    Node* scene = SceneSerializer::loadFile(path, &holder, result);
    if (!scene)
    {
        log_error("scene: cannot change to '%s': %s", path.c_str(), result.describe().c_str());
        return;
    }

    Node* previous = mCurrentScene;
    mCurrentScene = nullptr;
    if (previous)
    {
        if (previous->getParent()) previous->getParent()->removeChild(previous);
        delete previous;
    }

    scene->reparent(mRoot, true);
    mCurrentScene = scene;
    mCurrentScenePath = path;

    mTransition = Transition::None;
    mTransitionFresh = false;
    if (mPendingTransition != Transition::None)
    {
        Device* device = Device::singleton();
        if (device &&
                device->takeFrameCapture(mTransitionPixels, mTransitionWidth, mTransitionHeight))
        {
            mTransition = mPendingTransition;
            mTransitionSeconds = mPendingTransitionSeconds;
            mTransitionElapsed = 0.0;
            mTransitionFresh = true;
        }
    }
    mPendingTransition = Transition::None;
    mPendingTransitionSeconds = 0.0;
}

void SceneTree::drawNodes(Node* node)
{
    node->notification(Node::kNotificationDraw);
    for (std::size_t i = 0; i < node->mChildren.size(); ++i) drawNodes(node->mChildren[i]);
}

// Without an AudioListener3D the current camera hears: 3D sounds are placed against it, as in
// Godot, and not against the world origin.
void SceneTree::updateDefaultListener()
{
    if (AudioListener3D::getCurrent()) return;
    AudioServer* audio = AudioServer::singleton();
    Viewport* viewport = mRoot ? mRoot->castTo<Viewport>() : nullptr;
    if (!audio || !viewport) return;
    if (Camera3D* camera = viewport->getCamera3D())
        audio->setListenerTransform(camera->getGlobalTransform());
}

void SceneTree::processNodes(Node* node, int notification, bool& quit)
{
    const bool physics = notification == Node::kNotificationPhysicsProcess;
    if ((physics ? node->mPhysicsProcessCount : node->mProcessCount) == 0) return;
    if ((physics ? node->isPhysicsProcessing() : node->isProcessing()) && node->canProcess())
        node->notification(notification);
    quit = mQuit;

    for (std::size_t i = 0; i < node->mChildren.size();)
    {
        Node* child = node->mChildren[i];
        processNodes(child, notification, quit);
        if (i < node->mChildren.size() && node->mChildren[i] == child) ++i;
    }
}

} // namespace prisma3d
