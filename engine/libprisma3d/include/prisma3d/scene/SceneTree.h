#pragma once

#include "prisma3d/main/MainLoop.h"

#include <ct/function.hpp>
#include <ct/hashmap.hpp>
#include <ct/string.hpp>
#include <ct/vector.hpp>

#include <cstdint>

namespace prisma3d
{

class Node;
class Node3D;
class Skeleton3D;
class Tween;
class ParticleCollision3D;

class CollisionWorld;
class SceneTreeListener;
struct InputEvent;

class SceneTree : public MainLoop
{
public:
    SceneTree();
    ~SceneTree() override;
    SceneTree(const SceneTree&) = delete;
    SceneTree& operator=(const SceneTree&) = delete;

    static SceneTree* singleton();

    Node* getRoot() const;

    void initialize() override;
    bool physicsProcess(double time) override;
    bool process(double time) override;
    void finalize() override;
    void notification(int what) override;

    void quit(int exitCode = 0);
    int getExitCode() const;
    void setAutoAcceptQuit(bool enabled);
    bool isAutoAcceptQuit() const;
    void setPaused(bool paused);
    bool isPaused() const;

    std::uint64_t getFrame() const;
    std::size_t getNodeCount() const;
    std::uint32_t getPlanarReflectorCount() const;
    double getProcessDeltaTime() const;
    double getPhysicsProcessDeltaTime() const;

    void flushTransformNotifications();
    void flushSkeletonUpdates();
    CollisionWorld& getCollisionWorld();
    void queueDelete(Node* node);
    void flushDeleteQueue();
    Tween* createTween();
    void tweenOnce(const ct::Function<void(Tween&)>& build);
    bool hasTween(const Tween* tween) const;
    std::size_t getTweenCount() const;
    void addParticleCollider(ParticleCollision3D* collider);
    void removeParticleCollider(ParticleCollision3D* collider);
    const ct::Vector<ParticleCollision3D*>& getParticleColliders() const;
    bool hasGroup(const ct::String& group) const;
    std::size_t getNodeCountInGroup(const ct::String& group) const;
    Node* getFirstNodeInGroup(const ct::String& group) const;
    void getNodesInGroup(const ct::String& group, ct::Vector<Node*>& nodes) const;
    void addListener(SceneTreeListener* listener);
    void removeListener(SceneTreeListener* listener);

    void inputEvent(const InputEvent& event);
    void setInputAsHandled();
    bool isInputHandled() const;

    void setCurrentScene(Node* scene, const ct::String& path = ct::String());
    Node* getCurrentScene() const;
    const ct::String& getCurrentScenePath() const;
    enum class Transition
    {
        None,
        Fade,
        SlideLeft,
        SlideRight,
        SlideUp,
        SlideDown,
        ZoomIn,
        ZoomOut,
        WipeLeft,
        WipeRight
    };
    static Transition transitionFromName(const ct::String& name);

    void changeSceneToFile(const ct::String& path, Transition transition = Transition::None,
            double seconds = 0.0);
    Transition getTransition() const;
    double getTransitionProgress() const;
    bool takeTransitionPixels(ct::Vector<std::uint8_t>& pixels, int& width, int& height);
    void setProcessHook(const ct::Function<void()>& hook);
    bool reloadCurrentScene();
    bool isSceneChangePending() const;

private:
    friend class Node;
    friend class Node3D;
    friend class Skeleton3D;
    friend class PlanarReflector3D;

    void addToGroup(const ct::String& group, Node* node);
    void removeFromGroup(const ct::String& group, Node* node);
    void treeChanged();
    void nodeAdded(Node* node);
    void nodeRemoved(Node* node);
    void nodeRenamed(Node* node);
    void endNotify();
    void queueTransformChanged(Node3D* node);
    void queueSkeletonUpdate(Skeleton3D* skeleton);
    void dropSkeletonUpdate(Skeleton3D* skeleton);
    void dropTweenNode(Node* node);
    void processTweens(double delta, bool physics);
    void processNodes(Node* node, int notification, bool& quit);
    void updateDefaultListener();
    void drawNodes(Node* node);
    void collectInputNodes(Node* node, ct::Vector<Node*>& nodes) const;
    void flushSceneChange();

    static SceneTree* sSingleton;

    Node* mRoot;
    ct::Vector<Node3D*> mTransformChanged;
    std::uint32_t mPlanarReflectors = 0;
    ct::Vector<Skeleton3D*> mSkeletonUpdates;
    ct::Vector<Node*> mDeleteQueue;
    ct::Vector<Tween*> mTweens;
    ct::Vector<ParticleCollision3D*> mParticleColliders;
    ct::Vector<SceneTreeListener*> mListeners;
    ct::HashMap<ct::String, ct::Vector<Node*>> mGroups;
    int mNotifyDepth;
    bool mListenersDirty;
    std::uint64_t mFrame;
    std::size_t mNodeCount;
    double mProcessDelta;
    double mPhysicsDelta;
    bool mCloseRequestSent;
    int mExitCode;
    bool mQuit;
    bool mAutoAcceptQuit;
    bool mPaused;
    bool mInitialized;
    bool mInputHandled;
    bool mScenePending;
    Node* mCurrentScene;
    ct::String mCurrentScenePath;
    ct::String mPendingScenePath;
    Transition mPendingTransition;
    double mPendingTransitionSeconds;
    Transition mTransition;
    double mTransitionSeconds;
    double mTransitionElapsed;
    ct::Vector<std::uint8_t> mTransitionPixels;
    int mTransitionWidth;
    int mTransitionHeight;
    bool mTransitionFresh;
    ct::Function<void()> mProcessHook;
    CollisionWorld* mCollisionWorld;
};

} // namespace prisma3d
