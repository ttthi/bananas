#ifndef VISUALIZER_H_
#define VISUALIZER_H_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <OgreApplicationContext.h>
#include <OgreCameraMan.h>
#include <OgreInput.h>
#include <OgrePrerequisites.h>
#include <OgreQuaternion.h>
#include <OgreRoot.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>

#include "../affine_rotation.h"

namespace visualizer {

class ObjectHandle {
  public:
    explicit ObjectHandle(Ogre::SceneNode *node);

    void setVisible(bool visible);
    void setTransform(const affine_rotation::AffineRotation &transform);

  private:
    Ogre::SceneNode *node;
};

class KeyHandler : public OgreBites::InputListener {
  public:
    explicit KeyHandler(OgreBites::CameraMan *camera_manager);

    auto mousePressed(const OgreBites::MouseButtonEvent &evt) -> bool override;
    auto mouseReleased(const OgreBites::MouseButtonEvent &evt) -> bool override;

  private:
    OgreBites::CameraMan *camera_manager;
};

/// A wrapper that initializes an OgreBites::ApplicationContext and calls
/// Ogre::ApplicationContextBase::initApp() for it.
class InitializedContext : public OgreBites::ApplicationContext {
  public:
    InitializedContext();
};

class Visualizer {
  public:
    Visualizer();

    /// Render a new frame.
    void refresh();

    auto addCamera() -> ObjectHandle;
    auto addBox(float width, float height, float depth) -> ObjectHandle;
    auto addPlane(float width, float height) -> ObjectHandle;

  private:
    InitializedContext context{};
    Ogre::Root *root;
    Ogre::SceneManager *scene_manager;
    Ogre::SceneNode *camera_node;
    OgreBites::CameraMan camera_manager;
    KeyHandler key_handler;
};

} // namespace visualizer

#endif // VISUALIZER_H_
