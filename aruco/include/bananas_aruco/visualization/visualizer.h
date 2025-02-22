#ifndef VISUALIZER_H_
#define VISUALIZER_H_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gsl/pointers>
#include <gsl/span>

#include <OgreApplicationContext.h>
#include <OgreCameraMan.h>
#include <OgreInput.h>
#include <OgrePrerequisites.h>
#include <OgreRoot.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <unordered_map>

#include <bananas_aruco/box_board.h>
#include <bananas_aruco/world.h>

namespace visualizer {

class KeyHandler : public OgreBites::InputListener {
  public:
    explicit KeyHandler(OgreBites::CameraMan *camera_manager);

    auto mousePressed(const OgreBites::MouseButtonEvent &evt) -> bool override;
    auto mouseReleased(const OgreBites::MouseButtonEvent &evt) -> bool override;

  private:
    OgreBites::CameraMan *camera_manager_;
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

    void update(const world::FitResult &fit);

    /// Render a new frame.
    void refresh();

    void updateStaticEnvironment(
        gsl::span<const world::StaticEnvironment::PlacedObject> objects);
    void addBox(world::DynamicBoardId id, const board::BoxSize &size);

  private:
    InitializedContext context_{};
    gsl::not_null<Ogre::Root *> root_;
    gsl::not_null<Ogre::SceneManager *> scene_manager_;
    gsl::not_null<Ogre::SceneNode *> camera_node_;
    OgreBites::CameraMan camera_manager_;
    KeyHandler key_handler_;
    gsl::not_null<Ogre::SceneNode *> static_environment_;
    gsl::not_null<Ogre::SceneNode *> camera_visualization_;
    std::unordered_map<world::DynamicBoardId, Ogre::SceneNode *>
        dynamic_environment_{};
};

} // namespace visualizer

#endif // VISUALIZER_H_
