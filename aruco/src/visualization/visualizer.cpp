#include <bananas_aruco/visualization/visualizer.h>

#include <variant>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <gsl/pointers>
#include <gsl/span>

#include <OgreApplicationContext.h>
#include <OgreCamera.h>
#include <OgreCameraMan.h>
#include <OgreEntity.h>
#include <OgreInput.h>
#include <OgreLight.h>
#include <OgreMath.h>
#include <OgreNode.h>
#include <OgrePrerequisites.h>
#include <OgreQuaternion.h>
#include <OgreRenderWindow.h>
#include <OgreRoot.h>
#include <OgreSceneManager.h>
#include <OgreShaderGenerator.h>

#include <bananas_aruco/affine_rotation.h>
#include <bananas_aruco/box_board.h>
#include <bananas_aruco/grid_board.h>
#include <bananas_aruco/world.h>

namespace {

void set_transform(Ogre::SceneNode *node,
                   const affine_rotation::AffineRotation &transform) {
    node->setPosition(Ogre::Vector3f{transform.getTranslation().data()});

    const auto rotation{transform.getRotation()};
    node->setOrientation(Ogre::Quaternion{rotation.w(), rotation.x(),
                                          rotation.y(), rotation.z()});
}

/// An std::variant visitor for creating a node for an object.
class CreateObjectNodeVisitor {
  public:
    CreateObjectNodeVisitor(gsl::not_null<Ogre::SceneManager *> scene_manager,
                            gsl::not_null<Ogre::SceneNode *> parent)
        : scene_manager_{scene_manager}, parent_node_{parent} {}

    auto operator()(const board::BoxSettings &box_settings) const
        -> gsl::not_null<Ogre::SceneNode *> {
        const gsl::not_null<Ogre::Entity *> cube{
            scene_manager_->createEntity(Ogre::SceneManager::PT_CUBE)};
        const gsl::not_null<Ogre::SceneNode *> node{
            parent_node_->createChildSceneNode()};

        node->setScale({0.01F * box_settings.size.width,
                        0.01F * box_settings.size.height,
                        0.01F * box_settings.size.depth});
        node->attachObject(cube);

        return node;
    }

    auto operator()(const board::GridSettings &grid_settings) const
        -> gsl::not_null<Ogre::SceneNode *> {
        const gsl::not_null<Ogre::Entity *> plane{
            scene_manager_->createEntity(Ogre::SceneManager::PT_PLANE)};
        const gsl::not_null<Ogre::SceneNode *> node{
            parent_node_->createChildSceneNode()};

        // Make the plane point up (+Y direction) by default.
        const gsl::not_null<Ogre::SceneNode *> child_node{
            node->createChildSceneNode(
                Ogre::Vector3{0.0F, 0.0F, 0.0F},
                Ogre::Quaternion{Ogre::Degree{270}, Ogre::Vector3{1, 0, 0}})};
        child_node->attachObject(plane);

        node->setScale(0.005F * board::grid_width(grid_settings),
                       0.005F * board::grid_height(grid_settings), 1.0F);

        return node;
    }

  private:
    gsl::not_null<Ogre::SceneManager *> scene_manager_;
    gsl::not_null<Ogre::SceneNode *> parent_node_;
};

} // namespace

namespace visualizer {

KeyHandler::KeyHandler(OgreBites::CameraMan *camera_manager)
    : camera_manager_{camera_manager} {}

auto KeyHandler::mousePressed(const OgreBites::MouseButtonEvent & /*evt*/)
    -> bool {
    camera_manager_->setStyle(OgreBites::CameraStyle::CS_FREELOOK);
    return true;
}

auto KeyHandler::mouseReleased(const OgreBites::MouseButtonEvent & /*evt*/)
    -> bool {
    camera_manager_->setStyle(OgreBites::CameraStyle::CS_MANUAL);
    return true;
}

InitializedContext::InitializedContext() : OgreBites::ApplicationContext{} {
    initApp();
}

Visualizer::Visualizer()
    : root_{context_.getRoot()}, scene_manager_{root_->createSceneManager()},
      camera_node_{scene_manager_->getRootSceneNode()->createChildSceneNode()},
      camera_manager_{camera_node_}, key_handler_{&camera_manager_},
      static_environment_{
          scene_manager_->getRootSceneNode()->createChildSceneNode()},
      camera_visualization_{
          scene_manager_->getRootSceneNode()->createChildSceneNode()} {
    const gsl::not_null<Ogre::RTShader::ShaderGenerator *> shadergen{
        Ogre::RTShader::ShaderGenerator::getSingletonPtr()};
    shadergen->addSceneManager(scene_manager_);

    const gsl::not_null<Ogre::Light *> light{
        scene_manager_->createLight("light")};
    const gsl::not_null<Ogre::SceneNode *> lightNode{
        scene_manager_->getRootSceneNode()->createChildSceneNode()};
    lightNode->setPosition(10, 15, 10);
    lightNode->attachObject(light);

    camera_node_->setPosition(2, 1, 2);
    camera_node_->lookAt(Ogre::Vector3{0, 0, 0}, Ogre::Node::TS_PARENT);

    const gsl::not_null<Ogre::Camera *> camera{
        scene_manager_->createCamera("camera")};
    camera->setNearClipDistance(0.1);
    camera->setAutoAspectRatio(true);
    camera_node_->attachObject(camera);

    camera_manager_.setStyle(OgreBites::CameraStyle::CS_MANUAL);
    camera_manager_.setTopSpeed(2.0F);

    context_.getRenderWindow()->addViewport(camera);

    context_.addInputListener(&key_handler_);
    context_.addInputListener(&camera_manager_);

    const gsl::not_null<Ogre::Entity *> camera_visualization_entity{
        scene_manager_->createEntity(Ogre::SceneManager::PT_CUBE)};
    camera_visualization_->setScale(0.00025F, 0.00025F, 0.00125F);
    camera_visualization_->attachObject(camera_visualization_entity);
    camera_visualization_->setVisible(false);
}

void Visualizer::update(const world::FitResult &fit) {
    camera_visualization_->setVisible(fit.camera_to_world.has_value());
    if (fit.camera_to_world) {
        set_transform(camera_visualization_, *fit.camera_to_world);
    }
    for (const auto &box : dynamic_environment_) {
        const auto box_fit{fit.dynamic_boards_to_world.find(box.first)};
        const bool has_fit{box_fit != fit.dynamic_boards_to_world.cend()};
        box.second->setVisible(has_fit);
        if (has_fit) {
            set_transform(box.second, box_fit->second);
        }
    }
}

void Visualizer::refresh() { context_.getRoot()->renderOneFrame(); }

void Visualizer::updateStaticEnvironment(
    gsl::span<const world::StaticEnvironment::PlacedObject> objects) {
    static_environment_->removeAndDestroyAllChildren();
    const CreateObjectNodeVisitor visitor{scene_manager_, static_environment_};
    for (const auto &object : objects) {
        const auto node{std::visit(visitor, object.object)};
        set_transform(node, object.object_to_world);
    }
}

void Visualizer::addBox(world::DynamicBoardId id, const board::BoxSize &size) {
    const gsl::not_null<Ogre::Entity *> cube{
        scene_manager_->createEntity(Ogre::SceneManager::PT_CUBE)};
    const gsl::not_null<Ogre::SceneNode *> node{
        scene_manager_->getRootSceneNode()->createChildSceneNode()};

    node->setScale(
        {0.01F * size.width, 0.01F * size.height, 0.01F * size.depth});
    node->attachObject(cube);
    node->setVisible(false);

    dynamic_environment_.emplace(id, node);
}

} // namespace visualizer
