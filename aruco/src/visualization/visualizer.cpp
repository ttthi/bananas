#include <bananas_aruco/visualization/visualizer.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

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
#include <OgreShaderGenerator.h>

#include <bananas_aruco/affine_rotation.h>
#include <bananas_aruco/box_board.h>
#include <bananas_aruco/world.h>

namespace {

void set_transform(Ogre::SceneNode *node,
                   const affine_rotation::AffineRotation &transform) {
    node->setPosition(Ogre::Vector3f{transform.getTranslation().data()});

    const auto rotation{transform.getRotation()};
    node->setOrientation(Ogre::Quaternion{rotation.w(), rotation.x(),
                                          rotation.y(), rotation.z()});
}

} // namespace

namespace visualizer {

KeyHandler::KeyHandler(OgreBites::CameraMan *camera_manager)
    : camera_manager{camera_manager} {}

auto KeyHandler::mousePressed(const OgreBites::MouseButtonEvent & /*evt*/)
    -> bool {
    camera_manager->setStyle(OgreBites::CameraStyle::CS_FREELOOK);
    return true;
}

auto KeyHandler::mouseReleased(const OgreBites::MouseButtonEvent & /*evt*/)
    -> bool {
    camera_manager->setStyle(OgreBites::CameraStyle::CS_MANUAL);
    return true;
}

InitializedContext::InitializedContext() : OgreBites::ApplicationContext{} {
    initApp();
}

Visualizer::Visualizer()
    : root{context.getRoot()}, scene_manager{root->createSceneManager()},
      camera_node{scene_manager->getRootSceneNode()->createChildSceneNode()},
      camera_manager{camera_node}, key_handler{&camera_manager},
      static_environment{
          scene_manager->getRootSceneNode()->createChildSceneNode()},
      camera_visualization{
          scene_manager->getRootSceneNode()->createChildSceneNode()} {
    Ogre::RTShader::ShaderGenerator *shadergen{
        Ogre::RTShader::ShaderGenerator::getSingletonPtr()};
    shadergen->addSceneManager(scene_manager);

    Ogre::Light *light{scene_manager->createLight("light")};
    Ogre::SceneNode *lightNode{
        scene_manager->getRootSceneNode()->createChildSceneNode()};
    lightNode->setPosition(10, 10, -15);
    lightNode->attachObject(light);

    camera_node->setPosition(2, 2, -1);
    camera_node->lookAt(Ogre::Vector3{0, 0, 0}, Ogre::Node::TS_PARENT);
    camera_node->roll(Ogre::Degree{-90});

    Ogre::Camera *camera{scene_manager->createCamera("camera")};
    camera->setNearClipDistance(0.1);
    camera->setAutoAspectRatio(true);
    camera_node->attachObject(camera);

    camera_manager.setStyle(OgreBites::CameraStyle::CS_MANUAL);
    camera_manager.setTopSpeed(2.0F);

    context.getRenderWindow()->addViewport(camera);

    context.addInputListener(&key_handler);
    context.addInputListener(&camera_manager);

    Ogre::Entity *ground_plane{
        scene_manager->createEntity(Ogre::SceneManager::PT_PLANE)};
    // Turn the plane around. The boxes have negative Z coordinates in the
    // coordinate system we are using.
    Ogre::SceneNode *child_node{static_environment->createChildSceneNode(
        // TODO(vainiovano): Fix the static environment coordinate system to be
        // centered instead of trying to center the plane here.
        Ogre::Vector3{100.0F, 100.0F, 0.0F},
        Ogre::Quaternion{Ogre::Degree{180}, Ogre::Vector3{1, 0, 0}})};
    child_node->attachObject(ground_plane);
    static_environment->setVisible(false);

    Ogre::Entity *camera_visualization_entity{
        scene_manager->createEntity(Ogre::SceneManager::PT_CUBE)};
    camera_visualization->setScale(0.00025F, 0.00025F, 0.00125F);
    camera_visualization->attachObject(camera_visualization_entity);
    camera_visualization->setVisible(false);
}

void Visualizer::update(const world::FitResult &fit) {
    static_environment->setVisible(fit.camera_to_world.has_value());
    camera_visualization->setVisible(fit.camera_to_world.has_value());
    if (fit.camera_to_world) {
        set_transform(camera_visualization, *fit.camera_to_world);
    }
    for (const auto &box : dynamic_environment) {
        const auto box_fit{fit.dynamic_boards_to_world.find(box.first)};
        const bool has_fit{box_fit != fit.dynamic_boards_to_world.cend()};
        box.second->setVisible(has_fit);
        if (has_fit) {
            set_transform(box.second, box_fit->second);
        }
    }
}

void Visualizer::refresh() { context.getRoot()->renderOneFrame(); }

void Visualizer::setStaticEnvironmentSize(float width, float height) {
    static_environment->setScale(0.005F * width, 0.005F * height, 1.0F);
}

void Visualizer::addBox(world::DynamicBoardId id,
                        const box_board::BoxSize &size) {
    Ogre::Entity *cube{
        scene_manager->createEntity(Ogre::SceneManager::PT_CUBE)};
    Ogre::SceneNode *node{
        scene_manager->getRootSceneNode()->createChildSceneNode()};

    node->setScale(
        {0.01F * size.width, 0.01F * size.height, 0.01F * size.depth});
    node->attachObject(cube);
    node->setVisible(false);

    dynamic_environment.emplace(id, node);
}

} // namespace visualizer
