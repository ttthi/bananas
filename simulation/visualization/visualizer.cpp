#include "visualizer.h"

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

#include "../affine_rotation.h"

namespace visualizer {

void ObjectHandle::setVisible(bool visible) { node->setVisible(visible); }

void ObjectHandle::setTransform(
    const affine_rotation::AffineRotation &transform) {
    node->setPosition(Ogre::Vector3f{transform.getTranslation().data()});

    const auto rotation{transform.getRotation()};
    node->setOrientation(Ogre::Quaternion{rotation.w(), rotation.x(),
                                          rotation.y(), rotation.z()});
}

ObjectHandle::ObjectHandle(Ogre::SceneNode *node) : node{node} {};

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
      camera_manager{camera_node}, key_handler(&camera_manager) {
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
}

void Visualizer::refresh() { context.getRoot()->renderOneFrame(); }

auto Visualizer::addBox(float width, float height, float depth)
    -> ObjectHandle {
    Ogre::Entity *cube{
        scene_manager->createEntity(Ogre::SceneManager::PT_CUBE)};
    Ogre::SceneNode *node{
        scene_manager->getRootSceneNode()->createChildSceneNode()};

    node->setScale({0.01F * width, 0.01F * height, 0.01F * depth});
    node->attachObject(cube);
    node->setVisible(false);

    return ObjectHandle{node};
}

auto Visualizer::addPlane(float width, float height) -> ObjectHandle {
    Ogre::Entity *plane{
        scene_manager->createEntity(Ogre::SceneManager::PT_PLANE)};
    Ogre::SceneNode *node{
        scene_manager->getRootSceneNode()->createChildSceneNode()};
    // Turn the plane around. The boxes have negative Z coordinates in the
    // coordinate system we are using.
    Ogre::SceneNode *child_node{node->createChildSceneNode(
        // TODO(vainiovano): Fix the plane coordinate system to be centered
        // instead of trying to center the plane here.
        Ogre::Vector3{100.0F, 100.0F, 0.0F},
        Ogre::Quaternion{Ogre::Degree{180}, Ogre::Vector3{1, 0, 0}})};

    node->setScale({0.005F * width, 0.005F * height, 1.0F});
    child_node->attachObject(plane);
    node->setVisible(false);

    return ObjectHandle{node};
}

auto Visualizer::addCamera() -> ObjectHandle {
    return Visualizer::addBox(0.025F, 0.025F, 0.125F);
}

} // namespace visualizer
