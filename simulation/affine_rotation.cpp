#include "affine_rotation.h"

#include <utility>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <opencv2/core/eigen.hpp>
#include <opencv2/core/matx.hpp>

namespace affine_rotation {

AffineRotation::AffineRotation()
    : translation{0.0F, 0.0F, 0.0F}, rotation{1.0F, 0.0F, 0.0F, 0.0F} {};
AffineRotation::AffineRotation(Eigen::Vector3f translation,
                               Eigen::Quaternionf rotation)
    : translation{std::move(translation)}, rotation{std::move(rotation)} {};

auto AffineRotation::operator*(const AffineRotation &other) const
    -> AffineRotation {
    return {translation + rotation * other.translation,
            rotation * other.rotation};
}

void AffineRotation::operator*=(const AffineRotation &other) {
    translation += rotation * other.translation;
    rotation *= other.rotation;
}

auto AffineRotation::inverse() const -> AffineRotation {
    const auto rotation_inverse{rotation.inverse()};
    return {-(rotation_inverse * translation), rotation_inverse};
}

auto AffineRotation::getTranslation() const -> Eigen::Vector3f {
    return translation;
}

auto AffineRotation::getRotation() const -> Eigen::Quaternionf {
    return rotation;
}

auto from_cv(const cv::Vec3f &rvec, const cv::Vec3f &tvec) -> AffineRotation {
    Eigen::Vector3f rotation_vec;
    Eigen::Vector3f translation;
    cv::cv2eigen(rvec, rotation_vec);
    cv::cv2eigen(tvec, translation);

    const float angle{rotation_vec.norm()};
    const Eigen::AngleAxisf rotation{angle, rotation_vec / angle};
    return {translation, Eigen::Quaternionf{rotation}};
}

} // namespace affine_rotation
