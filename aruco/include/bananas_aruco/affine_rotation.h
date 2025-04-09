#ifndef AFFINE_ROTATION_H_
#define AFFINE_ROTATION_H_

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <nlohmann/json_fwd.hpp>

#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>

namespace affine_rotation {

/// A transform consisting of rotation and translation.
class AffineRotation {
  public:
    /// The identity transform.
    AffineRotation();

    /// The transform with the given rotation and translation.
    AffineRotation(Eigen::Quaternionf rotation, Eigen::Vector3f translation);

    /// Combine two affine rotations.
    ///
    /// @return A transform that has the same effect as first performing the
    /// transform @p other and then performing this transform.
    auto operator*(const AffineRotation &other) const -> AffineRotation;

    /// Apply the affine rotation to a point interpreted as a vector.
    ///
    /// @return The vector obtained by first rotating @p point by the rotation
    /// of this transform and then translating it by the translation of this
    /// transform.
    auto operator*(cv::Point3f point) const -> cv::Point3f;

    /// Return the left inverse of this transform.
    [[nodiscard]] auto inverse() const -> AffineRotation;

    /// Return the translation component of this transform.
    [[nodiscard]] auto getTranslation() const -> Eigen::Vector3f;
    /// Return the rotation component of this transform.
    [[nodiscard]] auto getRotation() const -> Eigen::Quaternionf;

  private:
    Eigen::Quaternionf rotation_;
    Eigen::Vector3f translation_;
};

void from_json(const nlohmann::json &j, AffineRotation &affine_rotation);
void to_json(nlohmann::json &j, const AffineRotation &affine_rotation);

auto from_cv(const cv::Vec3f &rvec, const cv::Vec3f &tvec) -> AffineRotation;

} // namespace affine_rotation

#endif // AFFINE_ROTATION_H_
