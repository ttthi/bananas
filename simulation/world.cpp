#include "world.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstddef>
#include <functional>
#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

#include <gsl/span>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/objdetect/aruco_board.hpp>

#include "affine_rotation.h"

namespace {

constexpr std::array<const int, 6> cube_id_offsets{3, 4, 2, 1, 0, 5};
const float quarter_circle{std::atanf(1.0F) * 2.0F};
const std::array<const float, 6> cube_face_angles{
    quarter_circle, -quarter_circle, 0, 0, 2 * quarter_circle, 0};

auto form_box_face(const world::BoxFaceSettings &face_settings)
    -> std::array<cv::Point2f, 4> {
    const float half_side{face_settings.side / 2.0F};
    std::array<cv::Point2f, 4> face{cv::Point2f{-half_side, -half_side},
                                    {half_side, -half_side},
                                    {half_side, half_side},
                                    {-half_side, half_side}};

    std::transform(
        face.cbegin(), face.cend(), face.begin(),
        [&face_settings](cv::Point2f point) -> cv::Point2f {
            const std::complex<float> point_complex{point.x, point.y};
            const std::complex<float> rotated{
                std::polar(1.0F, face_settings.rotation) * point_complex};
            // TODO(vainiovano): margins
            // const std::complex<float> translated{
            //     rotated.real() + face_settings.left_margin,
            //     rotated.imag() + face_settings.top_margin};
            return {rotated.real(), rotated.imag()};
        });

    return face;
}

auto cube_ids_from(int from) -> std::array<int, 6> {
    std::array<int, 6> result{};
    std::transform(cube_id_offsets.cbegin(), cube_id_offsets.cend(),
                   result.begin(), [from](int id) { return from + id; });
    return result;
}

auto make_cube_settings(float size, float margin, gsl::span<const int, 6> ids)
    -> world::BoxSettings {
    world::BoxSettings box_settings{};
    box_settings.size = {size, size, size};
    for (std::size_t i{0}; i < box_settings.faces.size(); ++i) {
        box_settings.faces[i].id = ids[i];
        box_settings.faces[i].left_margin = margin;
        box_settings.faces[i].top_margin = margin;
        box_settings.faces[i].side = size - 2 * margin;
    }
    for (std::size_t i{0}; i < box_settings.faces.size(); ++i) {
        box_settings.faces[i].left_margin = margin;
        box_settings.faces[i].top_margin = margin;
        box_settings.faces[i].side = size - 2 * margin;
        box_settings.faces[i].rotation = cube_face_angles[i];
    }
    return box_settings;
}

} // namespace

namespace world {

World::World(cv::Mat camera_matrix, cv::Mat distortion_coeffs,
             cv::aruco::Dictionary dictionary,
             cv::aruco::Board static_environment)
    : camera_matrix{std::move(camera_matrix)},
      distortion_coeffs{std::move(distortion_coeffs)},
      dictionary{std::move(dictionary)}, detector{this->dictionary, {}},
      static_environment(std::move(static_environment)) {}

auto World::addBoard(cv::aruco::Board board) -> DynamicBoardId {
    dynamic_boards.emplace(next_dynamic_board_id, std::move(board));
    return next_dynamic_board_id++;
}

auto World::addBox(const BoxSettings &settings) -> DynamicBoardId {
    return World::addBoard(formBoxBoard(settings));
}

auto World::addCube(float size, float margin, int start_id) -> DynamicBoardId {
    return World::addBox(
        make_cube_settings(size, margin, cube_ids_from(start_id)));
}

auto World::fit(const cv::Mat &image) const -> FitResult {
    std::vector<std::vector<cv::Point2f>> corners{};
    std::vector<std::vector<cv::Point2f>> rejected{};
    std::vector<int> ids{};

    detector.detectMarkers(image, corners, ids, rejected);

    detector.refineDetectedMarkers(image, static_environment, corners, ids,
                                   rejected, camera_matrix, distortion_coeffs);
    for (const auto &board : dynamic_boards) {
        detector.refineDetectedMarkers(image, board.second, corners, ids,
                                       rejected, camera_matrix,
                                       distortion_coeffs);
    }

    std::optional<affine_rotation::AffineRotation> camera_to_world{};
    std::unordered_map<DynamicBoardId, affine_rotation::AffineRotation>
        fit_boards{};
    const auto static_environment_fit{
        fitBoard(corners, ids, static_environment)};
    if (static_environment_fit) {
        camera_to_world = static_environment_fit->inverse();
        // TODO(vainiovano): Allow producing results even if the exact camera
        // location is not known.
        for (const auto &board : dynamic_boards) {
            const auto board_to_camera{fitBoard(corners, ids, board.second)};
            if (board_to_camera) {
                fit_boards.emplace(board.first,
                                   *camera_to_world * *board_to_camera);
            }
        }
    }

    return {std::move(corners), std::move(ids), std::move(camera_to_world),
            std::move(fit_boards)};
}

auto World::fitBoard(const std::vector<std::vector<cv::Point2f>> &corners,
                     const std::vector<int> &ids,
                     const cv::aruco::Board &board) const
    -> std::optional<affine_rotation::AffineRotation> {
    if (ids.empty()) {
        return {};
    }

    cv::Mat object_points;
    cv::Mat image_points;
    board.matchImagePoints(corners, ids, object_points, image_points);
    if (object_points.empty()) {
        return {};
    }

    cv::Vec3f rvec;
    cv::Vec3f tvec;
    if (!cv::solvePnP(object_points, image_points, camera_matrix,
                      distortion_coeffs, rvec, tvec)) {
        return {};
    }
    return affine_rotation::from_cv(rvec, tvec);
}

auto World::formBoxBoard(const world::BoxSettings &settings)
    -> cv::aruco::Board {
    std::vector<std::vector<cv::Point3f>> object_points(
        6, std::vector<cv::Point3f>(4));

    const auto set_face{
        [&settings, &object_points](
            int face_index,
            std::function<cv::Point3f(cv::Point2f)> to_space) -> void {
            const auto face{form_box_face(settings.faces[face_index])};
            std::transform(face.cbegin(), face.cend(),
                           object_points[face_index].begin(),
                           std::move(to_space));
        }};

    // TODO(vainiovano): Select a coordinate system that makes sense.
    set_face(0, [&settings](cv::Point2f point) -> cv::Point3f {
        return {point.x, point.y, -settings.size[0] / 2.0F};
    });
    set_face(1, [&settings](cv::Point2f point) -> cv::Point3f {
        return {-point.x, point.y, settings.size[0] / 2.0F};
    });
    set_face(2, [&settings](cv::Point2f point) -> cv::Point3f {
        return {-settings.size[1] / 2.0F, -point.x, -point.y};
    });
    set_face(3, [&settings](cv::Point2f point) -> cv::Point3f {
        return {settings.size[1] / 2.0F, -point.x, point.y};
    });
    set_face(4, [&settings](cv::Point2f point) -> cv::Point3f {
        return {point.x, -settings.size[2] / 2.0F, -point.y};
    });
    set_face(5, [&settings](cv::Point2f point) -> cv::Point3f {
        return {point.x, settings.size[2] / 2.0F, point.y};
    });

    std::array<int, 6> ids{};
    std::transform(
        settings.faces.cbegin(), settings.faces.cend(), ids.begin(),
        [](const world::BoxFaceSettings &face) -> int { return face.id; });
    return {object_points, dictionary, ids};
}

} // namespace world
