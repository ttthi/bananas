#include <bananas_aruco/world.h>

#include <optional>
#include <unordered_map>
#include <utility>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/objdetect/aruco_board.hpp>

#include <bananas_aruco/affine_rotation.h>
#include <bananas_aruco/box_board.h>

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

auto World::addBox(const box_board::BoxSettings &settings) -> DynamicBoardId {
    return World::addBoard(box_board::make_board(dictionary, settings));
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

} // namespace world
