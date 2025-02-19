#include <bananas_aruco/world.h>

#include <algorithm>
#include <iterator>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

#include <gsl/span>

#include <nlohmann/json_fwd.hpp>

#include <opencv2/calib3d.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/objdetect/aruco_board.hpp>
#include <opencv2/objdetect/aruco_dictionary.hpp>

#include <bananas_aruco/affine_rotation.h>
#include <bananas_aruco/board.h>
#include <bananas_aruco/box_board.h>
#include <bananas_aruco/grid_board.h>

namespace {

auto to_board(const cv::aruco::Dictionary &dictionary,
              gsl::span<const world::StaticEnvironment::PlacedObject> objects)
    -> cv::aruco::Board {
    std::vector<std::vector<cv::Point3f>> obj_points{};
    std::vector<int> ids{};

    for (const auto &object : objects) {
        const board::Board board{std::visit(
            [](const auto &config) { return board::make_board(config); },
            object.object)};

        obj_points.reserve(obj_points.size() + board.obj_points.size());
        std::transform(board.obj_points.cbegin(), board.obj_points.cend(),
                       std::back_inserter(obj_points),
                       [&object](const std::vector<cv::Point3f> &points) {
                           std::vector<cv::Point3f> res(4);
                           std::transform(
                               points.cbegin(), points.cend(), res.begin(),
                               [&object](cv::Point3f point) {
                                   return object.object_to_world * point;
                               });
                           return res;
                       });

        ids.insert(ids.end(), board.marker_ids.cbegin(),
                   board.marker_ids.cend());
    }
    return {obj_points, dictionary, ids};
}

} // namespace

namespace world {

StaticEnvironment::StaticEnvironment(
    const cv::aruco::Dictionary &dictionary,
    gsl::span<const StaticEnvironment::PlacedObject> objects)
    : board{to_board(dictionary, objects)},
      objects{objects.begin(), objects.end()} {};

void from_json(const nlohmann::json &j,
               StaticEnvironment::PlacedObject &object) {
    const auto type{j.at("type").get<std::string>()};
    const auto object_to_world{
        j.at("object_to_world").get<affine_rotation::AffineRotation>()};

    if (type == "box") {
        const auto box{j.at("settings").get<board::BoxSettings>()};
        object = {box, object_to_world};
    } else if (type == "grid") {
        const auto grid{j.at("settings").get<board::GridSettings>()};
        object = {grid, object_to_world};
    } else {
        throw std::runtime_error{"Bad static environment member type " + type};
    }
}

void from_json(const nlohmann::json &j, StaticEnvironment &environment) {
    const int dictionary_id{j.at("dictionary_id").get<int>()};

    std::vector<StaticEnvironment::PlacedObject> placed_objects{};
    j.at("objects").get_to(placed_objects);

    const auto &dictionary{cv::aruco::getPredefinedDictionary(dictionary_id)};
    environment = {dictionary, placed_objects};
}

World::World(cv::Mat camera_matrix, cv::Mat distortion_coeffs,
             const cv::aruco::Dictionary &dictionary,
             const StaticEnvironment &static_environment)
    : camera_matrix{std::move(camera_matrix)},
      distortion_coeffs{std::move(distortion_coeffs)}, dictionary{&dictionary},
      detector{dictionary, {}}, static_environment{&static_environment} {}

auto World::addBoard(cv::aruco::Board board) -> DynamicBoardId {
    dynamic_boards.emplace(next_dynamic_board_id, std::move(board));
    return next_dynamic_board_id++;
}

auto World::addBox(const board::BoxSettings &settings) -> DynamicBoardId {
    return World::addBoard(
        board::to_cv(*dictionary, board::make_board(settings)));
}

auto World::fit(const cv::Mat &image) const -> FitResult {
    std::vector<std::vector<cv::Point2f>> corners{};
    std::vector<std::vector<cv::Point2f>> rejected{};
    std::vector<int> ids{};
    const auto &static_board{static_environment->getBoard()};

    detector.detectMarkers(image, corners, ids, rejected);

    detector.refineDetectedMarkers(image, static_board, corners, ids, rejected,
                                   camera_matrix, distortion_coeffs);
    for (const auto &board : dynamic_boards) {
        detector.refineDetectedMarkers(image, board.second, corners, ids,
                                       rejected, camera_matrix,
                                       distortion_coeffs);
    }

    std::optional<affine_rotation::AffineRotation> camera_to_world{};
    std::unordered_map<DynamicBoardId, affine_rotation::AffineRotation>
        fit_boards{};
    const auto static_environment_fit{fitBoard(corners, ids, static_board)};
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
