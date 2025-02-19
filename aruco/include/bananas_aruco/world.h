#ifndef WORLD_H_
#define WORLD_H_

#include <cstdint>
#include <optional>
#include <unordered_map>
#include <variant>
#include <vector>

#include <gsl/pointers>
#include <gsl/span>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/objdetect/aruco_board.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/objdetect/aruco_dictionary.hpp>

#include <bananas_aruco/affine_rotation.h>
#include <bananas_aruco/box_board.h>
#include <bananas_aruco/grid_board.h>

namespace world {

using DynamicBoardId = std::uint32_t;

struct FitResult {
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<int> ids;
    std::optional<affine_rotation::AffineRotation> camera_to_world;
    std::unordered_map<DynamicBoardId, affine_rotation::AffineRotation>
        dynamic_boards_to_world;
};

class StaticEnvironment {
  public:
    using Object = std::variant<board::BoxSettings, board::GridSettings>;
    struct PlacedObject {
        Object object{};
        affine_rotation::AffineRotation object_to_world{};
    };

    StaticEnvironment(const cv::aruco::Dictionary &dictionary,
                      gsl::span<const PlacedObject> objects);

    // TODO(vainiovano): Allow adding objects to the static environment.

    [[nodiscard]]
    auto getBoard() const -> const cv::aruco::Board & {
        return board;
    }

  private:
    cv::aruco::Board board;
    /// The objects from which the static environment consists of. This may be
    /// used for serializing the object placements.
    /// TODO(vainiovanio): Implement serializing and deserializing the static
    /// environment.
    std::vector<PlacedObject> objects;
};

class World {
  public:
    // TODO(vainiovano): configurable detector parameters
    World(cv::Mat camera_matrix, cv::Mat distortion_coeffs,
          const cv::aruco::Dictionary &dictionary,
          const StaticEnvironment &environment);

    auto addBoard(cv::aruco::Board board) -> DynamicBoardId;
    auto addBox(const board::BoxSettings &settings) -> DynamicBoardId;

    auto fit(const cv::Mat &image) const -> FitResult;

  private:
    [[nodiscard]]
    auto fitBoard(const std::vector<std::vector<cv::Point2f>> &corners,
                  const std::vector<int> &ids,
                  const cv::aruco::Board &board) const
        -> std::optional<affine_rotation::AffineRotation>;

    cv::Mat camera_matrix;
    cv::Mat distortion_coeffs;
    gsl::not_null<const cv::aruco::Dictionary *> dictionary;
    cv::aruco::ArucoDetector detector;
    gsl::not_null<const StaticEnvironment *> static_environment;
    std::unordered_map<DynamicBoardId, cv::aruco::Board> dynamic_boards{};
    std::uint32_t next_dynamic_board_id{0};
};

} // namespace world

#endif // WORLD_H_
