#ifndef BANANAS_ARUCO_WORLD_H_
#define BANANAS_ARUCO_WORLD_H_

#include <cstdint>
#include <optional>
#include <unordered_map>
#include <vector>

#include <gsl/pointers>

#include <nlohmann/json_fwd.hpp>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/objdetect/aruco_board.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/objdetect/aruco_dictionary.hpp>

#include <bananas_aruco/affine_rotation.h>
#include <bananas_aruco/board.h>

/// Structures and functions related to the world model.
namespace bananas::world {

using BoardId = std::uint32_t;
using BoardPlacement =
    std::unordered_map<BoardId, affine_rotation::AffineRotation>;

void from_json(const nlohmann::json &j, BoardPlacement &placement);

struct UncertainPose {
    float reprojection_error{};
    affine_rotation::AffineRotation placement{};
};

using UncertainPlacement = std::unordered_map<BoardId, UncertainPose>;

struct FitResult {
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<int> ids;
    std::optional<UncertainPose> camera_to_world;
    UncertainPlacement dynamic_board_placements;
};

/// Our model of the world around us. This model consists of two main parts:
///
/// 1. The static environment, i.e., all boards whose exact locations we know.
///    This is used for finding where the camera is relative to the world.
/// 2. The dynamic environment, i.e., boxes. The locations and orientations of
///    these boxes are recomputed every time World::fit() is called.
class World {
  public:
    // TODO(vainiovano): configurable detector parameters
    World(cv::Mat camera_matrix, cv::Mat distortion_coeffs,
          const cv::aruco::Dictionary &dictionary);

    /// Add the given board to the world.
    ///
    /// @return The identifier of the added board.
    auto addBoard(const board::Board &board) -> BoardId;

    /// Move the given board into the static environment, locking its position
    /// and orientation.
    void makeStatic(BoardId id,
                    const affine_rotation::AffineRotation &board_to_world);

    /// Find the camera and box locations based on the given camera image.
    [[nodiscard]]
    auto fit(const cv::Mat &image) const -> FitResult;

  private:
    [[nodiscard]]
    auto fitBoard(const std::vector<std::vector<cv::Point2f>> &corners,
                  const std::vector<int> &ids, const cv::aruco::Board &board)
        const -> std::optional<UncertainPose>;

    void recomputeStaticEnvironment();

    // The reprojection error would be pretty misleading for a single marker
    // since the solver can just find a placement that just happens to fit.
    static constexpr int min_marker_count{2};

    cv::Mat camera_matrix_;
    cv::Mat distortion_coeffs_;
    gsl::not_null<const cv::aruco::Dictionary *> dictionary_;
    cv::aruco::ArucoDetector detector_;
    cv::aruco::Board static_environment_;
    BoardPlacement static_board_placements_{};
    std::vector<cv::aruco::Board> all_boards_{};
};

} // namespace bananas::world

#endif // BANANAS_ARUCO_WORLD_H_
