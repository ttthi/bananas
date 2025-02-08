#ifndef WORLD_H_
#define WORLD_H_

#include <cstdint>
#include <optional>
#include <unordered_map>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/matx.hpp>
#include <opencv2/objdetect/aruco_board.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/objdetect/aruco_dictionary.hpp>

#include "affine_rotation.h"

namespace world {

struct BoxFaceSettings {
    int id{};
    float left_margin{};
    float top_margin{};
    float rotation{};
    float side{1.0F};
};

struct BoxSettings {
    cv::Vec3f size{};
    // TODO(vainiovano): define the order
    std::array<BoxFaceSettings, 6> faces{};
};

using DynamicBoardId = std::uint32_t;

struct FitResult {
    std::vector<std::vector<cv::Point2f>> corners;
    std::vector<int> ids;
    std::optional<affine_rotation::AffineRotation> camera_to_world;
    std::unordered_map<DynamicBoardId, affine_rotation::AffineRotation>
        dynamic_boards_to_world;
};

class World {
  public:
    // TODO(vainiovano): configurable detector parameters
    World(cv::Mat camera_matrix, cv::Mat distortion_coeffs,
          cv::aruco::Dictionary dictionary,
          cv::aruco::Board static_environment);

    auto addBoard(cv::aruco::Board board) -> DynamicBoardId;
    auto addBox(const BoxSettings &settings) -> DynamicBoardId;
    auto addCube(float size, float margin, int start_id) -> DynamicBoardId;

    auto fit(const cv::Mat &image) const -> FitResult;

  private:
    auto fitBoard(const std::vector<std::vector<cv::Point2f>> &corners,
                  const std::vector<int> &ids,
                  const cv::aruco::Board &board) const
        -> std::optional<affine_rotation::AffineRotation>;

    auto formBoxBoard(const BoxSettings &settings) -> cv::aruco::Board;

    cv::Mat camera_matrix;
    cv::Mat distortion_coeffs;
    cv::aruco::Dictionary dictionary;
    cv::aruco::ArucoDetector detector;
    cv::aruco::Board static_environment;
    std::unordered_map<DynamicBoardId, cv::aruco::Board> dynamic_boards{};
    std::uint32_t next_dynamic_board_id{0};
};

} // namespace world

#endif // WORLD_H_
