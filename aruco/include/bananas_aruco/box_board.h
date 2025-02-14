#ifndef BOX_BOARD_H_
#define BOX_BOARD_H_

#include <array>
#include <vector>

#include <nlohmann/json.hpp>

#include <opencv2/objdetect/aruco_board.hpp>

namespace box_board {

/// Settings defining the location of a single marker relative to a box face.
struct MarkerSettings {
    int id{};
    /// The X offset of the marker from the center point of the face. Positive
    /// direction is right relative to the default orientation of the marker.
    float x_offset{};
    /// The Y offset of the marker from the center point of the face. Positive
    /// direction is up relative to the default orientation of the marker.
    float y_offset{};
    /// The rotation of the marker in radians. Positive direction is
    /// counterclockwise.
    float rotation{};
    /// The side length of the marker.
    float side{1.0F};
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(MarkerSettings, id, x_offset, y_offset,
                                   rotation, side);

/// The size of a box.
struct BoxSize {
    /// Extents in left-right axis.
    float width{1.0F};
    /// Extents in down-up axis.
    float height{1.0F};
    /// Extents in backward-forward axis.
    float depth{1.0F};
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(BoxSize, width, height, depth);

/// Settings defining the marker locations for a single box.
struct BoxSettings {
    /// The size of the box.
    BoxSize size{};

    /// The per-face markers, with the faces in the following order: forward,
    /// backward, left, right, up, down.
    ///
    /// For the forward, backward, left and right faces, the default orientation
    /// of the markers points up. For the up and down faces, the default
    /// orientation points right.
    std::array<std::vector<MarkerSettings>, 6> markers{};

    static constexpr int forward_face_index{0};
    static constexpr int backward_face_index{1};
    static constexpr int left_face_index{2};
    static constexpr int right_face_index{3};
    static constexpr int up_face_index{4};
    static constexpr int down_face_index{5};
};
void from_json(const nlohmann::json &j, BoxSettings &box_settings);

/// Produce an ArUco board for the given box. The markers are placed using the
/// glTF coordinate system: +X is left, +Y is up and +Z is forward.
auto make_board(const cv::aruco::Dictionary &dictionary,
                const BoxSettings &settings) -> cv::aruco::Board;

extern const BoxSettings example_box;

} // namespace box_board

#endif // BOX_BOARD_H_
