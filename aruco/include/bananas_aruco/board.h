#ifndef BANANAS_ARUCO_BOARD_H_
#define BANANAS_ARUCO_BOARD_H_

#include <vector>

#include <opencv2/core/types.hpp>
#include <opencv2/objdetect/aruco_board.hpp>
#include <opencv2/objdetect/aruco_dictionary.hpp>

/// Structures and functions related to ArUco boards.
namespace bananas::board {

/// A generic ArUco board. Use @ref to_cv() to convert a @ref Board into a
/// cv::aruco::Board that includes a dictionary.
struct Board {
    /// The markers contained in this board. A marker is encoded as a list of
    /// the four corner coordinates of the marker image in the glTF coordinate
    /// system: the top-left point, top-right point, bottom-right point, and the
    /// bottom-left point.
    std::vector<std::vector<cv::Point3f>> obj_points;
    /// The IDs of the markers in the ArUco dictionary that will be used with
    /// this board. The length of this array must match the length of @ref
    /// obj_points.
    std::vector<int> marker_ids;
};

/// Convert a @p board into a cv::aruco::Board using @p dictionary.
[[nodiscard]]
auto to_cv(const cv::aruco::Dictionary &dictionary,
           const Board &board) -> cv::aruco::Board;

} // namespace bananas::board

#endif // BANANAS_ARUCO_BOARD_H_
