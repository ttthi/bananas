#ifndef BANANAS_ARUCO_BOARD_H_
#define BANANAS_ARUCO_BOARD_H_

#include <vector>

#include <opencv2/core/types.hpp>
#include <opencv2/objdetect/aruco_board.hpp>
#include <opencv2/objdetect/aruco_dictionary.hpp>

/// Structures and functions related to ArUco boards.
namespace bananas::board {

struct Board {
    std::vector<std::vector<cv::Point3f>> obj_points;
    std::vector<int> marker_ids;
};

[[nodiscard]]
auto to_cv(const cv::aruco::Dictionary &dictionary,
           const Board &board) -> cv::aruco::Board;

} // namespace bananas::board

#endif // BANANAS_ARUCO_BOARD_H_
