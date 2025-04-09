#include <bananas_aruco/board.h>

#include <opencv2/objdetect/aruco_board.hpp>

namespace bananas::board {

auto to_cv(const cv::aruco::Dictionary &dictionary,
           const Board &board) -> cv::aruco::Board {
    return {board.obj_points, dictionary, board.marker_ids};
}

} // namespace bananas::board
