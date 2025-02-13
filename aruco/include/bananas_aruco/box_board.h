#ifndef BOX_BOARD_H_
#define BOX_BOARD_H_

#include <array>
#include <vector>

#include <opencv2/core/matx.hpp>
#include <opencv2/objdetect/aruco_board.hpp>

namespace box_board {

struct BoxFaceSettings {
    int id{};
    float x{};
    float y{};
    float rotation{};
    float side{1.0F};
};

struct BoxSettings {
    cv::Vec3f size{};
    // TODO(vainiovano): define the order
    std::array<std::vector<BoxFaceSettings>, 6> faces{};
};

auto make_board(const cv::aruco::Dictionary &dictionary,
                const BoxSettings &settings) -> cv::aruco::Board;

// TODO(vainiovano): Take the box data in as JSON or similar
extern const BoxSettings example_box;

} // namespace box_board

#endif // BOX_BOARD_H_
