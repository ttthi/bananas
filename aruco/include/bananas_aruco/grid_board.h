#ifndef GRID_BOARD_H_
#define GRID_BOARD_H_

#include <cstdint>

#include <Eigen/Geometry>

#include <nlohmann/json.hpp>

#include <bananas_aruco/board.h>

namespace board {

struct GridSize {
    std::uint32_t num_columns{5};
    std::uint32_t num_rows{5};
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(GridSize, num_columns, num_rows);

struct GridSettings {
    GridSize size{};
    float marker_side{1.0F};
    float marker_separation{1.0F};
    int start_id{0};
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(GridSettings, size, marker_side,
                                   marker_separation, start_id);

auto grid_width(const GridSettings &settings) -> float;

auto grid_height(const GridSettings &settings) -> float;

/// Produce an ArUco board for the given grid. The markers are placed facing up,
/// right-to-left and back-to-front in the glTF coordinate system (+X is left,
/// +Y is up and +Z is forward).
auto make_board(const GridSettings &settings) -> Board;

} // namespace board

#endif // GRID_BOARD_H_
