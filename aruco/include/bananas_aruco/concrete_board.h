#ifndef BANANAS_ARUCO_CONCRETE_BOARD_H_
#define BANANAS_ARUCO_CONCRETE_BOARD_H_

#include <variant>

#include <nlohmann/json_fwd.hpp>

#include <bananas_aruco/box_board.h>
#include <bananas_aruco/grid_board.h>

namespace bananas::board {

using ConcreteBoard = std::variant<BoxSettings, GridSettings>;
void from_json(const nlohmann::json &j, ConcreteBoard &board);

} // namespace bananas::board

#endif // BANANAS_ARUCO_CONCRETE_BOARD_H_
