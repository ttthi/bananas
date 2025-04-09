#include <cmath>
#include <vector>

#include <gmock/gmock-matchers.h>
#include <gtest/gtest.h>

#include <nlohmann/json.hpp>

#include <opencv2/core/types.hpp>

#include <bananas_aruco/box_board.h>

// NOLINTNEXTLINE(google-build-using-namespace)
using namespace nlohmann::json_literals;

// LINT: gtest macros quickly bump this metric up.
// NOLINTBEGIN(readability-function-cognitive-complexity)

namespace {

namespace board = bananas::board;

// NOLINTNEXTLINE(modernize-use-trailing-return-type)
MATCHER(PointsEq, std::string{"are "} + (negation ? "not " : "") +
                      "approximately equal points") {
    const cv::Point3f a{std::get<0>(arg)};
    const cv::Point3f b{std::get<1>(arg)};
    return testing::ExplainMatchResult(testing::FloatEq(b.x), a.x,
                                       result_listener) &&
           testing::ExplainMatchResult(testing::FloatEq(b.y), a.y,
                                       result_listener) &&
           testing::ExplainMatchResult(testing::FloatEq(b.z), a.z,
                                       result_listener);
}

} // namespace

// Tests that the default orientations of the ArUco markers are as expected.
TEST(BoxBoardTest, DefaultOrientationsAreCorrect) {
    const board::BoxSettings box{{1.0, 2.0, 3.0},
                                 {std::vector{board::BoxMarkerSettings{3}},
                                  {{1}},
                                  {{7}},
                                  {{5}},
                                  {{2}},
                                  {{9}}}};

    const auto board{board::make_board(box)};

    const std::vector<int> expected_ids{3, 1, 7, 5, 2, 9};
    EXPECT_EQ(board.marker_ids, expected_ids);

    const auto &object_points{board.obj_points};
    ASSERT_EQ(object_points.size(), 6);

    const std::vector<cv::Point3f> expected_forward{{-0.5F, 0.5F, 1.5F},
                                                    {0.5F, 0.5F, 1.5F},
                                                    {0.5F, -0.5F, 1.5F},
                                                    {-0.5F, -0.5F, 1.5F}};
    // We have a single marker per face, so we can just use the face index
    // constants. If the markers didn't match one-to-one to the faces, the
    // indices would be different.
    EXPECT_EQ(object_points[board::BoxSettings::forward_face_index],
              expected_forward);

    const std::vector<cv::Point3f> expected_backward{{0.5F, 0.5F, -1.5F},
                                                     {-0.5F, 0.5F, -1.5F},
                                                     {-0.5F, -0.5F, -1.5F},
                                                     {0.5F, -0.5F, -1.5F}};
    EXPECT_EQ(object_points[board::BoxSettings::backward_face_index],
              expected_backward);

    const std::vector<cv::Point3f> expected_left{{0.5F, 0.5F, 0.5F},
                                                 {0.5F, 0.5F, -0.5F},
                                                 {0.5F, -0.5F, -0.5F},
                                                 {0.5F, -0.5F, 0.5F}};
    EXPECT_EQ(object_points[board::BoxSettings::left_face_index],
              expected_left);

    const std::vector<cv::Point3f> expected_right{{-0.5F, 0.5F, -0.5F},
                                                  {-0.5F, 0.5F, 0.5F},
                                                  {-0.5F, -0.5F, 0.5F},
                                                  {-0.5F, -0.5F, -0.5F}};
    EXPECT_EQ(object_points[board::BoxSettings::right_face_index],
              expected_right);

    const std::vector<cv::Point3f> expected_up{{-0.5F, 1.0F, 0.5F},
                                               {-0.5F, 1.0F, -0.5F},
                                               {0.5F, 1.0F, -0.5F},
                                               {0.5F, 1.0F, 0.5F}};
    EXPECT_EQ(object_points[board::BoxSettings::up_face_index], expected_up);

    const std::vector<cv::Point3f> expected_down{{-0.5F, -1.0F, -0.5F},
                                                 {-0.5F, -1.0F, 0.5F},
                                                 {0.5F, -1.0F, 0.5F},
                                                 {0.5F, -1.0F, -0.5F}};
    EXPECT_EQ(object_points[board::BoxSettings::down_face_index],
              expected_down);
}

// Tests that rotated and offset markers are placed correctly.
TEST(BoxBoardTest, RotationsAndOffsetsWork) {
    const float quarter_circle{std::atan(1.0F) * 2.0F};
    const board::BoxSettings box{
        {3.0, 2.0, 1.0},
        {
            std::vector{
                board::BoxMarkerSettings{0, 0.5, -0.5, -quarter_circle, 0.5F},
                {1, -0.25, 0.5, 2.0F * quarter_circle, 0.25F}},
            {{{2, 0.0F, 0.5F, 0.0F, 0.5F}}},
            {},
            {},
            {{3, 0.25F, -0.25F, 3.0F * quarter_circle, 0.5F}},
            {},
        }};

    const auto board{board::make_board(box)};

    const std::vector<int> expected_ids{0, 1, 2, 3};
    EXPECT_EQ(board.marker_ids, expected_ids);

    const auto &object_points{board.obj_points};
    ASSERT_EQ(object_points.size(), 4);

    const std::vector<cv::Point3f> expected_forward1{{0.75F, -0.25F, 0.5F},
                                                     {0.75F, -0.75F, 0.5F},
                                                     {0.25F, -0.75F, 0.5F},
                                                     {0.25F, -0.25F, 0.5F}};
    // We don't require the rotations to be exact (but they actually are!), so
    // let's use approximate equality here.
    EXPECT_THAT(object_points[0],
                testing::Pointwise(PointsEq(), expected_forward1));

    const std::vector<cv::Point3f> expected_forward2{{-0.125F, 0.375F, 0.5F},
                                                     {-0.375F, 0.375F, 0.5F},
                                                     {-0.375F, 0.625F, 0.5F},
                                                     {-0.125F, 0.625F, 0.5F}};
    EXPECT_THAT(object_points[1],
                testing::Pointwise(PointsEq(), expected_forward2));

    const std::vector<cv::Point3f> expected_backward{{0.25F, 0.75F, -0.5F},
                                                     {-0.25F, 0.75F, -0.5F},
                                                     {-0.25F, 0.25F, -0.5F},
                                                     {0.25F, 0.25F, -0.5F}};
    EXPECT_THAT(object_points[2],
                testing::Pointwise(PointsEq(), expected_backward));

    const std::vector<cv::Point3f> expected_up{{0.0F, 1.0F, -0.5F},
                                               {0.5F, 1.0F, -0.5F},
                                               {0.5F, 1.0F, 0.0F},
                                               {0.0F, 1.0F, 0.0F}};
    EXPECT_THAT(object_points[3], testing::Pointwise(PointsEq(), expected_up));
}

// Tests that boards are correctly decoded from JSON.
TEST(BoxBoardTest, JSONDecodingWorks) {
    const board::BoxSettings settings{[]() {
        const auto json(R"(
            {
                "size": {"width": 1.0, "height": 2.0, "depth": 3.0},
                "markers": {
                    "forward": [
                        {
                            "id": 0,
                            "x_offset": 0.25,
                            "y_offset": 0.5,
                            "rotation": 0.75,
                            "side": 0.875
                        },
                        {
                            "id": 1,
                            "x_offset": 0.125,
                            "y_offset": 0.25,
                            "rotation": -0.25,
                            "side": 0.375
                        }
                    ],
                    "down": [
                        {
                            "id": 2,
                            "x_offset": -0.125,
                            "y_offset": -0.25,
                            "rotation": 3.0,
                            "side": 0.5
                        }
                    ],
                    "backward": []
                }
            }
        )"_json);
        board::BoxSettings settings;
        board::from_json(json, settings);
        return settings;
    }()};

    EXPECT_EQ(settings.size.width, 1.0F);
    EXPECT_EQ(settings.size.height, 2.0F);
    EXPECT_EQ(settings.size.depth, 3.0F);

    const auto &forward{
        settings.markers[board::BoxSettings::forward_face_index]};
    ASSERT_EQ(forward.size(), 2);

    EXPECT_EQ(forward[0].id, 0);
    EXPECT_EQ(forward[0].x_offset, 0.25F);
    EXPECT_EQ(forward[0].y_offset, 0.5F);
    EXPECT_EQ(forward[0].rotation, 0.75F);
    EXPECT_EQ(forward[0].side, 0.875F);

    EXPECT_EQ(forward[1].id, 1);
    EXPECT_EQ(forward[1].x_offset, 0.125F);
    EXPECT_EQ(forward[1].y_offset, 0.25F);
    EXPECT_EQ(forward[1].rotation, -0.25F);
    EXPECT_EQ(forward[1].side, 0.375F);

    const auto &backward{
        settings.markers[board::BoxSettings::backward_face_index]};
    EXPECT_TRUE(backward.empty());

    const auto &left{settings.markers[board::BoxSettings::left_face_index]};
    EXPECT_TRUE(left.empty());

    const auto &right{settings.markers[board::BoxSettings::right_face_index]};
    EXPECT_TRUE(right.empty());

    const auto &up{settings.markers[board::BoxSettings::up_face_index]};
    EXPECT_TRUE(up.empty());

    const auto &down{settings.markers[board::BoxSettings::down_face_index]};
    ASSERT_EQ(down.size(), 1);

    EXPECT_EQ(down[0].id, 2);
    EXPECT_EQ(down[0].x_offset, -0.125F);
    EXPECT_EQ(down[0].y_offset, -0.25F);
    EXPECT_EQ(down[0].rotation, 3.0F);
    EXPECT_EQ(down[0].side, 0.5F);
}

// NOLINTEND(readability-function-cognitive-complexity)
