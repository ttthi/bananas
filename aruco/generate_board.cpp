#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

#include <opencv2/core/mat.hpp>
#include <opencv2/core/persistence.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/objdetect/aruco_board.hpp>
#include <opencv2/objdetect/aruco_dictionary.hpp>

namespace {
const char *const about{"Create an ArUco grid board image"};
const char *const keys{
    "{@outfile |<none> | Output image }"
    "{w        |       | Number of markers in X direction }"
    "{h        |       | Number of markers in Y direction }"
    "{l        |       | Marker side length (in pixels) }"
    "{s        |       | Separation between two consecutive "
    "markers in the grid (in pixels)}"
    "{cd       |       | Input file with custom dictionary }"
    "{m        |       | Margins size (in pixels). Default is "
    "marker separation (-s) }"
    "{bb       | 1     | Number of bits in marker borders }"
    "{si       | 0     | The dictionary index from which the board is started "
    "}"};
} // namespace

auto main(int argc, char *argv[]) -> int {
    cv::CommandLineParser parser{argc, argv, keys};
    parser.about(about);

    if (argc < 7) {
        parser.printMessage();
        return EXIT_FAILURE;
    }

    const auto markersX{parser.get<int>("w")};
    const auto markersY{parser.get<int>("h")};
    const auto markerLength{parser.get<int>("l")};
    const auto markerSeparation{parser.get<int>("s")};
    int margins{markerSeparation};
    if (parser.has("m")) {
        margins = parser.get<int>("m");
    }

    const auto borderBits{parser.get<int>("bb")};

    const auto out{parser.get<cv::String>(0)};

    if (!parser.check()) {
        parser.printErrors();
        return EXIT_FAILURE;
    }

    cv::Size imageSize{};
    imageSize.width = markersX * (markerLength + markerSeparation) -
                      markerSeparation + 2 * margins;
    imageSize.height = markersY * (markerLength + markerSeparation) -
                       markerSeparation + 2 * margins;

    const cv::FileStorage fs{parser.get<std::string>("cd"),
                             cv::FileStorage::READ};
    cv::aruco::Dictionary dictionary{};
    const bool readOk{dictionary.readDictionary(fs.root())};
    if (!readOk) {
        std::cerr << "Invalid dictionary file\n";
        return EXIT_FAILURE;
    }

    const auto startIndex{parser.get<int>("si")};
    std::vector<int> indices(static_cast<std::size_t>(markersX * markersY));
    std::iota(indices.begin(), indices.end(), startIndex);

    const cv::aruco::GridBoard board{
        cv::Size{markersX, markersY}, static_cast<float>(markerLength),
        static_cast<float>(markerSeparation), dictionary, indices};

    cv::Mat boardImage{};
    board.generateImage(imageSize, boardImage, margins, borderBits);

    cv::imwrite(out, boardImage);
    return EXIT_SUCCESS;
}
