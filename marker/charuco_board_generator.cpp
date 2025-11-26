#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <marker_helpers.hpp>

void export_charuco_tiled_A4()
{
    int squaresX = 12;
    int squaresY = 17;

    float squareLength = 0.04f; // 40 mm
    float markerLength = 0.03f; // 30 mm

    // === YOUR OPENCV VERSION NEEDS A RAW DICTIONARY OBJECT ===
    cv::aruco::Dictionary dictionary =
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_1000);

    // === OLD API: Constructor takes dictionary BY REFERENCE, not a Ptr ===
    cv::Ptr<cv::aruco::CharucoBoard> board =
        cv::makePtr<cv::aruco::CharucoBoard>(
            cv::Size(squaresX, squaresY),
            squareLength,
            markerLength,
            dictionary   // ← NOTE: NOT a Ptr
        );

    // Output DPI
    int DPI = 300;
    float ppm = DPI / 0.0254f; // pixels per meter

    int width_px = int(squareLength * squaresX * ppm);
    int height_px = int(squareLength * squaresY * ppm);

    // Render board
    cv::Mat bigBoard;
    cv::aruco::drawPlanarBoard(board, cv::Size(width_px, height_px),
        bigBoard, 20, 1);

    // A4 = 2480 × 3508 px @ 300 DPI
    int A4_w = 2480;
    int A4_h = 3508;

    int page = 0;

    for (int y = 0; y < height_px; y += A4_h)
    {
        for (int x = 0; x < width_px; x += A4_w)
        {
            cv::Rect tile(
                x, y,
                std::min(A4_w, width_px - x),
                std::min(A4_h, height_px - y)
            );

            cv::Mat section = bigBoard(tile).clone();
            std::string filename =
                "charuco_page_" + std::to_string(++page) + ".png";

            cv::imwrite(filename, section);
        }
    }

    std::cout << "Generated " << page << " A4 tiles.\n";
}
