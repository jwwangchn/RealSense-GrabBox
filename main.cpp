#include <librealsense/rs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include "realsense.h"

using namespace std;
using namespace cv;
using namespace rs;

extern context _rs_ctx;
extern device *_rs_camera;
extern intrinsics _depth_intrin;
extern intrinsics _color_intrin;
extern bool _loop;

/////////////////////////////////////////////////////////////////////////////
// Main function
/////////////////////////////////////////////////////////////////////////////
int main() try
{
    rs::log_to_console(rs::log_severity::warn);

    if (!initialize_streaming())
    {
        std::cout << "Unable to locate a camera" << std::endl;
        rs::log_to_console(rs::log_severity::fatal);
        return EXIT_FAILURE;
    }

    setup_windows();
    RS_OPTION_R200_EMITTER_ENABLED;
    RS_OPTION_R200_LR_AUTO_EXPOSURE_ENABLED;
    // Loop until someone left clicks on either of the images in either window.
    while (_loop)
    {
        if (_rs_camera->is_streaming())
            _rs_camera->wait_for_frames();

        //display_next_frame();
        // Get current frames intrinsic data.
        _depth_intrin = _rs_camera->get_stream_intrinsics(rs::stream::depth);
        _color_intrin = _rs_camera->get_stream_intrinsics(rs::stream::color);

        // Create depth image
        cv::Mat depth16(_depth_intrin.height, _depth_intrin.width, CV_16U, (uchar *)_rs_camera->get_frame_data(rs::stream::depth));

        // Create color image
        cv::Mat rgb(_color_intrin.height, _color_intrin.width, CV_8UC3, (uchar *)_rs_camera->get_frame_data(rs::stream::color));

        // Create color and depth image
        cv::Mat depth16_rgb(_color_intrin.height, _color_intrin.width, CV_8UC3, (uchar *)_rs_camera->get_frame_data(rs::stream::color_aligned_to_depth));

        // < 800
        cv::Mat depth8u = depth16;
        depth8u.convertTo(depth8u, CV_8UC1, 255.0 / 1000);

        imshow(WINDOW_DEPTH, depth8u);
        cvWaitKey(1);

        cv::cvtColor(rgb, rgb, cv::COLOR_BGR2RGB);
        imshow(WINDOW_RGB, rgb);
        cvWaitKey(1);

        cv::cvtColor(depth16_rgb, depth16_rgb, cv::COLOR_BGR2RGB);
        imshow(WINDOW_DEPTH_RGB, depth16_rgb);
        cvWaitKey(1);

        // detectSquareBox();
        // cout << getDistance(_rs_camera, 200, 200) << endl;
        pair<Mat, float> distanceMatrixResult;
        distanceMatrixResult = getDistanceMatrix(_rs_camera);
        Mat distanceMatrix = distanceMatrixResult.first;
        imwrite("depth.png", distanceMatrix);
        float scale = distanceMatrixResult.second;
        cout << distanceMatrix.at<uint16_t>(200, 200) * scale << endl;
        detectSquareBox(rgb, distanceMatrix);
    }

    _rs_camera->stop();
    cv::destroyAllWindows();

    return EXIT_SUCCESS;
}


catch (const rs::error &e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception &e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}

// int main()
// {
//     rs::context ctx;
//     rs::device *dev = ctx.get_device(0);

//     if (ctx.get_device_count() == 0)
//     {
//         cout << "No device detected!!!" << endl;
//         return 0;
//     }

//     dev->enable_stream(rs::stream::color, INPUT_WIDTH, INPUT_HEIGHT, rs::format::rgb8, FRAME_RATE);
//     dev->enable_stream(rs::stream::depth, INPUT_WIDTH, INPUT_HEIGHT, rs::format::z16, FRAME_RATE);

//     dev->start();
//     while (1)
//     {
//         // Camera warmup - Dropped frames to allow stabilization
//         for (int i = 0; i < 2; i++)
//         {
//             dev->wait_for_frames();
//         }
//         uchar *pRgb = (uchar *)dev->get_frame_data(rs::stream::color);
//         uint16_t *depthImage = (uint16_t *)dev->get_frame_data(rs::stream::depth);
//         uchar *pCad = (uchar *)dev->get_frame_data(rs::stream::color_aligned_to_depth);

//         Mat rgb_show;
//         Mat rgb(INPUT_HEIGHT, INPUT_WIDTH, CV_8UC3, pRgb);
//         cvtColor(rgb, rgb_show, CV_BGR2RGB);

//         Mat depth(INPUT_HEIGHT, INPUT_WIDTH, CV_16UC1, depthImage);

//         Mat cad_show;
//         Mat cad(INPUT_HEIGHT, INPUT_WIDTH, CV_8UC3, pCad);
//         cvtColor(cad, cad_show, CV_BGR2RGB);

//         imshow("RGBImage", rgb_show);
//         imshow("DepthImage", depth * 15);
//         imshow("CADImage", cad_show);

//         cout << getDistance(dev, 200, 200) << endl;
//         waitKey(10);
//     }
//     return 0;
// }
