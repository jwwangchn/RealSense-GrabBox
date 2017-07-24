#include <librealsense/rs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include "realsense.h"

using namespace std;
using namespace cv;
using namespace rs;

// Window size and frame rate
int const INPUT_WIDTH = 320;
int const INPUT_HEIGHT = 240;
int const FRAMERATE = 60;

// Named windows
char *const WINDOW_DEPTH = "Depth Image";
char *const WINDOW_RGB = "RGB Image";
char *const WINDOW_DEPTH_RGB = "DEPTH RGB Image";

context _rs_ctx;
device *_rs_camera = NULL;
intrinsics _depth_intrin;
intrinsics _color_intrin;
bool _loop = true;

// Initialize the application state. Upon success will return the static app_state vars address
bool initialize_streaming()
{
    bool success = false;
    if (_rs_ctx.get_device_count() > 0)
    {
        _rs_camera = _rs_ctx.get_device(0);

        _rs_camera->enable_stream(rs::stream::color, INPUT_WIDTH, INPUT_HEIGHT, rs::format::rgb8, FRAMERATE);
        _rs_camera->enable_stream(rs::stream::depth, INPUT_WIDTH, INPUT_HEIGHT, rs::format::z16, FRAMERATE);

        _rs_camera->start();

        success = true;
    }
    return success;
}

/////////////////////////////////////////////////////////////////////////////
// If the left mouse button was clicked on either image, stop streaming and close windows.
/////////////////////////////////////////////////////////////////////////////
static void onMouse(int event, int x, int y, int, void *window_name)
{
    if (event == cv::EVENT_LBUTTONDOWN)
    {
        _loop = false;
    }
}

/////////////////////////////////////////////////////////////////////////////
// Create the depth and RGB windows, set their mouse callbacks.
// Required if we want to create a window and have the ability to use it in
// different functions
/////////////////////////////////////////////////////////////////////////////
void setup_windows()
{
    cv::namedWindow(WINDOW_DEPTH, WINDOW_AUTOSIZE);
    cv::namedWindow(WINDOW_RGB, WINDOW_AUTOSIZE);
    cv::namedWindow(WINDOW_DEPTH_RGB, WINDOW_AUTOSIZE);

    cv::setMouseCallback(WINDOW_DEPTH, onMouse, WINDOW_DEPTH);
    cv::setMouseCallback(WINDOW_RGB, onMouse, WINDOW_RGB);
    cv::setMouseCallback(WINDOW_DEPTH_RGB, onMouse, WINDOW_RGB);
}

/////////////////////////////////////////////////////////////////////////////
// Called every frame gets the data from streams and displays them using OpenCV.
/////////////////////////////////////////////////////////////////////////////
bool display_next_frame()
{
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
    return true;
}
/////////////////////////////////////////////////////////////////////////////
// Get distance function
/////////////////////////////////////////////////////////////////////////////
float getDistance(rs::device *dev, int x, int y)
{
    uint16_t *depthImage = (uint16_t *)dev->get_frame_data(rs::stream::depth);
    float scale = dev->get_depth_scale();
    rs::intrinsics depthIntrin = dev->get_stream_intrinsics(rs::stream::depth);
    uint16_t depthValue = depthImage[y * depthIntrin.width + x];
    cv::Mat depthValueMat = cv::Mat(depthIntrin.height, depthIntrin.width, CV_16U, depthImage);
    cv::Mat opened;
    cv::Mat element(5, 5, CV_8U, cv::Scalar(1));
    cv::morphologyEx(depthValueMat, opened, cv::MORPH_OPEN, element, cv::Point(-1,-1), 1);
    imshow("opened",opened*100);
    cvWaitKey(1);
    float depthInMeters = depthValue * scale;
    return depthInMeters;
}

/////////////////////////////////////////////////////////////////////////////
// Detect function
/////////////////////////////////////////////////////////////////////////////
pair<double, double> detectSquareBox()
{
    pair<double, double> result;
    int centerPosition = INPUT_WIDTH;
    double leftEdge = 0, minLeftEdge = INT_MAX;
    double rightEdge = 0, minRightEdge = INT_MAX;
    for (int y = 0; y < INPUT_HEIGHT; y++)
    {
        for (int x = 0; x < INPUT_WIDTH; x++)
        {
            float distance = getDistance(_rs_camera, x, y);
            if (distance > 0 && distance < minLeftEdge)
            {
                minLeftEdge = distance;
                // cout << "minLeftEdge: " << minLeftEdge << endl;
                leftEdge = x;
            }
        }
    }
    cout << "position: " << leftEdge << " distance: " << minLeftEdge << endl;
    cout << getDistance(_rs_camera, 200, 200) << endl;
    return result;
}



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

    // Loop until someone left clicks on either of the images in either window.
    while (_loop)
    {
        if (_rs_camera->is_streaming())
            _rs_camera->wait_for_frames();

        display_next_frame();
        // detectSquareBox();
        cout << getDistance(_rs_camera, 200, 200) << endl;
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