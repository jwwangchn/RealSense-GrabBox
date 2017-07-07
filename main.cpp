#include <librealsense/rs.hpp>
#include <opencv2/opencv.hpp>
#include "realsense.h"

using namespace std;
using namespace cv;
using namespace rs;

int main()
{
    rs::context ctx;
    rs::device *dev = ctx.get_device(0);

    if (ctx.get_device_count() <= 0)
    {
        cout << "can't open the realsense!!!" << endl;
        return 0;
    }

    dev->enable_stream(rs::stream::color, INPUT_WIDTH, INPUT_HEIGHT, rs::format::bgr8, FRAME_RATE);
    dev->enable_stream(rs::stream::depth, INPUT_WIDTH, INPUT_HEIGHT, rs::format::z16, FRAME_RATE);

    dev->start();
    while (1)
    {
        // Camera warmup - Dropped frames to allow stabilization
        for (int i = 0; i < 1; i++)
            dev->wait_for_frames();

        Mat bgr(Size(INPUT_WIDTH, INPUT_HEIGHT), CV_8UC3, (void *)dev->get_frame_data(rs::stream::color), Mat::AUTO_STEP);
        Mat depth(Size(INPUT_WIDTH, INPUT_HEIGHT), CV_8UC1, (void *)dev->get_frame_data(rs::stream::depth), Mat::AUTO_STEP);
        Mat cad(Size(INPUT_WIDTH, INPUT_HEIGHT), CV_8UC3, (void *)dev->get_frame_data(rs::stream::color_aligned_to_depth), Mat::AUTO_STEP);

        // equalizeHist(depth, depth);
        applyColorMap(depth, depth, COLORMAP_JET);

        namedWindow("Display Image RGB", WINDOW_AUTOSIZE);
        namedWindow("Display Image DEPTH", WINDOW_AUTOSIZE);
        namedWindow("Display Image CAD", WINDOW_AUTOSIZE);
        imshow("Display Image RGB", bgr);
        imshow("Display Image DEPTH", depth);
        imshow("Display Image CAD", cad);
        cout << getDistance(dev, 200, 200) << endl;
        waitKey(1);
    }

    return 0;
}