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

    if (ctx.get_device_count() == 0)
    {
        cout << "No device detected!!!" << endl;
        return 0;
    }

    dev->enable_stream(rs::stream::color, INPUT_WIDTH, INPUT_HEIGHT, rs::format::rgb8, FRAME_RATE);
    dev->enable_stream(rs::stream::depth, INPUT_WIDTH, INPUT_HEIGHT, rs::format::z16, FRAME_RATE);

    dev->start();
    while (1)
    {
        // Camera warmup - Dropped frames to allow stabilization
        for (int i = 0; i < 2; i++)
        {
            dev->wait_for_frames();
        }
        uchar *pRgb = (uchar *)dev->get_frame_data(rs::stream::color);
        uint16_t *depthImage = (uint16_t *)dev->get_frame_data(rs::stream::depth);
        uchar *pCad = (uchar *)dev->get_frame_data(rs::stream::color_aligned_to_depth);

        Mat rgb_show;
        Mat rgb(INPUT_HEIGHT, INPUT_WIDTH, CV_8UC3, pRgb);
        cvtColor(rgb, rgb_show, CV_BGR2RGB);
        Mat depth(INPUT_HEIGHT, INPUT_WIDTH, CV_16UC1, depthImage);
        Mat cad_show;
        Mat cad(INPUT_HEIGHT, INPUT_WIDTH, CV_8UC3, pCad);
        cvtColor(cad, cad_show, CV_BGR2RGB);


        imshow("RGBImage", rgb_show);
        imshow("DepthImage", depth * 15);
        imshow("CADImage", cad_show);
        
        for

        cout << getDistance(dev, 200, 200) << endl;
        waitKey(10);
    }
    return 0;
}