#ifndef GA_H
#define GA_H

#include <librealsense/rs.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;
using namespace rs;

#define INPUT_WIDTH 640
#define INPUT_HEIGHT 480
#define FRAME_RATE 30

float getDistance(rs::device *dev, int x, int y);


#endif