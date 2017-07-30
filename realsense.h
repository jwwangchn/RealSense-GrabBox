#ifndef GA_H
#define GA_H

#include <librealsense/rs.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;
using namespace rs;

// Window size and frame rate
int const INPUT_WIDTH = 640;
int const INPUT_HEIGHT = 480;
int const FRAMERATE = 30;

// Named windows
char *const WINDOW_DEPTH = "Depth Image";
char *const WINDOW_RGB = "RGB Image";
char *const WINDOW_DEPTH_RGB = "DEPTH RGB Image";

extern context _rs_ctx;
extern device *_rs_camera;
extern intrinsics _depth_intrin;
extern intrinsics _color_intrin;
extern bool _loop;

bool initialize_streaming();
void setup_windows();
bool display_next_frame();
float getDistance(rs::device *dev, int x, int y);
pair<Mat, float> getDistanceMatrix(rs::device *dev);
void detectSquareBox(Mat srcImage, Mat distanceMatrix);
static void onMouse(int event, int x, int y, int, void *window_name);


#endif