#ifndef GA_H
#define GA_H

#include <librealsense/rs.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;
using namespace rs;

// 滤波窗口大小 WINDOWS_SIZE - 1 为 4 的倍数
#define WINDOWS_SIZE 13


// 内层数量 = (WINDOWS_SIZE / 2 + 1) * (WINDOWS_SIZE / 2 + 1) - 1
// 外层数量 = WINDOWS_SIZE * WINDOWS_SIZE - ((WINDOWS_SIZE / 2 + 1) * (WINDOWS_SIZE / 2 + 1) - 1)

#define INNER_NUMBER (WINDOWS_SIZE / 2 + 1) * (WINDOWS_SIZE / 2 + 1) - 1
#define OUTER_NUMBER WINDOWS_SIZE * WINDOWS_SIZE - ((WINDOWS_SIZE / 2 + 1) * (WINDOWS_SIZE / 2 + 1))

// 内外层阈值
#define INNER_BAND_THRESHOLD INNER_NUMBER / 2 - 1
#define OUTER_BAND_THRESHOLD OUTER_NUMBER / 2 - 1


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

extern int IMAGE_HEIGHT;
extern int IMAGE_WIDTH;

bool initialize_streaming();
void setup_windows();
bool display_next_frame();
float getDistance(rs::device *dev, int x, int y);
pair<Mat, float> getDistanceMatrix(rs::device *dev);
void detectSquareBox(Mat srcImage, Mat distanceMatrix);
static void onMouse(int event, int x, int y, int, void *window_name);
Mat realSenseSmooth(Mat i_depth);

#endif