#include "realsense.h"

using namespace std;
using namespace cv;

float getDistance(rs::device *dev, int x, int y)
{
    uint16_t *depthImage = (uint16_t *) dev->get_frame_data(rs::stream::depth);
    float scale = dev->get_depth_scale();
    rs::intrinsics depthIntrin = dev->get_stream_intrinsics(rs::stream::depth);
    uint16_t depthValue = depthImage[y * depthIntrin.width + x];
    float depthInMeters = depthValue * scale;
    return depthInMeters;
}