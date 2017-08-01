#include "realsense.h"

using namespace std;
using namespace cv;
using namespace rs;

context _rs_ctx;
device *_rs_camera = NULL;
intrinsics _depth_intrin;
intrinsics _color_intrin;
bool _loop = true;

int IMAGE_HEIGHT;
int IMAGE_WIDTH;


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
    cv::morphologyEx(depthValueMat, opened, cv::MORPH_OPEN, element, cv::Point(-1, -1), 1);
    imshow("opened", opened * 100);
    cvWaitKey(1);
    float depthInMeters = depthValue * scale;
    return depthInMeters;
}

pair<Mat, float> getDistanceMatrix(rs::device *dev)
{
    uint16_t *depthImage = (uint16_t *)dev->get_frame_data(rs::stream::depth);
    float scale = dev->get_depth_scale();
    rs::intrinsics depthIntrin = dev->get_stream_intrinsics(rs::stream::depth);
    cv::Mat depthValueMat = cv::Mat(depthIntrin.height, depthIntrin.width, CV_16U, depthImage);

    pair<Mat, float> result;
    result.first = depthValueMat;
    result.second = scale;
    // Mat depthValueFloat = (float)(depthValueMat * scale);
    return result;
}

/////////////////////////////////////////////////////////////////////////////
// Detect function
/////////////////////////////////////////////////////////////////////////////
void detectSquareBox(Mat srcImage, Mat distanceMatrix)
{
    int rowNum = distanceMatrix.rows;
    int colNum = distanceMatrix.cols;

    int blackPixel[rowNum];
    int lineAxis[rowNum];
    bool lineFlag = false;
    vector<Point2f> points;
    Vec4f verticalLine;
    points.clear();
    //从下网上扫描
    for (int row = rowNum - 1; row >= 50; row--)
    {
        //黑色像素
        blackPixel[row] = 0;
        //从左往右扫描
        for (int col = 0; col < colNum - 1; col++)
        {
            //检测相邻两个元素的像素值
            int pixelValueNow = distanceMatrix.at<uint16_t>(row, col);
            int pixelValueNext = distanceMatrix.at<uint16_t>(row, col + 1);

            //如果当前像素为盒子所在像素
            if (pixelValueNow > 400 && pixelValueNow < 700)
            {
                //黑色像素计数加一
                blackPixel[row]++;
                //如果黑色像素数量大于某个值，同时下一个像素值为白色像素，认为出现线段
                if (blackPixel[row] > 10 && pixelValueNext > 700)
                {
                    //出现线段
                    lineFlag = true;
                    //计算中心位置
                    lineAxis[row] = (col + col - blackPixel[row]) / 2;
                    //存储当前行的中心像素位置
                    points.push_back(Point2f(lineAxis[row], row));
                    //cout << lineAxis << endl;
                    break;
                }
            }
            else
            {
                lineFlag = false;
                blackPixel[row] = 0;
            }
        }
    }
    if (points.size() > 5)
    {
        fitLine(Mat(points), verticalLine, CV_DIST_L1, 0, 0.01, 0.01);
    }

    double k = verticalLine[1] / verticalLine[0];
    double step = 100;
    cv::line(srcImage, cvPoint(verticalLine[2] - step, verticalLine[3] - k * step), cvPoint(verticalLine[2] + step, k * step + verticalLine[3]), Scalar(0, 0, 255), 5);
    imshow("dstImage", srcImage);
    //cout << "是否检测到盒子: " << lineFlag << endl;
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




Mat realSenseSmooth(Mat i_depth)
{
	//cout << "INNER_NUMBER: " << INNER_NUMBER << " OUTER_NUMBER: " << OUTER_NUMBER << endl;
	double minv = 0.0, maxv = 0.0;
    double* minp = &minv;
    double* maxp = &maxv;

    minMaxIdx(i_depth,minp,maxp);

    // cout << "Mat minv = " << minv << endl;
    // cout << "Mat maxv = " << maxv << endl;
//    Size size(512,424);
//    resize(i_depth,i_depth,size);

	IMAGE_HEIGHT = i_depth.rows;
	IMAGE_WIDTH = i_depth.cols;

    // cout << IMAGE_WIDTH << " " << IMAGE_HEIGHT<<endl;

	Mat i_before(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC4);  // 为了显示方便
	Mat i_after(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC4);   // 为了显示方便
	Mat i_result(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1); // 滤波结果

    unsigned int maxDepth = 0;
	//	unsigned short* depthArray = (unsigned short*)i_depth.data;
	unsigned char depthArray[IMAGE_HEIGHT * IMAGE_WIDTH];


	unsigned int iZeroCountBefore = 0;
	unsigned int iZeroCountAfter = 0;
	for (int i = 0; i < IMAGE_WIDTH * IMAGE_HEIGHT; i++)
	{
		int row = i / IMAGE_WIDTH;
		int col = i % IMAGE_WIDTH;
		depthArray[i] = i_depth.at<uint8_t>(row, col);
		unsigned char depthValue = depthArray[i];
		//        unsigned short depthValue = depthArray[row * IMAGE_WIDTH + col];
		// if (depthValue == 0)
		// {
		// 	i_before.data[i * 4] = 255;
		// 	i_before.data[i * 4 + 1] = 0;
		// 	i_before.data[i * 4 + 2] = 0;
		// 	i_before.data[i * 4 + 3] = depthValue / 256;
		// 	iZeroCountBefore++;
		// }
		// else
		// {
		// 	i_before.data[i * 4] = depthValue / 8000.0f * 256;
		// 	i_before.data[i * 4 + 1] = depthValue / 8000.0f * 256;
		// 	i_before.data[i * 4 + 2] = depthValue / 8000.0f * 256;
		// 	i_before.data[i * 4 + 3] = depthValue / 8000.0f * 256;
		// }
		maxDepth = depthValue > maxDepth ? depthValue : maxDepth;
	}
	// cout << "max depth value: " << maxDepth << endl;

	// 2. 像素滤波

	// 滤波后深度图的缓存
	unsigned char *smoothDepthArray = (unsigned char *)i_result.data;
	// 我们用这两个值来确定索引在正确的范围内
	int widthBound = IMAGE_WIDTH - 1;
	int heightBound = IMAGE_HEIGHT - 1;

	// 内（8个像素）外（16个像素）层阈值
	int innerBandThreshold = INNER_BAND_THRESHOLD;
	int outerBandThreshold = OUTER_BAND_THRESHOLD;

	// 处理每行像素
	for (int depthArrayRowIndex = 0; depthArrayRowIndex < IMAGE_HEIGHT; depthArrayRowIndex++)
	{
		// 处理一行像素中的每个像素
		for (int depthArrayColumnIndex = 0; depthArrayColumnIndex < IMAGE_WIDTH; depthArrayColumnIndex++)
		{
			int depthIndex = depthArrayColumnIndex + (depthArrayRowIndex * IMAGE_WIDTH);

			// 我们认为深度值为0的像素即为候选像素
			if (depthArray[depthIndex] == 0)
			{
				// 通过像素索引，我们可以计算得到像素的横纵坐标
				int x = depthIndex % IMAGE_WIDTH;
				int y = (depthIndex - x) / IMAGE_WIDTH;

				// filter collection 用来计算滤波器内每个深度值对应的频度，在后面
				// 我们将通过这个数值来确定给候选像素一个什么深度值。
				unsigned char filterCollection[WINDOWS_SIZE*WINDOWS_SIZE - 1][2] = {0};

				// 内外层框内非零像素数量计数器，在后面用来确定候选像素是否滤波
				int innerBandCount = 0;
				int outerBandCount = 0;

				// 下面的循环将会对以候选像素为中心的5 X 5的像素阵列进行遍历。这里定义了两个边界。如果在
				// 这个阵列内的像素为非零，那么我们将记录这个深度值，并将其所在边界的计数器加一，如果计数器
				// 高过设定的阈值，那么我们将取滤波器内统计的深度值的众数（频度最高的那个深度值）应用于候选
				// 像素上
				for (int yi = -(WINDOWS_SIZE / 2); yi < WINDOWS_SIZE / 2 + 1; yi++)
				{
					for (int xi = -(WINDOWS_SIZE / 2); xi < WINDOWS_SIZE / 2 + 1; xi++)
					{
						// yi和xi为操作像素相对于候选像素的平移量

						// 我们不要xi = 0&&yi = 0的情况，因为此时操作的就是候选像素
						if (xi != 0 || yi != 0)
						{
							// 确定操作像素在深度图中的位置
							int xSearch = x + xi;
							int ySearch = y + yi;

							// 检查操作像素的位置是否超过了图像的边界（候选像素在图像的边缘）
							if (xSearch >= 0 && xSearch <= widthBound &&
								ySearch >= 0 && ySearch <= heightBound)
							{
								int index = xSearch + (ySearch * IMAGE_WIDTH);
								// 我们只要非零量
								if (depthArray[index] != 0)
								{
									// 计算每个深度值的频度
									for (int i = 0; i < WINDOWS_SIZE; i++)
									{
										if (filterCollection[i][0] == depthArray[index])
										{
											// 如果在 filter collection中已经记录过了这个深度
											// 将这个深度对应的频度加一
											filterCollection[i][1]++;
											break;
										}
										else if (filterCollection[i][0] == 0)
										{
											// 如果filter collection中没有记录这个深度
											// 那么记录
											filterCollection[i][0] = depthArray[index];
											filterCollection[i][1]++;
											break;
										}
									}

									// 确定是内外哪个边界内的像素不为零，对相应计数器加一
									if (yi <= WINDOWS_SIZE / 2 && yi != -(WINDOWS_SIZE / 2) && xi != WINDOWS_SIZE / 2 && xi != -(WINDOWS_SIZE / 2))
										innerBandCount++;
									else
										outerBandCount++;
								}
							}
						}
					}
				}

				// 判断计数器是否超过阈值，如果任意层内非零像素的数目超过了阈值，
				// 就要将所有非零像素深度值对应的统计众数
				if (innerBandCount >= innerBandThreshold || outerBandCount >= outerBandThreshold)
				{
					char frequency = 0;
					char depth = 0;
					// 这个循环将统计所有非零像素深度值对应的众数
					for (int i = 0; i < WINDOWS_SIZE * WINDOWS_SIZE - 1; i++)
					{
						// 当没有记录深度值时（无非零深度值的像素）
						if (filterCollection[i][0] == 0)
							break;
						if (filterCollection[i][1] > frequency)
						{
							depth = filterCollection[i][0];
							frequency = filterCollection[i][1];
						}
					}

					smoothDepthArray[depthIndex] = depth;
				}
				else
				{
					smoothDepthArray[depthIndex] = 0;
				}
			}
			else
			{
				// 如果像素的深度值不为零，保持原深度值
				smoothDepthArray[depthIndex] = depthArray[depthIndex];
			}
		}
	}

	for (int i = 0; i < IMAGE_WIDTH * IMAGE_HEIGHT; i++)
	{
		int row = i / IMAGE_WIDTH;
		int col = i % IMAGE_WIDTH;

		unsigned char depthValue = smoothDepthArray[row * IMAGE_WIDTH + col];
		// if (depthValue == 0)
		// {
		// 	i_after.data[i * 4] = 255;
		// 	i_after.data[i * 4 + 1] = 0;
		// 	i_after.data[i * 4 + 2] = 0;
		// 	i_after.data[i * 4 + 3] = depthValue / 256;
		// 	iZeroCountAfter++;
		// }
		// else
		// {
		// 	i_after.data[i * 4] = depthValue / 8000.0f * 256;
		// 	i_after.data[i * 4 + 1] = depthValue / 8000.0f * 256;
		// 	i_after.data[i * 4 + 2] = depthValue / 8000.0f * 256;
		// 	i_after.data[i * 4 + 3] = depthValue / 8000.0f * 256;
		// }
	}
	// cout << "iZeroCountBefore:    " << iZeroCountBefore << "  depthArray[0]:  " << depthArray[0] << endl;
	// cout << "iZeroCountAfter:    " << iZeroCountAfter << "  smoothDepthArray[0]:  " << smoothDepthArray[0] << endl;
	return i_result;
}

Mat realSenseSmooth_uint16(Mat i_depth)
{
	//cout << "INNER_NUMBER: " << INNER_NUMBER << " OUTER_NUMBER: " << OUTER_NUMBER << endl;
	double minv = 0.0, maxv = 0.0;
    double* minp = &minv;
    double* maxp = &maxv;

    minMaxIdx(i_depth,minp,maxp);

    // cout << "Mat minv = " << minv << endl;
    // cout << "Mat maxv = " << maxv << endl;
//    Size size(512,424);
//    resize(i_depth,i_depth,size);

	IMAGE_HEIGHT = i_depth.rows;
	IMAGE_WIDTH = i_depth.cols;

    // cout << IMAGE_WIDTH << " " << IMAGE_HEIGHT<<endl;

	Mat i_before(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC4);  // 为了显示方便
	Mat i_after(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC4);   // 为了显示方便
	Mat i_result(IMAGE_HEIGHT, IMAGE_WIDTH, CV_16UC1); // 滤波结果

    unsigned int maxDepth = 0;
	//	unsigned short* depthArray = (unsigned short*)i_depth.data;
	unsigned short depthArray[IMAGE_HEIGHT * IMAGE_WIDTH];


	unsigned int iZeroCountBefore = 0;
	unsigned int iZeroCountAfter = 0;
	for (int i = 0; i < IMAGE_WIDTH * IMAGE_HEIGHT; i++)
	{
		int row = i / IMAGE_WIDTH;
		int col = i % IMAGE_WIDTH;
		depthArray[i] = i_depth.at<uint8_t>(row, col);
		unsigned short depthValue = depthArray[i];
		//        unsigned short depthValue = depthArray[row * IMAGE_WIDTH + col];
		// if (depthValue == 0)
		// {
		// 	i_before.data[i * 4] = 255;
		// 	i_before.data[i * 4 + 1] = 0;
		// 	i_before.data[i * 4 + 2] = 0;
		// 	i_before.data[i * 4 + 3] = depthValue / 256;
		// 	iZeroCountBefore++;
		// }
		// else
		// {
		// 	i_before.data[i * 4] = depthValue / 8000.0f * 256;
		// 	i_before.data[i * 4 + 1] = depthValue / 8000.0f * 256;
		// 	i_before.data[i * 4 + 2] = depthValue / 8000.0f * 256;
		// 	i_before.data[i * 4 + 3] = depthValue / 8000.0f * 256;
		// }
		maxDepth = depthValue > maxDepth ? depthValue : maxDepth;
	}
	// cout << "max depth value: " << maxDepth << endl;

	// 2. 像素滤波

	// 滤波后深度图的缓存
	unsigned short *smoothDepthArray = (unsigned short *)i_result.data;
	// 我们用这两个值来确定索引在正确的范围内
	int widthBound = IMAGE_WIDTH - 1;
	int heightBound = IMAGE_HEIGHT - 1;

	// 内（8个像素）外（16个像素）层阈值
	int innerBandThreshold = INNER_BAND_THRESHOLD;
	int outerBandThreshold = OUTER_BAND_THRESHOLD;

	// 处理每行像素
	for (int depthArrayRowIndex = 0; depthArrayRowIndex < IMAGE_HEIGHT; depthArrayRowIndex++)
	{
		// 处理一行像素中的每个像素
		for (int depthArrayColumnIndex = 0; depthArrayColumnIndex < IMAGE_WIDTH; depthArrayColumnIndex++)
		{
			int depthIndex = depthArrayColumnIndex + (depthArrayRowIndex * IMAGE_WIDTH);

			// 我们认为深度值为0的像素即为候选像素
			if (depthArray[depthIndex] == 0)
			{
				// 通过像素索引，我们可以计算得到像素的横纵坐标
				int x = depthIndex % IMAGE_WIDTH;
				int y = (depthIndex - x) / IMAGE_WIDTH;

				// filter collection 用来计算滤波器内每个深度值对应的频度，在后面
				// 我们将通过这个数值来确定给候选像素一个什么深度值。
				unsigned short filterCollection[WINDOWS_SIZE*WINDOWS_SIZE - 1][2] = {0};

				// 内外层框内非零像素数量计数器，在后面用来确定候选像素是否滤波
				int innerBandCount = 0;
				int outerBandCount = 0;

				// 下面的循环将会对以候选像素为中心的5 X 5的像素阵列进行遍历。这里定义了两个边界。如果在
				// 这个阵列内的像素为非零，那么我们将记录这个深度值，并将其所在边界的计数器加一，如果计数器
				// 高过设定的阈值，那么我们将取滤波器内统计的深度值的众数（频度最高的那个深度值）应用于候选
				// 像素上
				for (int yi = -(WINDOWS_SIZE / 2); yi < WINDOWS_SIZE / 2 + 1; yi++)
				{
					for (int xi = -(WINDOWS_SIZE / 2); xi < WINDOWS_SIZE / 2 + 1; xi++)
					{
						// yi和xi为操作像素相对于候选像素的平移量

						// 我们不要xi = 0&&yi = 0的情况，因为此时操作的就是候选像素
						if (xi != 0 || yi != 0)
						{
							// 确定操作像素在深度图中的位置
							int xSearch = x + xi;
							int ySearch = y + yi;

							// 检查操作像素的位置是否超过了图像的边界（候选像素在图像的边缘）
							if (xSearch >= 0 && xSearch <= widthBound &&
								ySearch >= 0 && ySearch <= heightBound)
							{
								int index = xSearch + (ySearch * IMAGE_WIDTH);
								// 我们只要非零量
								if (depthArray[index] != 0)
								{
									// 计算每个深度值的频度
									for (int i = 0; i < WINDOWS_SIZE; i++)
									{
										if (filterCollection[i][0] == depthArray[index])
										{
											// 如果在 filter collection中已经记录过了这个深度
											// 将这个深度对应的频度加一
											filterCollection[i][1]++;
											break;
										}
										else if (filterCollection[i][0] == 0)
										{
											// 如果filter collection中没有记录这个深度
											// 那么记录
											filterCollection[i][0] = depthArray[index];
											filterCollection[i][1]++;
											break;
										}
									}

									// 确定是内外哪个边界内的像素不为零，对相应计数器加一
									if (yi <= WINDOWS_SIZE / 2 && yi != -(WINDOWS_SIZE / 2) && xi != WINDOWS_SIZE / 2 && xi != -(WINDOWS_SIZE / 2))
										innerBandCount++;
									else
										outerBandCount++;
								}
							}
						}
					}
				}

				// 判断计数器是否超过阈值，如果任意层内非零像素的数目超过了阈值，
				// 就要将所有非零像素深度值对应的统计众数
				if (innerBandCount >= innerBandThreshold || outerBandCount >= outerBandThreshold)
				{
					short frequency = 0;
					short depth = 0;
					// 这个循环将统计所有非零像素深度值对应的众数
					for (int i = 0; i < WINDOWS_SIZE * WINDOWS_SIZE - 1; i++)
					{
						// 当没有记录深度值时（无非零深度值的像素）
						if (filterCollection[i][0] == 0)
							break;
						if (filterCollection[i][1] > frequency)
						{
							depth = filterCollection[i][0];
							frequency = filterCollection[i][1];
						}
					}

					smoothDepthArray[depthIndex] = depth;
				}
				else
				{
					smoothDepthArray[depthIndex] = 0;
				}
			}
			else
			{
				// 如果像素的深度值不为零，保持原深度值
				smoothDepthArray[depthIndex] = depthArray[depthIndex];
			}
		}
	}

	for (int i = 0; i < IMAGE_WIDTH * IMAGE_HEIGHT; i++)
	{
		int row = i / IMAGE_WIDTH;
		int col = i % IMAGE_WIDTH;

		unsigned short depthValue = smoothDepthArray[row * IMAGE_WIDTH + col];
		// if (depthValue == 0)
		// {
		// 	i_after.data[i * 4] = 255;
		// 	i_after.data[i * 4 + 1] = 0;
		// 	i_after.data[i * 4 + 2] = 0;
		// 	i_after.data[i * 4 + 3] = depthValue / 256;
		// 	iZeroCountAfter++;
		// }
		// else
		// {
		// 	i_after.data[i * 4] = depthValue / 8000.0f * 256;
		// 	i_after.data[i * 4 + 1] = depthValue / 8000.0f * 256;
		// 	i_after.data[i * 4 + 2] = depthValue / 8000.0f * 256;
		// 	i_after.data[i * 4 + 3] = depthValue / 8000.0f * 256;
		// }
	}
	// cout << "iZeroCountBefore:    " << iZeroCountBefore << "  depthArray[0]:  " << depthArray[0] << endl;
	// cout << "iZeroCountAfter:    " << iZeroCountAfter << "  smoothDepthArray[0]:  " << smoothDepthArray[0] << endl;
	return i_result;
}