

#include <opencv2/opencv.hpp>
#include <librealsense2/rs.hpp>
#include <librealsense2/cv-helpers.hpp>
#include <librealsense2/rsutil.h>
#include "api_how_to.h"

#pragma comment(lib,"opencv_world340d.lib")
#pragma comment(lib,"realsense2.lib")

using namespace std;
using namespace cv;

///////////
// realsense opencv sample stanley
//////////
Mat depth_mat;
Mat color_mat;

#define depth_window_name "depth"
#define rgb_window_name "rgb"
#define depth_filter_window_name "depth_filter"

static void onMouse(int event, int x, int y, int f, void*)
{
	Mat image = color_mat.clone();
	double depth = depth_mat.at<double>(y, x);
	
	char name[30];
	sprintf_s(name, "depth=%4.2f mm", depth*1000);
	putText(image, name, Point(150, 40), FONT_HERSHEY_SIMPLEX, .7, Scalar(0, 255, 0), 2, 8, false);

	sprintf_s(name, "X=%d", x);
	putText(image, name, Point(25, 300), FONT_HERSHEY_SIMPLEX, .7, Scalar(0, 0, 255), 2, 8, false);

	sprintf_s(name, "Y=%d", y);
	putText(image, name, Point(25, 340), FONT_HERSHEY_SIMPLEX, .7, Scalar(0, 0, 255), 2, 8, false);

	//imwrite("hsv.jpg",image);
	imshow(rgb_window_name, image);
}

void TestDirectRead()
{
	VideoCapture cap(0);

	Mat src;

	while (1)
	{
		cap >> src;
		imshow("src", src);
		waitKey(20);
	}
}

void TestGetIntrinsic()
{
	rs2::device dev = how_to::get_a_realsense_device();
	//how_to::print_device_information(dev);

	print_separator();
	
	rs2::sensor sensor = how_to::get_a_sensor_from_a_device(dev);

	rs2::stream_profile selected_profile = how_to::choose_a_streaming_profile(sensor);
	how_to::get_field_of_view(selected_profile); //show intrincsic
}


struct  rs2_intrinsics mRsDepthIntrinsic;

Mat DH_HomoTran(double d,double theta,double a,double alpha)
{
	Mat Metrix_r = (Mat_<double>(4, 4) <<	cos(theta), -sin(theta)*cos(alpha),  sin(theta)*sin(alpha),  a*cos(theta),
											sin(theta),  cos(theta)*cos(alpha), -cos(theta)*sin(alpha),  a*sin(theta),
											0         ,  sin(alpha)           ,   cos(alpha)          ,  d,
											0         , 0                     , 0                     ,  1);

	return Metrix_r;

}
#define DEF_PI (3.1415926F)
void calculate_obj(double *Image_coor,double *Robot_coor)
{
	
	//double LH1 = 140.0*0.001;//m
	//double LH2 = 20.0*0.001;//m
	//double LH3 = 10.0*0.001;//m

	double thetaH = -50.0 / 180 * DEF_PI; //for sewing
	double LH1 = 150.0*0.001;//m
	double LH2 = 40.0*0.001;//m
	double LH3 = 15.0*0.001;//m


	Mat mImage_coor = (Mat_<double>(4, 1) << Image_coor[0],
		Image_coor[1],
		Image_coor[2],
		1);

	
	const int PNUM = 5;
	double ALPHA[PNUM] = {0,0,0.5*DEF_PI,-0.5*DEF_PI,-0.5*DEF_PI };   //DH中的alpha  沿著x轉的角度
	double THETHA_HOME[PNUM] = {0,0,0,0,-0.5*DEF_PI }; //DH中的theta  沿著Z轉的角度
	double d_Axis_j[PNUM] = {LH1,0,0,0,0 }; //軸向距d
	double a_Radia_j[PNUM] = {0,LH2,0,LH3,0}; //徑向距a
	double JointTheta_j[PNUM] = { 0,0,thetaH,0,0 };
	double Theta_j[PNUM] = {0};

	Theta_j[0] = THETHA_HOME[0];
	Theta_j[1] = THETHA_HOME[1] + JointTheta_j[0]; //座標90度旋轉 + 各軸旋轉角度
	Theta_j[2] = THETHA_HOME[2] + JointTheta_j[1];
	Theta_j[3] = THETHA_HOME[3] + JointTheta_j[2];
	Theta_j[4] = THETHA_HOME[4] + JointTheta_j[3];


	Mat T0_1 = DH_HomoTran(d_Axis_j[0], Theta_j[0], a_Radia_j[0], ALPHA[0]);
	Mat T1_2 = DH_HomoTran(d_Axis_j[1], Theta_j[1], a_Radia_j[1], ALPHA[1]);
	Mat T2_3 = DH_HomoTran(d_Axis_j[2], Theta_j[2], a_Radia_j[2], ALPHA[2]);
	Mat T3_4 = DH_HomoTran(d_Axis_j[3], Theta_j[3], a_Radia_j[3], ALPHA[3]);
	Mat T4_5 = DH_HomoTran(d_Axis_j[4], Theta_j[4], a_Radia_j[4], ALPHA[4]);

	Mat T0_2 = T0_1*T1_2;
	Mat T0_3 = T0_2*T2_3;
	Mat T0_4 = T0_3*T3_4;
	Mat T0_5 = T0_4*T4_5;

	Mat mRobot_coor = T0_5*mImage_coor;
	double* pmRobot_coor = mRobot_coor.ptr<double>(0);
	Robot_coor[0] = pmRobot_coor[0];
	Robot_coor[1] = pmRobot_coor[1];
	Robot_coor[2] = pmRobot_coor[2];
}

void TestHomotran()
{
	double Image_coor[3] = { 0 };
	double Robot_coor[3] = { 0 };

	calculate_obj(Image_coor, Robot_coor);


}

static void onMouseDeproject(int event, int x, int y, int f, void*)
{
	Mat image = color_mat.clone();
	double depth = depth_mat.at<double>(y, x);

	struct  rs2_intrinsics intrin = mRsDepthIntrinsic;
	intrin.fx = 600;
	intrin.fy = 600;
	double Camera_coor[3] = { 0 };;
	double pixel[2] = { x,y };
	rs2_deproject_pixel_to_point(Camera_coor, &intrin, pixel, depth);

	double Robot_coor[3] = {0};
	calculate_obj(Camera_coor, Robot_coor);

	//from_point[0] = from_point[0] + 20.0/1000;
	//from_point[1] = from_point[1] + 140.0/1000;
	
	//rs2_extrinsics extrin;
	//double theta_y = 0*3.1415;
	//extrin.rotation[0] = 0;
	//extrin.rotation[1] = -1;
	//extrin.rotation[2] = 0;
	//extrin.rotation[3] = -sin(theta_y);
	//extrin.rotation[4] = 0;
	//extrin.rotation[5] = -cos(theta_y);
	//extrin.rotation[6] = cos(theta_y);
	//extrin.rotation[7] = 0;
	//extrin.rotation[8] = -sin(theta_y);
	//extrin.translation[0] = 0 / 1000;
	//extrin.translation[1] = 0.0 / 1000;
	//extrin.translation[2] = 0;

	//unity
	//extrin.rotation[0] = 1;
	//extrin.rotation[1] = 0;
	//extrin.rotation[2] = 0;
	//extrin.rotation[3] = 0;
	//extrin.rotation[4] = 1;
	//extrin.rotation[5] = 0;
	//extrin.rotation[6] = 0;
	//extrin.rotation[7] = 0;
	//extrin.rotation[8] = 1;

	

	char name[50];
	sprintf_s(name, "Image coor point=[%d,%d]",x, y);
	putText(image, name, Point(100, 40), FONT_HERSHEY_SIMPLEX, .7, Scalar(0, 255, 0), 2, 8, false);

	sprintf_s(name, "Camera coor point=[%2.2f,%2.2f,%2.2f] mm", Camera_coor[0]*1000, Camera_coor[1]*1000, Camera_coor[2]*1000);
	putText(image, name, Point(100, 60), FONT_HERSHEY_SIMPLEX, .7, Scalar(0, 255, 0), 2, 8, false);

	sprintf_s(name, "Robot coor point=[%2.2f,%2.2f,%2.2f] mm", Robot_coor[0] * 1000, Robot_coor[1] * 1000, Robot_coor[2] * 1000);
	putText(image, name, Point(100, 80), FONT_HERSHEY_SIMPLEX, .7, Scalar(0, 255, 0), 2, 8, false);

	//此張影像中心的紅色十字架
	//circle(dst,image_cen,3,Scalar(0,0,200),3);
	Point2f image_cen = Point2f(image.cols*0.5f, image.rows*0.5f);
	int cross_len = 10;
	line(image, Point2f(image_cen.x - cross_len, image_cen.y), Point2f(image_cen.x + cross_len, image_cen.y), Scalar(0, 0, 200), 2);  //crosshair horizontal
	line(image, Point2f(image_cen.x, image_cen.y - cross_len), Point2f(image_cen.x, image_cen.y + cross_len), Scalar(0, 0, 200), 2);  //crosshair horizontal


	//char name[30];
	//sprintf_s(name, "depth=%4.2f mm", depth * 1000);
	//putText(image, name, Point(150, 40), FONT_HERSHEY_SIMPLEX, .7, Scalar(0, 255, 0), 2, 8, false);

	//sprintf_s(name, "X=%d", x);
	//putText(image, name, Point(25, 300), FONT_HERSHEY_SIMPLEX, .7, Scalar(0, 0, 255), 2, 8, false);

	//sprintf_s(name, "Y=%d", y);
	//putText(image, name, Point(25, 340), FONT_HERSHEY_SIMPLEX, .7, Scalar(0, 0, 255), 2, 8, false);

	//imwrite("hsv.jpg",image);
	circle(image, cv::Point(200, 200),16, Scalar(0, 0, 200), 5);//圈出center點

	imshow(rgb_window_name, image);
}

void TestDeproject()
{
	rs2::colorizer color_map;
	namedWindow(depth_window_name, WINDOW_AUTOSIZE);
	namedWindow(rgb_window_name, WINDOW_AUTOSIZE);
	setMouseCallback(rgb_window_name, onMouseDeproject, 0);

	rs2::align align_to(RS2_STREAM_COLOR);

	// Declare RealSense pipeline, encapsulating the actual device and sensors
	rs2::config c;
	c.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
	c.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

	rs2::pipeline pipe;
	// Start streaming with default recommended configuration
	auto PipeProfile = pipe.start(c);
	auto depth_stream = PipeProfile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
	auto color_stream = PipeProfile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
	
	
	

	//rs2_extrinsics e2 = color_stream.get_extrinsics_to(color_stream);
	//auto mRsDepthIntrinsic = depth_stream.get_intrinsics();
	auto mRsColorIntrinsic = color_stream.get_intrinsics();
	mRsDepthIntrinsic = depth_stream.get_intrinsics();
	//rs2::stream_profile selected_profile = how_to::choose_a_streaming_profile(sensor);
	//auto video_stream = stream.as<rs2::video_stream_profile>
	//{
	//	try
	//	{
	//		//If the stream is indeed a video stream, we can now simply call get_intrinsics()
	//		rs2_intrinsics intrinsics = video_stream.get_intrinsics();

	while (cvGetWindowHandle(depth_window_name))
	{
		rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
		rs2::frameset aligned_set = align_to.process(data);

		rs2::frame depth_frame = color_map(aligned_set.get_depth_frame());
		depth_mat = depth_frame_to_meters(pipe, aligned_set.get_depth_frame());

		color_mat = frame_to_mat(aligned_set.get_color_frame());

		// Query frame size (width and height)
		const int w = depth_frame.as<rs2::video_frame>().get_width();
		const int h = depth_frame.as<rs2::video_frame>().get_height();

		// Create OpenCV matrix of size (w,h) from the colorized depth data
		Mat depth_color_mat(Size(w, h), CV_8UC3, (void*)depth_frame.get_data(), Mat::AUTO_STEP);

		Point2f image_cen = Point2f(mRsDepthIntrinsic.ppx, mRsDepthIntrinsic.ppy);
		int cross_len = 10;
		line(depth_color_mat, Point2f(image_cen.x - cross_len, image_cen.y), Point2f(image_cen.x + cross_len, image_cen.y), Scalar(0, 0, 200), 2);  //crosshair horizontal
		line(depth_color_mat, Point2f(image_cen.x, image_cen.y - cross_len), Point2f(image_cen.x, image_cen.y + cross_len), Scalar(0, 0, 200), 2);  //crosshair horizontal

		circle(depth_color_mat, cv::Point(200, 200),16, Scalar(0, 0, 200),5);//圈出center點

		// Update the window with new data
		imshow(depth_window_name, depth_color_mat);

		waitKey(30);
	}



}

int main(int argc, char * argv[]) try
{	
	//TestHomotran();
	TestDeproject();
	//TestGetIntrinsic();
	return 0;

	//Declare depth colorizer for pretty visualization of depth data
	rs2::colorizer color_map;
	rs2::align align_to(RS2_STREAM_COLOR);

	
	// Declare RealSense pipeline, encapsulating the actual device and sensors
	rs2::config c;
	c.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
	c.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);

	rs2::pipeline pipe;
	// Start streaming with default recommended configuration
	pipe.start(c);

	using namespace cv;
	namedWindow(depth_window_name, WINDOW_AUTOSIZE);
	namedWindow(rgb_window_name, WINDOW_AUTOSIZE);
	setMouseCallback(rgb_window_name, onMouse, 0);

	//initial first fram to rgb
	rs2::frameset data = pipe.wait_for_frames(); 
	rs2::frameset aligned_set = align_to.process(data);
	color_mat = frame_to_mat(aligned_set.get_color_frame());
	imshow("rgb", color_mat);

	//while (waitKey(1) < 0 && cvGetWindowHandle(depth_window_name))
	while (cvGetWindowHandle(depth_window_name))
	{
		rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
		rs2::frameset aligned_set = align_to.process(data);

		rs2::frame depth_frame = color_map(aligned_set.get_depth_frame());
		depth_mat = depth_frame_to_meters(pipe, aligned_set.get_depth_frame());
		
		color_mat = frame_to_mat(aligned_set.get_color_frame());

		// Query frame size (width and height)
		const int w = depth_frame.as<rs2::video_frame>().get_width();
		const int h = depth_frame.as<rs2::video_frame>().get_height();

		// Create OpenCV matrix of size (w,h) from the colorized depth data
		Mat depth_color_mat(Size(w, h), CV_8UC3, (void*)depth_frame.get_data(), Mat::AUTO_STEP);

		// Update the window with new data
		imshow(depth_window_name, depth_color_mat);

		waitKey(30);
	}

	return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
	return EXIT_FAILURE;
}
catch (const std::exception& e)
{
	std::cerr << e.what() << std::endl;
	return EXIT_FAILURE;
}

//======================
//==originalfrom github
//======================
//#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
//#include <opencv2/opencv.hpp>   // Include OpenCV API
//
//int main(int argc, char * argv[]) try
//{
//	// Declare depth colorizer for pretty visualization of depth data
//	rs2::colorizer color_map;
//
//	// Declare RealSense pipeline, encapsulating the actual device and sensors
//	rs2::pipeline pipe;
//	// Start streaming with default recommended configuration
//	pipe.start();
//
//	using namespace cv;
//	const auto window_name = "Display Image";
//	namedWindow(window_name, WINDOW_AUTOSIZE);
//
//	while (waitKey(1) < 0 && cvGetWindowHandle(window_name))
//	{
//		rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera
//		rs2::frame depth = color_map(data.get_depth_frame());
//
//		// Query frame size (width and height)
//		const int w = depth.as<rs2::video_frame>().get_width();
//		const int h = depth.as<rs2::video_frame>().get_height();
//
//		// Create OpenCV matrix of size (w,h) from the colorized depth data
//		Mat image(Size(w, h), CV_8UC3, (void*)depth.get_data(), Mat::AUTO_STEP);
//
//		// Update the window with new data
//		imshow(window_name, image);
//	}
//
//	return EXIT_SUCCESS;
//}
//catch (const rs2::error & e)
//{
//	std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
//	return EXIT_FAILURE;
//}
//catch (const std::exception& e)
//{
//	std::cerr << e.what() << std::endl;
//	return EXIT_FAILURE;
//}