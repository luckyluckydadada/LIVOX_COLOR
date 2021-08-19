

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud2.h>
#include <time.h>

#define Hmax 2048
#define Wmax 3072

#define FIT_LIDAR_CUT_IMAGE False

// 变更相机和雷达的位置关系后要从新修改下面的值
#if FIT_LIDAR_CUT_IMAGE
	#define FIT_min_x 420
	#define FIT_min_y 70
	#define FIT_max_x 2450
	#define FIT_max_y 2000
	#define H  (FIT_max_y - FIT_min_y)
	#define W  (FIT_max_x - FIT_min_x)
#else
	#define H Hmax
	#define W Wmax
#endif 

#define MAX_MIN_RESOLUTION false

/* 自定义的PointXYZRGBIL（pcl没有PointXYZRGBIL、PointXYZRGBI结构体）*/
// struct PointXYZRGBIL
// {
// 	PCL_ADD_POINT4D;
// 	PCL_ADD_RGB;
// 	uint32_t label;
// 	PCL_ADD_INTENSITY;
// 	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
// } EIGEN_ALIGN16;
// POINT_CLOUD_REGISTER_POINT_STRUCT(
// 	PointXYZRGBIL,
// 	(float, x, x)(float, y, y)(float, z, z)(float, rgb, rgb)(uint32_t, label, label)(float, intensity, intensity))
// typedef PointXYZRGBIL PointType;

// 使用pcl::PointXYZRGBNormal结构体代替自定义的PointXYZRGBI，为了livox_color_mapping建图，自定义结构体不支持很多内置函数
typedef pcl::PointXYZRGBNormal PointType;

//全局变量都能访问，图像回调中写，点云回调中读
cv::Vec3b image_color[H][W];

// 内外参
cv::Mat distCoeffs(5, 1, cv::DataType<double>::type);	   // 畸变向量
cv::Mat intrisic = cv::Mat::eye(3, 3, CV_64F);			   // 内参3*3矩阵
cv::Mat intrisicMat(3, 4, cv::DataType<double>::type);	   // 内参3*4的投影矩阵，最后一列是三个零
cv::Mat extrinsicMat_RT(4, 4, cv::DataType<double>::type); // 外参旋转矩阵3*3和平移向量3*1
void CalibrationData(void)
{
	// extrinsic
	// 0.0451423  -0.998715  0.0230348  0.00925535
	// 0.0558064  -0.0205011  -0.998231  0.0499455
	// 0.997421  0.046348  0.0548092  0.42788
	// 0  0  0  1
	extrinsicMat_RT.at<double>(0, 0) = 0.0715019;
	extrinsicMat_RT.at<double>(0, 1) = -0.997146;
	extrinsicMat_RT.at<double>(0, 2) =0.024241;
	extrinsicMat_RT.at<double>(0, 3) =  0.168924;
	extrinsicMat_RT.at<double>(1, 0) = 0.0598713;
	extrinsicMat_RT.at<double>(1, 1) = -0.0199688;
	extrinsicMat_RT.at<double>(1, 2) = -0.998006;
	extrinsicMat_RT.at<double>(1, 3) = -0.119454;
	extrinsicMat_RT.at<double>(2, 0) = 0.995642;
	extrinsicMat_RT.at<double>(2, 1) = 0.0728107;
	extrinsicMat_RT.at<double>(2, 2) = 0.0582726;
	extrinsicMat_RT.at<double>(2, 3) = 0.400851;
	extrinsicMat_RT.at<double>(3, 0) = 0.0;
	extrinsicMat_RT.at<double>(3, 1) = 0.0;
	extrinsicMat_RT.at<double>(3, 2) = 0.0;
	extrinsicMat_RT.at<double>(3, 3) = 1.0;
	
	// intrinsic
	// 2875.097131590431  0  1369.668059923329;
	//  0  2896.420251825658  1114.244269170673;
	//  0  0  1
	// ditortion
	// -0.008326874784366894 -0.06967846599874981 0.006185220615585947 -0.01133018681519818  0.5462976722456516
	intrisicMat.at<double>(0, 0) = intrisic.at<double>(0, 0) = 2875.097131590431;
	intrisicMat.at<double>(0, 1) = 0.000000e+00;
	intrisicMat.at<double>(0, 2) = intrisic.at<double>(0, 2) = 1369.668059923329;
	intrisicMat.at<double>(0, 3) = 0.000000e+00;
	intrisicMat.at<double>(1, 0) = 0.000000e+00;
	intrisicMat.at<double>(1, 1) = intrisic.at<double>(1, 1) = 2896.420251825658;
	intrisicMat.at<double>(1, 2) = intrisic.at<double>(1, 2) = 1114.244269170673;
	intrisicMat.at<double>(1, 3) = 0.000000e+00;
	intrisicMat.at<double>(2, 0) = 0.000000e+00;
	intrisicMat.at<double>(2, 1) = 0.000000e+00;
	intrisicMat.at<double>(2, 2) = 1.000000e+00;
	intrisicMat.at<double>(2, 3) = 0.000000e+00;
	distCoeffs.at<double>(0) = -0.008326874784366894;
	distCoeffs.at<double>(1) = -0.06967846599874981;
	distCoeffs.at<double>(2) = 0.006185220615585947;
	distCoeffs.at<double>(3) = -0.01133018681519818;
	distCoeffs.at<double>(4) = 0.5462976722456516;
}

class livox_lidar_color
{
public:
	ros::NodeHandle n;
	sensor_msgs::PointCloud2 msg;																									   //接收到的点云消息
	sensor_msgs::PointCloud2 fusion_msg;																							   //等待发送的点云消息
	ros::Subscriber subCloud = n.subscribe<sensor_msgs::PointCloud2>("/livox/lidar", 1, &livox_lidar_color::pointCloudCallback, this); //接收点云数据，进入回调函数pointCloudCallback
	ros::Publisher pubCloud = n.advertise<sensor_msgs::PointCloud2>("/livox/color_lidar", 1);										   //建立了一个发布器，方便之后发布加入颜色之后的点云

private:
	//点云回调函数
	void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg)
	{
// clock_t start,ends;
// start=clock();
		pcl::PointCloud<pcl::PointXYZI>::Ptr raw_pcl_ptr(new pcl::PointCloud<pcl::PointXYZI>); //livox点云消息包含xyz和intensity
		pcl::fromROSMsg(*laserCloudMsg, *raw_pcl_ptr);										   //把msg消息指针转化为PCL点云
		cv::Mat X(4, 1, cv::DataType<double>::type);
		cv::Mat Y(3, 1, cv::DataType<double>::type);

		pcl::PointCloud<PointType>::Ptr fusion_pcl_ptr(new pcl::PointCloud<PointType>); //放在这里是因为，每次都需要重新初始化
#if MAX_MIN_RESOLUTION		
		cv::Point pt_min,pt_max; // 统计点云在画面中的最大像素和最小像素
		pt_min.x=Wmax;pt_max.x=0; // 赋初值
		pt_min.y=Hmax;pt_max.x=0; // 赋初值
#endif

		for (int i = 0; i < raw_pcl_ptr->points.size(); i++)
		{
			X.at<double>(0, 0) = raw_pcl_ptr->points[i].x;
			X.at<double>(1, 0) = raw_pcl_ptr->points[i].y;
			X.at<double>(2, 0) = raw_pcl_ptr->points[i].z;
			X.at<double>(3, 0) = 1;
			Y = intrisicMat * extrinsicMat_RT * X; //雷达坐标转换到相机坐标，相机坐标投影到像素坐标
			cv::Point pt;						   // (x,y) 像素坐标
			// Y是3*1向量，pt.x是Y的第一个值除以第三个值，pt.y是Y的第二个值除以第三个值，为什么是下面这种写法？？
			pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0);
			pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0);
			// std::cout<<Y<<pt<<std::endl;
#if FIT_LIDAR_CUT_IMAGE
// 经过外参计算的点云投影到相机上的像素位置是2078x3072为标准，相机topic照片我们剪裁了，所以要加上偏移
		if (pt.x >= FIT_min_x && pt.x < FIT_max_x && pt.y >= FIT_min_y && pt.y < FIT_max_y && raw_pcl_ptr->points[i].x > 0)
#else
		if (pt.x >= 0 && pt.x < W && pt.y >= 0 && pt.y < H && raw_pcl_ptr->points[i].x > 0) //&& raw_pcl_ptr->points[i].x>0去掉图像后方的点云
#endif 
			{
#if MAX_MIN_RESOLUTION
				if (pt.x < pt_min.x) {
					pt_min.x = pt.x; 
					}
				if (pt.y < pt_min.y) {
					pt_min.y = pt.y; 
					}
				if (pt.x > pt_max.x) {
					pt_max.x = pt.x; 
					}
				if (pt.y > pt_max.y) {
					pt_max.y = pt.y; 
					}
#endif
				PointType p;
				p.x = raw_pcl_ptr->points[i].x;
				p.y = raw_pcl_ptr->points[i].y;
				p.z = raw_pcl_ptr->points[i].z;
				//点云颜色由图像上对应点确定
#if FIT_LIDAR_CUT_IMAGE
				p.b = image_color[pt.y-FIT_min_y][pt.x-FIT_min_x][0];
				p.g = image_color[pt.y-FIT_min_y][pt.x-FIT_min_x][1];
				p.r = image_color[pt.y-FIT_min_y][pt.x-FIT_min_x][2];
#else
				p.b = image_color[pt.y][pt.x][0];
				p.g = image_color[pt.y][pt.x][1];
				p.r = image_color[pt.y][pt.x][2];
#endif 

				fusion_pcl_ptr->points.push_back(p);
			}
		}
#if MAX_MIN_RESOLUTION
		std::cout<< "pt_min.x ="<<pt_min.x<<"  "<< "pt_min.y ="<<pt_min.y<<"  "<< "pt_max.x ="<<pt_max.x<<"  "<< "pt_max.y ="<<pt_max.y<<std::endl;
#endif
		fusion_pcl_ptr->width = fusion_pcl_ptr->points.size();
		fusion_pcl_ptr->height = 1;
		// std::cout<<  fusion_pcl_ptr->points.size() << std::endl;
		pcl::toROSMsg(*fusion_pcl_ptr, fusion_msg);			   //将点云转化为消息才能发布
		fusion_msg.header.frame_id = "livox_frame";			   //帧id改成和/livox/lidar一样的，同一坐标系
		fusion_msg.header.stamp = laserCloudMsg->header.stamp; // 时间戳和/livox/lidar 一致
		// std::cout<<fusion_msg;
		pubCloud.publish(fusion_msg);						   //发布调整之后的点云数据
// ends=clock();
// std::cout<<"fusion call:"<<(double (ends-start)/CLOCKS_PER_SEC)<<std::endl;
	}
};

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
// clock_t start,ends;
// start=clock();	
	try
	{
		cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image; //image_raw就是我们得到的图像了
		int rows = image.rows;//std::cout<<"rows="<<rows<<std::endl; 
		int cols = image.cols;//std::cout<<"cols="<<cols<<std::endl; 
		// 去畸变，可选		
		// cv::Mat map1, map2;
		// cv::Size imageSize = image.size();
		// cv::initUndistortRectifyMap(intrisic, distCoeffs, cv::Mat(), cv::getOptimalNewCameraMatrix(intrisic, distCoeffs, imageSize, 1, imageSize, 0), imageSize, CV_16SC2, map1, map2);
		// cv::remap(image, image, map1, map2, cv::INTER_LINEAR); // correct the distortion
		// cv::imwrite("1.bmp",image);
// ends=clock();
// std::cout<<"img call1:"<<(double (ends-start)/CLOCKS_PER_SEC)<<std::endl;
		for (int row = 0; row < rows; row++)
		{
			for (int col = 0; col < cols; col++)
			{
				image_color[row][col] = (cv::Vec3b)image.at<cv::Vec3b>(row, col);
			}
		}


	}
	catch (cv_bridge::Exception &e)
	{
		ROS_ERROR("Could not conveextrinsicMat_RT from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
// ends=clock();
// std::cout<<"img call:"<<(double (ends-start)/CLOCKS_PER_SEC)<<std::endl;
}

int main(int argc, char **argv)
{
	CalibrationData();
	ros::init(argc, argv, "livox_color");
	livox_lidar_color llc;

	ros::NodeHandle n;
	image_transport::ImageTransport it(n);
	image_transport::Subscriber sub = it.subscribe("/hikrobot_camera/rgb", 1, &imageCallback);
	ros::spin();
}
