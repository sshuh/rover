//#include <string.h>

#include <ros/ros.h>
#include <std_msgs/String.h>


//#include "flycapture/FlyCapture2.h"
//#include "flycapture/FlyCapture2GUI.h"

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "CamPointgrey.h"



//using namespace std;
//#include "opencv2/opencv.hpp"
using namespace FlyCapture2;
using namespace cv;

class ImageConverter
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;

public:
	cv_bridge::CvImagePtr cvimg_ptr;

	ImageConverter() : it_(nh_)
	{
		//image_sub_ = it_.subscribe("/camera/image_raw", 1, &ImageConverter::imageCb, this);
		image_pub_ = it_.advertise("/image_converter/output_video", 1);
		//cv::namedWindow( "imgConverter" );
		//cvimg_ptr->image.create( 480, 640, CV_8UC3 );
		//cvimg_ptr->encoding = sensor_msgs::image_encodings::BGR8;
	}

	~ImageConverter()
	{
		//cv::destroyWindow( "imgConverter" );
	}

	void PubCvtImage( Mat imgSrc )
	{
		try
		{
			//cvimg_ptr->header = "/image_converter/output_video";

			//cvimg_ptr->image = imgSrc;
			cvimg_ptr->encoding = cv_bridge::getCvType( "bgr8" );
			//cvimg_ptr = cv_bridge::toCvShare( msg, sensor_msgs::image_encodings::BGR8 );
			//printf("%d %d\n", cvimg_ptr->image.rows, cvimg_ptr->image.cols );
		}
		catch( cv_bridge::Exception& e )
		{
			ROS_ERROR("cv_bridge exception: %s", e.what() );
			return;
		}
		//image_pub_.publish( cvimg_ptr->toImageMsg() );
	}


	void imageCb( const sensor_msgs::ImageConstPtr& msg )
	{
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			//cv_ptr->header = "/image_converter/output_video";
			//cv_ptr->encoding = sensor_msgs::image_encodings::BGR8;
			//cv_ptr->image = imgSrc;

			cv_ptr = cv_bridge::toCvCopy( msg, sensor_msgs::image_encodings::BGR8 );
		}
		catch( cv_bridge::Exception& e )
		{
			ROS_ERROR("cv_bridge exception: %s", e.what() );
			return;
		}

		//if( cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60 )
		//  cv::circle( cv_ptr->image, cv::Point(50,50), 10, CV_RGB(255,0,0) );

		//cv::imshow( "imgConverter", cv_ptr->image );
		//cv::waitKey( 3 );
	}

};


#define TRANSPORT_IMAGE_TO	1 // 1, 2


#if( TRANSPORT_IMAGE_TO == 1 )
int main( int argc, char **argv )
{
	printf("ptgrey_cam Image to ROS Image message\n");

	ros::init( argc, argv, "ptgrey_cam" );
	ros::NodeHandle hNode;

	ClassCamPointgrey cam1;
	cam1.InitiateCamera(0);

	image_transport::ImageTransport it( hNode );
	image_transport::Publisher image_pub_ = it.advertise("camera/cv_image", 1);

	// ----- ptgrey_cam Image to ROS Image message: Initiate -----
	sensor_msgs::ImagePtr msg_image = boost::make_shared<sensor_msgs::Image>();

	msg_image->header.frame_id = "ptgrey_cam";
	msg_image->width = cam1.m_Image.GetCols();
	msg_image->height = cam1.m_Image.GetRows();

	msg_image->encoding = sensor_msgs::image_encodings::BGR8;
	msg_image->is_bigendian = false;
	msg_image->step = 3*msg_image->width;

	size_t size_img = 3*msg_image->width*msg_image->height;
	msg_image->data.resize(size_img);
	// -----

	while( ros::ok() )
	{
		// ----- ptgrey_cam Image to ROS Image message: Loop -----
		cam1.GrabImageForRosMsg( (char*)(&msg_image->data[0]) );
		// -----
		msg_image->header.stamp = ros::Time::now();

//		cam1.m_Error = cam1.m_Cam.RetrieveBuffer(&cam1.m_Image);
//		cam1.m_Error = cam1.m_Image.Convert(PIXEL_FORMAT_BGR, &cam1.m_ImageColor);// convert to rgb type
//		memcpy( (char*)(&msg_image->data[0]), cam1.m_ImageColor.GetData(), cam1.m_ImageColor.GetDataSize() );

		image_pub_.publish( msg_image );

		ros::spinOnce();
		usleep(30000);
	}

	//ros::spin();
	printf("terminated.\n");

	return 0;
}
#endif


#if( TRANSPORT_IMAGE_TO == 2 )
int main( int argc, char **argv )
{
	printf("ptgrey_cam Image to OpenCV Mat, and then transport to ROS Image message\n");

	ros::init( argc, argv, "ptgrey_cam" );
	ros::NodeHandle hNode;

	ClassCamPointgrey cam1;

	cam1.InitiateCamera(0);

	int key = 0;

	image_transport::ImageTransport it( hNode );
	image_transport::Publisher image_pub_ = it.advertise("camera/cv_image", 1);

	cv_bridge::CvImagePtr cvimg_ptr( new cv_bridge::CvImage );

	while( key != 27 )
	{
		cam1.GrabImage();

		imshow("imgSrc1", cam1.m_imgSrcColor );

		cvimg_ptr->encoding = sensor_msgs::image_encodings::BGR8;
		cvimg_ptr->image = cam1.m_imgSrcColor;

		sensor_msgs::ImagePtr msg = cvimg_ptr->toImageMsg();
		image_pub_.publish( msg );

		ros::spinOnce();

		key = cv::waitKey( 33 );
	}

	//ros::spin();
	printf("terminated.\n");

	return 0;
}
#endif

/*
int main( int argc, char **argv )
{
	printf("ptgrey_cam Image to OpenCV Mat\n");

	ros::init( argc, argv, "ptgrey_cam" );
	ros::NodeHandle hNode;

	ClassCamPointgrey cam1;

	cam1.InitiateCamera(0);

	int key = 0;

	//ImageConverter ImgCvt;
	//image_transport::ImageTransport it_(&hNode);

	//image_transport::Publisher image_pub_;
	//image_pub_ = it_.advertise("/image_converter/output_video", 1);

	//cv_bridge::CvImagePtr cvimgPtr;

	image_transport::ImageTransport it( hNode );
	image_transport::Publisher image_pub_ = it.advertise("camera/cv_image", 1);

	//sensor_msgs::ImagePtr msg = sensor_msgs::CvBridge::cvToImgMsg( cam1.m_imgSrcColor, "bgr8" );
	cv_bridge::CvImagePtr cvimg_ptr( new cv_bridge::CvImage );

	while( key != 27 )
	{
		cam1.GrabImage();

		imshow("imgSrc1", cam1.m_imgSrcColor );

		//ImgCvt.PubCvtImage( cam1.m_imgSrcColor );
		//cvimgPtr->header = "/image_converter/output_video";
		cvimg_ptr->encoding = sensor_msgs::image_encodings::BGR8;
		cvimg_ptr->image = cam1.m_imgSrcColor;

		sensor_msgs::ImagePtr msg = cvimg_ptr->toImageMsg();
		image_pub_.publish( msg );

		ros::spinOnce();

		key = cv::waitKey( 33 );
	}

	//ros::spin();
	printf("terminated.\n");



	return 0;
}
*/


/*
class ClassCamPointgrey
{
public:
	int width;
	int height;

	// ---- FlyCapture 2.3 ----
	FlyCapture2::BusManager m_BusManager;
	FlyCapture2::PGRGuid m_Guid;
	FlyCapture2::Camera m_Cam;
	FlyCapture2::FC2Config m_Config;
	FlyCapture2::Image m_Image;
	FlyCapture2::Image m_ImageColor;
	FlyCapture2::Error m_Error;

	int InitiateCamera( int iParam );
	void StopCamera();
	int GrabImage();

	// for camera parameter setting
	FlyCapture2::TimeStamp m_stTimeStamp;
	unsigned int m_uiTimestamp;
	unsigned int m_uiTimestampUser;
	int m_iCamSetting;
	void SetCameraParameter();

	cv::Mat m_imgSrcColor;
	cv::Mat m_imgSrcGray;
};


int ClassCamPointgrey::InitiateCamera( int iParam )
{
	m_iCamSetting = iParam;	// 0:None, 1:Fixed, 2:Adaptive and fix
	m_uiTimestampUser = 0;

	FlyCapture2::FC2Version version;
	FlyCapture2::Utilities::GetLibraryVersion( &version );

	printf("Flycapture ver %d.%d.%d.%d\n", version.major, version.minor, version.type, version.build );

	m_Error = m_BusManager.GetCameraFromIndex(0, &m_Guid);
	if( m_Error != PGRERROR_OK )
		return -1;

	m_Error = m_Cam.Connect(&m_Guid);
	if( m_Error != PGRERROR_OK )
		return -1;

	m_Error = m_Cam.StartCapture();

	m_imgSrcColor.create( 960, 1280, CV_8UC3 );
	m_imgSrcGray.create( 960, 1280, CV_8UC1 );
	//m_imgSrcColor.create( 480, 640, CV_8UC3 );
	//m_imgSrcGray.create( 480, 640, CV_8UC1 );

	return 1;
}


int ClassCamPointgrey::GrabImage()	// ---- FlyCapture 2.3 ----
{
	SetCameraParameter();

	m_Error = m_Cam.RetrieveBuffer(&m_Image);
	m_Error = m_Image.Convert(PIXEL_FORMAT_BGR, &m_ImageColor);// convert to rgb type
	memcpy( m_imgSrcColor.data, m_ImageColor.GetData(), m_ImageColor.GetDataSize() );

	m_stTimeStamp = m_Image.GetTimeStamp();
	m_uiTimestamp = (unsigned int)(m_stTimeStamp.seconds*1000) + (unsigned int)(m_stTimeStamp.microSeconds*0.001);
	//printf("%d\n", m_uiTimestamp);
	return 1;
}


void ClassCamPointgrey::StopCamera()	// ---- FlyCapture 2.3 ----
{
	m_Error = m_Cam.StopCapture();
	m_Error = m_Cam.Disconnect();

}


void ClassCamPointgrey::SetCameraParameter()
{
	// m_iCamSetting = 0:None, 1:Fixed, 2:Adaptive fixed, 3:
	switch( m_iCamSetting )
	{
	case 0:
		{

			break;
		}
	case 1:
		{
			if( m_uiTimestampUser == 10 )
			{
				FlyCapture2::Property pCamProperty( SHUTTER );
				m_Cam.GetProperty( &pCamProperty );
				pCamProperty.absValue = 0.06f;
				pCamProperty.autoManualMode = false;
				//pCamProperty.onOff = false;
				//pCamProperty.onePush = false;
				//pCamProperty.absControl = false;
				//pCamProperty.valueA = 1;
				m_Cam.SetProperty( &pCamProperty );
			}

			if( m_uiTimestampUser == 50 )
			{
				FlyCapture2::Property pCamProperty( FRAME_RATE );
				m_Cam.GetProperty( &pCamProperty );
				pCamProperty.absValue = 5.00f;
				pCamProperty.autoManualMode = false;
				pCamProperty.onOff = false;
				//pCamProperty.onePush = false;
				pCamProperty.absControl = true;
				//pCamProperty.valueA = 1;
				m_Cam.SetProperty( &pCamProperty );
			}

			if( m_uiTimestampUser++ == 100 )
			{

				FlyCapture2::Property pCamProperty( SHUTTER );
				m_Error = m_Cam.GetProperty( &pCamProperty );
				pCamProperty.absValue = 0.06f;
				pCamProperty.autoManualMode = false;
				//pCamProperty.onOff = false;
				//pCamProperty.onePush = false;
				//pCamProperty.absControl = false;
				//pCamProperty.valueA = 1;
				m_Error = m_Cam.SetProperty( &pCamProperty );
				usleep(100);
				m_Error = m_Cam.GetProperty( &pCamProperty );
				usleep(100);

				//pCamProperty.absValue = 0.096f;
				//pCamProperty.autoManualMode = false;
				//pCamProperty.onOff = false;

				//m_Cam.SetProperty( &pCamProperty );
				//Sleep(100);
				//m_Cam.GetProperty( &pCamProperty );
				printf("Cam setting: Fixed(shutter=%.3f)\n", pCamProperty.absValue );
				m_iCamSetting = 0;
			}
			break;
		}
	case 2:
		{
			if( m_uiTimestampUser == 10 )
			{
				FlyCapture2::Property pCamProperty( SHUTTER );
				m_Cam.GetProperty( &pCamProperty );
				pCamProperty.autoManualMode = true;
				m_Cam.SetProperty( &pCamProperty );
			}

			if( m_uiTimestampUser++ == 100 )
			{
				FlyCapture2::Property pCamProperty( SHUTTER );
				m_Cam.GetProperty( &pCamProperty );
				pCamProperty.autoManualMode = false;
				m_Cam.SetProperty( &pCamProperty );
				m_Cam.GetProperty( &pCamProperty );
				printf("Cam setting: Adaptive fixed(shutter=%.3f)\n", pCamProperty.absValue );
				m_iCamSetting = 0;
			}

			break;
		}
	}

}

*/
