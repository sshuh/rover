/*
 * CamPointgrey.h
 *
 *  Created on: Feb 6, 2014
 *      Author: sshuh
 */

#ifndef CAMPOINTGREY_H_
#define CAMPOINTGREY_H_

// ------------------------------ _WIN32 --------------------------------
#ifdef _WIN32
#pragma once
#include <Windows.h>
#include "OpenCV2.4.3/include/opencv2/opencv.hpp"

// PGR Includes
#include "ptgrey2/FlyCapture2.h"
#include "ptgrey2/FlyCapture2GUI.h"
using namespace FlyCapture2;

#endif

// ------------------------------ __linux__ --------------------------------
#ifdef __linux__
// PGR Includes
#include "flycapture/FlyCapture2.h"
#include "flycapture/FlyCapture2GUI.h"

//#include "opencv2/opencv.hpp"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//using namespace FlyCapture2;

#ifndef Sleep
#define Sleep(x)	usleep(x*1000)
#endif

#endif



//using namespace cv;

class ClassCameraImage
{
public:
	unsigned int m_timestamp;
	ClassCameraImage()
	{
		m_timestamp = 0;
		//imgSrc.create( 480, 640, CV_8UC3 );
		//imgSrc.setTo( 255 );
		//circle( imgSrc, Point(200,200), 50, CV_RGB(255,0,0), 1 );
	}

	int Initiate( int height, int width, int channel )
	{
		imgSrc.create( height, width, CV_MAKETYPE(CV_8U,channel) );
		imgSrc.setTo( 255 );
		cv::circle( imgSrc, cv::Point(200,200), 50, CV_RGB(255,0,0), 1 );
		return 1;
	}

	~ClassCameraImage()
	{
		imgSrc.release();
	}

	cv::Mat imgSrc;

};


class ClassCamPointgrey
{
public:
	//int width;
	//int height;

	cv::Size SizeOfCamImage;

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
	int GrabImageForRosMsg( char *pData );

	// for camera parameter setting
	FlyCapture2::TimeStamp m_stTimeStamp;
	unsigned int m_uiTimestamp;
	unsigned int m_uiTimestampUser;
	int m_iCamSetting;
	void SetCameraParameter();

public:
	cv::Mat m_imgSrcColor;
	cv::Mat m_imgSrcGray;


//	cv::VideoCapture m_capture;
//	cv::VideoWriter m_videoWriter;
//	int m_iVideoWriterOn;

	//cv::Mat m_imgUndistorted;
	//cv::Mat mat_CamIntrinsic;
	//cv::Mat mat_CamDistCoeff;
	//cv::Mat m_imgMap1;
	//cv::Mat m_imgMap2;
};



using namespace FlyCapture2;
using namespace cv;

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
	m_Error = m_Cam.RetrieveBuffer(&m_Image);

	SizeOfCamImage.width = m_Image.GetCols();
	SizeOfCamImage.height = m_Image.GetRows();

	printf("Size of image: %d x %d\n", SizeOfCamImage.width, SizeOfCamImage.height );


	m_imgSrcColor.create( SizeOfCamImage, CV_8UC3 );
	m_imgSrcGray.create( SizeOfCamImage, CV_8UC1 );

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


int ClassCamPointgrey::GrabImageForRosMsg( char *pData )
{
	//SetCameraParameter();

	m_Error = m_Cam.RetrieveBuffer(&m_Image);
	m_Error = m_Image.Convert(PIXEL_FORMAT_BGR, &m_ImageColor);// convert to rgb type
	memcpy( pData, m_ImageColor.GetData(), m_ImageColor.GetDataSize() );

	//m_stTimeStamp = m_Image.GetTimeStamp();
	//m_uiTimestamp = (unsigned int)(m_stTimeStamp.seconds*1000) + (unsigned int)(m_stTimeStamp.microSeconds*0.001);
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
				Sleep(100);
				m_Error = m_Cam.GetProperty( &pCamProperty );
				Sleep(100);

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


#endif /* CAMPOINTGREY_H_ */

