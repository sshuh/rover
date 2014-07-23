/*
 * HS2_LaserLMS151.h
 *
 *  Created on: Apr 16, 2014
 *      Author: sshuh
 */

#ifndef HS2_LASERLMS151_H_
#define HS2_LASERLMS151_H_
#endif /* HS2_LASERLMS151_H_ */

#pragma once

#ifdef _WIN32
#include <winsock2.h>
#include <stdio.h>
#include "hs2def.h"
#include "HS2_TcpipComm.h"
#endif

#ifdef _WIN32
#define _AFXDLL		// for HS2_Console
#include <afxsock.h>// for HS2_Console

#include <winsock2.h>
#endif

#ifdef __linux__
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>

//#include "hs2def.h"
#include "HS2_TcpipComm.h"

#define SOCKET	int

#endif

#include <math.h>

#ifndef PI
	#define PI		3.14159265358979
#endif
#ifndef PIf
	#define PIf		3.14159265358979f
#endif
#ifndef D2R
	#define D2R	0.01745329251994327	//(PI/180.)
#endif
#ifndef R2D
	#define R2D	57.2957795130823799	//(180./PI)
#endif
#ifndef D2RF
	#define D2RF	0.01745329251994327f	//(PI/180.)
#endif
#ifndef R2DF
	#define R2DF	57.2957795130823799f	//(180./PI)
#endif

#define LMS151_MODE_ASCII	1
#define LMS151_MODE_BINARY	2
#define LMS151_MODE			LMS151_MODE_BINARY

#define LMS151_NUMBER_OF_DATA		1081	//541

//#define SWITCH_ENDIAN_2BYTE	
// intel PC writes small order of number.  dec 1 --> u_short : 0x0001 --> u_int : 0x00000001
// LMS151 writes big order of number.   dec 1 --> 0x0100 --> u_int : 0x01000000


#if( LMS151_MODE == LMS151_MODE_BINARY )
#pragma pack(push, 1)
typedef struct
{
	char Header[8];
	char Command[16];
	unsigned short VersionNumber;

	unsigned short DeviceNumber;
	unsigned int SerialNumber;
	unsigned char DeviceStatus[2];

	unsigned short TelegrammCounter;	// MessageCounter
	unsigned short ScanCounter;
	unsigned int TimeSinceStartUp;	// PowerUpDuration
	unsigned int TimeOfTransmission;	// TransmissionDuration
	unsigned char StatusOfDigitalInputs[2];	// InputStatus
	unsigned char StatusOfDigitalOutputs[2];	// OutputStatus
	unsigned short ReservedByteA;

	unsigned int ScanFrequency;
	unsigned int MeasurementFrequency;

	unsigned short NumberOfEncoders;	// 0

	unsigned short NumberChannels16Bit;
	char MeasuredDataContents[5];	// DIST1_
	float ScaleFactor;
	float ScaleFactorOffset;
	int StartAngle;
	unsigned short Steps;	// AngularStepWidth
	unsigned short NumberOfData;	// Amount of Data
	unsigned short Data[LMS151_NUMBER_OF_DATA];

	unsigned short NumberChannels8Bit;	// 0
	unsigned short PositionInfo;	// 0
	unsigned short Name;	// 0
	unsigned short Comment;	// 0
	unsigned short TimeInfo;	// 0
	unsigned short EventInfo;	// 0

	unsigned char temp;
} LMS151DataFormat_Single;

#pragma pack(push, 1)
struct LMS151DataFormat_Continuous
{
	char Header[8];
	char Command[16];
	//char temp2[26];

	unsigned short VersionNumber;

	unsigned short DeviceNumber;
	unsigned int SerialNumber;
	unsigned char DeviceStatus[2];

	unsigned short TelegrammCounter;	// MessageCounter
	unsigned short ScanCounter;
	unsigned int TimeSinceStartUp;	// PowerUpDuration
	unsigned int TimeOfTransmission;	// TransmissionDuration
	unsigned char StatusOfDigitalInputs[2];	// InputStatus
	unsigned char StatusOfDigitalOutputs[2];	// OutputStatus
	unsigned short ReservedByteA;

	unsigned int ScanFrequency;
	unsigned int MeasurementFrequency;

	unsigned short NumberOfEncoders;	// 0

	unsigned short NumberChannels16Bit;
	char MeasuredDataContents[5];	// DIST1_
	float ScaleFactor;
	float ScaleFactorOffset;
	int StartAngle;
	unsigned short Steps;	// AngularStepWidth
	unsigned short NumberOfData;	// Amount of Data
	unsigned short Data[LMS151_NUMBER_OF_DATA];

	unsigned short NumberChannels8Bit;	// 0
	unsigned short PositionInfo;	// 0
	unsigned short Name;	// 0
	unsigned short Comment;	// 0
	unsigned short TimeInfo;	// 0
	unsigned short EventInfo;	// 0

	unsigned char temp;
};
#endif


#if( LMS151_MODE == LMS151_MODE_ASCII )

struct LMS151DataFormat_Single_Ascii
{
	char Header;
	char Command[16];
	char VersionNumber[2];

	char DeviceNumber[2];
	char SerialNumber[7];
	char DeviceStatus[4];
/*
	char TelegrammCounter[5];	// MessageCounter
	char ScanCounter[5];
	char TimeSinceStartUp[9];	// PowerUpDuration
	char TimeOfTransmission[9];	// TransmissionDuration
	char StatusOfDigitalInputs[4];	// InputStatus
	char StatusOfDigitalOutputs[4];	// OutputStatus
	char ReservedByteA[2];

	char ScanFrequency[5];
	char MeasurementFrequency[4];

	char NumberOfEncoders[2];	// 0

	char NumberChannels16Bit[2];
	char MeasuredDataContents[6];	// DIST1_
	char ScaleFactor[9];
	char ScaleFactorOffset[9];
	char StartAngle[9];
	char Steps[5];	// AngularStepWidth
	char NumberOfData[4];	// Amount of Data
*/
	char Data[LMS151_NUMBER_OF_DATA*5+13+100];

//	char NumberChannels8Bit[2];	// 0
//	char PositionInfo[2];	// 0
//	char Name[2];	// 0
//	char Comment[2];	// 0
//	char TimeInfo[2];	// 0
//	char EventInfo[2];	// 0

//	char temp[1];
};
#endif


class HS2_LaserLMS151 : HS2_TcpipComm
{
public:

#if( LMS151_MODE == LMS151_MODE_BINARY )
	LMS151DataFormat_Single		LrfDataS;
	LMS151DataFormat_Continuous		LrfDataC;
#endif

#if( LMS151_MODE == LMS151_MODE_ASCII )
	LMS151DataFormat_Single_Ascii	LrfDataC_Ascii;
#endif

	int OpenLaserScanner( const char* cIpAddress, const unsigned short usPort );
	int CloseLaserScanner(void);

	//StructLaserParameters LaserParam1;
	int InitializeLaserParams(void);

	int StartMeasurement();
	int StopMeasurement();
	void QueryStatus();
	//void ReadScanData_Single();
	//void ReadScanData_Continuous();

	int ReadScanData_Single_Binary( unsigned short* LRFdata );
	void ReadScanData_ContinuousStart_Binary();
	void ReadScanData_ContinuousStop_Binary();
	int ReadScanData_Answer_Binary( unsigned short* LRFdata );

	int ReadScanData_Single_Ascii( unsigned short* LRFdata );
};


int HS2_LaserLMS151::OpenLaserScanner( const char* cIpAddress, const unsigned short usPort )
{
	HS2_TcpipComm::SetSocketIPandPort( cIpAddress, usPort );

	int nResult = HS2_TcpipComm::ConnectToTcpServer( m_cIpAddress, m_usPort );
	return nResult;
}


int HS2_LaserLMS151::CloseLaserScanner(void)
{
	return HS2_TcpipComm::CloseTcpConnection_Client();
}


int HS2_LaserLMS151::InitializeLaserParams(void)
{
/*
	strcpy( LaserParam1.Laser_Model, "SICK_LMS151" );
	LaserParam1.Laser_MaxDist = 50000;
	LaserParam1.Laser_TotalSteps = 1081;

	LaserParam1.NumTotalMeas = 1081;	//LaserParam1.Laser_LastStep -LaserParam1.Laser_FirstStep +1;
	LaserParam1.LeftEndStep = 0;	//LaserParam1.Laser_FirstStep;
	LaserParam1.RightEndStep = 1080;	//LaserParam1.Laser_LastStep -LaserParam1.Laser_FirstStep;
	LaserParam1.CenterFrontStep = 540;	//LaserParam1.Laser_FrontStep -LaserParam1.Laser_FirstStep;

	LaserParam1.AngleResolution = 0.25;	// LMS151
	LaserParam1.LeftEndAngle = -135.0;
	LaserParam1.RightEndAngle = 135.0;

	LaserParam1.Left90DegStep = 180;
	LaserParam1.Right90DegStep = 900;

	for(int i=LaserParam1.LeftEndStep; i<=LaserParam1.RightEndStep; i++)
	{
		LaserParam1.LUT_CosScanAngle[i] = (float)( cos(D2R*( LaserParam1.LeftEndAngle+((double)(i-LaserParam1.LeftEndStep)*LaserParam1.AngleResolution) )) );
		LaserParam1.LUT_SinScanAngle[i] = (float)( sin(D2R*( LaserParam1.LeftEndAngle+((double)(i-LaserParam1.LeftEndStep)*LaserParam1.AngleResolution) )) );
	}
*/
	return 1;
}

void SwapOrder(unsigned char *dest, unsigned char *orig, int size)
{
	for (int ii=0;ii<size;ii++)
		dest[ii]=orig[size-ii-1];
}

int HS2_LaserLMS151::ReadScanData_Single_Binary( unsigned short* usData )
{
	char cRequest[30] = {0x02, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x0F,   0x73, 0x52, 0x4E,   0x20,
									0x4C, 0x4D, 0x44, 0x73, 0x63, 0x61, 0x6E, 0x64, 0x61, 0x74, 0x61,   0x05 };

	int nResult = SendDataToServer( cRequest, 24 );
	//printf("(R%d)", nResult );

	nResult = RecvDataFromServer( (char*)(&LrfDataS), sizeof(LMS151DataFormat_Single) );
	//printf("(R%d) ", nResult );

	//for( int i=0; i<541; i++ )
	//	LRFdata[540-i] = (LrfDataS.Data[i]<<8) | (LrfDataS.Data[i]>>8);

	//for( int i=0; i<541; i++ )
	//	usData[1080-i*2] = (LrfDataS.Data[i]<<8) | (LrfDataS.Data[i]>>8);

//	for( int i=0; i<LaserParam1.NumTotalMeas; i++ )
//		usData[LaserParam1.RightEndStep-i] = (LrfDataS.Data[i]<<8) | (LrfDataS.Data[i]>>8);



	nResult = strncmp( (LrfDataS.Command), "sRA LMDscandata", 15 );
	printf("(%d)\n", nResult );
	if( nResult != 0 )
		return 0;

	for( int i=0; i<1081; i++ )
		usData[i] = (LrfDataS.Data[i]<<8) | (LrfDataS.Data[i]>>8);

//	for( int i=0; i<1081; i++ )
//	{
//		SwapOrder( (unsigned char*)&usData[i], (unsigned char*)&LrfDataS.Data[i], 2 );
//	}

//	SwapOrder( (unsigned char*)usData, (unsigned char*)LrfDataS.Data, 2*1081 );

	for( int i=0; i<16; i++ )
		printf("%c", LrfDataS.Command[i] );


	unsigned short usNumofData = (LrfDataS.NumberOfData<<8) | (LrfDataS.NumberOfData>>8);
	unsigned short usTelegrammCounter = (LrfDataS.TelegrammCounter<<8) | (LrfDataS.TelegrammCounter>>8);
	printf("  %d  %d  ", usNumofData, usTelegrammCounter);

//	for( int i=0; i<1081; i++ )
//		usData[i] = (LrfDataS.Data[i]<<8) | (LrfDataS.Data[i]>>8);

	//printf("L %d  C %d  R %d\n", LrfDataS.Data[180], LrfDataS.Data[540], LrfDataS.Data[900] );



	return 1;
}




#if( LMS151_MODE == LMS151_MODE_ASCII )

int HS2_LaserLMS151::ReadScanData_Single_Ascii( unsigned short* LRFdata )
{
	char strRequest[20] = {0x02,    0x73, 0x52, 0x4E,    0x20,    0x4C, 0x4D, 0x44,   0x73, 0x63, 0x61, 0x6E, 0x64, 0x61, 0x74, 0x61,    0x03 };

	int nLen = (int)strlen( strRequest );
//	printf("(%d)", nLen );
	int nResult;
	nResult = send( m_hClntSock, strRequest, 17, 0 );
//	printf("(R%d)", nResult );

	nResult = recv( m_hClntSock, (char*)(&LrfDataC_Ascii), sizeof(LrfDataC_Ascii), 0 );
//	printf("(R%d) ", nResult );

	int dataStart = 0;

	char Compare1[9] = "DIST1";
	for( int i=0; i<100; i++ )
	{
		if( LrfDataC_Ascii.Data[i] == Compare1[0] )
		{
			if( LrfDataC_Ascii.Data[i+1] == Compare1[1] )
			{
				if( LrfDataC_Ascii.Data[i+2] == Compare1[2] )
				{
					if( LrfDataC_Ascii.Data[i+3] == Compare1[3] )
					{
						if( LrfDataC_Ascii.Data[i+4] == Compare1[4] )
						{
							dataStart = i+5;
							break;
						}
					}
				}
			}
		}
	}

	int k=dataStart+25;

	char Compare2[9] = "1388 21D";
	for( int i=0; i<20; i++ )
	{
		if( LrfDataC_Ascii.Data[k+i] == Compare2[0] )
		{
			if( LrfDataC_Ascii.Data[k+i+1] == Compare2[1] )
			{
				if( LrfDataC_Ascii.Data[k+i+2] == Compare2[2] )
				{
					if( LrfDataC_Ascii.Data[k+i+3] == Compare2[3] )
					{
						if( LrfDataC_Ascii.Data[k+i+4] == Compare2[4] )
						{
							if( LrfDataC_Ascii.Data[k+i+5] == Compare2[5] )
							{
								if( LrfDataC_Ascii.Data[k+i+6] == Compare2[6] )
								{
									if( LrfDataC_Ascii.Data[k+i+7] == Compare2[7] )
									{
										dataStart = k+i+9;
										break;
									}
								}
							}
						}
					}
				}
			}
		}
	}

	k=dataStart;
	char Range_ascii[4];
	for( int i=0; i<541; i++ )
	{
		int m = 0;
		while( LrfDataC_Ascii.Data[k] != ' ' )
		{
			Range_ascii[m] = LrfDataC_Ascii.Data[k];
			k++;
			m++;
		}
		k++;

		unsigned short lrfdata = 0;
		for( int j=0; j<m; j++ )
		{
			if( j != 0 )
				lrfdata = lrfdata<<4;

			if( Range_ascii[j] >= 'A' )
				lrfdata += 10 + Range_ascii[j] - 'A';
			else
				lrfdata += Range_ascii[j] - '0';
		}

		//LRFdata[LMS151_NUMBER_OF_DATA-1-i] = lrfdata;
		LRFdata[1080-i*2] = lrfdata;
	}
/*
	for(int i=0; i<541; i++ )
		printf("[%d]%d ", i, LRFdataS[i] );

	printf("\n");
*/
	return 1;
}

#endif


void HS2_LaserLMS151::ReadScanData_ContinuousStart_Binary()
{
	char strRequest[30] = {0x02, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x11,   0x73, 0x45, 0x4E,   0x20,
									0x4C, 0x4D, 0x44, 0x73, 0x63, 0x61, 0x6E, 0x64, 0x61, 0x74, 0x61,   0x20, 0x01,    0x33 };

	int nLen = (int)strlen( strRequest );
	printf("(%d)", nLen );
	int nResult;
	nResult = send( m_hClntSock, strRequest, 26, 0 );
	printf("(S%d)", nResult );

	char strAnswer[30];
	nResult = recv( m_hClntSock, strAnswer, 26, 0 );
	printf("(R%d)", nResult );

	nResult = strncmp( &(strAnswer[1]), "sEA LMCstartmeas 1", 18 );
	printf("(%d)", nResult );

	return;
}

void HS2_LaserLMS151::ReadScanData_ContinuousStop_Binary()
{
	char strRequest[30] = {0x02, 0x02, 0x02, 0x02, 0x00, 0x00, 0x00, 0x11,   0x73, 0x45, 0x4E,   0x20,
									0x4C, 0x4D, 0x44, 0x73, 0x63, 0x61, 0x6E, 0x64, 0x61, 0x74, 0x61,   0x20, 0x00,    0x32 };	//checksum = 0x32???

	int nLen = (int)strlen( strRequest );
	printf("(%d)", nLen );
	int nResult;
	nResult = send( m_hClntSock, strRequest, 26, 0 );
	printf("(S%d)", nResult );

	char strAnswer[30];
	nResult = recv( m_hClntSock, strAnswer, 26, 0 );
	printf("(R%d)", nResult );

	nResult = strncmp( &(strAnswer[1]), "sEA LMCstartmeas 0", 18 );
	printf("(%d)", nResult );

	return;
}

int HS2_LaserLMS151::ReadScanData_Answer_Binary( unsigned short* usData )
{
/*
	int nResult = recv( m_hClntSock, (char*)(&LrfDataC), sizeof(LrfDataC), 0 );
	//printf("(LRF%d) ", nResult );

	for( int i=0; i<541; i++ )
	{
		//LRFdata[540-i] = LrfData.Data[i];
		LRFdata[540-i] = (LrfDataC.Data[i]<<8) | (LrfDataC.Data[i]>>8);
		//LRFdata[540-i] *= 5;
	}
*/
	int nResult = RecvDataFromServer( (char*)(LrfDataC.Header), sizeof(LMS151DataFormat_Continuous) );

//	for( int i=0; i<LaserParam1.NumTotalMeas; i++ )
//		usData[LaserParam1.RightEndStep-i] = (LrfDataS.Data[i]<<8) | (LrfDataS.Data[i]>>8);

	nResult = strncmp( (LrfDataC.Command), "sSN LMDscandata", 15 );
	printf("(%d) ", nResult );

	for( int i=0; i<16; i++ )
		printf("%c", LrfDataC.Command[i] );

	if( nResult != 0 )
	{
		return 0;
	}


	for( int i=0; i<1081; i++ )
		usData[i] = (LrfDataC.Data[i]<<8) | (LrfDataC.Data[i]>>8);

//	SwapOrder( (unsigned char*)usData, (unsigned char*)LrfDataC.Data, 2*1081 );

//	for( int i=0; i<1081; i++ )
//	{
//		SwapOrder( (unsigned char*)&usData[i], (unsigned char*)&LrfDataC.Data[i], 2 );
//	}

//	for( int i=0; i<8; i++ )
//		printf("%02d", LrfDataC.Header[i] );

//	for( int i=0; i<16; i++ )
//		printf("%c", LrfDataC.Command[i] );


	unsigned short usNumofData = (LrfDataC.NumberOfData<<8) | (LrfDataC.NumberOfData>>8);
	unsigned short usTelegrammCounter = (LrfDataC.TelegrammCounter<<8) | (LrfDataC.TelegrammCounter>>8);
	printf("  %d  %d  ", usNumofData, usTelegrammCounter);
	//printf("[%d]\n", usData[540] );
	return 1;
}

int HS2_LaserLMS151::StartMeasurement()
{
	char strRequest[30] = "_sMN LMCstartmeas_";
	strRequest[0] = 0x02;
	strRequest[17] = 0x03;

	int nLen = (int)strlen( strRequest );
	printf("(%d)", nLen );
	int nResult;
	nResult = send( m_hClntSock, strRequest, 18, 0 );
	printf("(R%d)", nResult );

	char strAnswer[30];
	nResult = recv( m_hClntSock, strAnswer, 18, 0 );
	printf("(R%d)", nResult );

	nResult = strncmp( strAnswer+1, "sAN LMCstartmeas 0", 18 );
	printf("(R%d)", nResult );

	if( nResult == 18 )
		return 1;
	else
		return 0;
}

int HS2_LaserLMS151::StopMeasurement()
{
	char strRequest[20] = " sMN LMCstopmeas ";
	strRequest[0] = 0x02;
	strRequest[16] = 0x03;

	int nLen = (int)strlen( strRequest );
	int nResult;
	nResult = send( m_hClntSock, strRequest, nLen, 0 );

	char strAnswer[20];
	nResult = recv( m_hClntSock, strAnswer, 15, 0 );

	if( strcmp( strAnswer+1, "sAN LMCstopmeas 0" ) )
		return 1;
	else
		return 0;
}

void HS2_LaserLMS151::QueryStatus()
{
}


#if( LMS151_MODE == LMS151_MODE_ASCII )

int HS2_LaserLMS151::ReadScanData_Single()
{
	char strRequest[30] = "_sMN LMCstartmeas_";
	strRequest[0] = 0x02;
	strRequest[16] = 0x03;

	int nLen = (int)strlen( strRequest );
	printf("(%d)", nLen );
	int nResult;
	nResult = send( m_hClntSock, strRequest, 24, 0 );
	printf("(R%d)", nResult );

	//char strAnswer[8000];
	//nResult = recv( m_hClntSock, strAnswer, 8000, 0 );
	
	nResult = recv( m_hClntSock, (char*)(&LrfData), sizeof(LrfData), 0 );
	printf("(R%d) ", nResult );

	for( int i=0; i<541; i++ )
	{
		//LRFdata[540-i] = LrfData.Data[i];
		LRFdata[540-i] = (LrfData.Data[i]<<8) | (LrfData.Data[i]>>8);
	}
	//nResult = strncmp( strAnswer+1, "sRA LMDscandata 0", 17 );
	//printf("(R%d)", nResult );
/*
	for(int i=0; i<541; i++ )
		printf("[%d]%d ", i, LRFdata[i] );

	printf("\n");
*/
	//for(int i=0; i<nResult; i++ )
	//	printf("%02x ", strAnswer[i] );
	return 1;
}

/*

int LRF_SickLMS151::ReadScanData_Single()
{
	char strRequest[30] = " sRN LMDscandata ";
	strRequest[0] = 0x02;
	strRequest[16] = 0x03;

	int nLen = (int)strlen( strRequest );
	printf("(%d)", nLen );
	int nResult;
	nResult = send( m_hClntSock, strRequest, 17, 0 );
	printf("(R%d)", nResult );


	char strAnswer[8000];
	nResult = recv( m_hClntSock, strAnswer, 8000, 0 );
	printf("(R%d)", nResult );

	nResult = strncmp( strAnswer+1, "sRA LMDscandata 0", 17 );
	printf("(R%d)", nResult );


	for(int i=0; i<nResult; i++ )
		printf("%c", strAnswer[i] );

	return 1;
}
*/

void HS2_LaserLMS151::ReadScanData_ContinuousStart()
{
	char strRequest[30] = " sEN LMDscandata 1 ";
	strRequest[0] = 0x02;
	strRequest[18] = 0x03;

	int nLen = (int)strlen( strRequest );
	printf("(%d)", nLen );
	int nResult;
	nResult = send( m_hClntSock, strRequest, 19, 0 );
	printf("(R%d)", nResult );

	return;
}

void HS2_LaserLMS151::ReadScanData_ContinuousStop()
{
	char strRequest[30] = " sEN LMDscandata 0 ";
	strRequest[0] = 0x02;
	strRequest[18] = 0x03;

	int nLen = (int)strlen( strRequest );
	printf("(%d)", nLen );
	int nResult;
	nResult = send( m_hClntSock, strRequest, 19, 0 );
	printf("(R%d)", nResult );

	return;
}

int HS2_LaserLMS151::ReadScanData_Answer()
{
	char strAnswer[8000];
	int nResult = recv( m_hClntSock, strAnswer, 8000, 0 );
	printf("(R%d)", nResult );

	nResult = strncmp( strAnswer+1, "sRA LMDscandata 0", 17 );
	printf("(R%d)", nResult );
	if( nResult != 16 )
		printf("Error : ReadScanData_Answer");

	for(int i=0; i<nResult; i++ )
		printf("%c", strAnswer[i] );

	return 1;
}

int HS2_LaserLMS151::StartMeasurement()
{
	char strRequest[30] = "_sMN LMCstartmeas_";
	strRequest[0] = 0x02;
	strRequest[17] = 0x03;

	int nLen = (int)strlen( strRequest );
	printf("(%d)", nLen );
	int nResult;
	nResult = send( m_hClntSock, strRequest, 18, 0 );
	printf("(R%d)", nResult );

	char strAnswer[30];
	nResult = recv( m_hClntSock, strAnswer, 18, 0 );
	printf("(R%d)", nResult );

	nResult = strncmp( strAnswer+1, "sAN LMCstartmeas 0", 18 );
	printf("(R%d)", nResult );

	if( nResult == 18 )
		return 1;
	else
		return 0;
}

int HS2_LaserLMS151::StopMeasurement()
{
	char strRequest[20] = " sMN LMCstopmeas ";
	strRequest[0] = 0x02;
	strRequest[16] = 0x03;

	int nLen = (int)strlen( strRequest );
	int nResult;
	nResult = send( m_hClntSock, strRequest, nLen, 0 );

	char strAnswer[20];
	nResult = recv( m_hClntSock, strAnswer, 15, 0 );

	if( strcmp( strAnswer+1, "sAN LMCstopmeas 0" ) )
		return 1;
	else
		return 0;
}

#endif
