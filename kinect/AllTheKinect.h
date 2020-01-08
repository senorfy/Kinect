#pragma once
#include "AllFile.h"
class AllTheKinect
{
public:
	AllTheKinect();
	bool GetAndShowDepthData();
	bool GetAndShowColorData();
	bool GetAndShowBoyIndexData();
	bool GetAndShowBoyData();
	bool GetAndDealHandImg();
	~AllTheKinect();
private:
	IKinectSensor*  m_pKinectSensor;  // 获取Kinect设备
	IDepthFrameSource*  pDepthFrameSource;//获得深度信息传感器
	IDepthFrameReader*  m_pDepthFrameReader;//打开深度信息帧读取器
	IFrameDescription*  depthFrameDescription;
	int nDepthWidth;
	int nDepthHeight;

	IColorFrameSource* pColorFrameSource;//获得彩色信息传感器
	IColorFrameReader* m_pColorFrameReader;//打开彩色信息帧读取器  	
	IFrameDescription* colorFrameDescription;
	int nColorWidth;
	int nColorHeight;

	IBodyIndexFrameSource* pBodyIndexFrameSource;//获得身体信息传感器
	IBodyIndexFrameReader* pBodyIndexFrameReader;//打开身体信息帧读取器

	IBodyFrameSource* pBodyFrameSource;//获得骨骼信息传感器
	IBodyFrameReader* pBodyFrameReader;//打开骨骼信息帧读取器

	ICoordinateMapper* coordinateMapper;//坐标数据转换指针

	HRESULT hr;
	Joint aJoints[JointType_Count];

	Mat MatDepth8;
	Mat MatDepth16;
	Mat MatRGB;      //注意：这里必须为4通道的图，Kinect的数据只能以Bgra格式传出
	Mat MatDepthToColor1;
};

extern Point ppp;
extern bool flag;

