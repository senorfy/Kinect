#include "AllTheKinect.h"
#include <vector>
#include "kinect_deal.h"

AllTheKinect::AllTheKinect() :
	m_pKinectSensor(NULL),
	pDepthFrameSource(NULL),
	m_pDepthFrameReader(NULL),
	depthFrameDescription(NULL),
	nDepthWidth(0),
	nDepthHeight(0),
	pColorFrameSource(NULL),
	m_pColorFrameReader(NULL),
	colorFrameDescription(NULL),
	nColorWidth(0),
	nColorHeight(0),
	pBodyIndexFrameSource(NULL),
	pBodyIndexFrameReader(NULL),
	pBodyFrameSource(NULL),
	pBodyFrameReader(NULL),
	coordinateMapper(NULL)
{
	MatRGB.create(1080, 1920, CV_8UC4);
	MatDepth8.create(424, 512, CV_8UC1);
	MatDepth16.create(424, 512, CV_16UC1);
	MatDepthToColor1.create(424, 512, CV_8UC4);
	hr = GetDefaultKinectSensor(&m_pKinectSensor);
	while (NULL == coordinateMapper)
		m_pKinectSensor->get_CoordinateMapper(&coordinateMapper);
	if (m_pKinectSensor) {
		hr = m_pKinectSensor->Open();
	}
	else {
		cout << "获取Kinect设备失败" << endl;
		return ;
	}
	if (SUCCEEDED(hr)) {
		//获得深度信息传感器
		hr = m_pKinectSensor->get_DepthFrameSource(&pDepthFrameSource);
		//打开深度信息帧读取器
		hr = pDepthFrameSource->OpenReader(&m_pDepthFrameReader);
	}
	if (SUCCEEDED(hr)) {
		//获得彩色信息传感器  
		hr = m_pKinectSensor->get_ColorFrameSource(&pColorFrameSource);
		//打开彩色信息帧读取器  
		hr = pColorFrameSource->OpenReader(&m_pColorFrameReader);
	}
	if (SUCCEEDED(hr)) {
		//获得身体信息传感器
		hr = m_pKinectSensor->get_BodyIndexFrameSource(&pBodyIndexFrameSource);
		//打开身体信息帧读取器
		hr = pBodyIndexFrameSource->OpenReader(&pBodyIndexFrameReader);
	}
	if (SUCCEEDED(hr)) {
		//获得骨骼信息传感器
		hr = m_pKinectSensor->get_BodyFrameSource(&pBodyFrameSource);
		//打开骨骼信息帧读取器
		hr = pBodyFrameSource->OpenReader(&pBodyFrameReader);
	}
	
}

bool AllTheKinect::GetAndShowDepthData()
{
	IDepthFrame* m_pDepthFrame = NULL;//深度信息数据
	while (m_pDepthFrame == NULL)
		hr = m_pDepthFrameReader->AcquireLatestFrame(&m_pDepthFrame);//读取深度数据
	m_pDepthFrame->get_FrameDescription(&depthFrameDescription);
	depthFrameDescription->get_Height(&nDepthHeight);
	depthFrameDescription->get_Width(&nDepthWidth);
	if (SUCCEEDED(hr))
		hr = m_pDepthFrame->CopyFrameDataToArray(nDepthHeight * nDepthWidth, reinterpret_cast<UINT16*>(MatDepth16.data));
	//depth变成8位数据——>>显示明显
	UINT16* depthData = new UINT16[512 * 424];
	hr = m_pDepthFrame->CopyFrameDataToArray(nDepthHeight * nDepthWidth, depthData);
	for (int i = 0; i < nDepthHeight * nDepthWidth; i++) {
		BYTE intensity = static_cast<BYTE>(depthData[i] / 256);
		reinterpret_cast<BYTE*>(MatDepth8.data)[i] = intensity;
	}
	equalizeHist(MatDepth8, MatDepth8);
	
	imshow("MatDepth8", MatDepth8);
	imshow("MatDepth16", MatDepth16);
	m_pDepthFrame->Release();
	if (waitKey(1) == VK_ESCAPE)
		return false;
	delete[] depthData;
	return true;
}

bool AllTheKinect::GetAndShowColorData()
{
	
	IColorFrame* m_pColorFrame = NULL;//彩色信息数据
	while (m_pColorFrame == NULL)
		hr = m_pColorFrameReader->AcquireLatestFrame(&m_pColorFrame);//读取彩色数据
	m_pColorFrame->get_FrameDescription(&colorFrameDescription);
	colorFrameDescription->get_Height(&nColorHeight);
	colorFrameDescription->get_Width(&nColorWidth);
	UINT nColorBufferSize = nColorHeight * nColorWidth * 4;
	if (SUCCEEDED(hr))
		hr = m_pColorFrame->CopyConvertedFrameDataToArray(nColorBufferSize, reinterpret_cast<BYTE*>(MatRGB.data), ColorImageFormat::ColorImageFormat_Bgra);
	Mat MatRGB_resize = MatRGB.clone();       // 缩小方便看
	cv::resize(MatRGB_resize, MatRGB_resize, Size(960, 540));
	//putText(MatRGB_resize, "cdn", Point(100, 100), CV_FONT_HERSHEY_COMPLEX, 1, Scalar(0, 0, 0), 5, 8);
	imshow("MatRGB_resize", MatRGB_resize);
	//Mat channel[4];
	//split(MatRGB_resize, channel);
	setMouseCallback("MatRGB_resize", onMouse, 0);
	if (flag) {
		flag = false;
		cout << MatRGB_resize.at<Vec4b>(ppp) << endl;
		//printf("%d", channel[1].at<uchar>(ppp));
		//cout << channel[0].at<uchar>(ppp)<< endl;
	}
	//cvtColor(MatRGB_resize, MatRGB_resize, COLOR_RGB2YCrCb);
	//imshow("B", channel[0]);
	//imshow("G", channel[1]);
	//imshow("R", channel[2]);
	//putText(MatRGB_resize, "The result is : 1", Point(50, 50), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 3, 8);
	imshow("MatRGB_resize", MatRGB_resize);
	//if (waitKey(27) == VK_SPACE)
	//	imwrite("E:/abc.bmp", MatRGB);
	m_pColorFrame->Release();
	return true;
	
}

bool AllTheKinect::GetAndShowBoyIndexData()
{
	Mat MatDepthToColor(424, 512, CV_8UC4, Scalar::all(0));//保存深度映射到彩色
	
	IBodyIndexFrame* m_pBodyIndexFrame = NULL;//身体信息数据
	while (m_pBodyIndexFrame == NULL) 
		hr = pBodyIndexFrameReader->AcquireLatestFrame(&m_pBodyIndexFrame);//读取身体数据
	UINT nBodyIndexBufferSize = 0;
	BYTE* pBodyIndexBuffer = NULL;
	m_pBodyIndexFrame->AccessUnderlyingBuffer(&nBodyIndexBufferSize, &pBodyIndexBuffer);
	
	//ColorSpacePoint* colorSpacePoint = new ColorSpacePoint[512 * 424];
	vector<ColorSpacePoint> colorSpacePoint(512 * 424);
	hr = coordinateMapper->MapDepthFrameToColorSpace(nDepthHeight * nDepthWidth, reinterpret_cast<UINT16*>(MatDepth16.data), nDepthHeight * nDepthWidth, &colorSpacePoint[0]);
	if (SUCCEEDED(hr)) {
		//cout << nBodyIndexBufferSize << ":" << ":" << endl;
		for (int i = 0; i < nDepthHeight; i++)
		{
			for (int j = 0; j < nDepthWidth; j++)
			{
				unsigned int index = i * nDepthWidth + j;
				ColorSpacePoint csp = colorSpacePoint[index];
				int colorX = static_cast<int>(floor(csp.X + 0.5));
				int colorY = static_cast<int>(floor(csp.Y + 0.5));
				//选取落在彩色图像上的点并对前景背景进行阈值分割,再次更改阈值大小
				if (colorX >= 0 && colorX < nColorWidth && colorY >= 0 && colorY < nColorHeight /*&& *depthData < 650*/)
				{
					BYTE player = pBodyIndexBuffer[index];
					if (player != 0xff)//*depthData < 650
					//拷贝彩色信息
						MatDepthToColor.at<cv::Vec4b>(i, j) = MatRGB.at<cv::Vec4b>(colorY, colorX);
					//if (*depthData == 0)  iDepthToColor.at<cv::Vec4b>(i, j) = 0;
				}
			}

		}
	}
	MatDepthToColor1 = MatDepthToColor.clone();
	m_pBodyIndexFrame->Release();
	//imshow("MatDepthToColor", MatDepthToColor1);
	//if (waitKey(1) == VK_SPACE)
		//imwrite("E:/abc1.bmp", MatDepthToColor);
	return true;
}

bool AllTheKinect::GetAndShowBoyData()
{
	IBodyFrame* m_pBodyFrame = NULL;
	while (m_pBodyFrame == NULL)
		hr = pBodyFrameReader->AcquireLatestFrame(&m_pBodyFrame);//读取身体数据
	IBody* ppBodies[BODY_COUNT] = { 0 };
	m_pBodyFrame->GetAndRefreshBodyData(_countof(ppBodies), ppBodies);
	Mat MatBoy(nColorHeight, nColorWidth, CV_8UC4);
	for (int i = 0; i < BODY_COUNT; ++i) {
		IBody* pBody = ppBodies[i];
		if (pBody)
		{
			BOOLEAN bTracked = false;
			hr = pBody->get_IsTracked(&bTracked);

			if (SUCCEEDED(hr) && bTracked)
			{

				hr = pBody->GetJoints(_countof(aJoints), aJoints);

				DrawLine(MatBoy, aJoints[JointType_SpineBase], aJoints[JointType_SpineMid], coordinateMapper);
				DrawLine(MatBoy, aJoints[JointType_SpineMid], aJoints[JointType_SpineShoulder], coordinateMapper);
				DrawLine(MatBoy, aJoints[JointType_SpineShoulder], aJoints[JointType_Neck], coordinateMapper);
				DrawLine(MatBoy, aJoints[JointType_Neck], aJoints[JointType_Head], coordinateMapper);

				DrawLine(MatBoy, aJoints[JointType_SpineShoulder], aJoints[JointType_ShoulderLeft], coordinateMapper);
				DrawLine(MatBoy, aJoints[JointType_ShoulderLeft], aJoints[JointType_ElbowLeft], coordinateMapper);
				DrawLine(MatBoy, aJoints[JointType_ElbowLeft], aJoints[JointType_WristLeft], coordinateMapper);
				DrawLine(MatBoy, aJoints[JointType_WristLeft], aJoints[JointType_HandLeft], coordinateMapper);
				DrawLine(MatBoy, aJoints[JointType_HandLeft], aJoints[JointType_HandTipLeft], coordinateMapper);
				DrawLine(MatBoy, aJoints[JointType_HandLeft], aJoints[JointType_ThumbLeft], coordinateMapper);

				DrawLine(MatBoy, aJoints[JointType_SpineShoulder], aJoints[JointType_ShoulderRight], coordinateMapper);
				DrawLine(MatBoy, aJoints[JointType_ShoulderRight], aJoints[JointType_ElbowRight], coordinateMapper);
				DrawLine(MatBoy, aJoints[JointType_ElbowRight], aJoints[JointType_WristRight], coordinateMapper);
				DrawLine(MatBoy, aJoints[JointType_WristRight], aJoints[JointType_HandRight], coordinateMapper);
				DrawLine(MatBoy, aJoints[JointType_HandRight], aJoints[JointType_HandTipRight], coordinateMapper);
				DrawLine(MatBoy, aJoints[JointType_HandRight], aJoints[JointType_ThumbRight], coordinateMapper);

				DrawLine(MatBoy, aJoints[JointType_SpineBase], aJoints[JointType_HipLeft], coordinateMapper);
				DrawLine(MatBoy, aJoints[JointType_HipLeft], aJoints[JointType_KneeLeft], coordinateMapper);
				DrawLine(MatBoy, aJoints[JointType_KneeLeft], aJoints[JointType_AnkleLeft], coordinateMapper);
				DrawLine(MatBoy, aJoints[JointType_AnkleLeft], aJoints[JointType_FootLeft], coordinateMapper);

				DrawLine(MatBoy, aJoints[JointType_SpineBase], aJoints[JointType_HipRight], coordinateMapper);
				DrawLine(MatBoy, aJoints[JointType_HipRight], aJoints[JointType_KneeRight], coordinateMapper);
				DrawLine(MatBoy, aJoints[JointType_KneeRight], aJoints[JointType_AnkleRight], coordinateMapper);
				DrawLine(MatBoy, aJoints[JointType_AnkleRight], aJoints[JointType_FootRight], coordinateMapper);
			}
		}
	}
	//imshow("MatBoy", MatBoy);
	m_pBodyFrame->Release();
	return true;
}

bool AllTheKinect::GetAndDealHandImg()
{
	DepthSpacePoint LeftHandPoint , RightHandPoint ;
	coordinateMapper->MapCameraPointToDepthSpace(aJoints[JointType_WristLeft].Position, &LeftHandPoint);
	coordinateMapper->MapCameraPointToDepthSpace(aJoints[JointType_HandTipLeft].Position, &RightHandPoint);
	//circle(MatDepthToColor1, Point(LeftHandPoint.X, LeftHandPoint.Y), 4, Vec3b(0, 0, 255), 5);
	//circle(MatDepthToColor1, Point(RightHandPoint.X, RightHandPoint.Y), 4, Vec3b(0, 0, 255), 5);
	if (RightHandPoint.X < 70) RightHandPoint.X = 70;
	else if (RightHandPoint.X > 442) RightHandPoint.X = 442;
	if (RightHandPoint.Y < 70) RightHandPoint.Y = 70;
	else if(RightHandPoint.Y > 354) RightHandPoint.Y = 354;
	Rect abv(RightHandPoint.X-70, RightHandPoint.Y-70, 140, 140);
	rectangle(MatDepthToColor1, Point(RightHandPoint.X - 70, RightHandPoint.Y - 70), Point(RightHandPoint.X + 70, RightHandPoint.Y + 70), Scalar(0, 0, 255), 1, 1, 0);
	
	Mat MatHand1= MatDepthToColor1(abv);
	Mat MatHand = MatHand1.clone();
	if (waitKey(27) == VK_SPACE)
		imwrite("E:/abvoova.bmp", MatDepthToColor1);
	Mat ycrcb_img;
	cvtColor(MatHand, ycrcb_img, COLOR_RGB2YCrCb);
	Mat output_mask = Mat::zeros(MatHand.size(), CV_8UC1);
	//resize(MatHand, MatHand, Size(64, 64));
	vector<Mat> channel;
	split(MatHand, channel);
	//Mat aa = MatHand;
	//Mat output_mask = Mat::zeros(MatHand.size(), CV_8UC1);
	for (int i = 0; i < MatHand.rows; i++)
	{
		for (int j = 0; j < MatHand.cols; j++)
		{
			uchar* p_mask = output_mask.ptr<uchar>(i, j);
			uchar* p_src = ycrcb_img.ptr<uchar>(i, j);
			int flag = 0;
			//Cr[132, 136], Cb[125, 130]，若再次范围内，则将掩模像素值置为255
			if (p_src[0] >= 30 && p_src[0] <= 233&&p_src[1] >= 80 && p_src[1] <= 127 && p_src[2] >= 133 && p_src[2] <= 180)
			{
				p_mask[0] = 255;
				flag = 1;
			}
			if (flag == 0) {
				MatHand.at<Vec4b>(i, j) = 0;
			}
		}
	}
	//Mat detect;
	//MatHand.copyTo(detect, output_mask);
	/*for (int j = 0; j < MatHand.cols; ++j) {
		for (int i = 0; i < MatHand.rows; ++i) {
			int flag = 0;
			if (channel[0].at<uchar>(i, j) > 35 && channel[0].at<uchar>(i, j) < 230) {
				if (channel[1].at<uchar>(i, j) > 80 && channel[1].at<uchar>(i, j) < 130) {
					if (channel[2].at<uchar>(i, j) > 130 && channel[2].at<uchar>(i, j) < 180) {
						flag = 1;
					}
				}
			}
			if (flag == 0) {
				MatHand.at<Vec4b>(i, j) = 0;
			}
		}
	}*/
	//cout << MatHand.cols << "   " << MatHand.rows << endl;
	imshow("MatDepthToColor1", MatHand);
	
	//if (waitKey(1) == VK_SPACE)
		//imwrite("E:/abc123.bmp", MatDepthToColor1);
	if (waitKey(27) == VK_SPACE) {
		string path = "E:/abca.jpg";
		static char num = '1';
		path[5] = num;
		++num;
		imwrite(path, MatHand);
	}
	return true;
}	

AllTheKinect::~AllTheKinect()
{
	cv::destroyAllWindows();
	m_pKinectSensor->Close();
	std::system("pause");
}
