#include "kinect_deal.h"

void GetKinectAllData(void)
{

}

HRESULT  GetScreenshotFileName(_Out_writes_z_(nFilePathSize) LPWSTR lpszFilePath, UINT nFilePathSize) {
	WCHAR* pszKnownPath = NULL;
	HRESULT hr = SHGetKnownFolderPath(FOLDERID_Pictures, 0, NULL, &pszKnownPath);
	if (SUCCEEDED(hr))
	{
		// Get the time
		WCHAR szTimeString[MAX_PATH];
		GetTimeFormatEx(NULL, 0, NULL, L"hh'-'mm'-'ss", szTimeString, _countof(szTimeString));

		// File name will be KinectScreenshotColor-HH-MM-SS.bmp
		StringCchPrintfW(lpszFilePath, nFilePathSize, L"%s\\KinectScreenshot-Color-%s.bmp", pszKnownPath, szTimeString);
	}

	if (pszKnownPath)
	{
		CoTaskMemFree(pszKnownPath);
	}
	return hr;
}
Point ppp;
bool flag = false;
HRESULT SaveBitmapToFile(BYTE* pBitmapBits, LONG lWidth, LONG lHeight, WORD wBitsPerPixel, LPCWSTR lpszFilePath)
{
	return E_NOTIMPL;
}
void DrawLine(Mat& Img, const Joint& r1, const Joint& r2, ICoordinateMapper* pMapper)
{
	//用两个关节点来做线段的两端，并且进行状态过滤
	if (r1.TrackingState == TrackingState_NotTracked || r2.TrackingState == TrackingState_NotTracked)
		return;
	//要把关节点用的摄像机坐标下的点转换成彩色空间的点
	ColorSpacePoint p1, p2;
	pMapper->MapCameraPointToColorSpace(r1.Position, &p1);
	pMapper->MapCameraPointToColorSpace(r2.Position, &p2);

	line(Img, Point(p1.X, p1.Y), Point(p2.X, p2.Y), Vec3b(0, 0, 255), 5);
}

void onMouse(int event, int x, int y, int flags, void* param)
{
	if (event == CV_EVENT_LBUTTONDOWN)//左键按下，读取初始坐标，并在图像上该点处划圆  
	{
		flag = true;
		ppp.x = x;
		ppp.y = y;
		cout << ppp << endl;
	}
}

