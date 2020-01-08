#pragma once
#include "AllFile.h"
void GetKinectAllData(void);

HRESULT  GetScreenshotFileName(_Out_writes_z_(nFilePathSize) LPWSTR lpszFilePath, UINT nFilePathSize);
HRESULT  SaveBitmapToFile(BYTE* pBitmapBits, LONG lWidth, LONG lHeight, WORD wBitsPerPixel, LPCWSTR lpszFilePath);
void DrawLine(Mat& Img, const Joint& r1, const Joint& r2, ICoordinateMapper* pMapper);
void onMouse(int event, int x, int y, int flags, void* param);