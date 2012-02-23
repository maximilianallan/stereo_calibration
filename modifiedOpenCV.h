#ifndef __MOD_OPEN_CV_H
#define __MOD_OPEN_CV_H

#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxmisc.h>
#include "ATANCamera.h"
#include <TooN/TooN.h>
template<int Size> TooN::Vector<Size> normalizeLast(TooN::Vector<Size> vec){
  int size = (int)vec.size();
  for(int i=0;i<size;i++)
    vec[i] /= vec[size-1];
  return vec;
}

void new_stereoRectify(const CvMat *leftMatrix, const CvMat *rightMatrix, const float leftDistortion, const float rightDistortion,/* const CvMat *, const CvMat *,*/   CvSize imageSize, const CvMat *rotation, const CvMat *translation, CvMat *R1, CvMat *R2, CvMat *P1, CvMat *P2, CvMat *Q,int flags CV_DEFAULT(CV_CALIB_ZERO_DISPARITY), double alpha CV_DEFAULT(-1), CvSize new_image_size CV_DEFAULT(cvSize(0,0)), CvRect* valid_pix_ROI1 CV_DEFAULT(0), CvRect* valid_pix_ROI2 CV_DEFAULT(0));

void getRectangles(const CvMat *, const float, const CvMat *, const CvMat *, CvSize, cv::Rect_<float>&, cv::Rect_<float>&);

void undistort_points(const CvMat* _src, CvMat* _dst, 
		      const CvMat* _cameraMatrix,
		      const float _distCoeffs,
		      const CvMat* matR = 0, const CvMat* matP = 0);

void InitUndistortRectifyMap(const CvMat *, const float distortion, const CvMat *, const CvMat *, CvArr *, CvArr *);

void UndistortRectifyMap( const cv::Mat& cameraMatrix, float distCoeffs, const cv::Mat& matR, const cv::Mat& newCameraMatrix, cv::Size size, int m1type, cv::Mat& map1, cv::Mat& map2 );


#endif
