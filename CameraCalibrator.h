// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited

#ifndef __CAMERACALIBRATOR_H
#define __CAMERACALIBRATOR_H
#include "CalibImage.h"
#include "VideoSource.h"
#include <gvars3/gvars3.h>
#include <vector>
#include "GLWindow2.h"
#include <utility>

enum ImageSide{LEFT,RIGHT}; 

class CameraCalibrator
{
public:
  CameraCalibrator();
  void Run();
  ~CameraCalibrator();
    
protected:
  void Reset();
  void HandleFrame(CVD::Image<CVD::byte> imFrame);
  static void MainLoopCallback(void* pvUserData);
  void MainLoopStep();
  
  VideoSource mVideoSource;
  void GetRelativePoseEstimate(std::pair<CalibImage,CalibImage> &imgs, SE3<> &l2rse3);
  void GetExtrinsicMedian(SE3<> &se3);
  void CalculateExtrinsic(SE3<> &se3);
  
  GLWindow2 mGLWindowL;
  GLWindow2 mGLWindowR;
  ATANCamera *mCamera_LeftPTR;
  ATANCamera *mCamera_RightPTR;
  ATANCamera &mCamera_Left;
  ATANCamera &mCamera_Right;
  bool mbDone;
  bool mbExtrinsic;
  int count;
  std::vector<std::pair<CalibImage,CalibImage> > mvCalibImgs;
  
  bool mbGrabNextFrame;
  GVars3::gvar3<int> mgvnOptimizing;
  GVars3::gvar3<int> mgvnShowImage;
  GVars3::gvar3<int> mgvnDisableDistortion;
  bool goodGrid;
  bool badGrid;
  bool assessGrid;
  double mdMeanPixelError_LEFT;
  double mdMeanPixelError_RIGHT;

  void GUICommandHandler(std::string sCommand, std::string sParams);
  static void GUICommandCallBack(void* ptr, std::string sCommand, std::string sParams);
  void OptimizeOneStepLEFT();
  void OptimizeOneStepRIGHT();
  //void OptimizeOneStep(ATANCamera &Cam, ImageSide);
  void oldExt(SE3<> &se3);

};

#endif
