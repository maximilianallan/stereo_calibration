// -*- c++ *--
// Copyright 2008 Isis Innovation Limited
//
// VideoSource.h
// Declares the VideoSource class
// 
// This is a very simple class to provide video input; this can be
// replaced with whatever form of video input that is needed.  It
// should open the video input on construction, and provide two
// function calls after construction: Size() must return the video
// format as an ImageRef, and GetAndFillFrameBWandRGB should wait for
// a new frame and then overwrite the passed-as-reference images with
// GreyScale and Colour versions of the new frame.
//

#ifndef __VIDEOSOURCE_H
#define __VIDEOSOURCE_H
#include <cvd/image.h>
#include <cvd/byte.h>
#include <cvd/rgb.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include <opencv/cv.h>
#include "constants.h"
struct VideoSourceData;

//#define OPENCV_VIDEO_W 720 
// feed is 1440 wide, split into two images
//#define OPENCV_VIDEO_H 576
class VideoSource{
 public:
  VideoSource();
  void GetAndFillFrameBWandRGB(CVD::Image<CVD::byte> &imL_BW, 
			       CVD::Image<CVD::byte> &imR_BW, 
			       CVD::Image<CVD::Rgb<CVD::byte> > &imL_RGB, 
			       CVD::Image<CVD::Rgb<CVD::byte> > &imR_RGB);
 
  void GetAndFillFrameBWandRGB(CVD::Image<CVD::byte> &imL_BW, 
			       CVD::Image<CVD::byte> &imR_BW, 
			       CVD::Image<CVD::Rgb<CVD::byte> > &imL_RGB, 
			       CVD::Image<CVD::Rgb<CVD::byte> > &imR_RGB, 
			       const cv::Mat &, const cv::Mat &, 
			       const cv::Mat &, const cv::Mat &);
  CVD::ImageRef Size();
  CvSize getCVSize();
 private:
  void *mptr1;
  //  void *mptr2;
  CVD::ImageRef mirSize;
  CvSize cv_Size;
};

#endif
