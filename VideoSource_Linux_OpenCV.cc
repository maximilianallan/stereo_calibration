/*
* Autor : Arnaud GROSJEAN (VIDE SARL)
* This implementation of VideoSource allows to use OpenCV as a source for the video input
* I did so because libCVD failed getting my V4L2 device
*
* INSTALLATION :
* - Copy the VideoSource_Linux_OpenCV.cc file in your PTAM directory
* - In the Makefile:
*	- set the linkflags to
	LINKFLAGS = -L MY_CUSTOM_LINK_PATH -lblas -llapack -lGVars3 -lcvd -lcv -lcxcore -lhighgui
*	- set the videosource to 
	VIDEOSOURCE = VideoSource_Linux_OpenCV.o
* - Compile the project
* - Enjoy !
* 
* Notice this code define two constants for the image width and height (OPENCV_VIDEO_W and OPENCV_VIDEO_H)
*/

#include "CameraCalibrator.h"
#include "VideoSource.h"
#include <cvd/Linux/v4lbuffer.h>
#include <cvd/colourspace_convert.h>
#include <cvd/colourspaces.h>
#include <gvars3/instances.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include <opencv/cv.h>


using namespace CVD;
using namespace std;
using namespace GVars3;
using namespace cv;
// old video aspect ratio
//#define OPENCV_VIDEO_W 640
//#define OPENCV_VIDEO_H 480



VideoSource::VideoSource()
{
    cout << "  VideoSource_Linux: Opening video source..." << endl;
    // mptr = new VideoCapture("http://192.168.0.5:8080/videofeed");
    //mptr1 = new VideoCapture(1);
    //mptr2 = new VideoCapture(2);
    //((VideoCapture*)mptr1)->set(CV_CAP_PROP_FRAME_WIDTH, OPENCV_VIDEO_W);
    //((VideoCapture*)mptr1)->set(CV_CAP_PROP_FRAME_HEIGHT, OPENCV_VIDEO_H);
    //((VideoCapture*)mptr2)->set(CV_CAP_PROP_FRAME_WIDTH, OPENCV_VIDEO_W);
    //((VideoCapture*)mptr2)->set(CV_CAP_PROP_FRAME_HEIGHT, OPENCV_VIDEO_H);

    mptr1 = new VideoCapture("/home/max/Documents/university/computer_science/individual_project/openCV/projects/longercopyx2.avi");
    //mptr = new VideoCapture("/home/max/Documents/university/computer_science/individual_project/data_set/calibration_pattern/video/stereo_1440x576_25hz/capture-20110405T100726Z.avi");
    VideoCapture* cap1 = (VideoCapture*) mptr1;
    //VideoCapture* cap2 = (VideoCapture*) mptr2;
    //cout << "video cap 1 size is " << cap1->get(CV_CAP_PROP_FRAME_HEIGHT) << " and video cap2 size is " << cap2->get(CV_CAP_PROP_FRAME_HEIGHT) << endl;
    if (!cap1->isOpened()/* || !cap2->isOpened()*/) {
        cerr << "Unable to get the camera" << endl;
        exit(-1);
    }
    mirSize = ImageRef(OPENCV_VIDEO_W, OPENCV_VIDEO_H);
    cv_Size = cvSize(OPENCV_VIDEO_W,OPENCV_VIDEO_H);
    cout << "  ... got video source." << endl;    
    
};


ImageRef VideoSource::Size()
{ 
  return mirSize;
};

CvSize VideoSource::getCVSize(){
  return cv_Size;
}

//version for converting single image into Image<byte>
void conversionNB(Mat frame, Image<byte> &imBW){
  Mat clone = frame.clone();
  Mat_<Vec3b>& frame_p = (Mat_<Vec3b>&)clone;
  for (int i = 0; i < OPENCV_VIDEO_H; i++){
    for (int j = 0; j < OPENCV_VIDEO_W; j++){	
      imBW[i][j] = (frame_p(i,j)[0] + frame_p(i,j)[1] + frame_p(i,j)[2]) / 3;
    }
  }
}

void conversionRGB(Mat frame, Image<Rgb<byte> > &imRGB){
	Mat clone = frame.clone();
	Mat_<Vec3b>& frame_p = (Mat_<Vec3b>&)clone;
	for (int i = 0; i < OPENCV_VIDEO_H; i++){
		for (int j = 0; j < OPENCV_VIDEO_W; j++){	
		imRGB[i][j].red = frame_p(i,j)[2];
		imRGB[i][j].green = frame_p(i,j)[1];
		imRGB[i][j].blue = frame_p(i,j)[0];
		}
	}
}

//version for converting large stereo image into two Image<byte>
void conversionNB(Mat frame, Image<byte> &imL_BW, Image<byte> &imR_BW){
  Mat clone = frame.clone(); 
  Mat_<Vec3b>& frame_p = (Mat_<Vec3b>&)clone;
  for (int i = 0; i < OPENCV_VIDEO_H; i++){
    for (int j = 0; j < OPENCV_VIDEO_W; j++){	
      imL_BW[i][j] = (frame_p(i,j)[0] + frame_p(i,j)[1] + frame_p(i,j)[2]) / 3;
    }
                for (int a = OPENCV_VIDEO_W; a<(OPENCV_VIDEO_W*2);a++){
		  imR_BW[i][a-OPENCV_VIDEO_W] = (frame_p(i,a)[0] + frame_p(i,a)[1] + frame_p(i,a)[2]) / 3;
		}
	}

}

//version for converting large stereo image into two Image<byte>
void conversionRGB(Mat frame, Image<Rgb<byte> > &imL_RGB, Image<Rgb<byte> > &imR_RGB){
  Mat clone = frame.clone();
  Mat_<Vec3b>& frame_p = (Mat_<Vec3b>&)clone;
  for (int i = 0; i < OPENCV_VIDEO_H; i++){
    for (int j = 0; j < OPENCV_VIDEO_W; j++){	
      imL_RGB[i][j].red = frame_p(i,j)[2];
      imL_RGB[i][j].green = frame_p(i,j)[1];
      imL_RGB[i][j].blue = frame_p(i,j)[0];
    }
    for (int a = OPENCV_VIDEO_W; a < (OPENCV_VIDEO_W*2); a++){
      imR_RGB[i][a-OPENCV_VIDEO_W].red = frame_p(i,a)[2];
      imR_RGB[i][a-OPENCV_VIDEO_W].green = frame_p(i,a)[1];
      imR_RGB[i][a-OPENCV_VIDEO_W].blue = frame_p(i,a)[0];
    }
  }
}


void VideoSource::GetAndFillFrameBWandRGB(Image<byte> &imL_BW, Image<byte> &imR_BW, Image<Rgb<byte> > &imL_RGB,Image<Rgb<byte> > &imR_RGB){
  Mat frame;
  VideoCapture* cap = (VideoCapture*)mptr1;
  *cap >> frame;
  conversionNB(frame, imL_BW, imR_BW);
  conversionRGB(frame, imL_RGB, imR_RGB);
}

void VideoSource::GetAndFillFrameBWandRGB(CVD::Image<CVD::byte> &imL_BW,CVD::Image<CVD::byte> &imR_BW,CVD::Image<CVD::Rgb<CVD::byte> > &imL_RGB,CVD::Image<CVD::Rgb<CVD::byte> > &imR_RGB,const cv::Mat &leftX, const cv::Mat &leftY, const cv::Mat &rightX, const cv::Mat &rightY){
  Mat frame;
  VideoCapture* cap1 = (VideoCapture*)mptr1;
  *cap1 >> frame;
  CvRect l = cvRect(0,0,OPENCV_VIDEO_W,OPENCV_VIDEO_H);
  CvRect r = cvRect(OPENCV_VIDEO_W-1,0,OPENCV_VIDEO_W,OPENCV_VIDEO_H);
  
  Mat frame_left(l.height,l.width,CV_8UC3);
  Mat frame_right(l.height,l.width,CV_8UC3);
 
  frame_left = frame(l);
  frame_right = frame(r);
  //Mat rectified_left, rectified_right;
  //VideoCapture *cap1 = (VideoCapture*)mptr1;
  //VideoCapture *cap2 = (VideoCapture*)mptr2;
  
  //*cap1 >> rectified_left;
  //*cap2 >> rectified_right;


  
  //double varianceFactor = 1,meanFactor = 1; 
  //preprocessFrames(frame_left,frame_right, meanFactor, varianceFactor);
  Mat rectified_left(l.height,l.width,CV_8UC3);
  Mat rectified_right(l.height,l.width,CV_8UC3);
  remap(frame_left,rectified_left,leftX,leftY, CV_INTER_LINEAR);
  remap(frame_right,rectified_right,rightX,rightY, CV_INTER_LINEAR);
  conversionNB(rectified_left, imL_BW);
  //cout << "here" << endl;
  conversionNB(rectified_right, imR_BW);
  //cout << "here" << endl;
  conversionRGB(rectified_left, imL_RGB);
  //cout << "here" << endl;
  conversionRGB(rectified_right, imR_RGB);
  //cout << "here" << endl;
}
