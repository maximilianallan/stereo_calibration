// Copyright 2008 Isis Innovation Limited
#include "OpenGL.h"
#include <gvars3/instances.h>
#include "CameraCalibrator.h"
#include "CalibImage.h"
#include <TooN/SVD.h>
#include <TooN/TooN.h>
#include <fstream>
#include <stdlib.h>
#include <string.h>
//#include <cstring>
#include <stdio.h>
//#include "HomographyInit.h"

#include "modifiedOpenCV.h"
#include "SmallMatrixOpts.h"
//#include <utility>
#include <TooN/se3.h>
#include <TooN/SVD.h>
#include <TooN/SymEigen.h>
#include <TooN/wls.h>
#include "MEstimator.h"

using namespace CVD;
using namespace std;
using namespace GVars3;

/* CAMERAS MUST BE NAMED AS Camera_Left AND Camera_Right FOR
   GVARS GET() TO WORK...
*/

int main() {
 
  cout << "  Will now calibrate the stereo pair" << endl;
  
  cout << "  Parsing calibrator_settings.cfg ...." << endl;
  
  GUI.LoadFile("calibrator_settings.cfg");
  GUI.StartParserThread();
  atexit(GUI.StopParserThread); // Clean up readline when program quits
  
  //there has got to be a better way of getting all
  //these gvars...
  GV3::get<Vector<NUMTRACKERCAMPARAMETERS> >( "CameraLeft.Parameters", ATANCamera::mvDefaultParams, SILENT);
  GV3::get<Vector<NUMTRACKERCAMPARAMETERS> >("CameraRight.Parameters", ATANCamera::mvDefaultParams, SILENT);
  GV3::get<Vector<6> >("CameraLeftExtrinsic.Parameters", ATANCamera::mvDefaultExtrinsicVector,SILENT);
  GV3::get<Vector<6> >("CameraRightExtrinsic.Parameters", ATANCamera::mvDefaultExtrinsicVector,SILENT);
  //GV3::get<Vector<9> >("Camera_Left_cvRectifyRotate.Parameters",ATANCamera::mvDefaultRectifyRotate,SILENT);
  //GV3::get<Vector<12> >("Camera_Left_cvRectifyProject.Parameters", ATANCamera::mvDefaultRectifyProject,SILENT);
  //GV3::get<Vector<16> >("Camera_Left_cvRectifyReProject.Parameters", ATANCamera::mvDefaultRectifyReProject,SILENT);
  //GV3::get<Vector<9> >("Camera_Right_cvRectifyRotate.Parameters",ATANCamera::mvDefaultRectifyRotate,SILENT);
  //GV3::get<Vector<12> >("Camera_Right_cvRectifyProject.Parameters", ATANCamera::mvDefaultRectifyProject,SILENT);
  //GV3::get<Vector<16> >("Camera_Right_cvRectifyReProject.Parameters", ATANCamera::mvDefaultRectifyReProject,SILENT);
  //GV3::get<Vector<9> >("Camera_Left_cvMatrix.Parameters", ATANCamera::mvDefaultcvCameraMatrix,SILENT);
  //GV3::get<Vector<9> >("Camera_Right_cvMatrix.Parameters", ATANCamera::mvDefaultcvCameraMatrix,SILENT);
  //GV3::get<Vector<8> >("Camera_Left_cvDistortion.Parameters", ATANCamera::mvDefaultcvDistortion,SILENT);
  //GV3::get<Vector<8> >("Camera_Right_cvDistortion.Parameters", ATANCamera::mvDefaultcvDistortion,SILENT);
  //GV3::get<Vector<4> >("Camera_Left_cvRectangle.Parameters", ATANCamera::mvDefaultcvRectangle, SILENT);
  //GV3::get<Vector<4> >("Camera_Right_cvRectangle.Parameters", ATANCamera::mvDefaultcvRectangle, SILENT);
  
  try {
    CameraCalibrator c;
    c.Run();
  } catch (CVD::Exceptions::All e) {
    cout << endl;
    cout << "!! Failed to run CameraCalibrator; got exception. " << endl;
    cout << "   Exception was: " << endl;
    cout << e.what << endl;
  }
}

CameraCalibrator::CameraCalibrator()
  : mGLWindowL(mVideoSource.Size(), "Camera Calibrator Left"), mGLWindowR(mVideoSource.Size(),"Camera Calibrator Right"), mCamera_LeftPTR(new ATANCamera("CameraLeft")), mCamera_RightPTR(new ATANCamera("CameraRight")), mCamera_Left(*mCamera_LeftPTR), mCamera_Right(*mCamera_RightPTR) {
  
  count = 0;
  assessGrid = false;
  goodGrid = false;
  badGrid = false;
  mbDone = false;
  mbExtrinsic = false;
  mGLWindowL.make_current();
  GUI.RegisterCommand("CameraCalibrator.GrabNextFrame", GUICommandCallBack, this);
  GUI.RegisterCommand("CameraCalibrator.Reset", GUICommandCallBack, this);
  GUI.RegisterCommand("CameraCalibrator.ShowNext", GUICommandCallBack, this);
  GUI.RegisterCommand("CameraCalibrator.SaveCalib", GUICommandCallBack, this);
  GUI.RegisterCommand("CameraCalibrator.GoodGrid",GUICommandCallBack, this);
  GUI.RegisterCommand("CameraCalibrator.BadGrid",GUICommandCallBack,this);
  GUI.RegisterCommand("quit", GUICommandCallBack, this);
  GUI.RegisterCommand("exit", GUICommandCallBack, this);
  GV3::Register(mgvnOptimizing, "CameraCalibrator.Optimize", 0, SILENT);
  GV3::Register(mgvnShowImage, "CameraCalibrator.Show", 0, SILENT);
  GV3::Register(mgvnDisableDistortion, "CameraCalibrator.NoDistortion", 0, SILENT);
  GUI.ParseLine("GLWindow.AddMenu CalibMenu");
  GUI.ParseLine("CalibMenu.AddMenuButton Live GrabFrame CameraCalibrator.GrabNextFrame");
  GUI.ParseLine("CalibMenu.AddMenuButton Live GoodGrid CameraCalibrator.GoodGrid");
  GUI.ParseLine("CalibMenu.AddMenuButton Live BadGrid CameraCalibrator.BadGrid");
  GUI.ParseLine("CalibMenu.AddMenuButton Live Reset CameraCalibrator.Reset");
  GUI.ParseLine("CalibMenu.AddMenuButton Live Optimize \"CameraCalibrator.Optimize=1\"");
  GUI.ParseLine("CalibMenu.AddMenuToggle Live NoDist CameraCalibrator.NoDistortion");
  GUI.ParseLine("CalibMenu.AddMenuSlider Opti \"Show Img\" CameraCalibrator.Show 0 10");
  GUI.ParseLine("CalibMenu.AddMenuButton Opti \"Show Next\" CameraCalibrator.ShowNext");
  GUI.ParseLine("CalibMenu.AddMenuButton Opti \"Grab More\" CameraCalibrator.Optimize=0 ");
  GUI.ParseLine("CalibMenu.AddMenuButton Opti Reset CameraCalibrator.Reset");
  GUI.ParseLine("CalibMenu.AddMenuToggle Opti NoDist CameraCalibrator.NoDistortion");
  GUI.ParseLine("CalibMenu.AddMenuButton Opti Save&External CameraCalibrator.SaveCalib");
  mvCalibImgs.reserve(20);
  Reset();
}

CameraCalibrator::~CameraCalibrator(){
  delete mCamera_LeftPTR;
  delete mCamera_RightPTR;
}
void CameraCalibrator::Run() {
  while (!mbDone) 
    {
        // We use two versions of each video frame:
        // One black and white (for processing by the tracker etc)
        // and one RGB, for drawing.

      Image<Rgb<byte> > imFrameL_RGB(mVideoSource.Size());
      Image<byte> imFrameL_BW(mVideoSource.Size());
      Image<Rgb<byte> > imFrameR_RGB(mVideoSource.Size());
      Image<byte> imFrameR_BW(mVideoSource.Size());
      // Grab new video frame...
	
      mVideoSource.GetAndFillFrameBWandRGB(imFrameL_BW, imFrameR_BW, imFrameL_RGB, imFrameR_RGB);
      // Set up openGL
      mGLWindowL.SetupViewport();
      mGLWindowL.SetupVideoOrtho();
      mGLWindowL.SetupVideoRasterPosAndZoom();

      mGLWindowR.SetupViewport();
      mGLWindowR.SetupVideoOrtho();
      mGLWindowR.SetupVideoRasterPosAndZoom();

      if (mvCalibImgs.size() < 1)
	*mgvnOptimizing = 0;

      if (!*mgvnOptimizing) 
	{
	  GUI.ParseLine("CalibMenu.ShowMenu Live");
	    
	  //draws the left and right images
	  mGLWindowL.make_current();
	  glDrawPixels(imFrameL_BW);
	  mGLWindowR.make_current();
	  glDrawPixels(imFrameR_BW);
	  mGLWindowL.make_current();
	  CalibImage c_Left;
	  CalibImage c_Right;
	  if(mbGrabNextFrame){
	    mGLWindowL.make_current();
	    if(c_Left.MakeFromImage(imFrameL_BW, mGLWindowL)){
	      mGLWindowR.make_current();
	      if(c_Right.MakeFromImage(imFrameR_BW,mGLWindowR)){
		mGLWindowL.make_current();
		while(badGrid == false && goodGrid == false){
		  assessGrid = true;
		  mGLWindowL.DrawMenus();
		  mGLWindowR.DrawMenus();
		  mGLWindowL.HandlePendingEvents();
		  mGLWindowR.HandlePendingEvents();
		  mGLWindowL.swap_buffers();
		  mGLWindowR.swap_buffers();
		}
		assessGrid = false;
		pair<CalibImage,CalibImage>p(c_Left,c_Right);
		if(badGrid == false && goodGrid == true && (int)p.first.orderedGridCorners.size() == (int)p.second.orderedGridCorners.size() && (int)p.first.orderedGridCorners.size() == X_GRID*Y_GRID){
		  count++;
		  p.first.setGridCorners();
		  p.second.setGridCorners();
		  //p.first.goodGrid = p.second.goodGrid = true;
		}else{
		  //p.first.goodGrid = p.second.goodGrid = false;
		}
		goodGrid = false;
		badGrid = false;
		mvCalibImgs.push_back(p);
		//mvLeftImages.push_back(c_Left);
		//mvRightImages.push_back(c_Right);
		  
		mvCalibImgs.back().first.GuessInitialPose(mCamera_Left);
		mvCalibImgs.back().first.Draw3DGrid(mCamera_Left, false);
		  
		mGLWindowR.make_current();
		mvCalibImgs.back().second.GuessInitialPose(mCamera_Right);
		mvCalibImgs.back().second.Draw3DGrid(mCamera_Right, false);
		mGLWindowL.make_current();
		  
	      }
	    }
	  }
	  mbGrabNextFrame = false;
	} else {
	  
	//leftImages.reserve(mvCalibImgs.size());
	//rightImages.reserve(mvCalibImgs.size());
	  
	OptimizeOneStepLEFT();
	OptimizeOneStepRIGHT();
	GUI.ParseLine("CalibMenu.ShowMenu Opti");
	int nToShow = *mgvnShowImage - 1; //which image to use
	  
	if (nToShow < 0)
	  nToShow = 0;
	if (nToShow >= (int) mvCalibImgs.size())
	  nToShow = mvCalibImgs.size() - 1;

	*mgvnShowImage = nToShow + 1;
	mGLWindowL.make_current();
	glDrawPixels(mvCalibImgs[nToShow].first.mim);
	mvCalibImgs[nToShow].first.Draw3DGrid(mCamera_Left, true);
	mGLWindowR.make_current();
	glDrawPixels(mvCalibImgs[nToShow].second.mim);
	mvCalibImgs[nToShow].second.Draw3DGrid(mCamera_Right,true);
	mGLWindowL.make_current();
      }
	
      ostringstream ost;
      ost << "Camera Calibration: Grabbed " << mvCalibImgs.size() << " images from cameras and " << count << " images with corresponding points " << endl;
      if (!*mgvnOptimizing) {
	ost << "Take snapshots of the calib grid with the \"GrabFrame\" button," << endl;
	ost << "and then press \"Optimize\"." << endl;
	ost << "Take enough shots (4+) at different angles to get points " << endl;
      } else {
	ost << "Current RMS pixel error on left camera is " << mdMeanPixelError_LEFT << " and RMS pixel error on right camera is " << mdMeanPixelError_RIGHT << endl;
	ost << "Current left camera params are  " << GV3::get_var("CameraLeft.Parameters") << endl;
	ost << "Current right camera params are  " << GV3::get_var("CameraRight.Parameters") << endl;
	ost << "(That would be a pixel aspect ratio of "
	    << mCamera_Left.PixelAspectRatio() << " on the left camera and " << mCamera_Right.PixelAspectRatio() << " on the right camera)" << endl;
	ost << "Check fit by looking through the grabbed images." << endl;
	ost << "RMS should go below 0.5, typically below 0.3 for a wide lens." << endl;
	ost << "Press \"save\" to save calibration to camera config file and begin external calibration." << endl;
      }
      if(mbExtrinsic){
	  
	// GET SE3 BETWEEN TWO CAMERAS //
	
	SE3<> L2R;
	// Find and select the median (based on translation distance) SE3
	// for relative pose between two cameras based on calculated pose
	// of each individual camera w.r.t the grid
	GetExtrinsicMedian(L2R);
	// Use Gauss-Newton minimisation to find best estimate of pose
	// over all images based on median start position
	CalculateExtrinsic(L2R);
	// Scale w.r.t known grid size
	L2R.get_translation() *= GRID_SQUARE_SIZE;
	cout << "Relative pose from left to right camera is " << endl << L2R << endl;
	
	SO3<> rInv = (L2R.get_rotation()).inverse();
	Vector<3> rTrans = -1*(L2R.get_translation());
	SE3<> R2L;
	R2L.get_rotation() = rInv;
	R2L.get_translation() = rTrans;
	  
	//sets the member variables of SE3 and cvR and cvT
	mCamera_Left.UpdateExtrinsicParams(L2R);
	mCamera_Right.UpdateExtrinsicParams(R2L);
		  
	ofstream ofs("CameraLeft.cfg");
	if(ofs.good()){
	  GV2.PrintVar("CameraLeft.Parameters",ofs,true);
	  GV2.PrintVar("CameraLeftExtrinsic.Parameters",ofs, true); //true prints \n and allows multiple gvars in 1 file
	  ofs.close();
	}else{
	  cout << "Could not open CameraLeft.cfg" << " for writing."<<endl;
	  GV2.PrintVar("CameraLeft.Parameters",cout);
	  GV2.PrintVar("CameraLeftExtrinsic.Parameters",cout);
	  cout << "Copy-paste the above matrix to CameraLeft.cfg" << endl;
	}
	ofs.open("CameraRight.cfg");
	if(ofs.good()){
	  GV2.PrintVar("CameraRight.Parameters",ofs,true);
	  GV2.PrintVar("CameraRightExtrinsic.Parameters",ofs,true);
	  ofs.close();
	}else{
	  cout << "Could not open CameraRight.cfg" << " for writing."<<endl;
	  GV2.PrintVar("CameraRight.Parameters",cout,true);
	  GV2.PrintVar("CameraRightExtrinsic.Parameters",cout);
	  cout << "Copy-paste the above matrix to CameraRight.cfg" << endl;
	}
	cout << "Goodbye ... " << endl;
	mbDone = true;
      }
      mGLWindowL.DrawCaption(ost.str());
      mGLWindowR.DrawCaption(ost.str());
      mGLWindowL.DrawMenus();
      mGLWindowR.DrawMenus();
      mGLWindowL.HandlePendingEvents();
      mGLWindowR.HandlePendingEvents();
      mGLWindowL.swap_buffers();
      mGLWindowR.swap_buffers();
    }
}

// do R_lr = R_r * inverse(R_l)
// then T_lr = T_r -  R_lr*T_r

void CameraCalibrator::GetRelativePoseEstimate(pair<CalibImage,CalibImage> &imgs, SE3<> &L2RSE3){
  Matrix<3> rotation = (imgs.second.mse3CamFromWorld.get_rotation().get_matrix())*(imgs.first.mse3CamFromWorld.get_rotation().get_matrix().T());
  Vector<3> translation = imgs.second.mse3CamFromWorld.get_translation() - (rotation * imgs.first.mse3CamFromWorld.get_translation());
  L2RSE3.get_rotation() = rotation;
  L2RSE3.get_translation() = translation;
}

// Comparision function for Quicksort
bool CompareSe3(const SE3<> &a, const SE3<> &b){
  return a.get_translation()*a.get_translation() > b.get_translation()*b.get_translation();
}

// Sort the vector of SE3 transformations into order based on length of translation 
vector<SE3<> > Quicksort(vector<SE3<> > vUnsorted){
  if((int)vUnsorted.size() <= 1)
    return vUnsorted;
  vector<SE3<> > vSmaller;
  vector<SE3<> > vLarger;
  vector<SE3<> >::iterator Pivot = vUnsorted.begin();
  vector<SE3<> >::iterator Compare = Pivot;

  for(Compare++;Compare!=vUnsorted.end();Compare++)
    {
      if(CompareSe3((*Pivot),(*Compare)))
	vSmaller.push_back(*Compare);
      else
	vLarger.push_back(*Pivot);
    }
  
  vSmaller = Quicksort(vSmaller);
  vLarger = Quicksort(vLarger);
  vSmaller.push_back(*Pivot);
  vSmaller.insert(vSmaller.end(),vLarger.begin(),vLarger.end());
  return vSmaller;
} 

// Get Median SE3 for start estimate for G.N minimisation
void CameraCalibrator::GetExtrinsicMedian(SE3<> &Se3Extrinsic){
  vector<SE3<> > vUnSortedSe3;
  SE3<> RelativePose;
  for(int i=0;i<(int)mvCalibImgs.size();i++){
    if(!mvCalibImgs[i].first.goodGrid || (int)mvCalibImgs[i].first.mvGridCorners.size() != X_GRID*Y_GRID  || (int)mvCalibImgs[i].second.mvGridCorners.size() != X_GRID*Y_GRID)
      continue;
    GetRelativePoseEstimate(mvCalibImgs[i],RelativePose);
    vUnSortedSe3.push_back(RelativePose);
  }
  vector<SE3<> > vSortedSe3 = Quicksort(vUnSortedSe3);
  Se3Extrinsic = vSortedSe3[(int)vSortedSe3.size()/2];
}

// Reset camera calibratior - set intrinsic params to default, delete all data and reset image size
void CameraCalibrator::Reset() {
    *mCamera_Left.mgvvCameraParams = ATANCamera::mvDefaultParams;
    *mCamera_Right.mgvvCameraParams = ATANCamera::mvDefaultParams;
    if (*mgvnDisableDistortion){
        mCamera_Left.DisableRadialDistortion();
        mCamera_Right.DisableRadialDistortion();
    }
    mCamera_Left.SetImageSize(mVideoSource.Size());
    mCamera_Right.SetImageSize(mVideoSource.Size());
    mbGrabNextFrame = false;
    *mgvnOptimizing = false;
    mvCalibImgs.clear();
}

// GUI function - calls the command handler
void CameraCalibrator::GUICommandCallBack(void* ptr, string sCommand, string sParams) {
    ((CameraCalibrator*) ptr)->GUICommandHandler(sCommand, sParams);
}

// GUI function - calls appropriate function to handle command
void CameraCalibrator::GUICommandHandler(string sCommand, string sParams) // Called by the callback func..
{
    if (sCommand == "CameraCalibrator.Reset") {
        Reset();
        return;
    };
    if (sCommand == "CameraCalibrator.GrabNextFrame") {
        mbGrabNextFrame = true;
        return;
    }
    if(sCommand == "CameraCalibrator.GoodGrid"){
      if(assessGrid)
	goodGrid = true;
      return;
    }
    if(sCommand == "CameraCalibrator.BadGrid"){
      if(assessGrid)
	badGrid = true;
      return;
    }
    if (sCommand == "CameraCalibrator.ShowNext") {
        int nToShow = (*mgvnShowImage - 1 + 1) % mvCalibImgs.size();
        *mgvnShowImage = nToShow + 1;
        return;
    }
    if (sCommand == "CameraCalibrator.SaveCalib") 
      {
	cout << " Left camera calib is " << GV3::get_var("CameraLeft.Parameters") << endl
	     << " and right camera calib is " << GV3::get_var("CameraRight.Parameters") << endl
	     << "  Saving camera calibration " << endl;
      	mbExtrinsic = true; // Now perform extrinsic calibration
      }

    if (sCommand == "exit" || sCommand == "quit") 
      {
	mbDone = true;
      }
}
/* Performs optimization on the calibrated images for left camera
   TO MODIFY : ditch left and right version and pair of images
   instead have separate image vectors and pass as arg.
*/

void CameraCalibrator::OptimizeOneStepLEFT() {
  int nViews = mvCalibImgs.size(); 
  int nDim = 6 * nViews + NUMTRACKERCAMPARAMETERS; // 6 degrees of freedom (3 rotational 3 translation)
  int nCamParamBase = nDim - NUMTRACKERCAMPARAMETERS;

  Matrix<> mJTJ_left(nDim, nDim);
  Vector<> vJTe_left(nDim);
  mJTJ_left = Identity; // Weak stabilizing prior
  vJTe_left = Zeros;
  
  if (*mgvnDisableDistortion){ 
    mCamera_Left.DisableRadialDistortion();
  }
  double dSumSquaredError_left = 0.0;
  int nTotalMeas_left = 0;

  for (int n = 0; n < nViews; n++) {
    int nMotionBase = n * 6;
    vector<CalibImage::ErrorAndJacobians> vEAJ_left = mvCalibImgs[n].first.Project(mCamera_Left);
    
      /*vEAJ - for each grid corner, transform this to the real grid 
	orientation then project to the image coordiantes (u,v). 
	size is the number of grid corners in image n */
      for (unsigned int i = 0; i < vEAJ_left.size(); i++) {
	CalibImage::ErrorAndJacobians &EAJ_left = vEAJ_left[i];
	// All the below should be +=, but the MSVC compiler doesn't seem to understand that. :(
	mJTJ_left.slice(nMotionBase, nMotionBase, 6, 6) =
	  mJTJ_left.slice(nMotionBase, nMotionBase, 6, 6) + EAJ_left.m26PoseJac.T() * EAJ_left.m26PoseJac;
	mJTJ_left.slice(nCamParamBase, nCamParamBase, NUMTRACKERCAMPARAMETERS, NUMTRACKERCAMPARAMETERS) =
	  mJTJ_left.slice(nCamParamBase, nCamParamBase, NUMTRACKERCAMPARAMETERS, NUMTRACKERCAMPARAMETERS) + EAJ_left.m2NCameraJac.T() * EAJ_left.m2NCameraJac;
	mJTJ_left.slice(nMotionBase, nCamParamBase, 6, NUMTRACKERCAMPARAMETERS) =
	  mJTJ_left.slice(nMotionBase, nCamParamBase, 6, NUMTRACKERCAMPARAMETERS) + EAJ_left.m26PoseJac.T() * EAJ_left.m2NCameraJac;
	mJTJ_left.T().slice(nMotionBase, nCamParamBase, 6, NUMTRACKERCAMPARAMETERS) =
	  mJTJ_left.T().slice(nMotionBase, nCamParamBase, 6, NUMTRACKERCAMPARAMETERS) + EAJ_left.m26PoseJac.T() * EAJ_left.m2NCameraJac;
	// Above does twice the work it needs to, but who cares..

	vJTe_left.slice(nMotionBase, 6) =
	  vJTe_left.slice(nMotionBase, 6) + EAJ_left.m26PoseJac.T() * EAJ_left.v2Error;
	vJTe_left.slice(nCamParamBase, NUMTRACKERCAMPARAMETERS) =
	  vJTe_left.slice(nCamParamBase, NUMTRACKERCAMPARAMETERS) + EAJ_left.m2NCameraJac.T() * EAJ_left.v2Error;


	dSumSquaredError_left += EAJ_left.v2Error * EAJ_left.v2Error;
	++nTotalMeas_left;
      }
  };

  mdMeanPixelError_LEFT = sqrt(dSumSquaredError_left / nTotalMeas_left);
 

  SVD<> svd_left(mJTJ_left);
  Vector<> vUpdate_left(nDim);
  vUpdate_left = svd_left.backsub(vJTe_left);
  //solves mJTJ*vUpdate = vJTe -. vUpdate = mJTJ^-1 *vJTe 
 // Slow down because highly nonlinear...
  vUpdate_left *= 0.1;
  for (int n = 0; n < nViews; n++){
    mvCalibImgs[n].first.mse3CamFromWorld = SE3<>::exp(vUpdate_left.slice(n * 6, 6)) * mvCalibImgs[n].first.mse3CamFromWorld;
  }
  mCamera_Left.UpdateParams(vUpdate_left.slice(nCamParamBase, NUMTRACKERCAMPARAMETERS));
}

/*
  void CameraCalibrator::OptimizeOneStep(ATANCamera Camera, Image eSide) {
  int nViews = mvCalibImgs.size(); 
  int nDim = 6 * nViews + NUMTRACKERCAMPARAMETERS; // 6 degrees of freedom (3 rotational 3 translation)
  int nCamParamBase = nDim - NUMTRACKERCAMPARAMETERS;

  Matrix<> mJTJ(nDim, nDim);
  Vector<> vJTe(nDim);
  mJTJ_right = Identity;
  vJTe_right = Zeros;

  if (*mgvnDisableDistortion){ 
    mCamera_Right.DisableRadialDistortion();
  }
  double dSumSquaredError = 0.0;
  int nTotalMeas_right = 0;

  for (int n = 0; n < nViews; n++) {
    int nMotionBase = n * 6;
    vector<CalibImage::ErrorAndJacobians> vEAJ;
    if(eSide == LEFT)
    vEAJ = mvCalibImgs[n].first.Project(Camera);
    else
    vEAJ = mvCalibImgs[n].second.Project(Camera);
    //vEAJ - for each grid corner, transform this to the real grid 
    //  orientation then project to the image coordiantes (u,v). 
    //  size is the number of grid corners in image n
    for (unsigned int i = 0; i < vEAJ_right.size(); i++) {
      CalibImage::ErrorAndJacobians &EAJ = vEAJ[i];
      // All the below should be +=, but the MSVC compiler doesn't seem to understand that. :(
      
      mJTJ.slice(nMotionBase, nMotionBase, 6, 6) =
	mJTJ.slice(nMotionBase, nMotionBase, 6, 6) + EAJ.m26PoseJac.T() * EAJ.m26PoseJac;
      mJTJ.slice(nCamParamBase, nCamParamBase, NUMTRACKERCAMPARAMETERS, NUMTRACKERCAMPARAMETERS) =
	mJTJ.slice(nCamParamBase, nCamParamBase, NUMTRACKERCAMPARAMETERS, NUMTRACKERCAMPARAMETERS) + EAJ.m2NCameraJac.T() * EAJ.m2NCameraJac;
      mJTJ.slice(nMotionBase, nCamParamBase, 6, NUMTRACKERCAMPARAMETERS) =
	mJTJ.slice(nMotionBase, nCamParamBase, 6, NUMTRACKERCAMPARAMETERS) + EAJ.m26PoseJac.T() * EAJ.m2NCameraJac;
      mJTJ.T().slice(nMotionBase, nCamParamBase, 6, NUMTRACKERCAMPARAMETERS) =
	mJTJ.T().slice(nMotionBase, nCamParamBase, 6, NUMTRACKERCAMPARAMETERS) + EAJ.m26PoseJac.T() * EAJ.m2NCameraJac;
      // Above does twice the work it needs to, but who cares..

      vJTe.slice(nMotionBase, 6) =
	vJTe.slice(nMotionBase, 6) + EAJ.m26PoseJac.T() * EAJ.v2Error;
      vJTe.slice(nCamParamBase, NUMTRACKERCAMPARAMETERS) =
	vJTe.slice(nCamParamBase, NUMTRACKERCAMPARAMETERS) + EAJ.m2NCameraJac.T() * EAJ.v2Error;
	    
      dSumSquaredError += EAJ.v2Error * EAJ.v2Error;
      ++nTotalMeas;

      }
  };

  mdMeanPixelError = sqrt(dSumSquaredError / nTotalMeas);
  
  SVD<> svd(mJTJ);
  Vector<> vUpdate(nDim);
  vUpdate = svd.backsub(vJTe);
  //solves mJTJ*vUpdate = vJTe -. vUpdate = mJTJ^-1 *vJTe 
  vUpdate *= 0.1; // Slow down because highly nonlinear...
  for (int n = 0; n < nViews; n++){
    mvCalibImgs[n].second.mse3CamFromWorld = SE3<>::exp(vUpdate_right.slice(n * 6, 6)) * mvCalibImgs[n].second.mse3CamFromWorld;
  }
  mCamera_Right.UpdateParams(vUpdate_right.slice(nCamParamBase, NUMTRACKERCAMPARAMETERS));
}



 */

void CameraCalibrator::OptimizeOneStepRIGHT() {
  int nViews = mvCalibImgs.size(); 
  int nDim = 6 * nViews + NUMTRACKERCAMPARAMETERS; // 6 degrees of freedom (3 rotational 3 translation)
  int nCamParamBase = nDim - NUMTRACKERCAMPARAMETERS;

  Matrix<> mJTJ_right(nDim, nDim);
  Vector<> vJTe_right(nDim);
  mJTJ_right = Identity;
  vJTe_right = Zeros;

  if (*mgvnDisableDistortion){ 
    mCamera_Right.DisableRadialDistortion();
  }
  double dSumSquaredError_right = 0.0;
  int nTotalMeas_right = 0;

  for (int n = 0; n < nViews; n++) {
    int nMotionBase = n * 6;
    vector<CalibImage::ErrorAndJacobians> vEAJ_right = mvCalibImgs[n].second.Project(mCamera_Right);
    /*vEAJ - for each grid corner, transform this to the real grid 
      orientation then project to the image coordiantes (u,v). 
      size is the number of grid corners in image n */
    for (unsigned int i = 0; i < vEAJ_right.size(); i++) {
      CalibImage::ErrorAndJacobians &EAJ_right = vEAJ_right[i];
      // All the below should be +=, but the MSVC compiler doesn't seem to understand that. :(
      
      mJTJ_right.slice(nMotionBase, nMotionBase, 6, 6) =
	mJTJ_right.slice(nMotionBase, nMotionBase, 6, 6) + EAJ_right.m26PoseJac.T() * EAJ_right.m26PoseJac;
      mJTJ_right.slice(nCamParamBase, nCamParamBase, NUMTRACKERCAMPARAMETERS, NUMTRACKERCAMPARAMETERS) =
	mJTJ_right.slice(nCamParamBase, nCamParamBase, NUMTRACKERCAMPARAMETERS, NUMTRACKERCAMPARAMETERS) + EAJ_right.m2NCameraJac.T() * EAJ_right.m2NCameraJac;
      mJTJ_right.slice(nMotionBase, nCamParamBase, 6, NUMTRACKERCAMPARAMETERS) =
	mJTJ_right.slice(nMotionBase, nCamParamBase, 6, NUMTRACKERCAMPARAMETERS) + EAJ_right.m26PoseJac.T() * EAJ_right.m2NCameraJac;
      mJTJ_right.T().slice(nMotionBase, nCamParamBase, 6, NUMTRACKERCAMPARAMETERS) =
	mJTJ_right.T().slice(nMotionBase, nCamParamBase, 6, NUMTRACKERCAMPARAMETERS) + EAJ_right.m26PoseJac.T() * EAJ_right.m2NCameraJac;
      // Above does twice the work it needs to, but who cares..

      vJTe_right.slice(nMotionBase, 6) =
	vJTe_right.slice(nMotionBase, 6) + EAJ_right.m26PoseJac.T() * EAJ_right.v2Error;
      vJTe_right.slice(nCamParamBase, NUMTRACKERCAMPARAMETERS) =
	vJTe_right.slice(nCamParamBase, NUMTRACKERCAMPARAMETERS) + EAJ_right.m2NCameraJac.T() * EAJ_right.v2Error;
	    
      dSumSquaredError_right += EAJ_right.v2Error * EAJ_right.v2Error;
      ++nTotalMeas_right;

      }
  };

  mdMeanPixelError_RIGHT = sqrt(dSumSquaredError_right / nTotalMeas_right);
  
  SVD<> svd_right(mJTJ_right);
  Vector<> vUpdate_right(nDim);
  vUpdate_right = svd_right.backsub(vJTe_right);
  //solves mJTJ*vUpdate = vJTe -. vUpdate = mJTJ^-1 *vJTe 
  vUpdate_right *= 0.1; // Slow down because highly nonlinear...
  for (int n = 0; n < nViews; n++){
    mvCalibImgs[n].second.mse3CamFromWorld = SE3<>::exp(vUpdate_right.slice(n * 6, 6)) * mvCalibImgs[n].second.mse3CamFromWorld;
  }
  mCamera_Right.UpdateParams(vUpdate_right.slice(nCamParamBase, NUMTRACKERCAMPARAMETERS));
}

/* written by Max */
void CameraCalibrator::CalculateExtrinsic(SE3<> &se3){
  
  static gvar3<string> gvsEstimator("MEstimator","Tukey",SILENT);
  float LastError = 999999.9;
  Vector<2> v2Error;
  // perform 35 iterations of gauss-newton minimisation
  ofstream pose("pose.txt");
  for(int i=0;i<35;i++)
    {
      pose << "Update " << i << endl
	   << "####################" << endl;
      SE3<> inv_se3 = se3.inverse();
      vector<double> vdErrorSquared;
      vector<Matrix<2,6> > vmJacs;
      vector<Vector<2> > vvCovariance;
      int NoImages = (int)mvCalibImgs.size();
      
      for(int n=0;n<NoImages;n++)
	{
	  if((int)mvCalibImgs[n].first.mvGridCorners.size() + (int)mvCalibImgs[n].second.mvGridCorners.size() < 2*X_GRID*Y_GRID)
	    continue;
	  
	  CalibImage &Left = mvCalibImgs[n].first;
	  CalibImage &Right = mvCalibImgs[n].second;
	
	  for(int j=0;j<(X_GRID*Y_GRID);j++)
	    {
	      CalibGridCorner &LeftCorner = Left.mvGridCorners[j];
	      CalibGridCorner &RightCorner = Right.mvGridCorners[j];
	      
	      //the world position of the point in object space: (x,y,0)
	      Vector<3> WorldPos;
	      WorldPos.slice(0,2) = vec(LeftCorner.irGridPos);
	      WorldPos[2] = 0.0;
	    
	      //the coordinate of the point in the reference frame of the left camera,
	      //the right camera, the pixel coordinates of the projected point and the covariance
	      Vector<3> v3LeftFrame = Left.mse3CamFromWorld*WorldPos;
	      Vector<3> v3RightFrame = se3*v3LeftFrame;
	      Vector<2> v2RightProj = mCamera_Right.Project(project(v3RightFrame));
	      Vector<2> Covariance = (RightCorner.Params.v2Pos - v2RightProj);
	      
	      //find the camera jacobian: (du/dmu, dv/dmu)
	      Matrix<2,6> m26CamJacobian;
	      double dOneOverCamZ = 1.0/v3RightFrame[2];
	      for(int m=0;m<6;m++)
		{
		  const Vector<4> v4Motion = SE3<>::generator_field(m,unproject(v3LeftFrame));
		  Vector<2> v2CamFrameMotion;
		  v2CamFrameMotion[0] = (v4Motion[0] - v3RightFrame[0]*v4Motion[2]*dOneOverCamZ)*dOneOverCamZ;
		  v2CamFrameMotion[1] = (v4Motion[1] - v3RightFrame[1]*v4Motion[2]*dOneOverCamZ)*dOneOverCamZ;
		  m26CamJacobian.T()[m] = mCamera_Right.GetProjectionDerivs()*v2CamFrameMotion;
		}
	    
	      //add the errors to the error vectors
	      vmJacs.push_back(m26CamJacobian);
	      vdErrorSquared.push_back(Covariance*Covariance);
	      vvCovariance.push_back(Covariance);
	      pose << Covariance << endl;
	      
	    } // end for all corners
	} //end for all images 
      
      double dSigmaSquared = Tukey::FindSigmaSquared(vdErrorSquared);
      WLS<6> wls;
      wls.add_prior(300.0);
      
      int GoodPoints = (int)vvCovariance.size();
      
      for(int f = 0;f<GoodPoints;f++)
	{

	  Vector<2> &v2 = vvCovariance[f];
	  double dErrorSq = v2*v2;
	  double dWeight = Tukey::Weight(dErrorSq,dSigmaSquared);
	  
	  Matrix<2,6> &m26Jacs = vmJacs[f];
	  
	  wls.add_mJ(v2[0], m26Jacs[0], dWeight);
	  wls.add_mJ(v2[1], m26Jacs[1], dWeight);
	}

      wls.compute();
	  
      v2Error = Zeros;
      for(int a=0;a<GoodPoints;a++)
	for(int b=0;b<2;b++)
	  v2Error[b] += sqrt((vvCovariance[a])[b]*(vvCovariance[a])[b]);

      v2Error[0] /= NoImages*X_GRID*Y_GRID;
      v2Error[1] /= NoImages*X_GRID*Y_GRID;
      
      //if((v2Error*v2Error) > LastError)
      // break;
      LastError = v2Error*v2Error;
      se3 = SE3<>::exp(wls.get_mu())*se3; 
      //cout << "error after run " << i << " is: " << sqrt(LastError) << endl;
      pose << endl;
    }
  cout << "Mean pixel error is " << v2Error << endl;
  pose.close();
}








