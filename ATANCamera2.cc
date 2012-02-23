// Copyright 2008 Isis Innovation Limited
#include "ATANCamera.h"
#include <TooN/helpers.h>
#include <cvd/vector_image_ref.h>
#include <iostream>
#include <gvars3/instances.h>
using namespace std;
using namespace CVD;
using namespace GVars3;

ATANCamera::ATANCamera(string sName)
{
  // The camera name is used to find the camera's parameters in a GVar.
  msName = sName;
  GV2.Register(mgvvCameraParams, 
	       sName+".Parameters", 
	       mvDefaultParams, 
	       HIDDEN | FATAL_IF_NOT_DEFINED);
  GV2.Register(mgvvSE3Parameters_Vector,
	       sName+"_Extrinsic.Parameters",
	       mvDefaultExtrinsicVector , 
	       HIDDEN|FATAL_IF_NOT_DEFINED);
  /*GV2.Register(mgvvCVCameraMatrix,
	       sName+"_cvMatrix.Parameters",
	       mvDefaultcvCameraMatrix,
	       HIDDEN|FATAL_IF_NOT_DEFINED);
  GV2.Register(mgvvRectifyRotate,
	       sName+"_cvRectifyRotate.Parameters",
	       mvDefaultRectifyRotate,
	       HIDDEN|FATAL_IF_NOT_DEFINED);
  GV2.Register(mgvvRectifyProject,
	       sName+"_cvRectifyProject.Parameters",
	       mvDefaultRectifyProject,
	       HIDDEN|FATAL_IF_NOT_DEFINED);
  GV2.Register(mgvvRectifyReProject,
	       sName+"_cvRectifyReProject.Parameters",
	       mvDefaultRectifyReProject,
	       HIDDEN|FATAL_IF_NOT_DEFINED);
  GV2.Register(mgvvCvDistortion,
	       sName+"_cvDistortion.Parameters",
	       mvDefaultcvDistortion,
	       HIDDEN|FATAL_IF_NOT_DEFINED);
  GV2.Register(mgvvCvRectangle,
	       sName+"_cvRectangle.Parameters",
	       mvDefaultcvRectangle,
	       HIDDEN|FATAL_IF_NOT_DEFINED);
  */
  mvImageSize[0] = OPENCV_VIDEO_W;
  mvImageSize[1] = OPENCV_VIDEO_H;
  /* 
  //needs to be done before RefreshParams call
  cvDistortion = cvCreateMat(8,1,CV_64F);
  cvRectifyRotate = cvCreateMat(3,3,CV_64F);
  cvRectifyProject = cvCreateMat(3,4,CV_64F);
  cvRectifyReProject = cvCreateMat(4,4,CV_64F);
  cvCameraMatrix = cvCreateMat(3,3,CV_64F);
  cvRectangle = new CvRect;
  cvR = cvCreateMat(3,3,CV_64F);
  cvT = cvCreateMat(3,1,CV_64F);*/
  RefreshParams();
  /*  RefreshCvParams();
  
  map1 = new cv::Mat(OPENCV_VIDEO_H,
		     OPENCV_VIDEO_W,
		     CV_32FC1);
  map2 = new cv::Mat(OPENCV_VIDEO_H,
  		     OPENCV_VIDEO_W,
		     CV_32FC1);*/
}
//delete openCV heap data. not done with destructor 
//as this is invoked whenever bundleAdjustor deletes
//the copy of the main camera it makes...
//FIX suggestions: smart pointers? gc? 
//at the moment deleted at system.cc:195 and 
//calibrator.cc
void ATANCamera::deleteCv(){
  cvReleaseMat(&cvRectifyRotate);
  cvReleaseMat(&cvRectifyProject);
  cvReleaseMat(&cvRectifyReProject);
  cvReleaseMat(&cvCameraMatrix);
  cvReleaseMat(&cvR);
  cvReleaseMat(&cvDistortion);
  cvReleaseMat(&cvT);
  delete map1;
  delete map2;
  delete cvRectangle;
}

void ATANCamera::setcvMatrixToIntrinsic(){
  cvmSet(cvCameraMatrix,0,0,getFocal(0));
  cvmSet(cvCameraMatrix,0,1,0);
  cvmSet(cvCameraMatrix,0,2,mvCenter[0]);
  cvmSet(cvCameraMatrix,1,0,0);
  cvmSet(cvCameraMatrix,1,1,getFocal(1));
  cvmSet(cvCameraMatrix,1,2,mvCenter[1]);
  cvmSet(cvCameraMatrix,2,0,0);
  cvmSet(cvCameraMatrix,2,1,0);
  cvmSet(cvCameraMatrix,2,2,1);
}

void ATANCamera::SetParametersToRectified(){
  cout << "still need to remove this " << endl;
  mvFocal[0] = mvFocal[1] = cvmGet(cvRectifyProject,0,0);
  mvInvFocal[0] = 1/mvFocal[0];
  
  mvInvFocal[1] = 1/mvFocal[1];
  
  mvCenter[0] = cvmGet(cvRectifyProject,0,2);
  
  mvCenter[1] = cvmGet(cvRectifyProject,1,2);
  mdW = 0;
  md2Tan = 0;
  mdWinv = 0;
  mdDistortionEnabled = 0.0;

  Vector<2> v2;
  double prX_N = mvCenter[0]/mvImageSize[0];
  double prY_N = mvCenter[1]/mvImageSize[1];
  double fcX_N = mvFocal[0]/mvImageSize[0];
  double fcY_N = mvFocal[1]/mvImageSize[1];
  v2[0]= max(prX_N, 1.0 - prX_N) / fcX_N;
  v2[1]= max(prY_N, 1.0 - prY_N) / fcY_N;
  mdLargestRadius = invrtrans(sqrt(v2*v2));
  
  // At what stage does the model become invalid?
  mdMaxR = 1.5 * mdLargestRadius; // (pretty arbitrary)

  // work out world radius of one pixel
  // (This only really makes sense for square-ish pixels)
  {
    Vector<2> v2Center = UnProject(mvImageSize / 2);
    Vector<2> v2RootTwoAway = UnProject(mvImageSize / 2 + vec(ImageRef(1,1)));
    Vector<2> v2Diff = v2Center - v2RootTwoAway;
    mdOnePixelDist = sqrt(v2Diff * v2Diff) / sqrt(2.0);
  }
  
  // Work out the linear projection values for the UFB
  {
    // First: Find out how big the linear bounding rectangle must be
    vector<Vector<2> > vv2Verts;
    vv2Verts.push_back(UnProject(makeVector( -0.5, -0.5)));
    vv2Verts.push_back(UnProject(makeVector( mvImageSize[0]-0.5, -0.5)));
    vv2Verts.push_back(UnProject(makeVector( mvImageSize[0]-0.5, mvImageSize[1]-0.5)));
    vv2Verts.push_back(UnProject(makeVector( -0.5, mvImageSize[1]-0.5)));
    Vector<2> v2Min = vv2Verts[0];
    Vector<2> v2Max = vv2Verts[0];
    for(int i=0; i<4; i++)
      for(int j=0; j<2; j++)
	{
	  if(vv2Verts[i][j] < v2Min[j]) v2Min[j] = vv2Verts[i][j];
	  if(vv2Verts[i][j] > v2Max[j]) v2Max[j] = vv2Verts[i][j];
	}
    mvImplaneTL = v2Min;
    mvImplaneBR = v2Max;
    
    // Store projection parameters to fill this bounding box
    Vector<2> v2Range = v2Max - v2Min;
    mvUFBLinearInvFocal = v2Range;
    mvUFBLinearFocal[0] = 1.0 / mvUFBLinearInvFocal[0];
    mvUFBLinearFocal[1] = 1.0 / mvUFBLinearInvFocal[1];
    mvUFBLinearCenter[0] = -1.0 * v2Min[0] * mvUFBLinearFocal[0];
    mvUFBLinearCenter[1] = -1.0 * v2Min[1] * mvUFBLinearFocal[1];
  }

  for(int i=0;i<4;i++)
    for(int j=0;j<4;j++)
      mvReProjectMatrix(i,j) = cvmGet(cvRectifyReProject,i,j);
  for(int i=0;i<3;i++)
    for(int j=0;j<4;j++)
      mvProjectMatrix(i,j) = cvmGet(cvRectifyProject,i,j);
}

//rectifed project takes a 3 vector map point and returns a 2d image points
// for the camera in question.
Vector<2> ATANCamera::RectifiedProject(const Vector<3> pointCoord){
  
  Vector<3> tmp = pointCoord;
  normalizeLast(tmp);
  mvLastCam = tmp.slice(0,2);
  mdLastR = sqrt(mvLastCam * mvLastCam);
  mbInvalid = (mdLastR > mdMaxR);
  mdLastFactor = 1;
  mdLastDistR = mdLastR; //there is no distortion now 
  mvLastDistCam = mvLastCam; //aim to remove these 3 vars ...

  float _outputcv[3];
  CvMat outputcv = cvMat(3,1,CV_64FC1,_outputcv);
  cvMatMul(&outputcv,cvRectifyProject,&outputcv);
  Vector<3> output;
  for(int i=0;i<3;i++) output[i] = _outputcv[i];
  normalizeLast(output);
  mvLastIm[0] = output[0];
  mvLastIm[1] = output[1];
  return mvLastIm;
}
//rectified unproject takes a point in the left image and projects in into
//a 3 vector representing the point's location in 3d space.
Vector<3> ATANCamera::RectifiedUnProjectToWorld(const Vector<2> imgPoint, float disparity){
  if(disparity < 0)
    disparity *= -1;
  //modify x,y to x-c_x and y-c_y
  mvLastIm = imgPoint;
  Vector<4> inV;
  inV.slice(0,2) = imgPoint;
  inV[2] = disparity;
  inV[3] = 1;
  Vector<4> outV = mvReProjectMatrix*inV;
  
  Vector<4> outV3 = normalizeLast(outV);
  if(outV3[3] != 1)
    cout << "ERROR - normalizeLast is not doing its job"<< endl;
  mvLastDistCam[0] = mvLastCam[0] = outV3[0]/outV3[2];
  mvLastDistCam[1] = mvLastCam[1] = outV3[1]/outV3[2];
  mdLastDistR = sqrt(mvLastDistCam * mvLastDistCam);
  mdLastR = mdLastDistR;
  mvLastWorld = outV3.slice(0,3);
  return mvLastWorld;
}

//a z=1 version of above for 1 camera unproject
Vector<2> ATANCamera::RectifiedUnProject(const Vector<2> imPoint){
  cout << "This function should not be used!!!" << endl;
  float invF = cvmGet(cvRectifyProject,0,0);
  float cX = cvmGet(cvRectifyProject,0,2);
  float cY = cvmGet(cvRectifyProject,1,1);
  Vector<2> returnV;
  returnV[0] = (imPoint[0] - cX)*invF;
  returnV[1] = (imPoint[1] - cY)*invF;
  mvLastDistCam[0] = mvLastCam[0] = returnV[0];
  mvLastDistCam[1] = mvLastCam[1] = returnV[1];
  mdLastDistR = sqrt(mvLastDistCam * mvLastDistCam);
  mdLastR = mdLastDistR;
  return returnV;
}

Vector<3> ATANCamera::RectifiedUnProjectToWorld(const ImageRef imPoint, float disparity){
  //Vector<2> imp;
  //imp[0] = imPoint.x;
  //imp[1] = imPoint.y;
  return RectifiedUnProjectToWorld(vec(imPoint),disparity);
}
Vector<2> ATANCamera::RectifiedUnProject(const ImageRef imPoint){
  cout << "this function should not be used!" << endl;
  Vector<2> imP;
  imP[0] = imPoint.x;
  imP[1] = imPoint.y;
  return RectifiedUnProject(imP);
}
void ATANCamera::SetImageSize(Vector<2> vImageSize)
{
  mvImageSize = vImageSize;
  RefreshParams();
};

void ATANCamera::RefreshParams() 
{
  // This updates internal member variables according to the current camera parameters,
  // and the currently selected target image size.
  //
  
  // First: Focal length and image center in pixel coordinates
  mvFocal[0] = mvImageSize[0] * (*mgvvCameraParams)[0];
  mvFocal[1] = mvImageSize[1] * (*mgvvCameraParams)[1];
  mvCenter[0] = mvImageSize[0] * (*mgvvCameraParams)[2] - 0.5;
  mvCenter[1] = mvImageSize[1] * (*mgvvCameraParams)[3] - 0.5;
  
  // One over focal length
  mvInvFocal[0] = 1.0 / mvFocal[0];
  mvInvFocal[1] = 1.0 / mvFocal[1];

  // Some radial distortion parameters..
  mdW =  (*mgvvCameraParams)[4];
  if(mdW != 0.0)
    {
      md2Tan = 2.0 * tan(mdW / 2.0);
      mdOneOver2Tan = 1.0 / md2Tan;
      mdWinv = 1.0 / mdW;
      mdDistortionEnabled = 1.0;
    }
  else
    {
      mdWinv = 0.0;
      md2Tan = 0.0;
      mdDistortionEnabled = 0.0;
    }
  
  // work out biggest radius in image
  Vector<2> v2;
  v2[0]= max((*mgvvCameraParams)[2], 1.0 - (*mgvvCameraParams)[2]) / (*mgvvCameraParams)[0];
  v2[1]= max((*mgvvCameraParams)[3], 1.0 - (*mgvvCameraParams)[3]) / (*mgvvCameraParams)[1];
  mdLargestRadius = invrtrans(sqrt(v2*v2));
  
  // At what stage does the model become invalid?
  mdMaxR = 1.5 * mdLargestRadius; // (pretty arbitrary)

  // work out world radius of one pixel
  // (This only really makes sense for square-ish pixels)
  {
    Vector<2> v2Center = UnProject(mvImageSize / 2);
    Vector<2> v2RootTwoAway = UnProject(mvImageSize / 2 + vec(ImageRef(1,1)));
    Vector<2> v2Diff = v2Center - v2RootTwoAway;
    mdOnePixelDist = sqrt(v2Diff * v2Diff) / sqrt(2.0);
  }
  
  // Work out the linear projection values for the UFB
  {
    // First: Find out how big the linear bounding rectangle must be
    vector<Vector<2> > vv2Verts;
    vv2Verts.push_back(UnProject(makeVector( -0.5, -0.5)));
    vv2Verts.push_back(UnProject(makeVector( mvImageSize[0]-0.5, -0.5)));
    vv2Verts.push_back(UnProject(makeVector( mvImageSize[0]-0.5, mvImageSize[1]-0.5)));
    vv2Verts.push_back(UnProject(makeVector( -0.5, mvImageSize[1]-0.5)));
    Vector<2> v2Min = vv2Verts[0];
    Vector<2> v2Max = vv2Verts[0];
    for(int i=0; i<4; i++)
      for(int j=0; j<2; j++)
	{
	  if(vv2Verts[i][j] < v2Min[j]) v2Min[j] = vv2Verts[i][j];
	  if(vv2Verts[i][j] > v2Max[j]) v2Max[j] = vv2Verts[i][j];
	}
    mvImplaneTL = v2Min;
    mvImplaneBR = v2Max;
    
    // Store projection parameters to fill this bounding box
    Vector<2> v2Range = v2Max - v2Min;
    mvUFBLinearInvFocal = v2Range;
    mvUFBLinearFocal[0] = 1.0 / mvUFBLinearInvFocal[0];
    mvUFBLinearFocal[1] = 1.0 / mvUFBLinearInvFocal[1];
    mvUFBLinearCenter[0] = -1.0 * v2Min[0] * mvUFBLinearFocal[0];
    mvUFBLinearCenter[1] = -1.0 * v2Min[1] * mvUFBLinearFocal[1];
  }
  Extrinsic = SE3<>::exp(*mgvvSE3Parameters_Vector);

  mvCameraMatrix(0,0) = mvFocal[0];
  mvCameraMatrix(0,1) = mvCameraMatrix(1,0) = mvCameraMatrix(2,0) = mvCameraMatrix(2,1) = 0;
  mvCameraMatrix(0,2) = mvCenter[0];
  mvCameraMatrix(1,1) = mvFocal[1];
  mvCameraMatrix(1,2) = mvCenter[1];
  mvCameraMatrix(2,2) = 1;
}

//sets the openCV parameters to the gvars variables - with 
//exception of extrinsic params: set from member var
void ATANCamera::RefreshCvParams(){
  cvRectangle->x = (*mgvvCvRectangle)[0];
  cvRectangle->y = (*mgvvCvRectangle)[1];
  cvRectangle->width = (*mgvvCvRectangle)[2];
  cvRectangle->height = (*mgvvCvRectangle)[3];
  for(int i=0;i<8;i++)
    cvmSet(cvDistortion,i,0,(*mgvvCvDistortion)[i]);
  for(int i=0;i<9;i++)
    cvmSet(cvRectifyRotate, (int)i/3, i%3, 
    	   (*mgvvRectifyRotate)[i]);
  for(int i=0;i<3;i++){
    for(int j=0;j<4;j++){
      cvmSet(cvRectifyProject,i,j,
	   (*mgvvRectifyProject)[i*4+j]);
    }
  }
  for(int i=0;i<16;i++)
    cvmSet(cvRectifyReProject, (int)i/4, i%4,
	   (*mgvvRectifyReProject)[i]);
  for(int i=0;i<3;i++)
    for(int j=0;j<3;j++)
      cvmSet(cvCameraMatrix,i,j,(*mgvvCVCameraMatrix)[(i*3)+j]);
  for(int i=0;i<3;i++){
    cvmSet(cvT,i,0,Extrinsic.get_translation()[i]);
    for(int j=0;j<3;j++){
      cvmSet(cvR,i,j,Extrinsic.get_rotation().get_matrix()[i][j]);
    }
  }

}

//sets the 4 Gvars: cam matrix, R, P, Q associated with 
//openCV and each camera to whatever the member variables
//are set to.
void ATANCamera::UpdateCvParams(const CvMat &R, const CvMat &P,
				const CvMat &Q){
  cvRectifyRotate = cvCloneMat(&R);
  cvRectifyProject = cvCloneMat(&P);
  cvRectifyReProject = cvCloneMat(&Q);
  for(int k=0;k<8;k++)
    (*mgvvCvDistortion)[k] = cvmGet(cvDistortion,k,0);
  for(int i=0;i<9;i++){
    (*mgvvRectifyRotate)[i] = cvmGet(cvRectifyRotate,
				     (int)i/3,i%3);
    (*mgvvCVCameraMatrix)[i] = cvmGet(cvCameraMatrix,(int)i/3,i%3);
  }
  for(int i=0;i<3;i++){
    for(int j=0;j<4;j++)
      (*mgvvRectifyProject)[(i*4)+j] = 
	cvmGet(cvRectifyProject,i,j);
    
  }
  
  for(int i=0;i<16;i++)
    (*mgvvRectifyReProject)[i] = cvmGet(cvRectifyReProject,
    (int)i/4,i%4);

  (*mgvvCvRectangle)[0] = cvRectangle->x;
  (*mgvvCvRectangle)[1] = cvRectangle->y;
  (*mgvvCvRectangle)[2] = cvRectangle->width;
  (*mgvvCvRectangle)[3] = cvRectangle->height;
}

// Project from the camera z=1 plane to image pixels,
// while storing intermediate calculation results in member variables
Vector<2> ATANCamera::Project(const Vector<2>& vCam){
  mvLastCam = vCam;
  mdLastR = sqrt(vCam * vCam); 
  mbInvalid = (mdLastR > mdMaxR);
  mdLastFactor = rtrans_factor(mdLastR); //gives r'/r from pinhole camera model
  mdLastDistR = mdLastFactor * mdLastR;
  mvLastDistCam = mdLastFactor * mvLastCam;
  
  mvLastIm[0] = mvCenter[0] + mvFocal[0] * mvLastDistCam[0];
  mvLastIm[1] = mvCenter[1] + mvFocal[1] * mvLastDistCam[1];
  
  return mvLastIm;
}


// Un-project from image pixel coords to the camera z=1 plane
// while storing intermediate calculation results in member variables

Vector<2> ATANCamera::UnProject(const Vector<2>& v2Im)
{
  mvLastIm = v2Im;
  mvLastDistCam[0] = (mvLastIm[0]-mvCenter[0]) * mvInvFocal[0];
  mvLastDistCam[1] = (mvLastIm[1]-mvCenter[1]) * mvInvFocal[1];
    
 //subtracts principle points and divides by focal lengths
  mdLastDistR = sqrt(mvLastDistCam * mvLastDistCam);
  mdLastR = invrtrans(mdLastDistR);

  double dFactor;
  if(mdLastDistR > 0.01){
    dFactor =  mdLastR / mdLastDistR;
  }else{
    dFactor = 1.0;
  }
  mdLastFactor = 1.0 / dFactor;
  mvLastCam = dFactor * mvLastDistCam;
  return mvLastCam;
}

// Utility function for easy drawing with OpenGL
// C.f. comment in top of ATANCamera.h
Matrix<4> ATANCamera::MakeUFBLinearFrustumMatrix(double near, double far)
{
  Matrix<4> m4 = Zeros;
  

  double left = mvImplaneTL[0] * near;
  double right = mvImplaneBR[0] * near;
  double top = mvImplaneTL[1] * near;
  double bottom = mvImplaneBR[1] * near;
  
  // The openGhelL frustum manpage is A PACK OF LIES!!
  // Two of the elements are NOT what the manpage says they should be.
  // Anyway, below code makes a frustum projection matrix
  // Which projects a RHS-coord frame with +z in front of the camera
  // Which is what I usually want, instead of glFrustum's LHS, -z idea.
  m4[0][0] = (2 * near) / (right - left);
  m4[1][1] = (2 * near) / (top - bottom);
  
  m4[0][2] = (right + left) / (left - right);
  m4[1][2] = (top + bottom) / (bottom - top);
  m4[2][2] = (far + near) / (far - near);
  m4[3][2] = 1;
  
  m4[2][3] = 2*near*far / (near - far);

  return m4;
};

Matrix<2,2> ATANCamera::GetProjectionDerivs()
{
  // get the derivative of image frame wrt camera z=1 frame at the last computed projection
  // in the form (d im1/d cam1, d im1/d cam2)
  //             (d im2/d cam1, d im2/d cam2)
  
  double dFracBydx;
  double dFracBydy;
  
  double &k = md2Tan; //2*tan(omega/2)
  double &x = mvLastCam[0];
  //get last z = 1 coords from last point projection in mvLast.
  double &y = mvLastCam[1];
  double r = mdLastR * mdDistortionEnabled;
  //if r is really small either v far away (z large) or 
  //x^2 + y^2 is really small... or both
  if(r < 0.01) 
    {
      dFracBydx = 0.0; //differential of
      dFracBydy = 0.0; //radial distortion
    }
  else
    {
      dFracBydx = 
	mdWinv * (k * x) / (r*r*(1 + k*k*r*r)) - x * mdLastFactor / (r*r); 
      dFracBydy = 
	mdWinv * (k * y) / (r*r*(1 + k*k*r*r)) - y * mdLastFactor / (r*r); 
    }
  
  Matrix<2> m2Derivs;
  
  m2Derivs[0][0] = mvFocal[0] * (dFracBydx * x + mdLastFactor);  
  m2Derivs[1][0] = mvFocal[1] * (dFracBydx * y);  
  m2Derivs[0][1] = mvFocal[0] * (dFracBydy * x);  
  m2Derivs[1][1] = mvFocal[1] * (dFracBydy * y + mdLastFactor);  
  return m2Derivs;
}

Matrix<2,NUMTRACKERCAMPARAMETERS> ATANCamera::GetCameraParameterDerivs()
{
  // Differentials wrt to the camera parameters
  // Use these to calibrate the camera
  // No need for this to be quick, so do them numerically
  
  Matrix<2, NUMTRACKERCAMPARAMETERS> m2NNumDerivs;
  Vector<NUMTRACKERCAMPARAMETERS> vNNormal = *mgvvCameraParams;
  Vector<2> v2Cam = mvLastCam;
  Vector<2> v2Out = Project(v2Cam);
  for(int i=0; i<NUMTRACKERCAMPARAMETERS; i++)
    {
      if(i == NUMTRACKERCAMPARAMETERS-1 && mdW == 0.0)
	continue;
      Vector<NUMTRACKERCAMPARAMETERS> vNUpdate;
      vNUpdate = Zeros;
      vNUpdate[i] += 0.001;
      UpdateParams(vNUpdate); 
      Vector<2> v2Out_B = Project(v2Cam);
      m2NNumDerivs.T()[i] = (v2Out_B - v2Out) / 0.001;
      *mgvvCameraParams = vNNormal;
      RefreshParams();
    }
  if(mdW == 0.0)
    m2NNumDerivs.T()[NUMTRACKERCAMPARAMETERS-1] = Zeros;
  return m2NNumDerivs;
}

void ATANCamera::UpdateParams(Vector<5> vUpdate)
{
  // Update the camera parameters; use this as part of camera calibration.
  (*mgvvCameraParams) = (*mgvvCameraParams) + vUpdate;
  RefreshParams();
}

void ATANCamera::UpdateExtrinsicParams(SE3<> extrinsic){
  (*mgvvSE3Parameters_Vector) = extrinsic.ln();
  Extrinsic = extrinsic;
  for(int i=0;i<3;i++){
    cvmSet(cvT,i,0,Extrinsic.get_translation()[i]);
    for(int j=0;j<3;j++){
      cvmSet(cvR,i,j,Extrinsic.get_rotation().get_matrix()[i][j]);
    }
  }
}

void ATANCamera::DisableRadialDistortion()
{
  // Set the radial distortion parameter to zero
  // This disables radial distortion and also disables its differentials
  (*mgvvCameraParams)[NUMTRACKERCAMPARAMETERS-1] = 0.0;
  RefreshParams();
}

Vector<2> ATANCamera::UFBProject(const Vector<2>& vCam)
{
  // Project from camera z=1 plane to UFB, storing intermediate calc results.
  mvLastCam = vCam;
  mdLastR = sqrt(vCam * vCam);
  mbInvalid = (mdLastR > mdMaxR);
  mdLastFactor = rtrans_factor(mdLastR);
  mdLastDistR = mdLastFactor * mdLastR;
  mvLastDistCam = mdLastFactor * mvLastCam;
  
  mvLastIm[0] = (*mgvvCameraParams)[2]  + (*mgvvCameraParams)[0] * mvLastDistCam[0];
  mvLastIm[1] = (*mgvvCameraParams)[3]  + (*mgvvCameraParams)[1] * mvLastDistCam[1];
  return mvLastIm;
}

Vector<2> ATANCamera::UFBUnProject(const Vector<2>& v2Im)
{
  mvLastIm = v2Im;
  mvLastDistCam[0] = (mvLastIm[0] - (*mgvvCameraParams)[2]) / (*mgvvCameraParams)[0];
  mvLastDistCam[1] = (mvLastIm[1] - (*mgvvCameraParams)[3]) / (*mgvvCameraParams)[1];
  mdLastDistR = sqrt(mvLastDistCam * mvLastDistCam);
  mdLastR = invrtrans(mdLastDistR);
  double dFactor;
  if(mdLastDistR > 0.01)
    dFactor =  mdLastR / mdLastDistR;
  else
    dFactor = 1.0;
  mdLastFactor = 1.0 / dFactor;
  mvLastCam = dFactor * mvLastDistCam;
  return mvLastCam;
}

CvMat *ATANCamera::getCvCameraMatrix(){
  return cvCameraMatrix;
}

CvMat *ATANCamera::getCvDistortion(){
  return cvDistortion;
}
CvMat *ATANCamera::getCvR(){
  return cvR;
}
CvMat *ATANCamera::getCvT(){
  return cvT;
}
CvMat *ATANCamera::getCvRectifyRotate(){
  return cvRectifyRotate;
}
CvMat *ATANCamera::getCvRectifyProject(){
  return cvRectifyProject;
}
CvMat *ATANCamera::getCvRectifyReProject(){
  return cvRectifyReProject;
}
cv::Mat *ATANCamera::getMap1(){
  return map1;
}
cv::Mat *ATANCamera::getMap2(){
  return map2;
}
CvRect *ATANCamera::getCvRectangle(){
  return cvRectangle;
}
const Vector<NUMTRACKERCAMPARAMETERS> ATANCamera::mvDefaultParams = makeVector(0.5, 0.75, 0.5, 0.5, 0.1);
//const SE3<> ATANCamera::mvDefaultSE3;
const Vector<6> ATANCamera::mvDefaultExtrinsicVector = makeVector(0.5,0.5,0.5,0,0,0);
const Vector<9> ATANCamera::mvDefaultRectifyRotate = makeVector(1,0,0,0,1,0,0,0,1);
const Vector<12> ATANCamera::mvDefaultRectifyProject = makeVector(1,0,0,0,0,1,0,0,0,0,1,0);
const Vector<16> ATANCamera::mvDefaultRectifyReProject = makeVector(1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1);
const Vector<9> ATANCamera::mvDefaultcvCameraMatrix = makeVector(1,0,0,0,1,0,0,0,1);
const Vector<8> ATANCamera::mvDefaultcvDistortion = makeVector(0,0,0,0,0,0,0,0);
const Vector<4> ATANCamera::mvDefaultcvRectangle = makeVector(0,0,0,0);
