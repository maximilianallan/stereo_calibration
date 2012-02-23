// -*- c++ -*-
// Copyright 2008 Isis Innovation Limited

#ifndef __CALIB_IMAGE_H
#define __CALIB_IMAGE_H
#include "ATANCamera.h"
#include "CalibCornerPatch.h"
#include "GLWindow2.h"
#include <vector>
#include <TooN/se3.h>
#include "constants.h"
//#define X_GRID 9
//#define Y_GRID 7
const int N_NOT_TRIED=-1;
const int N_FAILED=-2;

struct CalibGridCorner
{
  struct NeighborState
  {
    NeighborState() {val = N_NOT_TRIED;}
    int val;
  };
  
  CalibCornerPatch::Params Params;
  CVD::ImageRef irGridPos;
  NeighborState aNeighborStates[4];
  Matrix<2> GetSteps(std::vector<CalibGridCorner> &vgc); 
  Matrix<2> mInheritedSteps;
  
  void Draw();
  
  double ExpansionPotential();
};

class CalibImage
{
public:
 
  bool MakeFromImage(CVD::Image<CVD::byte> &im, GLWindow2 &w);
  SE3<> mse3CamFromWorld;
  void DrawImageGrid();
  void Draw3DGrid(ATANCamera &Camera, bool bDrawErrors);
  void GuessInitialPose(ATANCamera &Camera);
  inline void setGridCorners(){
    mvGridCorners = orderedGridCorners;
  }
  //CalibImage();
  struct ErrorAndJacobians
  {
    Vector<2> v2Error;
    Matrix<2,6> m26PoseJac;
    Matrix<2,NUMTRACKERCAMPARAMETERS> m2NCameraJac;
  };
  bool goodGrid;
  std::vector<ErrorAndJacobians> Project(ATANCamera &Camera);
  CVD::Image<CVD::byte> mim;
  friend void eraseBadPoints(CalibImage &, CalibImage &);
  std::vector<CalibGridCorner> mvGridCorners; // all the grid corners in the image
  std::vector<CalibGridCorner> orderedGridCorners;
protected:
  std::vector<CVD::ImageRef> mvCorners; //all the corners in the image
 
  bool reorderPoints(Vector<2> x, Vector<2> y, Vector<2> x_2, CVD::ImageRef c);
  void reorderPoints2(Vector<2> x, Vector<2> y, Vector<2> x_2, CVD::ImageRef c);
  
  bool ExpandByAngle(int nSrc, int nDirn);
  int NextToExpand();
  void ExpandByStep(int n);
  CVD::ImageRef IR_from_dirn(int nDirn);
  
 
};




#endif

