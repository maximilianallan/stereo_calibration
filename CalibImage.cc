// Copyright 2008 Isis Innovation Limited
#include "OpenGL.h"
#include "CameraCalibrator.h"
#include "CalibImage.h"
#include <stdlib.h>
#include <gvars3/instances.h>
#include <cvd/utility.h>
#include <cvd/convolution.h>
#include <cvd/fast_corner.h>
#include <cvd/vector_image_ref.h>
#include <cvd/image_interpolate.h>

#include <TooN/se3.h>
#include <TooN/SVD.h>
#include <TooN/wls.h>

using namespace std;
using namespace CVD;
using namespace GVars3;

inline bool IsCorner(Image<byte> &im, ImageRef ir, int nGate)
{ // Does a quick check to see if a point in an image could be a grid corner.
  // Does this by going around a 16-pixel ring, and checking that there's four
  // transitions (black - white- black - white - )
  // Also checks that the central pixel is blurred.

  // Find the mean intensity of the pixel ring...
  int nSum = 0;
  static byte abPixels[16];
  for(int i=0; i<16; i++)
    {
      abPixels[i] = im[ir + fast_pixel_ring[i]];
      //fast_pixel_ring returns the 16 pixels around ir in an octagon
      nSum += abPixels[i];
    };
  int nMean = nSum / 16;
  int nHiThresh = nMean + nGate;
  int nLoThresh = nMean - nGate;

  // If the center pixel is roughly the same as the mean, this isn't a corner.
  int nCenter = im[ir];
  if(nCenter <= nLoThresh || nCenter >= nHiThresh)
    return false;
  
  // Count transitions around the ring... there should be four!
  // a chessboard corner at ir (or very very close to it) would result in 4 orthogonal lines moving away from ir rather than two or zero for edges of 
  // squares or middle of squares respectively
  bool bState = (abPixels[15] > nMean);
  int nSwaps = 0;
  for(int i=0; i<16; i++)
    {
      byte bValNow = abPixels[i];
      if(bState)
	{
	  if(bValNow < nLoThresh)
	    {
	      bState = false;
	      nSwaps++;
	    }
	}
      else
	if(bValNow > nHiThresh)
	  {
	    bState = true;
	    nSwaps++;
	  };
    }
  return (nSwaps == 4);
};


//determines if a point ref is inside the drawn grid. vectors a,b,c create two triangles out of the quadrilateral
bool inside(CVD::ImageRef &ref, Vector<2> &a, Vector<2> &b, Vector<2> &c){
  Vector<2> r;
  r[0] = ref.x;
  r[1] = ref.y;
  r -= c;
  float alpha = ((b*b)*(r*a) - (a*b)*(r*b))/((a*a)*(b*b) - (a*b)*(a*b));
  float beta = ((r*b) - alpha*(a*b))/(b*b);
  if(alpha < 1 && beta < 1 && alpha > 0 && beta > 0 && (alpha+beta)<1)
    return true;
  else return false;
}

//sorts the points in the vector by their distance from the start point (represented by second element in the float array)
// pair<int A, float *B> where A is the index in the vector of corner points and float[0] is the dot product that point has with 
// the vector running parallel to the edge of the quad and float[1] is the length of the line from the start of that vector and the
// point of intest
void sort_by_length(std::vector<std::pair<int,float*> > &ordered){
  //shortest first!
  std::vector<std::pair<int,float*> > tmp;
  tmp.reserve(ordered.size());
  tmp.push_back(ordered.front());
  for(int n=1;n<(int)ordered.size();n++)
    {
      for(vector<std::pair<int,float*> >::iterator m=tmp.begin();m!=tmp.end();m++)
	{
	  if((*m).second[1]>=ordered[n].second[1]){
	    tmp.insert(m,ordered[n]);
	    break;
	  }else{
	    if(*m==tmp.back()){
	      tmp.push_back(ordered[n]);
	      break;
	    }
	  }
	}
    }
  ordered = tmp;
}
	  
//adds one of these pairs to the vector at index defined by last
void add_to_ordered(std::vector<std::pair<int,float*> > &ordered,double dot, float len, float index, int last){
  std::pair<int,float*> tmp;// = new pair<int,float*>;
  tmp.first = index;
  float *array = new float[2];
  array[0] = dot;
  array[1] = len;
  tmp.second = array;
  if(last == -1){
    ordered.push_back(tmp);
  }else{
    vector<pair<int,float*> >::iterator it = ordered.begin();
    for(int n=0;n<last;n++)
      it++;
    ordered.insert(it,tmp);
  }
}
//by calculating dot product of the point to grid origin and the line from grid origin parallel to the quadrilateral side
//create an ordered list of the points from largest dot to smallest - ignore negatives
void check_insert(double dot,std::vector<std::pair<int,float*> > &ordered, float len, int index){
  if(dot < 0 || dot != dot)
    return;
  if((int)ordered.empty()){
    add_to_ordered(ordered,dot,len,index,-1);
    return;
  }

  if(dot <= ordered.back().second[0]){
    add_to_ordered(ordered,dot,len,index,-1);
    return;
  }

  for(int n=0;n<(int)ordered.size();n++){
    if(dot < ordered[n].second[0]){
      continue;
    }else{
      add_to_ordered(ordered,dot,len,index,n);
      return;
    }
  }
}

Vector<2> GuessInitialAngles(Image<byte> &im, ImageRef irCenter)
{
  // The iterative patch-finder works better if the initial guess
  // is roughly aligned! Find one of the line-axes by searching round 
  // the circle for the strongest gradient, and use that and +90deg as the
  // initial guesses for patch angle.
  //
  // Yes, this is a very poor estimate, but it's generally (hopefully?) 
  // enough for the iterative finder to converge.
  
  image_interpolate<Interpolate::Bilinear, byte> imInterp(im);
  double dBestAngle = 0;
  double dBestGradMag = 0;
  double dGradAtBest = 0;
  for(double dAngle = 0.0; dAngle < M_PI; dAngle += 0.1)
    {
      Vector<2> v2Dirn;
      v2Dirn[0] = cos(dAngle);      v2Dirn[1] = sin(dAngle);
      Vector<2> v2Perp;
      v2Perp[1] = -v2Dirn[0];      v2Perp[0] = v2Dirn[1];
      
      double dG =     imInterp[vec(irCenter) + v2Dirn * 3.0 + v2Perp * 0.1] - 
	              imInterp[vec(irCenter) + v2Dirn * 3.0 - v2Perp * 0.1]
	       +      imInterp[vec(irCenter) - v2Dirn * 3.0 - v2Perp * 0.1] - 
		      imInterp[vec(irCenter) - v2Dirn * 3.0 + v2Perp * 0.1];
      if(fabs(dG) > dBestGradMag)
	{
	  dBestGradMag = fabs(dG);
	  dGradAtBest = dG;
	  dBestAngle = dAngle;
	};
    }
  
  Vector<2> v2Ret;
  if(dGradAtBest < 0)
    {   v2Ret[0] = dBestAngle; v2Ret[1] = dBestAngle + M_PI / 2.0;    }
  else
    {   v2Ret[1] = dBestAngle; v2Ret[0] = dBestAngle - M_PI / 2.0;    }
  return v2Ret;
}

bool CalibImage::MakeFromImage(Image<byte> &im, GLWindow2 &glWindow)
{
  static gvar3<int> gvnCornerPatchSize("CameraCalibrator.CornerPatchPixelSize", 20, SILENT);
  mvCorners.clear();
  mvGridCorners.clear();
  
  mim = im;
  mim.make_unique(); 
  //rather than being a reference to a shared image, make a quick copy.
  int number_of_corners = 0;
  vector<CVD::ImageRef> corner_of_grid;
  corner_of_grid.reserve(4);
  while(number_of_corners < 4){
    glWindow.HandlePendingEvents();
    if(glWindow.wasClicked)
      {
	corner_of_grid.push_back(glWindow.getClick());
	number_of_corners++;
      }
  }
  //Draw the bounding box around the chessboard
  glPointSize(3);
  glColor3f(1,1,0);
  glBegin(GL_LINE_LOOP);
  for(int n=0;n<4;n++)
    glVertex2i(corner_of_grid[n].x,corner_of_grid[n].y);
  glEnd();
  glWindow.swap_buffers();
  // Find potential corners..
  // This works better on a blurred image, so make a blurred copy
  // and run the corner finding on that.
  Vector<2> to_basis;
  Vector<2> x_basis;
  Vector<2> y_basis;
  Vector<2> to_basis_2;
  Vector<2> x_basis_2;
  Vector<2> y_basis_2;
  {
    Image<byte> imBlurred = mim;
    imBlurred.make_unique();
    convolveGaussian(imBlurred, GV2.GetDouble("CameraCalibrator.BlurSigma", 1.0, SILENT));
    
    to_basis[0] = corner_of_grid[0].x;
    to_basis[1] = corner_of_grid[0].y;
    x_basis[0] = x_basis_2[0] = corner_of_grid[1].x;
    x_basis[1] = x_basis_2[1] = corner_of_grid[1].y;
    to_basis_2[0] = corner_of_grid[2].x;
    to_basis_2[1] = corner_of_grid[2].y;
    y_basis[0] = y_basis_2[0] = corner_of_grid[3].x;
    y_basis[1] = y_basis_2[1] = corner_of_grid[3].y;
    
    
    x_basis_2 = x_basis_2 - to_basis_2;
    y_basis_2 = y_basis_2 - to_basis_2;
    
    x_basis = x_basis - to_basis;
    y_basis = y_basis - to_basis;
    
    ImageRef topLeft(5,5);
    ImageRef botRight = mim.size() - topLeft;
    ImageRef ir = topLeft;
    glPointSize(4);
    glColor3f(1,0,1);
    glBegin(GL_POINTS);
    int nGate = GV2.GetInt("CameraCalibrator.MeanGate", 10, SILENT);
    mvCorners.reserve(50);
    do
      if(inside(ir,x_basis,y_basis,to_basis) || inside(ir,x_basis_2,y_basis_2,to_basis_2))
	{
	  if(IsCorner(imBlurred, ir, nGate))
	    {
	      //glColor3f(1,0,1);
	      mvCorners.push_back(ir);
	      glVertex(ir);
	    }
	}
    while(ir.next(topLeft,botRight));
    glEnd();
  }
  //glWindow.HandlePendingEvents();
  glWindow.swap_buffers();
   
  ImageRef irCenterOfImage = mim.size()/2;
  ImageRef irCornerOfGrid;
  unsigned int nBestDistSquared = 99999999;
  for(unsigned int i=0; i<mvCorners.size(); i++)
    {
      unsigned int nDist = (mvCorners[i] - irCenterOfImage).mag_squared();
      if(nDist < nBestDistSquared)
	{
	  nBestDistSquared = nDist;
	  irCornerOfGrid = mvCorners[i];
	}
    }
 
  // ... and try to fit a corner-patch to that.
  CalibCornerPatch Patch(*gvnCornerPatchSize); //create a 20x20(pixels) patch
  CalibCornerPatch::Params Params; //create a params struct: warp matrix, pos, angles..
  Params.v2Pos = vec(irCornerOfGrid);
  Params.v2Angles = GuessInitialAngles(mim,irCornerOfGrid); 
  //warp is set to a 2x2 matrix of cos and sin of v2angles
  Params.dGain = 80.0;
  Params.dMean = 120.0;
  //this modifies the patch params to fit it with viewed patch
  if(!Patch.IterateOnImageWithDrawing(Params, mim)) //mim is blurry image
    {
      //cout << "Failed in iterateonImagewithdrawing 1" << endl;
      return false;
    }
  // The first found corner patch becomes the origin of the detected grid.
  CalibGridCorner cFirst;
  cFirst.Params = Params;
  mvGridCorners.push_back(cFirst);
  cFirst.Draw();
  
  // Next, go in two compass directions from the origin patch, and see if 
  // neighbors can be found. expand by angle creates a source gridcorner
  // then creates a direction vector from the number (0,1,2,3) and a row 
  // (actually column as this is transpose) so this is the basis vector -
  // the warping matrix 0 and 1 given first and second row respectively, 2
  // and 3 give the same vector * -1. then the size of the vector between the
  // source of the grid and each corner is calculated, less than 10 ignore, bigger
  // than some large value, ignore. if angle between this vector and the target
  // direction is too low also ignore - eliminates really accute surface orient
  
  if(!(ExpandByAngle(0,0) || ExpandByAngle(0,2)))
    {
      //cout << "Failed on expand by angle 1" << endl;
      return false;
    }
  if(!(ExpandByAngle(0,1) || ExpandByAngle(0,3)))
    {
      //cout << "Failed on expand by angle 2" << endl;
      return false;
    }      
  //sets which directions neighbours are in with vectors - includes lengths!
  mvGridCorners[1].mInheritedSteps = mvGridCorners[2].mInheritedSteps = mvGridCorners[0].GetSteps(mvGridCorners);

  // The three initial grid elements are enough to find the rest of the grid.
  int nNext;
  int nSanityCounter = 0; // Stop it getting stuck in an infinite loop...
  const int nSanityCounterLimit = 500;
  while((nNext = NextToExpand()) >= 0 && nSanityCounter < nSanityCounterLimit )
    {
      /*NextToExpand finds best grid corner number. this allows scaling of the 
	direction vector in expand by step.
       */
      ExpandByStep(nNext);
      nSanityCounter++;
    }
  if(nSanityCounter == nSanityCounterLimit)
    {
      //cout << "Failed on expanding grid " << endl;
      return false;
    }
  
  if((int)mvGridCorners.size() == Y_GRID*X_GRID)
    reorderPoints2(x_basis, y_basis, x_basis_2, corner_of_grid[0]);
  //else
  //for(int j=0;j<(int)mvGridCorners.size();j++)
  //mvGridCorners[j].irGridPos *= GRID_SQUARE_SIZE;
  DrawImageGrid();
  return true;
}
  
void eraseBadPoints(CalibImage &a, CalibImage &b){
  float difference =0;
  float mean_difference = difference/(int)a.mvGridCorners.size();
  for(int i=0;i<(int)a.mvGridCorners.size();i++){
    if(sqrt((a.mvGridCorners[i].Params.v2Pos - b.mvGridCorners[i].Params.v2Pos)*(a.mvGridCorners[i].Params.v2Pos - b.mvGridCorners[i].Params.v2Pos)) > 2*mean_difference)
      {
	(a.mvGridCorners).erase(a.mvGridCorners.begin() + i);
	(b.mvGridCorners).erase(b.mvGridCorners.begin()+ i);
	cout << "erased the " << i << "th point " << endl;
      }
  }
  
}

void sortByHeight(vector<pair<CalibGridCorner,double> > &ord){
  vector<pair<CalibGridCorner, double> > tmp;
  tmp.reserve(ord.size());
  ord.erase(ord.begin()+Y_GRID,ord.end());
  tmp.push_back(ord.front());
  for(int n=1;n<(int)ord.size();n++)
    {
      for(vector<pair<CalibGridCorner,double> >::iterator m=tmp.begin();m!=tmp.end();m++)
	{
	  if((*m).first.Params.v2Pos[1]>=ord[n].first.Params.v2Pos[1]){
	    tmp.insert(m,ord[n]);
	    break;
	  }else{
	    if(((*m).first.Params.v2Pos)==(tmp.back().first.Params.v2Pos)){
	      tmp.push_back(ord[n]);
	      break;
	    }
	  }
	}
    }
  ord = tmp;
  
}
bool ordered(CalibGridCorner &x, std::vector<CalibGridCorner> &oc){
  for(int i=0;i<(int)oc.size();i++){
    Vector<2> temp = x.Params.v2Pos - oc[i].Params.v2Pos;
    //if(x.Params.v2Pos == oc[i].Params.v2Pos)
    //  return true;
    if(sqrt(temp*temp < 8))
      return true;
  }
  return false;
}

double minDistance(const Vector<2> &point, const Vector<2> &line){
  Vector<2> point_norm = point;
  Vector<2> line_norm = line;
  normalize(point_norm);
  normalize(line_norm);
  double angle = acos(point_norm*line_norm);
  angle = M_PI/2 - angle;
  double a = sqrt(point*point)*cos(angle);
  if(a < 10) return 999999;
  else return a;
}

void CalibImage::reorderPoints2(Vector<2> x, Vector<2> y, Vector <2>x_2, ImageRef corner){

  float bestDistance = 9999.0;
  std::vector<CalibGridCorner> orderedCorners;
  orderedCorners.reserve(mvGridCorners.size());
  orderedCorners.push_back(mvGridCorners[0]);
  for(int i=0; i< (int)mvGridCorners.size(); i++){
    CVD::ImageRef ir;
    ir.x = mvGridCorners[i].Params.v2Pos[0];
    ir.y = mvGridCorners[i].Params.v2Pos[1];
    float nDist = sqrt( (ir.x - corner.x)*(ir.x - corner.x) + (ir.y - corner.y)*(ir.y - corner.y) );
    if(nDist < bestDistance)
      {
	bestDistance = nDist;
	orderedCorners.front() = mvGridCorners[i];
      }
  }
  
  //now (hopefully) orderedCorners[0] has the "top left" corner of the grid
  
  normalize(x);
  normalize(y);
  
  std::vector<std::pair<int,float*> >orderedColumn;
  orderedColumn.reserve(Y_GRID);
  for(int k=0;k<(int)mvGridCorners.size();k++){
    Vector<2> yNorm = mvGridCorners[k].Params.v2Pos - orderedCorners.back().Params.v2Pos;
    float len = sqrt(yNorm*yNorm);
    normalize(yNorm);
    check_insert((yNorm*y),orderedColumn,len,k);
  }
  
  orderedColumn.erase(orderedColumn.begin()+(Y_GRID - 1),orderedColumn.end());
  
  //sort the points in order of how far they are from the top left
  sort_by_length(orderedColumn);
  vector<pair<CalibGridCorner,double> > newOrderedColumn;
  newOrderedColumn.reserve(mvGridCorners.size());
  pair<CalibGridCorner,double> tmp(orderedCorners.front(),0.0);
  newOrderedColumn.push_back(tmp);
  orderedCorners.clear();
  for(int i=0;i<(int)orderedColumn.size();i++){
    pair<CalibGridCorner,double> tmp1 (mvGridCorners[orderedColumn[i].first],0.0);
    newOrderedColumn.push_back(tmp1);
  }
  //newOrderedColumn now contains the grid square corners
  
  //make the grid go from left to right!
  
  for(int i=1;i<X_GRID;i++){
    Vector<2> column = newOrderedColumn.back().first.Params.v2Pos - newOrderedColumn.front().first.Params.v2Pos;
    Vector<2> origin = newOrderedColumn.front().first.Params.v2Pos;
    for(int c=0;c<(int)newOrderedColumn.size();c++){
      orderedCorners.push_back(newOrderedColumn[c].first);
      orderedCorners.back().irGridPos = CVD::ImageRef(i-1,c);
    }
    //add all corners to final grid
    
    newOrderedColumn.clear();
    CalibGridCorner fakeCorner;
    //fakeCorner.Params.v2Pos = Vector<2> tmp(99999,99999);
    //pair<CalibGridCorner,double> first_point (mvGridCorners[0],minDistance( (mvGridCorners[0].Params.v2Pos - origin), column));
    pair<CalibGridCorner,double> first_point (fakeCorner,99999);
    newOrderedColumn.push_back(first_point);
    for(int k=0;k<(int)mvGridCorners.size();k++){
      if(ordered(mvGridCorners[k],orderedCorners))
	continue;
      double dist = minDistance( (mvGridCorners[k].Params.v2Pos - origin) ,column);
      pair<CalibGridCorner,double>a_pair(mvGridCorners[k],dist);
      //  if(newOrderedColumn.empty()){
      //	newOrderedColumn.push_back(a_pair);
      //	continue;
      //}
      for(vector<pair<CalibGridCorner,double> >::iterator m=newOrderedColumn.begin();m!=newOrderedColumn.end();m++){
	if( dist <= (*m).second ){
	  newOrderedColumn.insert(m,a_pair);
	  break;
	}else{
	  if( (*m).first.Params.v2Pos == newOrderedColumn.back().first.Params.v2Pos){
	    newOrderedColumn.push_back(a_pair);
	    break;
	  }
	}
      }
    }
    //now have a vector newOrderedColum of all points from 
    //smallest distance to largest distance
    sortByHeight(newOrderedColumn);
    
  }
  //add the points from the last loop
  for(int c=0;c<(int)newOrderedColumn.size();c++)
      {
	orderedCorners.push_back(newOrderedColumn[c].first);
	orderedCorners.back().irGridPos = CVD::ImageRef(X_GRID-1,c);
      }
  //glLineWidth(2);
  //glColor3f(0,0,1);
  //glEnable(GL_LINE_SMOOTH);
  //glEnable(GL_BLEND);
  // glBegin(GL_LINE_LOOP);
  
  // for(int i=0;i<(int)orderedCorners.size();i++)
  // {
  //    glVertex(orderedCorners[i].Params.v2Pos);
  //  }
  //glEnd();
  
  for(int c=0;c<(int)orderedCorners.size();c++)
     {
       //if not in the last column
       if(orderedCorners[c].irGridPos.x != (X_GRID-1))
	 orderedCorners[c].aNeighborStates[0].val = c+Y_GRID;
       else
	 orderedCorners[c].aNeighborStates[0].val = N_FAILED;
       //if not in the top row
       if(orderedCorners[c].irGridPos.y != 0)
	 orderedCorners[c].aNeighborStates[1].val = c-1;
       else
	 orderedCorners[c].aNeighborStates[1].val = N_FAILED;
       //of not in the first column
       if(orderedCorners[c].irGridPos.x != 0)
	 orderedCorners[c].aNeighborStates[2].val = c-Y_GRID;
       else
	 orderedCorners[c].aNeighborStates[2].val = N_FAILED;
       //if not in the bottom row
       if(orderedCorners[c].irGridPos.y != Y_GRID-1)
	 orderedCorners[c].aNeighborStates[3].val = c+1;
       else
	 orderedCorners[c].aNeighborStates[3].val = N_FAILED;
     }
   
  mvGridCorners = orderedGridCorners = orderedCorners;
}


bool CalibImage::reorderPoints(Vector<2> x, Vector<2> y, Vector<2> x_2, ImageRef corner){
  float bestDistance = 9999.0;
  std::vector<CalibGridCorner> orderedCorners;
  orderedCorners.push_back(mvGridCorners[0]);
  for(int i=0; i< (int)mvGridCorners.size(); i++){
    CVD::ImageRef ir;
    ir.x = mvGridCorners[i].Params.v2Pos[0];
    ir.y = mvGridCorners[i].Params.v2Pos[1];
    float nDist = sqrt( (ir.x - corner.x)*(ir.x - corner.x) + (ir.y - corner.y)*(ir.y - corner.y) );
    if(nDist < bestDistance)
      {
	bestDistance = nDist;
	orderedCorners.front() = mvGridCorners[i];
      }
  }
  // cout << "the top corner of the grid is at " << orderedCorners.front().Params.v2Pos[0] << "," << orderedCorners.front().Params.v2Pos[1] << endl;
  //orderedCorners.front().irGridPos = CVD::ImageRef(0,0);
    //now (hopefully) orderedCorners[0] has the "top left" corner of the grid
  
  normalize(x);
  normalize(y);
  
  std::vector<std::pair<int,float*> >orderedColumn;
  for(int k=0;k<(int)mvGridCorners.size();k++){
    Vector<2> yNorm = mvGridCorners[k].Params.v2Pos - orderedCorners.back().Params.v2Pos;
    float len = sqrt(yNorm*yNorm);
    normalize(yNorm);
    check_insert((yNorm*y),orderedColumn,len,k);
  }
  
  orderedColumn.erase(orderedColumn.begin()+(Y_GRID - 1),orderedColumn.end());
  //sort the points in order of how far they are from the top left
  sort_by_length(orderedColumn);
  
  std::vector<std::pair<int,float*> >orderedRow;
  for(int r=0;r<Y_GRID;r++){
    if(r!=0) //the first "start" point is already added!
      orderedCorners.push_back(mvGridCorners[orderedColumn[r-1].first]);
    orderedCorners.back().irGridPos = CVD::ImageRef(0,r);
    //insert element of orderedColumn into orderedCOrners
    orderedRow.clear();
    	//iterate over columns
    for(int i=0;i<(int)mvGridCorners.size();i++)
      {
	//search all the grid corners
	if(ordered(mvGridCorners[i],orderedCorners))
	  { 
	    continue;
	  }
	Vector<2> xNorm = mvGridCorners[i].Params.v2Pos - orderedCorners.back().Params.v2Pos; 
	//ordered corners.back() gives the "first" 
	//point in the row - inserted at start of for loop
	double len= sqrt(xNorm*xNorm);
	normalize(xNorm);
	check_insert((xNorm*x),orderedRow,len,i);
      }	  
    orderedRow.erase(orderedRow.begin()+(X_GRID-1),orderedRow.end());
    sort_by_length(orderedRow);
    if(r > (int)Y_GRID/2){
      x = x_2; 
      normalize(x);
    }
    for(int c=0;c<(int)orderedRow.size();c++){
      orderedCorners.push_back(mvGridCorners[orderedRow[c].first]);
      orderedCorners.back().irGridPos = CVD::ImageRef(c+1,r);
    }
  }
  if(orderedCorners.size() != mvGridCorners.size())
    {
      cout << "Didn't get all points..." << orderedCorners.size() << " and " << mvGridCorners.size() << endl;
      return false;
    }
  glLineWidth(2);
  glColor3f(0,0,1);
  glEnable(GL_LINE_SMOOTH);
  glEnable(GL_BLEND);
  glBegin(GL_LINE_LOOP);
  
  for(int i=0;i<(int)orderedCorners.size();i++)
    {
      glVertex(orderedCorners[i].Params.v2Pos);
    }
  glEnd();
  mvGridCorners = orderedCorners;
  return true;
}

bool CalibImage::ExpandByAngle(int nSrc, int nDirn)
{
  static gvar3<int> gvnCornerPatchSize("CameraCalibrator.CornerPatchPixelSize", 20, SILENT);
  CalibGridCorner &gSrc = mvGridCorners[nSrc];
  
  ImageRef irBest;
  double dBestDist = 99999;
  Vector<2> v2TargetDirn = gSrc.Params.m2Warp().T()[nDirn%2];
  if(nDirn >= 2)
    v2TargetDirn *= -1;
  //mvCorners is a std::vector of image refs
  // finds most suitable image corner for 
  // start point of next patch
  for(unsigned int i=0; i<mvCorners.size(); i++)
    {
      Vector<2> v2Diff = vec(mvCorners[i]) - gSrc.Params.v2Pos;
      if(v2Diff * v2Diff < 100)
	continue;
      if(v2Diff * v2Diff > dBestDist * dBestDist)
	continue;
      Vector<2> v2Dirn = v2Diff;
      normalize(v2Dirn);
      if(v2Dirn * v2TargetDirn < cos(M_PI / 18.0))
	continue;
      dBestDist = sqrt(v2Diff * v2Diff);
      irBest = mvCorners[i];
    }
  
  CalibGridCorner gTarget;
  gTarget.Params = gSrc.Params;
  gTarget.Params.v2Pos = vec(irBest);
  gTarget.Params.dGain *= -1; //should be "opposite" colour than current source patch
  
  CalibCornerPatch Patch(*gvnCornerPatchSize);
  if(!Patch.IterateOnImageWithDrawing(gTarget.Params, mim))
    {
      gSrc.aNeighborStates[nDirn].val = N_FAILED;
      return false;
    }

  gTarget.irGridPos = gSrc.irGridPos;
  if(nDirn < 2)
    gTarget.irGridPos[nDirn]++;
  else gTarget.irGridPos[nDirn%2]--;
  // Update connection states:
  mvGridCorners.push_back(gTarget); // n.b. This invalidates gSrc!
  mvGridCorners.back().aNeighborStates[(nDirn + 2) % 4].val = nSrc;
  mvGridCorners[nSrc].aNeighborStates[nDirn].val = mvGridCorners.size() - 1;
  
  mvGridCorners.back().Draw();
  return true;
}


void CalibGridCorner::Draw()
{
  glColor3f(0,1,0);
  glEnable(GL_LINE_SMOOTH);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glBegin(GL_LINES);
  glVertex(Params.v2Pos + Params.m2Warp() * vec(ImageRef( 10,0)));
  glVertex(Params.v2Pos + Params.m2Warp() * vec(ImageRef(-10,0)));
  glVertex(Params.v2Pos + Params.m2Warp() * vec(ImageRef( 0, 10)));
  glVertex(Params.v2Pos + Params.m2Warp() * vec(ImageRef( 0,-10)));
  glEnd();
}


double CalibGridCorner::ExpansionPotential()
{
  // Scoring function. How good would this grid corner be at finding a neighbor?
  // The best case is if it's already surrounded by three neighbors and only needs
  // to find the last one (because it'll have the most accurate guess for where
  // the last one should be) and so on.
  int nMissing = 0;
  for(int i=0; i<4; i++)
    if(aNeighborStates[i].val == N_NOT_TRIED)
      nMissing++;

  if(nMissing == 0)
    return 0.0;
  
  if(nMissing == 1)
    return 100.0;
  
  if(nMissing == 3)
    return 1.0;

  if(nMissing == 2)
    {
      int nFirst = 0;
      while(aNeighborStates[nFirst].val != N_NOT_TRIED)
	nFirst++;
      if(aNeighborStates[(nFirst + 2) % 4].val == N_NOT_TRIED)
	return 10.0;
      else
	return 20.0;
    }
  assert(0); // should never get here
  return 0.0;
};


Matrix<2> CalibGridCorner::GetSteps(vector<CalibGridCorner> &vgc)
{
  Matrix<2> m2Steps;
  for(int dirn=0; dirn<2; dirn++)
    {
      Vector<2> v2Dirn;
      int nFound = 0;
      v2Dirn = Zeros;
      if(aNeighborStates[dirn].val >=0)
	{
	  v2Dirn += vgc[aNeighborStates[dirn].val].Params.v2Pos - Params.v2Pos;
	  nFound++;
	}
      if(aNeighborStates[dirn+2].val >=0)
	{
	  v2Dirn -= vgc[aNeighborStates[dirn+2].val].Params.v2Pos - Params.v2Pos;
	  nFound++;
	}
      if(nFound == 0)
	m2Steps[dirn] = mInheritedSteps[dirn];
      else
	m2Steps[dirn] = v2Dirn / nFound;
    }
  return m2Steps;
};

int CalibImage::NextToExpand()
{
  int nBest = -1;
  double dBest = 0.0;
  
  for(unsigned int i=0; i<mvGridCorners.size(); i++)
    {
      double d = mvGridCorners[i].ExpansionPotential();
      if(d > dBest)
	{
	  nBest = i;
	  dBest = d;
	}
    }
  return nBest;
}
/*
  For each element in the mvGridCorners array after the first
  3 have been found, expand by step.
  
  for the nth grid corner, each neighbour state is checked, 
  right,up,left,down and the first one which is found which has
  not been tried, but the one on the opposite side of the
  corner has been found then it will check it. if no
  neighbour point has been found then one of the 4 non checked
  neighbours is used.

  then an ir_from_dirn is found for the direction of travel. 
  this is just an image ref that is (1,0) for a left step, 
  (0,-1) for a down step etc.

  
 


*/


void CalibImage::ExpandByStep(int n)
{
  static gvar3<double> gvdMaxStepDistFraction("CameraCalibrator.ExpandByStepMaxDistFrac", 0.3, SILENT);
  static gvar3<int> gvnCornerPatchSize("CameraCalibrator.CornerPatchPixelSize", 20, SILENT);
  
  CalibGridCorner &gSrc = mvGridCorners[n];
  
  // First, choose which direction to expand in...
  // Ideally, choose a dirn for which the Step calc is good!
  int nDirn = -10;
  for(int i=0; nDirn == -10 && i<4; i++)
    {
      if(gSrc.aNeighborStates[i].val == N_NOT_TRIED &&
	 gSrc.aNeighborStates[(i+2) % 4].val >= 0)
	nDirn = i;
    }
  if(nDirn == -10)
  for(int i=0; nDirn == -10 && i<4; i++)
    {
      if(gSrc.aNeighborStates[i].val == N_NOT_TRIED)
	nDirn = i;
    }
  assert(nDirn != -10);

  Vector<2> v2Step;
  ImageRef irGridStep = IR_from_dirn(nDirn);
  
  v2Step = gSrc.GetSteps(mvGridCorners).T() * vec(irGridStep);
  
  Vector<2> v2SearchPos = gSrc.Params.v2Pos + v2Step;
  
  // Before the search: pre-fill the failure result for easy returns.
  gSrc.aNeighborStates[nDirn].val = N_FAILED;
  
  ImageRef irBest;
  double dBestDist = 99999;
  for(unsigned int i=0; i<mvCorners.size(); i++)
    {
      Vector<2> v2Diff = vec(mvCorners[i]) - v2SearchPos;
      if(v2Diff * v2Diff > dBestDist * dBestDist)
	continue;
      dBestDist = sqrt(v2Diff * v2Diff);
      irBest = mvCorners[i];
    }
  
  double dStepDist= sqrt(v2Step * v2Step);
  if(dBestDist > *gvdMaxStepDistFraction * dStepDist)
    return;
  
  CalibGridCorner gTarget;
  gTarget.Params = gSrc.Params;
  gTarget.Params.v2Pos = vec(irBest);
  gTarget.Params.dGain *= -1;
  gTarget.irGridPos = gSrc.irGridPos + irGridStep;
  gTarget.mInheritedSteps = gSrc.GetSteps(mvGridCorners);
  CalibCornerPatch Patch(*gvnCornerPatchSize);
  if(!Patch.IterateOnImageWithDrawing(gTarget.Params, mim))
    return;
  
  // Update connection states:
  int nTargetNum = mvGridCorners.size();
  for(int dirn = 0; dirn<4; dirn++)
    {
      ImageRef irSearch = gTarget.irGridPos + IR_from_dirn(dirn);
      for(unsigned int i=0; i<mvGridCorners.size(); i++)
	if(mvGridCorners[i].irGridPos == irSearch)
	  {
	    gTarget.aNeighborStates[dirn].val = i;
	    mvGridCorners[i].aNeighborStates[(dirn + 2) % 4].val = nTargetNum;
	  }
    }
  mvGridCorners.push_back(gTarget);
  mvGridCorners.back().Draw();
}

void CalibImage::DrawImageGrid()
{
  glLineWidth(2);
  glColor3f(0,0,1);
  glEnable(GL_LINE_SMOOTH);
  glEnable(GL_BLEND);
  glBegin(GL_LINES);
  
  for(int i=0; i< (int) mvGridCorners.size(); i++)
    {
      for(int dirn=0; dirn<4; dirn++)
	if(mvGridCorners[i].aNeighborStates[dirn].val > i)
	  {
	    glVertex(mvGridCorners[i].Params.v2Pos);
	    glVertex(mvGridCorners[mvGridCorners[i].aNeighborStates[dirn].val].Params.v2Pos);
	  }
    }
  glEnd();
  
  glPointSize(5);
  glEnable(GL_POINT_SMOOTH);
  glColor3f(1,1,0);
  glBegin(GL_POINTS);
  for(unsigned int i=0; i<mvGridCorners.size(); i++)
    glVertex(mvGridCorners[i].Params.v2Pos);
  glEnd();
};

void CalibImage::Draw3DGrid(ATANCamera &Camera, bool bDrawErrors)
{
  //if(side == Right)
  //  glRasterPos2i(720,0);
  glLineWidth(2);
  glColor3f(0,0,1);
  glEnable(GL_LINE_SMOOTH);
  glEnable(GL_BLEND);
  glBegin(GL_LINES);
  //if(goodGrid)
  //mvGridCorners = orderedGridCorners;
  for(int i=0; i< (int) mvGridCorners.size(); i++)
    {
      for(int dirn=0; dirn<4; dirn++)
	if(mvGridCorners[i].aNeighborStates[dirn].val > i)
	  {
	    Vector<3> v3; v3[2] = 0.0;
	    v3.slice<0,2>() = vec(mvGridCorners[i].irGridPos);
		//project() projects a homogenous vector down to a non-homogenous one
	    glVertex(Camera.Project(project(mse3CamFromWorld * v3)));
		//Cam.Proj() will take a point in the world coordinate frame (given by
		// via an image reference) and give the 3D coords of where cam will see 
	    v3.slice<0,2>() = vec(mvGridCorners[mvGridCorners[i].aNeighborStates[dirn].val].irGridPos);
	    glVertex(Camera.Project(project(mse3CamFromWorld * v3)));
	  }
    }
  glEnd();

  if(bDrawErrors)
    {
      //if(side == Right)
      //glRasterPos2i(720,0);
      glColor3f(1,0,0);
      glLineWidth(1);
      glBegin(GL_LINES);
      for(int i=0; i< (int) mvGridCorners.size(); i++)
	{
	  Vector<3> v3; v3[2] = 0.0;
	  v3.slice<0,2>() = vec(mvGridCorners[i].irGridPos);
	  Vector<2> v2Pixels_Projected = Camera.Project(project(mse3CamFromWorld * v3));
	  Vector<2> v2Error = mvGridCorners[i].Params.v2Pos - v2Pixels_Projected;
	  glVertex(v2Pixels_Projected);
	  glVertex(v2Pixels_Projected + 10.0 * v2Error);
	}
      glEnd();
    }
};

ImageRef CalibImage::IR_from_dirn(int nDirn)
{
  ImageRef ir;
  ir[nDirn%2] = (nDirn < 2) ? 1: -1;
  return ir;
}


void CalibImage::GuessInitialPose(ATANCamera &Camera)
{
  // First, find a homography which maps the grid (x,y)
  // (e.g. 0,0 for origin 0,1 ...to the unprojected 
  //image coords (u,v)
  // Use the standard null-space-of-SVD-thing to find 9 homography parms
  // (c.f. appendix of thesis)
  //std::vector<CalibGridCorner> *corners = &mvGridCorners;
  //if(goodGrid)
  //  mvGridCorners = orderedGridCorners;
    
  int nPoints = mvGridCorners.size();
  Matrix<> m2Nx9(2*nPoints, 9);
  for(int n=0; n<nPoints; n++)
    {
      // First, un-project the points to the image plane
      Vector<2> v2UnProj = Camera.UnProject(mvGridCorners[n].Params.v2Pos);
      double u = v2UnProj[0];
      double v = v2UnProj[1];
      //cout << "u = " << u << " and v = " << v << endl;
      // Then fill in the matrix..
      double x = mvGridCorners[n].irGridPos.x;
      double y = mvGridCorners[n].irGridPos.y;
      //cout << "x = " << x << " and y = " << y << endl;
      
      m2Nx9[n*2+0][0] = x;
      m2Nx9[n*2+0][1] = y;
      m2Nx9[n*2+0][2] = 1;
      m2Nx9[n*2+0][3] = 0;
      m2Nx9[n*2+0][4] = 0;
      m2Nx9[n*2+0][5] = 0;
      m2Nx9[n*2+0][6] = -x*u;
      m2Nx9[n*2+0][7] = -y*u;
      m2Nx9[n*2+0][8] = -u;

      m2Nx9[n*2+1][0] = 0;
      m2Nx9[n*2+1][1] = 0;
      m2Nx9[n*2+1][2] = 0;
      m2Nx9[n*2+1][3] = x;
      m2Nx9[n*2+1][4] = y;
      m2Nx9[n*2+1][5] = 1;
      m2Nx9[n*2+1][6] = -x*v;
      m2Nx9[n*2+1][7] = -y*v;
      m2Nx9[n*2+1][8] = -v;
    }

  // The right null-space (should only be one) of the matrix gives the homography...
  SVD<> svdHomography(m2Nx9);
  Vector<9> vH = svdHomography.get_VT()[8];
  Matrix<3> m3Homography;
  m3Homography[0] = vH.slice<0,3>();
  m3Homography[1] = vH.slice<3,3>();
  m3Homography[2] = vH.slice<6,3>();
  
  
  // Fix up possibly poorly conditioned bits of the homography
  //eq d13 in thesis
  {
    SVD<2> svdTopLeftBit(m3Homography.slice<0,0,2,2>());
    Vector<2> v2Diagonal = svdTopLeftBit.get_diagonal();
    m3Homography = m3Homography / v2Diagonal[0];
    v2Diagonal = v2Diagonal / v2Diagonal[0];
    double dLambda2 = v2Diagonal[1];
    
    Vector<2> v2b;   // This is one hypothesis for v2b ; the other is the negative.
    v2b[0] = 0.0;
    v2b[1] = sqrt( 1.0 - (dLambda2 * dLambda2)); 
    
    Vector<2> v2aprime = v2b * svdTopLeftBit.get_VT();
    
    Vector<2> v2a = m3Homography[2].slice<0,2>();
    double dDotProd = v2a * v2aprime;
    
    if(dDotProd>0) 
      m3Homography[2].slice<0,2>() = v2aprime;
    else
      m3Homography[2].slice<0,2>() = -v2aprime;
  }
 
  
  //OK, now turn homography into something 3D ...simple gram-schmidt ortho-norm
  // Take 3x3 matrix H with column: abt
  // And add a new 3rd column: abct
  Matrix<3> mRotation;
  Vector<3> vTranslation;
  double dMag1 = sqrt(m3Homography.T()[0] * m3Homography.T()[0]);
  m3Homography = m3Homography / dMag1;
  
  mRotation.T()[0] = m3Homography.T()[0];
  
  // ( all components of the first vector are removed from the second...
  
  mRotation.T()[1] = m3Homography.T()[1] - m3Homography.T()[0]*(m3Homography.T()[0]*m3Homography.T()[1]); 
  mRotation.T()[1] /= sqrt(mRotation.T()[1] * mRotation.T()[1]);
  mRotation.T()[2] = mRotation.T()[0]^mRotation.T()[1];
  vTranslation = m3Homography.T()[2];
  
  // Store result
  mse3CamFromWorld.get_rotation()=mRotation; 
  mse3CamFromWorld.get_translation() = vTranslation;
};

vector<CalibImage::ErrorAndJacobians> CalibImage::Project(ATANCamera &Camera)
{
  //std::vector<CalibGridCorner> *corners = &mvGridCorners;
  //if(goodGrid)
  //  corners = &orderedGridCorners;
  vector<ErrorAndJacobians> vResult;
  for(unsigned int n=0; n<mvGridCorners.size(); n++)
    {
      ErrorAndJacobians EAJ;
      
      // First, project into image...
      Vector<3> v3World;
      v3World[2] = 0.0;
      v3World.slice<0,2>() = vec(mvGridCorners[n].irGridPos);
      /* take the grid position of each grid corner that was found and rotate+translate it to get it from ((0,1)...) object 
	 coordinates to world coordinates. then project it into the image
       */
      Vector<3> v3Cam = mse3CamFromWorld * v3World;
      // puts the vector in world coordinate frame
      if(v3Cam[2] <= 0.001)
	continue;
      
      Vector<2> v2Image = Camera.Project(project(v3Cam));
      if(Camera.Invalid())
	continue;
      
      EAJ.v2Error = mvGridCorners[n].Params.v2Pos - v2Image;
      
      // Now find motion jacobian..
      double dOneOverCameraZ = 1.0 / v3Cam[2];
      Matrix<2> m2CamDerivs = Camera.GetProjectionDerivs();
      
      for(int dof=0; dof<6; dof++)
	{
	  const Vector<4> v4Motion = SE3<>::generator_field(dof, unproject(v3Cam)); //returns the dof th generator * unproject(
	  
	  Vector<2> v2CamFrameMotion;
	  v2CamFrameMotion[0] = (v4Motion[0] - v3Cam[0] * v4Motion[2] * dOneOverCameraZ) * dOneOverCameraZ;
	  v2CamFrameMotion[1] = (v4Motion[1] - v3Cam[1] * v4Motion[2] * dOneOverCameraZ) * dOneOverCameraZ;
	  EAJ.m26PoseJac.T()[dof] = m2CamDerivs * v2CamFrameMotion;
	};

      // Finally, the camera provids its own jacobian
      EAJ.m2NCameraJac = Camera.GetCameraParameterDerivs();
      vResult.push_back(EAJ);
    }
  return vResult;
};







