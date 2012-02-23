#include "modifiedOpenCV.h"
#include <iostream>
#include <math.h>
//#include "ATANCamera.h"
using namespace std;
using namespace cv;

void new_stereoRectify(const CvMat* _cameraMatrix1, 
		       const CvMat* _cameraMatrix2,
		       const float _distCoeffs1, 
		       const float _distCoeffs2, 
		       CvSize imageSize, 
		       const CvMat* matR, 
		       const CvMat* matT,
		       CvMat* _R1, CvMat* _R2, 
		       CvMat* _P1, CvMat* _P2,
		       CvMat* matQ, int flags, 
		       double alpha, CvSize newImgSize,
		       CvRect* roi1, CvRect* roi2 )
{
  double _om[3], _t[3], _uu[3]={0,0,0}, _r_r[3][3], _pp[3][4];
  double _ww[3], _wr[3][3], _z[3] = {0,0,0}, _ri[3][3];
  cv::Rect_<float> inner1, inner2, outer1, outer2;
    
  CvMat om  = cvMat(3, 1, CV_64F, _om);
  CvMat t   = cvMat(3, 1, CV_64F, _t);
  CvMat uu  = cvMat(3, 1, CV_64F, _uu);
  CvMat r_r = cvMat(3, 3, CV_64F, _r_r);
  CvMat pp  = cvMat(3, 4, CV_64F, _pp);
  CvMat ww  = cvMat(3, 1, CV_64F, _ww); // temps
  CvMat wR  = cvMat(3, 3, CV_64F, _wr);
  CvMat Z   = cvMat(3, 1, CV_64F, _z);
  CvMat Ri  = cvMat(3, 3, CV_64F, _ri);
  double nx = imageSize.width, ny = imageSize.height;
  int i, k;
  
  if( matR->rows == 3 && matR->cols == 3 )
    cvRodrigues2(matR, &om);          // get vector rotation
  else
    cvConvert(matR, &om); // it's already a rotation vector
  cvConvertScale(&om, &om, -0.5); // get average rotation

  cvRodrigues2(&om, &r_r);        // rotate cameras to same orientation by averaging
  cvMatMul(&r_r, matT, &t);
  
  int idx = fabs(_t[0]) > fabs(_t[1]) ? 0 : 1;
  double c = _t[idx], nt = cvNorm(&t, 0, CV_L2);
  _uu[idx] = c > 0 ? 1 : -1; // unit vector to fix new Y axis to be orthogonal to 
  // new X axis, which is parallel to baseline.

  // calculate global Z rotation
  cvCrossProduct(&t,&uu,&ww);
  double nw = cvNorm(&ww, 0, CV_L2);
  cvConvertScale(&ww, &ww, acos(fabs(c)/nt)/nw);
  cvRodrigues2(&ww, &wR);

  // apply to both views
  cvGEMM(&wR, &r_r, 1, 0, 0, &Ri, CV_GEMM_B_T);
  cvConvert( &Ri, _R1 );
  cvGEMM(&wR, &r_r, 1, 0, 0, &Ri, 0);
  cvConvert( &Ri, _R2 );
  cvMatMul(&Ri, matT, &t);

  // calculate projection/camera matrices
  // these contain the relevant rectified image internal params (fx, fy=fx, cx, cy)
  double fc_new = DBL_MAX;
  CvPoint2D64f cc_new[2] = {{0,0}, {0,0}};

  for( k = 0; k < 2; k++ ) {
    const CvMat* A = k == 0 ? _cameraMatrix1 : _cameraMatrix2;
    const float dk1 = k == 0 ? _distCoeffs1 : _distCoeffs2;
    double fc = cvmGet(A,idx^1,idx^1);
    double r2 = (nx*nx + ny*ny)/(4*fc*fc);
    double num = (1/dk1)*atan(2*sqrt(r2)*tan(dk1/2));
    fc *=  num/sqrt(r2);
    fc_new = MIN(fc_new, fc);
  }

  for( k = 0; k < 2; k++ )
    {
      const CvMat* A = k == 0 ? _cameraMatrix1 : _cameraMatrix2;
      const float Dk = k == 0 ? _distCoeffs1 : _distCoeffs2;
      
      CvPoint2D32f _pts[4];
      CvPoint3D32f _pts_3[4];
      
      CvMat pts = cvMat(1, 4, CV_32FC2, _pts);
      CvMat pts_3 = cvMat(1, 4, CV_32FC3, _pts_3);
      
      for( i = 0; i < 4; i++ )
	{
	  int j = (i<2) ? 0 : 1;
	  _pts[i].x = (float)((i % 2)*(nx-1));
	  _pts[i].y = (float)(j*(ny-1));
	}
      CvPoint2D32f _undist_pts[4];
      CvMat undist_pts = cvMat(1,4,CV_32FC2, _undist_pts);

      undistort_points( &pts, &undist_pts, A, Dk, 0, 0 );
      cvConvertPointsHomogeneous( &undist_pts, &pts_3 );
      
      //Change camera matrix to have cc=[0,0] and fc = fc_new
      double _a_tmp[3][3];
      CvMat A_tmp  = cvMat(3, 3, CV_64F, _a_tmp);
      _a_tmp[0][0]=fc_new;
      _a_tmp[1][1]=fc_new;
      _a_tmp[0][2]=0.0;
      _a_tmp[1][2]=0.0;
      cvProjectPoints2( &pts_3, k == 0 ? _R1 : _R2, &Z, &A_tmp, 0, &pts );
      CvScalar avg = cvAvg(&pts);
      cc_new[k].x = (nx-1)/2 - avg.val[0];
      cc_new[k].y = (ny-1)/2 - avg.val[1];
	
    }
  // vertical focal length must be the same for both images to keep the epipolar constraint
  // (for horizontal epipolar lines -- TBD: check for vertical epipolar lines)
  // use fy for fx also, for simplicity

  // For simplicity, set the principal points for both cameras to be the average
  // of the two principal points (either one of or both x- and y- coordinates)
  if( flags & CV_CALIB_ZERO_DISPARITY )
    {
      cc_new[0].x = cc_new[1].x = (cc_new[0].x + cc_new[1].x)*0.5;
      cc_new[0].y = cc_new[1].y = (cc_new[0].y + cc_new[1].y)*0.5;
    }
  else if( idx == 0 ) // horizontal stereo
    cc_new[0].y = cc_new[1].y = (cc_new[0].y + cc_new[1].y)*0.5;
  else // vertical stereo
    cc_new[0].x = cc_new[1].x = (cc_new[0].x + cc_new[1].x)*0.5;

  cvZero( &pp );
  _pp[0][0] = _pp[1][1] = fc_new;
  _pp[0][2] = cc_new[0].x;
  _pp[1][2] = cc_new[0].y;
  _pp[2][2] = 1;
  cvConvert(&pp, _P1);
  _pp[0][2] = cc_new[1].x;
  _pp[1][2] = cc_new[1].y;
  _pp[idx][3] = _t[idx]*fc_new; // baseline * focal length
  cvConvert(&pp, _P2);
  
  alpha = MIN(alpha, 1.);
  getRectangles(_cameraMatrix1, _distCoeffs1, _R1, _P1, imageSize, inner1, outer1 );
  getRectangles(_cameraMatrix2, _distCoeffs2, _R2, _P2, imageSize, inner2, outer2 );
  
  {
    newImgSize = newImgSize.width*newImgSize.height != 0 ? newImgSize : imageSize;
    double cx1_0 = cc_new[0].x;
    double cy1_0 = cc_new[0].y;
    double cx2_0 = cc_new[1].x;
    double cy2_0 = cc_new[1].y;
    double cx1 = newImgSize.width*cx1_0/imageSize.width;
    double cy1 = newImgSize.height*cy1_0/imageSize.height;
    double cx2 = newImgSize.width*cx2_0/imageSize.width;
    double cy2 = newImgSize.height*cy2_0/imageSize.height;
    double s = 1.;
    //cout << "alpha is "<< alpha << endl;
    if( alpha >= 0 )
      {
	double s0 = std::max(std::max(std::max((double)cx1/(cx1_0 - inner1.x), (double)cy1/(cy1_0 - inner1.y)),
				      (double)(newImgSize.width - cx1)/(inner1.x + inner1.width - cx1_0)),
			     (double)(newImgSize.height - cy1)/(inner1.y + inner1.height - cy1_0));
		
	s0 = std::max(std::max(std::max(std::max((double)cx2/(cx2_0 - inner2.x), (double)cy2/(cy2_0 - inner2.y)),
					(double)(newImgSize.width - cx2)/(inner2.x + inner2.width - cx2_0)),
			       (double)(newImgSize.height - cy2)/(inner2.y + inner2.height - cy2_0)),
		      s0);
	double s1 = std::min(std::min(std::min((double)cx1/(cx1_0 - outer1.x), (double)cy1/(cy1_0 - outer1.y)),
				      (double)(newImgSize.width - cx1)/(outer1.x + outer1.width - cx1_0)),
			     (double)(newImgSize.height - cy1)/(outer1.y + outer1.height - cy1_0));
	s1 = std::min(std::min(std::min(std::min((double)cx2/(cx2_0 - outer2.x), (double)cy2/(cy2_0 - outer2.y)),
					(double)(newImgSize.width - cx2)/(outer2.x + outer2.width - cx2_0)),
			       (double)(newImgSize.height - cy2)/(outer2.y + outer2.height - cy2_0)),
		      s1);
        
	s = s0*(1 - alpha) + s1*alpha;
      }
    cout << "s is " << s << endl;
    fc_new *= s;
    cc_new[0] = cvPoint2D64f(cx1, cy1);
    cc_new[1] = cvPoint2D64f(cx2, cy2);
    
    cvmSet(_P1, 0, 0, fc_new);
    cvmSet(_P1, 1, 1, fc_new);
    cvmSet(_P1, 0, 2, cx1);
    cvmSet(_P1, 1, 2, cy1);
    
    cvmSet(_P2, 0, 0, fc_new);
    cvmSet(_P2, 1, 1, fc_new);
    cvmSet(_P2, 0, 2, cx2);
    cvmSet(_P2, 1, 2, cy2);
    cvmSet(_P2, idx, 3, s*cvmGet(_P2, idx, 3));
    
    if(roi1)
      {
	*roi1 = cv::Rect(cvCeil((inner1.x - cx1_0)*s + cx1),
			 cvCeil((inner1.y - cy1_0)*s + cy1),
			 cvFloor(inner1.width*s), cvFloor(inner1.height*s))
	  & cv::Rect(0, 0, newImgSize.width, newImgSize.height);
      }
    
    if(roi2)
      {
	*roi2 = cv::Rect(cvCeil((inner2.x - cx2_0)*s + cx2),
			 cvCeil((inner2.y - cy2_0)*s + cy2),
			 cvFloor(inner2.width*s), cvFloor(inner2.height*s))
	  & cv::Rect(0, 0, newImgSize.width, newImgSize.height);
      }
  }

  if( matQ )
    {
      double q[] =
	{
	  1, 0, 0, -cc_new[0].x,
	  0, 1, 0, -cc_new[0].y,
	  0, 0, 0, fc_new,
	  0, 0, 1./_t[idx],
	  (idx == 0 ? cc_new[0].x - cc_new[1].x : cc_new[0].y - cc_new[1].y)/_t[idx]
	};
      CvMat Q = cvMat(4, 4, CV_64F, q);
      cvConvert( &Q, matQ );
    }
    
}
      
                  


void getRectangles(const CvMat *camMat, const float dist, const CvMat *R, const CvMat *newCamMat, CvSize imgSize, cv::Rect_<float> &inner, cv::Rect_<float> &outer){
  const int N = 9;
  cv::Ptr<CvMat> _pts = cvCreateMat(1,N*N,CV_32FC2);
  cv::Ptr<CvMat> _undist_pts = cvCreateMat(1,N*N,CV_32FC2);
  CvPoint2D32f* pts = (CvPoint2D32f*)(_pts->data.ptr);
  CvPoint2D32f* undist_pts = (CvPoint2D32f*)(_undist_pts->data.ptr);
  //get a series of progressively larger boxes from
  //the top left of the screen down down to the right
  for(int y=0, k=0; y<N;y++)
    for(int x=0;x<N;x++){
      pts[k++] = cvPoint2D32f((float)x*imgSize.width/(N-1),
			      (float)y*imgSize.height/(N-1));
    }
  
  undistort_points(_pts,_undist_pts,camMat,dist,R,newCamMat);

  float iX0=-FLT_MAX, iX1=FLT_MAX, iY0=-FLT_MAX, iY1=FLT_MAX;
  float oX0=FLT_MAX, oX1=-FLT_MAX, oY0=FLT_MAX, oY1=-FLT_MAX;
  // find the inscribed rectangle.
  // the code will likely not work with extreme rotation matrices (R) (>45%)
  for(int y=0, k = 0; y < N; y++ )
    for(int x = 0; x < N; x++ )
      {
	CvPoint2D32f p = undist_pts[k++];
	oX0 = MIN(oX0, p.x);
	oX1 = MAX(oX1, p.x);
	oY0 = MIN(oY0, p.y);
	oY1 = MAX(oY1, p.y);
	
            if( x == 0 )
                iX0 = MAX(iX0, p.x);
            if( x == N-1 )
                iX1 = MIN(iX1, p.x); 
            if( y == 0 )
                iY0 = MAX(iY0, p.y);
            if( y == N-1 )
                iY1 = MIN(iY1, p.y);
      }
  
  inner = cv::Rect_<float>(iX0, iY0, iX1-iX0, iY1-iY0);
  outer = cv::Rect_<float>(oX0, oY0, oX1-oX0, oY1-oY0);
  //cvReleaseMat(
}		   


void undistort_points(const CvMat* _src, CvMat* _dst, 
		      const CvMat* _cameraMatrix,
		      const float distortion,
		      const CvMat* matR, const CvMat* matP){
  double A[3][3], RR[3][3],fx, fy, ifx, ify, cx, cy;
  CvMat matA=cvMat(3, 3, CV_64F, A);
  CvMat _RR=cvMat(3, 3, CV_64F, RR);
  const CvPoint2D32f* srcf;
  const CvPoint2D64f* srcd;
  CvPoint2D32f* dstf;
  CvPoint2D64f* dstd;
  int stype, dtype;
  int sstep, dstep;
  int i, n;
  
  CV_Assert( CV_IS_MAT(_src) && CV_IS_MAT(_dst) &&
	     (_src->rows == 1 || _src->cols == 1) &&
	     (_dst->rows == 1 || _dst->cols == 1) &&
	     _src->cols + _src->rows - 1 == _dst->rows + _dst->cols - 1 &&
	     (CV_MAT_TYPE(_src->type) == CV_32FC2 || CV_MAT_TYPE(_src->type) == CV_64FC2) &&
	     (CV_MAT_TYPE(_dst->type) == CV_32FC2 || CV_MAT_TYPE(_dst->type) == CV_64FC2));
  
  CV_Assert( CV_IS_MAT(_cameraMatrix) &&
	     _cameraMatrix->rows == 3 && _cameraMatrix->cols == 3 );
  
  cvConvert( _cameraMatrix, &matA );
  
  if( matR )
    {
      CV_Assert( CV_IS_MAT(matR) && matR->rows == 3 && matR->cols == 3 );
      cvConvert( matR, &_RR );
    }
  else
    cvSetIdentity(&_RR);
  
  if( matP )
    {
      double PP[3][3];
      CvMat _P3x3, _PP=cvMat(3, 3, CV_64F, PP);
      CV_Assert( CV_IS_MAT(matP) && matP->rows == 3 && (matP->cols == 3 || matP->cols == 4));
      cvConvert( cvGetCols(matP, &_P3x3, 0, 3), &_PP );
      cvMatMul( &_PP, &_RR, &_RR );
    }
  
  srcf = (const CvPoint2D32f*)_src->data.ptr;
  srcd = (const CvPoint2D64f*)_src->data.ptr;
  dstf = (CvPoint2D32f*)_dst->data.ptr;
  dstd = (CvPoint2D64f*)_dst->data.ptr;
  stype = CV_MAT_TYPE(_src->type);
  dtype = CV_MAT_TYPE(_dst->type);
  sstep = _src->rows == 1 ? 1 : _src->step/CV_ELEM_SIZE(stype);
  dstep = _dst->rows == 1 ? 1 : _dst->step/CV_ELEM_SIZE(dtype);

  n = _src->rows + _src->cols - 1;
  
  fx = A[0][0];
  fy = A[1][1];
  ifx = 1./fx;
  ify = 1./fy;
  cx = A[0][2];
  cy = A[1][2];

  for( i = 0; i < n; i++ )
    {
      double x, y, x0, y0;
      if( stype == CV_32FC2 )
        {
	  x = srcf[i*sstep].x;
	  y = srcf[i*sstep].y;
        }
      else
        {
	  x = srcd[i*sstep].x;
	  y = srcd[i*sstep].y;
        }

      x0 = x = (x - cx)*ifx; //get into distorted x/z coords
      y0 = y = (y - cy)*ify;
      
      float dR=sqrt((x*x)+ (y*y));
      float r=dR;
      if(distortion != 0)
	r = tan(dR*distortion) / (2*tan(distortion/2));
      double dFactor; //ratio of undistorted radius to distorted
      //undistort it - but only if there is distortion!
      if(dR > 0.01)
	dFactor = r/dR;
      else
	dFactor = 1.0;
      x = x*dFactor; //undistorted x
      y = y*dFactor; //undistorted y
      
      double xx = RR[0][0]*x + RR[0][1]*y + RR[0][2];
      double yy = RR[1][0]*x + RR[1][1]*y + RR[1][2];
      double ww = 1./(RR[2][0]*x + RR[2][1]*y + RR[2][2]);
      x = xx*ww;
      y = yy*ww;
      
      if( dtype == CV_32FC2 )
        {
	  dstf[i*dstep].x = (float)x;
	  dstf[i*dstep].y = (float)y;
	}
      else
        {
	  dstd[i*dstep].x = x;
	  dstd[i*dstep].y = y;
        }
    }
}



void InitUndistortRectifyMap(const CvMat *CamMatrix, const float distortion, const CvMat *R, const CvMat *newCamMatrix, CvArr *mapX, CvArr* mapY){
  cv::Mat A = cv::cvarrToMat(CamMatrix), Rmat, newCamMatrixmat;
  cv::Mat mapx = cv::cvarrToMat(mapX), mapy, mapx0 = mapx, mapy0;
  
  if( mapY )
    mapy0 = mapy = cv::cvarrToMat(mapY);
  
  if( R )
    Rmat = cv::cvarrToMat(R);
  if( newCamMatrix)
    newCamMatrixmat = cv::cvarrToMat(newCamMatrix);

  UndistortRectifyMap( A, distortion, Rmat, newCamMatrixmat, mapx.size(), mapx.type(), mapx, mapy );
  CV_Assert( mapx0.data == mapx.data && mapy0.data == mapy.data );  
}

void UndistortRectifyMap( const Mat& cameraMatrix, float distortion, const Mat& matR, const Mat& newCameraMatrix, Size size, int m1type, Mat& map1, Mat& map2 )
{
  if( m1type <= 0 )
        m1type = CV_16SC2;
    CV_Assert( m1type == CV_16SC2 || m1type == CV_32FC1 || m1type == CV_32FC2 );
    map1.create( size, m1type );
    if( m1type != CV_32FC2 )
        map2.create( size, m1type == CV_16SC2 ? CV_16UC1 : CV_32FC1 );
    else
        map2.release();

    Mat_<double> R = Mat_<double>::eye(3, 3);
    Mat_<double> A = Mat_<double>(cameraMatrix), Ar;

    if( newCameraMatrix.data )
        Ar = Mat_<double>(newCameraMatrix);
    else
        Ar = getDefaultNewCameraMatrix( A, size, true );

    if( matR.data )
        R = Mat_<double>(matR);

    CV_Assert( A.size() == Size(3,3) && A.size() == R.size() );
    CV_Assert( Ar.size() == Size(3,3) || Ar.size() == Size(4, 3));
    Mat_<double> iR = (Ar.colRange(0,3)*R).inv(DECOMP_LU);
    const double* ir = &iR(0,0);

    double u0 = A(0, 2),  v0 = A(1, 2);
    double fx = A(0, 0),  fy = A(1, 1);
    
    for( int i = 0; i < size.height; i++ )
    {
        float* m1f = (float*)(map1.data + map1.step*i);
        float* m2f = (float*)(map2.data + map2.step*i);
        short* m1 = (short*)m1f;
        ushort* m2 = (ushort*)m2f;
        double _x = i*ir[1] + ir[2], _y = i*ir[4] + ir[5], _w = i*ir[7] + ir[8];
        for( int j = 0; j < size.width; j++, _x += ir[0], _y += ir[3], _w += ir[6] )
        {
	  double w = 1./_w, x = _x*w, y = _y*w;
	  double x2 = x*x, y2 = y*y;
	  double r2 = x2 + y2;
	  double r_d = sqrt(r2);
	  double r_u = tan(r_d*distortion)/(2*tan(distortion/2));
	  double kr = r_d/r_u;
	  
	  double u = fx*(x*kr) + u0;
	  double v = fy*(y*kr) + v0;
	
	  if( m1type == CV_16SC2 )
            {
	      int iu = saturate_cast<int>(u*INTER_TAB_SIZE);
	      int iv = saturate_cast<int>(v*INTER_TAB_SIZE);
	      m1[j*2] = (short)(iu >> INTER_BITS);
	      m1[j*2+1] = (short)(iv >> INTER_BITS);
	      m2[j] = (ushort)((iv & (INTER_TAB_SIZE-1))*INTER_TAB_SIZE + (iu & (INTER_TAB_SIZE-1)));
            }
            else if( m1type == CV_32FC1 )
            {
                m1f[j] = (float)u;
                m2f[j] = (float)v;
            }
            else
            {
                m1f[j*2] = (float)u;
                m1f[j*2+1] = (float)v;
            }
        }
	}
}




                        
