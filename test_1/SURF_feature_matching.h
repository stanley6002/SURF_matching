//
//  SURF_feature_matching.h
//  test_1
//
//  Created by chih-hsiang chang on 2/9/12.
//  Copyright 2012 __MyCompanyName__. All rights reserved.
//
#ifndef __included_SURF_feature_matching_h
#define __included_SURF_feature_matching_h

#include <cv.h>
#include <cxmisc.h>
#include <cxcore.h>
#include <highgui.h>
#include <cvaux.h>
#include <cxmisc.h> 
#include <iostream>
#include <math.h>
#include <opencv2/features2d/features2d.hpp>

#include "type.h"

using namespace cv;



vector<int> SURF_feature_matching (IplImage *result,IplImage *result1);
void findPair(CvSeq* ,  CvSeq* ,  CvSeq* , CvSeq *, vector<int>&);
void flannFindPairs (const CvSeq* ,  const CvSeq* ,  const CvSeq* , const CvSeq *, vector<int>&);
int neighbor_comparision (const float* ,CvSeq* , CvSeq* ,int,CvPoint *);
double  compareSURFDescriptor (const float* , const float*, double , int);
bool isColinear(int, CvPoint2D64f *);
void ComputeH(int , CvPoint2D64f *, CvPoint2D64f *, CvMat *);
int ComputeNumberOfInliers(int , CvPoint2D64f *, CvPoint2D64f *, CvMat *, CvMat *, double *);
void Normalization(int, CvPoint2D64f *, CvMat *);
void RANSAC_homography(int , CvPoint2D64f *, CvPoint2D64f *, CvMat *, CvMat *);
vector<int>determine_uniquness(CvPoint *,CvPoint *, IplImage *, int &);
void plot_epipolar_line(IplImage *I_in, CvMat* correspond_lines);
IplImage* plot_two_images(IplImage *IGray,IplImage *IGray1,CvPoint *corners1,CvPoint *corners3, int &vector_index);
void cvpoint2vector(int num_pts, CvPoint* matching_point, v3_t *point);
void featurematching_process(IplImage *IGray_r,IplImage *IGray_l,matched *pts);
vector<float> SIFT_feature_matching(IplImage *result,IplImage *result1);
IplImage* plot_two_imagesf(IplImage *IGray,IplImage *IGray1,Point2f *corners1,Point2f *corners3, int &vector_index);
void float_cvpoint2vector(int num_pts, Point2f* matching_point, v3_t *point);
void SIFT_feature_matching_real (IplImage *result,IplImage *result1);
DescriptorMatcher* createDescMatcher( const string& matherType = string() );
#endif;