
#include "SURF_feature_matching.h"
//#include "vector.h"


using namespace cv;

#define  T_DIST 10




// window size ued in corner detection
int mask_size=10;
int x_input=10;
int matching_index=0;
CvPoint matched_point;
using namespace cv;
void featurematching_process(IplImage *IGray_r,IplImage *IGray_l,matched *pts)
{
    
#define SURF
# ifdef SURF
    vector<int> featurematching_vector;
    featurematching_vector= SURF_feature_matching (IGray_r,IGray_l);
    CvPoint  putative_point1[(int)featurematching_vector.size()/4];
    CvPoint  putative_point2[(int)featurematching_vector.size()/4];
    for (int i=0; i<(int)featurematching_vector.size();i+=4)
    {
        putative_point1[i/4].x= featurematching_vector[i];
        putative_point1[i/4].y= featurematching_vector[i+1];
        putative_point2[i/4].x= featurematching_vector[i+2];
        putative_point2[i/4].y= featurematching_vector[i+3]; 
    }
    int size=(int)featurematching_vector.size()/4; 
    int num_pts= size;
    
    v3_t *l1_pt= new v3_t[num_pts];
    v3_t *r1_pt= new v3_t[num_pts];
    
    cvpoint2vector(num_pts, putative_point1, l1_pt);
    cvpoint2vector(num_pts, putative_point2, r1_pt);
    for (int i=0; i<num_pts; i++)
    {
        pts[0].R_pts.push_back(r1_pt[i]);
        pts[0].L_pts.push_back(l1_pt[i]);
    }

#endif   
//#define SIFT
# ifdef SIFT    
    vector<float> featurematching_vector_sift;
    
    // SIFT_feature_matching_real (IGray_r,IGray_l);
    //featurematching_vector_sift= SIFT_feature_matching_real (IGray_r,IGray_l);
   
    featurematching_vector_sift=SIFT_feature_matching(IGray_r,IGray_l);
    Point2f  *putative_pt1 =new Point2f[(int)featurematching_vector_sift.size()/4];
    Point2f  *putative_pt2= new Point2f[(int)featurematching_vector_sift.size()/4];
    for (int i=0; i<(int)featurematching_vector_sift.size();i+=4)
    {
        putative_pt2[i/4].x= featurematching_vector_sift[i];
        putative_pt2[i/4].y= featurematching_vector_sift[i+1];
        putative_pt1[i/4].x= featurematching_vector_sift[i+2];
        putative_pt1[i/4].y= featurematching_vector_sift[i+3]; 
    }
    int size=(int)featurematching_vector_sift.size()/4; 
       int num_pts= size;
        
        v3_t *l1_pt= new v3_t[num_pts];
        v3_t *r1_pt= new v3_t[num_pts];
        
        float_cvpoint2vector(num_pts, putative_pt1, l1_pt);
        float_cvpoint2vector(num_pts, putative_pt2, r1_pt);
        for (int i=0; i<num_pts; i++)
        {
             pts[0].R_pts.push_back(r1_pt[i]);
             pts[0].L_pts.push_back(l1_pt[i]);
        }
    
    IplImage* I_show;
    I_show=plot_two_imagesf(IGray_l,IGray_r,putative_pt1,putative_pt2, size);
    cvShowImage("test", I_show);
    //cout<<"test"<<endl;
    cvWaitKey(0);
    delete[] putative_pt1;
    delete[] putative_pt2;
    //  # endif
#endif    
   
  
//#define DEBUG1
# ifdef DEBUG1
    IplImage* I_show;
    I_show=plot_two_images(IGray_l,IGray_r,putative_point1,putative_point2, size);
    cvShowImage("test", I_show);
    //cout<<"test"<<endl;
    cvWaitKey(0);
# endif
        
}
//vector<float> SIFT_feature_matching(IplImage *result,IplImage *result1)
//{
///*
//    // int minHessian =80;
//    // SurfFeatureDetector detector(minHessian);
//    //std::vector<KeyPoint> Fast_Keypoint1, Fast_Keypoint2;
//    //SurfFeatureDetector::SurfFeatureDetector(minHessian,3,4);	
//    //FAST(result, Fast_Keypoint1, 30);
//    //FAST(result1, Fast_Keypoint2,30);
//    //cout<<Fast_Keypoint1.size()<<" "<<Fast_Keypoint2.size()<<endl;
//    std::vector<KeyPoint> keypoints_1, keypoints_2;
//    
//   
//    
//    //BriefDescriptorExtractor extractor1(32);
//    detector.detect( result, keypoints_1 );
//    detector.detect( result1, keypoints_2 );
//    cout<<keypoints_1.size()<<" "<<keypoints_2.size()<<endl;
//    //-- Step 2: Calculate descriptors (feature vectors)
//    //SurfDescriptorExtractor extractor(4,4,false);
//    SurfDescriptorExtractor extractor;
//    //cv::SurfDescriptorExtractor::SurfDescriptorExtractor(4,4,false);		
//    Mat descriptors_1, descriptors_2;
//    
//    //extractor.compute( result, Fast_Keypoint1, descriptors_1 );
//    //extractor.compute( result1,Fast_Keypoint2, descriptors_2 );
//    extractor.compute( result, keypoints_1, descriptors_1 );
//    extractor.compute( result1, keypoints_2, descriptors_2 );
//
//    
//    //-- Step 3: Matching descriptor vectors using FLANN matcher
//    FlannBasedMatcher matcher;
//    std::vector< DMatch > matches;
//    matcher.match( descriptors_1, descriptors_2, matches );
//    
//    double max_dist = 0; double min_dist = 20000;
//    
//    //-- Quick calculation of max and min distances between keypoints
//    for( int i = 0; i < descriptors_1.rows; i++ )
//    { double dist = matches[i].distance;
//        if( dist < min_dist ) min_dist = dist;
//        if( dist > max_dist ) max_dist = dist;
//    }
//    
//    printf("-- Max dist : %f \n", max_dist );
//    printf("-- Min dist : %f \n", min_dist );
//    
//    //-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist )
//    //-- PS.- radiusMatch can also be used here.
//    std::vector< DMatch > good_matches;
//    
//    for( int i = 0; i < descriptors_1.rows; i++ )
//    { if( matches[i].distance <5*min_dist )
//    { good_matches.push_back( matches[i]); }
//    }
//    
//    //   Mat img_matches;
//    //    drawMatches( result, keypoints_1, result1, keypoints_2,
//    //                good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
//    //                vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
//    //    imshow( "Good Matches", img_matches );
//    
//    for( int i = 0; i < good_matches.size(); i++ )
//    {
//    
//        //printf( "-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, good_matches[i].queryIdx, good_matches[i].trainIdx ); 
//    }
//    std::vector<Point2f> obj ; //keypt1= left;
//    std::vector<Point2f> scene;  //keypt2= right;
//    for( int i = 0; i < good_matches.size(); i++ )
//    {
//        //-- Get the keypoints from the good matches
//        obj.push_back ( keypoints_1[ good_matches[i].queryIdx ].pt );
//        scene.push_back( keypoints_2[ good_matches[i].trainIdx ].pt );
//    }
//    
//
//    std::vector<float> matching_vector;
//    for(int i=0;i< good_matches.size();i++)
//    {
//        matching_vector.push_back(obj[i].x);   // left 
//        matching_vector.push_back(obj[i].y);
//        matching_vector.push_back(scene[i].x); // right
//        matching_vector.push_back(scene[i].y);   
//    }
//    
//    int num=good_matches.size();
//    Point2f *corners1= new Point2f[num];
//    Point2f *corners2= new Point2f[num];
//   
//    for(int i=0;i<good_matches.size();i++)
//    {
//        corners1[i].x=obj[i].x ;  // left 
//        corners1[i].y=obj[i].y;
//        corners2[i].x=scene[i].x ;  // right
//        corners2[i].y=scene[i].y;
//    }
//    
//    //    IplImage* I_show;
//    //    
//    //    I_show=plot_two_imagesf(result,result1,corners1,corners2, num);
//    //    cvShowImage("test", I_show);
//    //    waitKey(0);
//    
//    return(matching_vector);
// */
//    
//}
void  SIFT_feature_matching_real (IplImage *result,IplImage *result1)
{
  
// //cv::SiftFeatureDetector::SiftFeatureDetector(100.0,100.0,2.0,4.0,4.0,0.0);	
//    SiftFeatureDetector detector(0.025,5.0);
//   
//   
//    SiftDescriptorExtractor extractor;
//    vector<KeyPoint> keypoints1, keypoints2;
//    detector.detect(result, keypoints1);
//    detector.detect(result1, keypoints2);
//    cout<< keypoints1.size()<<" "<<keypoints2.size()<<endl;
//  
//    Mat descriptors_1, descriptors_2;
//    extractor.compute( result, keypoints1, descriptors_1 );
//    extractor.compute( result1, keypoints2, descriptors_2 );
//    std::vector< DMatch > matches;
//    vector<int> matches1;
//    vector<double> distances;
//     BruteForceMatcher< L2<float> > matcher;
//    
//    //Mat descs1, descs2;
//    //Ptr<DescriptorMatcher> matcher1 = createDescMatcher();
//    //matcher1->add( descs2 );
//    matcher.match(descriptors_1, descriptors_2, matches);
//    double max_dist = 0; double min_dist = 20000;
//    
//  
//    std::vector< DMatch > good_matches;
//    
//    //    for( int i = 0; i < descriptors_1.rows; i++ )
//    //    { if( matches[i].distance < 1*min_dist )
//    //    { good_matches.push_back( matches[i]); }
//    //    }
//      Mat img_matches;
//      Mat img_matches1;
//      drawMatches( result, keypoints1, result1, keypoints2, matches, img_matches1 );
//     imshow( "Good Matches", img_matches1 );
//     waitKey(0);
//    // return(matching_vector);
    
}

vector<int> SURF_feature_matching (IplImage *result,IplImage *result1)
{

    CvSeq  *imageDescriptors = 0;
    CvSeq  *imageKeypoints =0;
    CvSeq  *objectDescriptors =0;
    CvSeq  *objectKeypoint =0;
    CvSURFParams params = cvSURFParams(300);
    CvMemStorage* storage = cvCreateMemStorage(0);
    cvExtractSURF(result, NULL, &imageKeypoints,  &imageDescriptors,storage,params);
    cvExtractSURF(result1, NULL, &objectKeypoint, &objectDescriptors,storage,params);
    //  CvMemStorage *memStorage = cvCreateMemStorage(0);
  
    //CvSeq  *descriptors=0;
   // descriptors = cvCreateSeq( 0, sizeof(CvSeq),20*CV_ELEM_SIZE(CV_32F), storage );
    
    //float uint =10.0;
    //float *vec;
    //    //    cvSeqPush(descriptors,&uint);
    //    vec = (float*) cvGetSeqElem(imageDescriptors, 0);
    //    for(int i=0;i<29;i++)
    //    {
    //        cout<<vec[i]<<endl;
            //vec[i]=0;
    //    }
         

    //    cout<<  vec_m[0]<<endl;
    //    
    //    //vec[1]=1.02;
    //    for(int i=0;i<2;i++)
    //    {
    //    float* rr= (float*)cvGetSeqElem(descriptors, i);
    //    cout<< *rr<<" "<<endl;
    //    }
    //    //cvSeqPush(seq, &points);
    //    cout<<sizeof(CvSeq)<<" "<<descriptors->total<<endl;
    //}
    //CvSeqReader *reader;
    //for (int i=0;i< objectDescriptors->total;i++)
    //{
    //CV_READ_SEQ_ELEM( objectDescriptors, reader );
    //    CvSURFPoint* rr= (CvSURFPoint*)cvGetSeqElem(objectDescriptors, i);
    //cout<< rr->pt.x<<" "<<rr->pt.y<<endl;
    //}
    
    vector <int> ptpair;
    flannFindPairs (objectKeypoint,objectDescriptors,imageKeypoints,imageDescriptors,ptpair);
    // findPair(objectKeypoint, objectDescriptors, imageKeypoints, imageDescriptors, ptpair);
    
    cvReleaseMemStorage(&storage);
    CvPoint2D64f center1[(int) ptpair.size()/2];
    CvPoint2D64f center2[(int) ptpair.size()/2];
    
    // std::cout<<"After_matching__"<<(int) ptpair.size()/2<<"\n";
    
    for (int i=0;i<(int) ptpair.size();i+=2)
    {
   
        CvSURFPoint* rr= (CvSURFPoint*)cvGetSeqElem(objectKeypoint, ptpair[i]);
        CvSURFPoint* rl= (CvSURFPoint*)cvGetSeqElem(imageKeypoints, ptpair[i+1]);
        center1[i/2].x= rr->pt.x;
        center1[i/2].y= rr->pt.y;
        center2[i/2].x= rl->pt.x;
        center2[i/2].y= rl->pt.y;
    }
    int num_matched=(int) ptpair.size()/2;
    CvMat    *points1=cvCreateMat(num_matched,2,CV_32FC1);
    CvMat    *points2=cvCreateMat(num_matched,2,CV_32FC1);
    CvPoint  putative_point1[num_matched];
    CvPoint  putative_point2[num_matched];
   
    for (int i=0; i<(int)num_matched;i++)
     {
        putative_point1[i].x= center1[i].x;
        putative_point1[i].y= center1[i].y;
        putative_point2[i].x= center2[i].x;
        putative_point2[i].y= center2[i].y; 
        cvCircle(result1,putative_point1[i], 5, CV_RGB(255,255,255));
        cvCircle(result,putative_point2[i], 5, CV_RGB(255,255,255));
     }
    
    //IplImage* I_show;
    //I_show=plot_two_images(result1,result,putative_point1,putative_point2,num_matched);
    //cvWaitKey(0);
    //cvShowImage("test", I_show);
    //cvShowImage("right_image", result);
    //cvShowImage("left_image",result1);
  
    CvMat *H = cvCreateMat(3,3,CV_64FC1);
    CvMat *inlier_mask = cvCreateMat(num_matched,1,CV_64FC1); 
    //RANSAC_homography(num_matched, center1, center2, H, inlier_mask);
    //cout<<cvmGet(H, 0, 0)<<" "<<cvmGet(H, 0, 1)<<" "<<cvmGet(H, 0, 2)<<endl;
    //cout<<cvmGet(H, 1, 0)<<" "<<cvmGet(H, 1, 1)<<" "<<cvmGet(H, 1, 2)<<endl;
    //cout<<cvmGet(H, 2, 0)<<" "<<cvmGet(H, 2, 1)<<" "<<cvmGet(H, 2, 2)<<endl;
    
//# define DEBUG_RANSAC
# ifdef  DEBUG_RANSAC
    int num_inlier = 0;
    for(int i=0; i<num_matched; i++)
    {
        if(cvmGet(inlier_mask,i,0) == 1)
        {
            
            num_inlier++;
        }
    }
    CvPoint  new_center1[num_inlier];
    CvPoint  new_center2[num_inlier];
    int new_center_index=0;
    for (int i=0; i<num_inlier; i++)
    {
        if(cvmGet(inlier_mask,i,0) == 1)
        {
            new_center1[new_center_index].x= (int)center1[i].x;
            new_center1[new_center_index].y= (int)center1[i].y;
            new_center2[new_center_index].x= (int)center2[i].x;
            new_center2[new_center_index].y= (int)center2[i].y; 
            new_center_index++;  
        }
    }
#  endif
    // int num_inlier = 0;
    CvPoint  new_center1[num_matched];
    CvPoint  new_center2[num_matched];
    for(int i=0; i<num_matched; i++)
    {
        //CvPoint  new_center1[num_inlier];
        //CvPoint  new_center2[num_inlier];
        //int new_center_index=0;
        //for (int i=0; i<num_inlier; i++)
        //{
        //    if(cvmGet(inlier_mask,i,0) == 1)
        //    {
                new_center1[i].x= (int)center1[i].x;
                new_center1[i].y= (int)center1[i].y;
                new_center2[i].x= (int)center2[i].x;
                new_center2[i].y= (int)center2[i].y; 
         //       new_center_index++;  
        //    }
       // }
    }
    
    vector<int> putative_point;
    putative_point=determine_uniquness(new_center1,new_center2, result,num_matched);
    cout<<"After_Ransac__ "<<(int)putative_point.size()/4<<endl;
   
//#define DEBUG1_TEST
#ifdef DEBUG1_TEST
    FILE *f1 = fopen("/Users/chih-hsiangchang/Desktop/Archive/featurePt1.txt", "w");
    FILE *f2 = fopen("/Users/chih-hsiangchang/Desktop/Archive/featurePt2.txt", "w");

    CvMat *points1_=cvCreateMat( (int)putative_point.size()/4,2,CV_32FC1);
    CvMat *points2_=cvCreateMat( (int)putative_point.size()/4,2,CV_32FC1);
    for (int i=0; i<(int)putative_point.size();i+=4)
    {
        putative_point1[i/4].x= putative_point[i];
        putative_point1[i/4].y= putative_point[i+1];
        putative_point2[i/4].x= putative_point[i+2];
        putative_point2[i/4].y= putative_point[i+3];
      
        ////  
        float location_x1= putative_point[i];
        float location_y1= putative_point[i+1];
        float location_x2= putative_point[i+2];
        float location_y2= putative_point[i+3];

        CvScalar s1=cvGet2D(result,location_y1,location_x1);
        CvScalar s2=cvGet2D(result1,location_y2,location_x2);
         
        int s1_in= (int) s1.val[0];
        int s2_in= (int) s2.val[0];
        fprintf(f1, "%d %d %d\n", putative_point[i],putative_point[i+1],s1_in);
        fprintf(f2, "%d %d %d\n", putative_point[i+2],putative_point[i+3],s2_in);
        
        //////
        cvCircle(result1,putative_point1[i/4], 5, CV_RGB(255,255,255));
        cvCircle(result,putative_point2[i/4], 5, CV_RGB(255,255,255));
    }
   fclose(f1);
   fclose(f2); 
    
    //    new_center_index=(int)putative_point.size()/4;
    //    for (int i=0; i<=1  ;i++)
    //    {
    //        for (int j=0;j< new_center_index;j++)
    //        {
    //            if (i==0)
    //            {
    //                cvmSet(points1,j,i,putative_point1[j].x);
    //                cvmSet(points2,j,i,putative_point2[j].x);     
    //            }
    //            else
    //            {                
    //                cvmSet(points1,j,i,putative_point2[j].y);
    //                cvmSet(points2,j,i,putative_point2[j].y);
    //            }
    //        }
    //    }
    //    new_center_index=(int)putative_point.size()/4;
    
    cout<<"putative_point"<<"\n";
    cvShowImage("right_image", result);
    cvShowImage("left_image",result1);
    cvSaveImage("/img.jpg", result);
    cvSaveImage("/img1.jpg", result1);
    cvWaitKey(0);
#endif
    return( putative_point);
}

vector<int>determine_uniquness(CvPoint *center1,CvPoint *center2, IplImage *Image1,int &num_ofinlier)
{
    vector<int> update_point;
    int const Ecliden_distance=3;
    int const neighbor_region=10;
    //cout<<num_ofinlier;
 if (num_ofinlier>0)
 {
    CvMat *neighbor_index = cvCreateMat(num_ofinlier, 1, CV_64FC1);
    cvZero(neighbor_index);

//    for (int i=0;i<num_ofinlier;i++)
//    {
//        
//        for (int j=i+1;j<num_ofinlier;j++)
//        {
//            float distance=  sqrt(pow(center1[i].x-center1[j].x,2.0)+pow(center1[i].y-center1[j].y,2.0));
//            if ( distance<Ecliden_distance && cvmGet(neighbor_index,j,0)==0)
//            {  
//                CvScalar MeanScalar_i1, StandardDeviationScalar_i1;
//                CvScalar MeanScalar_i2, StandardDeviationScalar_i2;
//                cvSetImageROI(Image1,cvRect(center1[i].x-(neighbor_region)/2,center1[i].y-(neighbor_region)/2,neighbor_region,neighbor_region)); 
//                cvAvgSdv(Image1,&MeanScalar_i1,&StandardDeviationScalar_i1);  
//                cvResetImageROI(Image1);
//                cvSetImageROI(Image1,cvRect(center1[j].x-(neighbor_region)/2,center1[j].y-(neighbor_region)/2,neighbor_region,neighbor_region)); 
//                cvAvgSdv(Image1,&MeanScalar_i2,&StandardDeviationScalar_i2);  
//                cvResetImageROI(Image1);
//                if (StandardDeviationScalar_i2.val[0]>=StandardDeviationScalar_i1.val[0])
//                {
//                    cvmSet(neighbor_index,i,0,1);      
//                }
//                else
//                {
//                    cvmSet(neighbor_index,j,0,1); 
//                }
//            }
//        }
//    }
//
    
    for (int i=0;i<neighbor_index->rows;i++)
    {
//        if (cvmGet(neighbor_index,i,0)==0)
//        {      
            
            update_point.push_back(center1[i].x);
            update_point.push_back(center1[i].y);
            update_point.push_back(center2[i].x);
            update_point.push_back(center2[i].y);       
 //       }
    }
 }
 //   cvReleaseMat(&neighbor_index);
    return(update_point);
}

void findPair(CvSeq* objectKeypoint,CvSeq* objectDescriptors,CvSeq* imageKeypoints,CvSeq *imageDescriptors, vector<int>& ptpair)
{  
    ptpair.clear(); 
    CvSeqReader reader;
    cvStartReadSeq(objectDescriptors, &reader);
    // for (int i=0; i< objectDescriptors->total;i++)
    CvPoint center1[1];
    for (int i=0;i<objectKeypoint->total;i++)
    {
        CvSURFPoint* o = (CvSURFPoint*)cvGetSeqElem( objectKeypoint,i);
        center1[0].x= o->pt.x;
        center1[0].y= o->pt.y;   
        const float * descriptor =(const float*)reader.ptr;
        CV_NEXT_SEQ_ELEM(reader.seq->elem_size,reader);
        int nearest_neighbor=neighbor_comparision(descriptor,imageDescriptors,imageKeypoints,o->laplacian,center1);
        if (nearest_neighbor>=0)
        {
            ptpair.push_back(i);
            ptpair.push_back(nearest_neighbor);
        }
        
    }
}
int neighbor_comparision(const float* vec,CvSeq *imageDescriptors,CvSeq *imageKeypoints, int laplacian, CvPoint *center)
{
    int length = (int)(imageDescriptors->elem_size/sizeof(float));
    vector <float> candidatepair;
    int i, neighbor = -1;
    double d, dist1 =1e6, dist2 = 1e6;
    CvSeqReader reader, kreader;
    cvStartReadSeq(imageKeypoints, &kreader);
    cvStartReadSeq(imageDescriptors, &reader);
    CvPoint centerimage[1];
    for( i = 0; i < imageKeypoints->total; i++)
    {
        CvSURFPoint* image_k = (CvSURFPoint*)cvGetSeqElem( imageKeypoints, i );
        centerimage[0].x= image_k->pt.x;
        centerimage[0].y= image_k->pt.y;  
        float distance_x = (centerimage[0].x-center[0].x)*(centerimage[0].x-center[0].x);
        float distance_y = (centerimage[0].y-center[0].y)*(centerimage[0].y-center[0].y);
        double distance= sqrt((distance_x+distance_y));
        if( laplacian != image_k->laplacian )
            continue;
        if (distance<=30)
        {
            const float *mvec = (float *) cvGetSeqElem(imageDescriptors, i);
         //   std::cout<<" image_x  "<< centerimage[0].x<<"  image_y"<< centerimage[0].y<<"\n";
         //   std::cout<<" object_x  "<< center[0].x<<"  object_y"<< center[0].y<<"\n";
            d = compareSURFDescriptor(vec, mvec, dist2, length );
            //std::cout<<"candiate"<<d<<"\n";
            candidatepair.push_back(d);
            candidatepair.push_back(i);
        }
    }
    std::cout<<"size"<<candidatepair.size()<<"\n";
    for (int i=0; i<(int)candidatepair.size();i+=2)
    {
        std::cout<<"candiate_distance"<<candidatepair[i]<<"\n";
        std::cout<<"candiate"<<candidatepair[i+1]<<"\n";
        d= candidatepair[i];
        if( d < dist1 )
        {
            dist1=d;
            neighbor=candidatepair[i+1];
        }
        
    }  
    return neighbor; 
    //  return -1;
}
double compareSURFDescriptor (const float*d1, const float*d2, double dist2, int length)
{   
    double total_cost=0;
    assert(length %4 ==0);
    for (int i=0;i<length; i+=4)
    {    
        double t0 = d1[i] - d2[i];
        double t1 = d1[i+1] - d2[i+1];
        double t2 = d1[i+2] - d2[i+2];
        double t3 = d1[i+3] - d2[i+3];
        total_cost += t0*t0 + t1*t1 + t2*t2 + t3*t3;
        if( total_cost > dist2 )
            break;  
    }
    return total_cost;
}
void flannFindPairs( const CvSeq*, const CvSeq* objectDescriptors,
                    const CvSeq*, const CvSeq* imageDescriptors, vector<int>& ptpairs )
{
	
    int length = (int)(objectDescriptors->elem_size/sizeof(float));
    
    cv::Mat m_object(objectDescriptors->total, length, CV_32F);
	cv::Mat m_image(imageDescriptors->total, length, CV_32F);
    
    
	// copy descriptors
    CvSeqReader obj_reader;
	float* obj_ptr = m_object.ptr<float>(0);
    cvStartReadSeq( objectDescriptors, &obj_reader );
    for(int i = 0; i < objectDescriptors->total; i++ )
    {
        const float* descriptor = (const float*)obj_reader.ptr;
        CV_NEXT_SEQ_ELEM( obj_reader.seq->elem_size, obj_reader );
        memcpy(obj_ptr, descriptor, length*sizeof(float));
        obj_ptr += length;
    }
    CvSeqReader img_reader;
	float* img_ptr = m_image.ptr<float>(0);
    cvStartReadSeq( imageDescriptors, &img_reader );
    for(int i = 0; i < imageDescriptors->total; i++ )
    {
        const float* descriptor = (const float*)img_reader.ptr;
        CV_NEXT_SEQ_ELEM( img_reader.seq->elem_size, img_reader );
        memcpy(img_ptr, descriptor, length*sizeof(float));
        img_ptr += length;
    }
    cv::Mat m_indices(objectDescriptors->total, 2, CV_32S);
    cv::Mat m_dists(objectDescriptors->total, 2, CV_32F);
    cv::flann::Index flann_index(m_image, cv::flann::KDTreeIndexParams(8));  // using 4 randomized kdtrees
    //cv::flann::Index flann_index(m_image, cv::flann::KDTreeIndexParams(1));  // using 4 randomized kdtrees
    //  flann_index.radiusSearch(m_object, m_indices, m_dists, 16, cv::flann::SearchParams(128));
    flann_index.knnSearch(m_object, m_indices, m_dists,2, cv::flann::SearchParams(32)); // maximum number of leafs checked

    int* indices_ptr = m_indices.ptr<int>(0);
    float* dists_ptr = m_dists.ptr<float>(0);
    for (int i=0;i<m_indices.rows;++i) { 
    	if (dists_ptr[2*i]<0.4*dists_ptr[2*i+1]) {
    		ptpairs.push_back(i);
    		ptpairs.push_back(indices_ptr[2*i]);
    	}
    }
}
bool isColinear(int num, CvPoint2D64f *p)
{
    //    Check colinearity of a set of pts input: p (pts to be checked)
    //    num (ttl number of pts) return true if some pts are coliner
    //        false if not
    
    int i,j,k; 
    bool iscolinear;
    double value; 
    CvMat *pt1 = cvCreateMat(3, 1, CV_64FC1);
    CvMat *pt2 = cvCreateMat(3, 1, CV_64FC1);
    CvMat *pt3 = cvCreateMat(3, 1, CV_64FC1);
    CvMat *line = cvCreateMat(3, 1, CV_64FC1);
    iscolinear = false;
    for(i=0; i<num-2; i++)
    {
        cvmSet(pt1,0,0,p[i].x); 
        cvmSet(pt1,1,0,p[i].y); 
        cvmSet(pt1,2,0,1);
        for(j=i+1; j<num-1; j++)
        {
            cvmSet(pt2,0,0,p[j].x); 
            cvmSet(pt2,1,0,p[j].y); 
            cvmSet(pt2,2,0,1); // compute the line connecting pt1 & pt2 cvCrossProduct(pt1, pt2, line);
            cvCrossProduct(pt1, pt2, line);
            for(k=j+1; k<num; k++)
            {
                cvmSet(pt3,0,0,p[k].x); 
                cvmSet(pt3,1,0,p[k].y);
                cvmSet(pt3,2,0,1); // check whether pt3 on the line value = cvDotProduct(pt3, line);
                value = cvDotProduct(pt3, line);
                if(abs(value) < 10e-2)
                {
                    iscolinear = true; 
                    break;
                }
            }
            if(iscolinear == true) 
                break;
        }
        if(iscolinear == true) 
            break;
    }
    cvReleaseMat(&pt1); 
    cvReleaseMat(&pt2); 
    cvReleaseMat(&pt3);
    cvReleaseMat(&line); 
    return iscolinear;
}

void ComputeH(int n, CvPoint2D64f *p1, CvPoint2D64f *p2, CvMat *H)
{
    int i; 
    CvMat *A = cvCreateMat(2*n, 9, CV_64FC1); 
    CvMat *U = cvCreateMat(2*n, 2*n, CV_64FC1); 
    CvMat *D = cvCreateMat(2*n, 9, CV_64FC1); 
    CvMat *V = cvCreateMat(9, 9, CV_64FC1);
    cvZero(A); 
    for(i=0; i<n;i++)
    {
        cvmSet(A,2*i,3,-p1[i].x);
        cvmSet(A,2*i,4,-p1[i].y); 
        cvmSet(A,2*i,5,-1); 
        cvmSet(A,2*i,6,p2[i].y*p1[i].x); 
        cvmSet(A,2*i,7,p2[i].y*p1[i].y);
        cvmSet(A,2*i,8,p2[i].y);
        
        // 2*i+1 row
        cvmSet(A,2*i+1,0,p1[i].x); 
        cvmSet(A,2*i+1,1,p1[i].y); 
        cvmSet(A,2*i+1,2,1); 
        cvmSet(A,2*i+1,6,-p2[i].x*p1[i].x); 
        cvmSet(A,2*i+1,7,-p2[i].x*p1[i].y); 
        cvmSet(A,2*i+1,8,-p2[i].x);
    }
    cvSVD(A, D, U, V, CV_SVD_U_T|CV_SVD_V_T);
    for(i=0; i<9; i++)
    {
        cvmSet(H, i/3, i%3, cvmGet(V, 8, i));
    }
    cvReleaseMat(&A);
    cvReleaseMat(&U); 
    cvReleaseMat(&D);
    cvReleaseMat(&V);
}

int ComputeNumberOfInliers(int num, CvPoint2D64f *p1, CvPoint2D64f *p2, CvMat *H, CvMat *inlier_mask, double *dist_std)
{
    int i, num_inlier; double curr_dist,curr_dist1,curr_dist2 , sum_dist, mean_dist;
    CvPoint2D64f tmp_pt;
    CvMat *dist = cvCreateMat(num, 1, CV_64FC1); 
    CvMat *x = cvCreateMat(3,1,CV_64FC1);
    CvMat *xp = cvCreateMat(3,1,CV_64FC1); 
    CvMat *pt = cvCreateMat(3,1,CV_64FC1); 
    CvMat *invH = cvCreateMat(3,3,CV_64FC1);
    cvInvert(H, invH);
    sum_dist = 0; 
    num_inlier = 0; 
    cvZero(inlier_mask); 
    for(i=0; i<num; i++){
        
        cvmSet(x,0,0,p1[i].x); 
        cvmSet(x,1,0,p1[i].y); 
        cvmSet(x,2,0,1); // initial point x'
        
        cvmSet(xp,0,0,p2[i].x); 
        cvmSet(xp,1,0,p2[i].y); 
        cvmSet(xp,2,0,1);
        cvMatMul(H, x, pt);
        tmp_pt.x = (int)(cvmGet(pt,0,0)/cvmGet(pt,2,0));
        tmp_pt.y = (int)(cvmGet(pt,1,0)/cvmGet(pt,2,0)); 
        curr_dist1 = pow(tmp_pt.x-p2[i].x, 2.0) + pow(tmp_pt.y-p2[i].y, 2.0); // d(x, invH x') 
        cvMatMul(invH, xp, pt);
        tmp_pt.x = (int)(cvmGet(pt,0,0)/cvmGet(pt,2,0)); 
        tmp_pt.y = (int)(cvmGet(pt,1,0)/cvmGet(pt,2,0)); 
        curr_dist2 = pow(tmp_pt.x-p1[i].x, 2.0) + pow(tmp_pt.y-p1[i].y, 2.0);
        curr_dist=sqrt(curr_dist1+curr_dist2);
       // cout<<curr_dist<<endl;
        if(curr_dist < T_DIST)
        { // an inlier
            num_inlier++; 
            cvmSet(inlier_mask,i,0,1); 
            cvmSet(dist,i,0,curr_dist); 
            sum_dist += curr_dist;
        }
    }
    
    mean_dist = sum_dist/(double)num_inlier; 
    *dist_std = 0; 
    for(i=0; i<num; i++)
    {
        if(cvmGet(inlier_mask,i,0) == 1) 
            *dist_std += pow(cvmGet(dist,i,0)-mean_dist,2.0);
    } 
    *dist_std /= (double) (num_inlier -1);
    
    cvReleaseMat(&dist);
    cvReleaseMat(&x);
    cvReleaseMat(&xp); 
    cvReleaseMat(&pt);
    cvReleaseMat(&invH);
    return num_inlier;
}
void Normalization(int num, CvPoint2D64f *p, CvMat *T)
{
    double scale, tx, ty;
    double meanx, meany;
    double value;
    int i; 
    CvMat *x = cvCreateMat(3,1,CV_64FC1); 
    CvMat *xp = cvCreateMat(3,1,CV_64FC1);
    meanx = 0; 
    meany = 0;
    for(i=0; i<num; i++)
    {
        meanx += p[i].x; 
        meany += p[i].y;
    } 
    meanx /= (double)num; 
    meany /= (double)num;
    value= 0;
    for(i=0; i<num; i++)
        value += sqrt(pow(p[i].x-meanx, 2.0) + pow(p[i].y-meany, 2.0));
    value /= (double)num;
    scale = sqrt(2.0)/value; 
    tx = -scale * meanx;
    ty = -scale * meany;
    cvZero(T); 
    cvmSet(T,0,0,scale);
    cvmSet(T,0,2,tx); 
    cvmSet(T,1,1,scale); 
    cvmSet(T,1,2,ty); 
    cvmSet(T,2,2,1.0);
    for(i=0; i<num; i++)
    { 
        cvmSet(x,0,0,p[i].x);
        cvmSet(x,1,0,p[i].y); 
        cvmSet(x,2,0,1.0); 
        cvMatMul(T,x,xp); 
        p[i].x = cvmGet(xp,0,0)/cvmGet(xp,2,0);
        p[i].y = cvmGet(xp,1,0)/cvmGet(xp,2,0);
    }
    cvReleaseMat(&x); 
    cvReleaseMat(&xp);
}
void RANSAC_homography(int num, CvPoint2D64f *m1, CvPoint2D64f *m2, CvMat *H, CvMat *inlier_mask)
{
    
    int i,j; 
    int N = 1, s = 4, sample_cnt = 0; 
    double e, p = 0.8; 
    int numinlier, MAX_num; 
    double curr_dist_std, dist_std; 
    bool iscolinear; 
    CvPoint2D64f *curr_m1 = new CvPoint2D64f[s]; 
    CvPoint2D64f *curr_m2 = new CvPoint2D64f[s]; 
    int *curr_idx = new int[s];
    CvMat *curr_inlier_mask = cvCreateMat(num,1,CV_64FC1);
    CvMat *curr_H = cvCreateMat(3,3,CV_64FC1);
    CvMat *T1 = cvCreateMat(3,3,CV_64FC1);
    CvMat *T2 = cvCreateMat(3,3,CV_64FC1);
    CvMat *invT2 = cvCreateMat(3,3,CV_64FC1); 
    CvMat *tmp_pt = cvCreateMat(3,1,CV_64FC1);
    // RANSAC algorithm (reject outliers and obtain the best H)
    //srand(time(0)); 
    MAX_num = -1;
    for (int test=0;test<10000;test++)
    {
    //while(N > sample_cnt)
    // {
        // for a randomly chosen non-colinear correspondances
        iscolinear = true;
        while(iscolinear == true)
        {
            iscolinear = false;
            for(i=0; i<s; i++)
            {
                curr_idx[i] = rand()%num; 
                for(j=0; j<i; j++)
                {
                    if(curr_idx[i] == curr_idx[j])
                    {
                        iscolinear = true; 
                        break;
                    }
                }
                
                if(iscolinear == true)
                    break; 
                curr_m1[i].x = m1[curr_idx[i]].x; 
                curr_m1[i].y = m1[curr_idx[i]].y;
                curr_m2[i].x = m2[curr_idx[i]].x; 
                curr_m2[i].y = m2[curr_idx[i]].y;
            }
            // Check whether these points are colinear
            if(iscolinear == false) 
                iscolinear = isColinear(s, curr_m1);
        }
        // Nomalized DLT 
        Normalization(s, curr_m1, T1); //curr_m1 <- T1 * curr_m1 
        Normalization(s, curr_m2, T2); //curr_m2 <- T2 * curr_m2
        // Compute the homography matrix H = invT2 * curr_H * T1
        ComputeH(s, curr_m1, curr_m2, curr_H); 
        cvInvert(T2, invT2);
        cvMatMul(invT2, curr_H, curr_H); // curr_H <- invT2 * curr_H 
        cvMatMul(curr_H, T1, curr_H);	// curr_H <- curr_H * T1
        // Calculate the distance for each putative correspondence // and compute the number of inliers 
        numinlier = ComputeNumberOfInliers(num,m1,m2,curr_H,curr_inlier_mask,&curr_dist_std);
        //cout<<numinlier<<endl;
        // Update a better H
        
       // if(numinlier > MAX_num || (numinlier == MAX_num && curr_dist_std < dist_std))
      if(numinlier > MAX_num) 
        {
            MAX_num = numinlier; 
            cvCopy(curr_H, H); 
            cvCopy(curr_inlier_mask, inlier_mask); 
            dist_std = curr_dist_std;
        }
   //     
   //     e = 1 - (double)numinlier / (double)num;
   //     N = (int)(log(1-p)/log(1-pow(1-e,s))); 
   //     cout<<"test N"<<N<<endl;
   //     sample_cnt++;
    }
    //   delete curr_m1, curr_m2, curr_idx; 
    cout<<MAX_num<<endl;
    cvReleaseMat(&curr_H); 
    cvReleaseMat(&T1); 
    cvReleaseMat(&T2); 
    cvReleaseMat(&invT2); 
    cvReleaseMat(&tmp_pt);
    cvReleaseMat(&curr_inlier_mask);
}

void plot_epipolar_line(IplImage *I_in, CvMat* correspond_lines)
{
    int width = I_in->width;
    int height = I_in->height;
    CvMat *line = cvCreateMat(3,1,CV_64FC1);
    CvMat *f1 = cvCreateMat(3,1,CV_64FC1);
    CvMat *f2 = cvCreateMat(3,1,CV_64FC1);
    CvMat *f3 = cvCreateMat(3,1,CV_64FC1);
    CvMat *f4 = cvCreateMat(3,1,CV_64FC1);
    CvMat *l1 = cvCreateMat(3,1,CV_64FC1);
    cvmSet(l1,0,0,0);
    cvmSet(l1,1,0,1);
    cvmSet(l1,2,0,0);
    CvMat *l2 = cvCreateMat(3,1,CV_64FC1);
    cvmSet(l2,0,0,0);
    cvmSet(l2,1,0,1);
    cvmSet(l2,2,0,-(height-1));
    CvMat *l3 = cvCreateMat(3,1,CV_64FC1);
    cvmSet(l3,0,0,1);
    cvmSet(l3,1,0,0);
    cvmSet(l3,2,0,0);
    CvMat *l4 = cvCreateMat(3,1,CV_64FC1);
    cvmSet(l4,0,0,1);
    cvmSet(l4,1,0,0);
    cvmSet(l4,2,0,-(width-1)); 
    float minmum_value;
    float pt_array[4][3];
    
    bool Flag1 = false, Flag2 = false;
    CvPoint point1, point2;
    int x, y;
    int i;
    
    
    for (int index=0; index<correspond_lines->rows; index++)
    {
        for (int j=0; j<3;j++)
            
        {
            minmum_value=(cvGet2D(correspond_lines,index,j).val[0]); 
            if (abs(minmum_value)<1e-8)   // once x_axis too small 
            {
                cvmSet(line,j,0,0) ;
            }
            else
            {
                cvmSet(line,j,0,(cvGet2D(correspond_lines,index,j).val[0])) ;
            }
            
        }  
        cvCrossProduct(line, l1, f1);
        cvCrossProduct(line, l2, f2);
        cvCrossProduct(line, l3, f3);
        cvCrossProduct(line, l4, f4);
        for (i=0;i<5;i++)
        {
            if (i==0)
            { 
                for (int j=0;j<=2;j++)
                {
                    pt_array[i][j]= (cvGet2D(f1,j,0).val[0]);   
                }
                
            }
            if (i==1)
            { 
                for (int j=0;j<=2;j++)
                {
                    pt_array[i][j]= (cvGet2D(f2,j,0).val[0]);   
                }
            }
            if (i==2)
            { 
                for (int j=0;j<=2;j++)
                {
                    pt_array[i][j]= (cvGet2D(f3,j,0).val[0]);   
                }
            }
            if (i==3)
            { 
                for (int j=0;j<=2;j++)
                {
                    pt_array[i][j]= (cvGet2D(f4,j,0).val[0]);   
                }
            }
        }
        for(i = 0; i < 4; i++)
        {
            if(pt_array[i][2] == 0)
            {
                x = 50000000;
                y = 50000000;
            }
            else
            {
                x = cvRound(pt_array [i][0] / pt_array [i][2]);
                y = cvRound(pt_array [i][1] / pt_array [i][2]);
                
            }
            if((x >= 0 && x < width) && (y >= 0 && y < height))
            {
                if(Flag1 == false)
                {
                    point1 = cvPoint(x, y);
                    Flag1 = true;
                }
                else if(Flag2 == false && (point1.x != x || point1.y != y))
                {
                    point2 = cvPoint(x,y);
                    Flag2 = true;
                }
            }
        }
        if(Flag1 == true && Flag2 == true)
        {
            cvLine(I_in, point1, point2, cvScalar(0, 0, 255), 2);
            Flag1 = false;
            Flag2 = false;
        }
        else
        {
            std::cout<<"can't plot epipolar line";
            exit(0);
        }
    }  
} 
IplImage* plot_two_images(IplImage *IGray,IplImage *IGray1,CvPoint *corners1,CvPoint *corners3, int &vector_index)
{
   

    CvPoint newmatched;       
    int  ttw =  IGray->width+ IGray->width;
    int  ttl  =  IGray->height;
   
    IplImage* Imagedisplay  = cvCreateImage(cvSize(2*IGray->width,IGray->height), IGray->depth, IGray->nChannels );
    for (int i=0; i<ttl;i++){
        for (int j=0; j<ttw; j++){
            if (j< IGray->width)
            {
                cvSet2D(Imagedisplay,i,j,cvGet2D(IGray,i,j));
            }
            if (j>IGray->width)
            {
                cvSet2D(Imagedisplay,i,j,cvGet2D(IGray1,i,j-(IGray1->width)));
            }
        }
    }
    
    
    for (int i=0;i<vector_index; i++)
    {           
        newmatched.x= (int)(corners3[i].x+(IGray->width));
        newmatched.y= (int)(corners3[i].y);
        cvLine(Imagedisplay, 
               cvPoint( corners1[i].x-1, corners1[i].y ), 
               cvPoint( newmatched.x, newmatched.y ), 
               CV_RGB(0,0,0)
               );
        cvCircle(Imagedisplay, newmatched, 3, CV_RGB(255,255,255));
        cvCircle(Imagedisplay, corners1[i], 3, CV_RGB(255,255,255));
        
    }    return (Imagedisplay);
}
IplImage* plot_two_imagesf(IplImage *IGray,IplImage *IGray1,Point2f *corners1,Point2f *corners3, int &vector_index)
{
    CvPoint newmatched;       
    int  ttw =  IGray->width+ IGray->width;
    int  ttl  =  IGray->height;
    IplImage* Imagedisplay  = cvCreateImage(cvSize(2*IGray->width,IGray->height), IGray->depth, IGray->nChannels );
    for (int i=0; i<ttl;i++){
        for (int j=0; j<ttw; j++){
            if (j< IGray->width)
            {
                cvSet2D(Imagedisplay,i,j,cvGet2D(IGray,i,j));
            }
            if (j>IGray->width)
            {
                cvSet2D(Imagedisplay,i,j,cvGet2D(IGray1,i,j-(IGray1->width)));
            }
        }
    }
    for (int i=0;i<vector_index; i++)
    {           
        newmatched.x= (int)(corners3[i].x+(IGray->width));
        newmatched.y= (int)(corners3[i].y);
        cvLine(Imagedisplay, 
               cvPoint( corners1[i].x-1, corners1[i].y ), 
               cvPoint( newmatched.x, newmatched.y ), 
               CV_RGB(0,0,0)
               );
        cvCircle(Imagedisplay, newmatched, 3, CV_RGB(255,255,255));
        cvCircle(Imagedisplay, corners1[i], 3, CV_RGB(255,255,255));
        
    }    return (Imagedisplay);
}
void cvpoint2vector(int num_pts, CvPoint* matching_point, v3_t *point)
{
    double x,y;
    for (int i=0; i<num_pts ;i++ )
    {
        x= matching_point[i].x;
        y= matching_point[i].y;
        point[i]= v3_new(x,y,1.0);
    }
}
void float_cvpoint2vector(int num_pts, Point2f* matching_point, v3_t *point)
{
    double x,y;
    for (int i=0; i<num_pts ;i++ )
    {
        x= matching_point[i].x;
        y= matching_point[i].y;
        point[i]= v3_new(x,y,1.0);
    }
}


//CvMat *compute_epipplar_line(double* F,int x,v3_t* l_ptinlier, v3_t* r_ptinlier)
//{
//    CvMat *correspondent_lines=cvCreateMat(x,3, CV_32FC1);
//    CvMat *l_points=cvCreateMat(x,2, CV_32FC1);
//    CvMat *r_points=cvCreateMat(x,2, CV_32FC1);
//    CvMat *F_matrix=cvCreateMat(3,3, CV_32FC1);
//    for (int i=0;i<x;i++)
//    {
//        cvmSet(l_points, i, 0, l_ptinlier[i].p[0]);
//        cvmSet(l_points, i, 1, l_ptinlier[i].p[1]);
//        cvmSet(r_points, i, 0, r_ptinlier[i].p[0]);
//        cvmSet(r_points, i, 1, r_ptinlier[i].p[1]);
//    }
//    
//    cvmSet(F_matrix,0,0, F[0]);
//    cvmSet(F_matrix,0,1, F[1]);
//    cvmSet(F_matrix,0,2, F[2]);
//    cvmSet(F_matrix,1,0, F[3]);
//    cvmSet(F_matrix,1,1, F[4]);
//    cvmSet(F_matrix,1,2, F[5]);
//    cvmSet(F_matrix,2,0, F[6]);
//    cvmSet(F_matrix,2,1, F[7]);
//    cvmSet(F_matrix,2,2, F[8]);
//    
//    
//    cvComputeCorrespondEpilines(l_points,1,F_matrix,correspondent_lines);
//    cout<<cvmGet(correspondent_lines,3,0)<<" "<<cvmGet(correspondent_lines,3,1)<<" "<<cvmGet(correspondent_lines,3,2)<<endl;
//    return(correspondent_lines);
//}


