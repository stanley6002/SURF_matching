

#include "F_matrix.h"


void F_matrix_process (int num_pts, v3_t* r_pt, v3_t* l_pt,double *F, int num_trial, int F_threshold, int essential, matched* refined_pts, int ith_pair)
{
    int F_estimated;
//    int estimate_fmatrix_ransac_matches(int num_pts, v3_t *a_pts, v3_t *b_pts, 
//                                        int num_trials, double threshold, 
//                                        double success_ratio,
//                                        int essential, double *F)
    //double success_ratio;
    F_estimated= estimate_fmatrix_ransac_matches(num_pts, r_pt, l_pt,1000,2.0,2.0,essential, F); 
    //matrix_print(3, 3, F);
    double  e1_tmp[3], e2_tmp[3];
   // F_estimated= estimate_fmatrix_linear(num_pts,r_pt,l_pt, 
   //                             1,F,  e1_tmp, e2_tmp); 
    double error_brefine=0;

    vector<int> inliers;
    
    for (int i=0; i<num_pts;i++)
    {
    double distance = fmatrix_compute_residual(F,r_pt[i],l_pt[i]);
       
        if (distance<100)
        { 
            //cout<<distance1<<endl;
        inliers.push_back(i);
        //cout<< l_pt[i].p[1]<<" "<<r_pt[i].p[1]<<" "<<distance<<" "<<distance1<<endl;
        //    cout<< l_pt[i].p[0]<<" "<<l_pt[i].p[1]<<" "<<r_pt[i].p[0]<<" "<<r_pt[i].p[1]<<endl;
        //  error_brefine= distance+error_brefine;
        //  cout<<"Geometry_distance "<<distance<<endl;
        }
    }
    
    cout<<"feature_point_Before"<<num_pts<<" "<< "feature_point_screened by f_matrix "<<(int)inliers.size()<<endl;
    //cout<< error_brefine/(int)inliers.size()<<endl;
    v3_t *l_ptinlier= new v3_t[(int)inliers.size()];
    v3_t *r_ptinlier= new v3_t[(int)inliers.size()];
    for (int i=0; i<(int)inliers.size();i++)
    {
        //CvPoint newmatched;  
        l_ptinlier[i]= l_pt[inliers[i]];
        r_ptinlier[i]= r_pt[inliers[i]];
        //newmatched.x= (int)l_ptinlier[i].p[0];
        //newmatched.y= (int)l_ptinlier[i].p[1];
        //cvCircle(IGray1, newmatched, 3, CV_RGB(255,255,255));
        //cvCircle(IGray_l, newmatched, 3, CV_RGB(255,255,255));
    }
    double F_refine[9];
    //F_estimated= estimate_fmatrix_ransac_matches((int)inliers.size(),r_ptinlier,l_ptinlier, 
    //                                             50, 1 ,1,essential, F_refine); 
    //cout<<"Fundamental_matrix"<<endl;
    //matrix_print(3, 3, F_refine);
    //for (int i=0; i<num_pts;i++)
    //{
    //    double distance = fmatrix_compute_residual(F_refine,r_pt[i],l_pt[i]);
    //    if (distance<1)
    //    { 
        //cout<<distance<<endl;
        //cout<< l_pt[i].p[0]<<" "<<l_pt[i].p[1]<<" "<<r_pt[i].p[0]<<" "<<r_pt[i].p[1]<<" "<<distance<<endl;
    //    }
    //}
    
    double F0[9];
    memcpy(F0,F,sizeof(double)*9);
    if (!essential)
    {
        for (int i=0; i<(int)inliers.size();i++)
        {
            //CvPoint newmatched;  
            l_ptinlier[i]= l_pt[inliers[i]];
            r_ptinlier[i]= r_pt[inliers[i]];
            //newmatched.x= (int)l_ptinlier[i].p[0];
            //newmatched.y= (int)l_ptinlier[i].p[1];
            //cvCircle(IGray1, newmatched, 3, CV_RGB(255,255,255));
            //cvCircle(IGray_l, newmatched, 3, CV_RGB(255,255,255));
        }
       refine_fmatrix_nonlinear_matches((int)inliers.size(), r_ptinlier, l_ptinlier, F0, F); 
    } 
    cout<<"nonlinear_fundamental_matrix: "<<endl;
    matrix_print(3,3,F);
    vector<int> non_inliers;
    for (int i=0; i<num_pts;i++)
    {
        double distance = fmatrix_compute_residual(F,r_pt[i],l_pt[i]);
        // double distance= fmatrix_compute_distance(F,  r_pt[i],  l_pt[i]); 
      
        if (distance<100)
        {
            non_inliers.push_back(i);
            // cout<< l_pt[i].p[0]<<" "<<l_pt[i].p[1]<<" "<<r_pt[i].p[0]<<" "<<r_pt[i].p[1]<<endl;
            //  cout<<"Geometry_distance "<<distance<<endl;
        }
    }
    v3_t *l_ptinlier_nonlinear= new v3_t[(int)non_inliers.size()];
    v3_t *r_ptinlier_nonlinear= new v3_t[(int)non_inliers.size()];
    for (int i=0; i<(int)non_inliers.size();i++)
    {
        //CvPoint newmatched;  
        l_ptinlier_nonlinear[i]= l_pt[non_inliers[i]];
        r_ptinlier_nonlinear[i]= r_pt[non_inliers[i]];
        //newmatched.x= (int)l_ptinlier[i].p[0];
        //newmatched.y= (int)l_ptinlier[i].p[1];
        //cvCircle(IGray1, newmatched, 3, CV_RGB(255,255,255));
        //cvCircle(IGray_l, newmatched, 3, CV_RGB(255,255,255));
        //cout<<"matchedPoint "<<l_ptinlier_nonlinear[i].p[0]<<" "<<l_ptinlier_nonlinear[i].p[1]<<" "<<r_ptinlier_nonlinear[i].p[0]<<" "<<r_ptinlier_nonlinear[i].p[1]<<endl;
    
    
    }

    //push_backpts(l_ptinlier, r_ptinlier, refined_pts,(int)inliers.size(),ith_pair);
     push_backpts(l_ptinlier_nonlinear, r_ptinlier_nonlinear, refined_pts,(int)non_inliers.size(),ith_pair);
    double error;
    double sum=0;
    for (int i=0;i<(int)inliers.size();i++)
    {
        error=fmatrix_compute_residual(F, r_ptinlier[i],l_ptinlier[i]) ;
        sum+=error;
    }
    //cout<<"nonlinear"<<endl;
    //cout<<sum/((int)inliers.size())<<endl;
    //cout<<endl;
}


void push_backpts(v3_t *lpts, v3_t*rpts, matched *pts, int inlier_size, int ith_pair)
{
    for (int i=0;i<inlier_size;i++)
    {
  
        pts[ith_pair].L_pts.push_back(lpts[i]);
        pts[ith_pair].R_pts.push_back(rpts[i]);
    }
}


void pop_backpts_WI(v2_t*lpts, v2_t*rpts, matched *pts, int ith_pair)
{
    for (int i=0; i<(int)pts[0].L_pts.size();i++)
    {
        lpts[i].p[0]= pts[ith_pair].L_pts[i].p[0];
        lpts[i].p[1]= pts[ith_pair].L_pts[i].p[1];       
        rpts[i].p[0]= pts[ith_pair].R_pts[i].p[0];
        rpts[i].p[1]= pts[ith_pair].R_pts[i].p[1];
    }
}

void pop_backpts(v3_t*lpts, v3_t*rpts, matched *pts)
{
    for (int i=0; i<(int)pts[0].L_pts.size();i++)
    {
        lpts[i]= pts[0].L_pts[i];
        rpts[i]= pts[0].R_pts[i];
    }
}

void pushback_Fmatrix(double* F, F_key_matrix *F_matrix,int i)
{
    F_matrix[i].p[0]=F[0];
    F_matrix[i].p[1]=F[1];
    F_matrix[i].p[2]=F[2];
    F_matrix[i].p[3]=F[3];
    F_matrix[i].p[4]=F[4];
    F_matrix[i].p[5]=F[5];
    F_matrix[i].p[6]=F[6];
    F_matrix[i].p[7]=F[7];
    F_matrix[i].p[8]=F[8];
}
void  EstimateTransform(v2_t*lpts, v2_t*rpts, Motion MotionSelect ,int num_size,  int _round /*128 m_homography_rounds*/, 
                         int _homography_threshold/*6.0m_homography_threshold*/,double *H, double *K)
{
    int min_matches;
    switch (MotionSelect) 
    {
        case MotionRigid:
            min_matches = 3;
            break;
        case MotionHomography:
            min_matches = 4;
            break;
    }

    int *match_idxs = new int[min_matches];
    int num_matches = num_size;
    int max_inliers = 0;
    double Mbest[9];
    
    if (num_matches < min_matches) {
        std::vector<int> empty;
        printf("Cannot estimate rigid transform\n");
        return empty;
    }
    
    v3_t *r_pts_ran = new v3_t[min_matches];
    v3_t *l_pts_ran = new v3_t[min_matches];
    double *weight = new double[min_matches];
    for (int round = 0; round < 64 ; round++) 
    {
        for (int i = 0; i < min_matches; i++) {
            bool found;
            int idx;
            
            do {
                found = true;
                idx = rand() % num_matches;
                
                for (int j = 0; j < i; j++) {
                    if (match_idxs[j] == idx) {
                        found = false;
                        break;
                    }
                }
            } while (!found);
            
            match_idxs[i] = idx;
        }
     
        /* Solve for the motion */
		
        for (int i = 0; i < min_matches; i++) 
        {
           int index = match_idxs[i];
            
            l_pts_ran[i].p[0] = lpts[index].p[0];
            l_pts_ran[i].p[1] = lpts[index].p[1];
            l_pts_ran[i].p[2] = 1.0;
		    
            r_pts_ran[i].p[0] = rpts[index].p[0];
            r_pts_ran[i].p[1] = rpts[index].p[1];
            r_pts_ran[i].p[2] = 1.0;
            
            weight[i] = 1.0;
        }
    
                
        double M_current[9];
    
        
        switch (MotionSelect) {
            case MotionRigid:
            {
                double R[9], T[9], Tout[9], scale;
                align_horn(min_matches, r_pts_ran, l_pts_ran, R, T, Tout, &scale, weight);
                memcpy(M_current , Tout, 9 * sizeof(double));
                break;
            }
                
            case MotionHomography: 
            {
               //align_homography(min_matches, r_pts, l_pts, Mcurr, 0);
               // break;
            }
        }
		
        
        std::vector<int> inliers;
        int num_inliers = CountInliers(lpts, rpts, M_current, 
                                       _homography_threshold, inliers, num_size);
        //cout<<"# inliers " <<num_inliers <<endl;
        if (num_inliers > max_inliers) 
        {
            max_inliers = num_inliers;
            memcpy(Mbest, M_current, 9 * sizeof(double));
        }
    }
    matrix_print(3,3,Mbest);


}

static int CountInliers(const v2_t* lpts, 
                        const v2_t* rpts, 
                        double *M, double thresh, std::vector<int> &inliers, int num_size)
{
    inliers.clear();
    int count = 0;
    
    for (unsigned int i = 0; i < num_size; i++) 
    {
        /* Determine if the ith feature in f1, when transformed by M,
         * is within RANSACthresh of its match in f2 (if one exists)
         *
         * if so, increment count and append i to inliers */
       
        double p[3];
        
        p[0] = lpts[i].p[0];
        p[1] = lpts[i].p[1];
        p[2] = 1.0;
        
        double q[3];
        matrix_product(3, 3, 3, 1, M, p, q);
        
        double qx = q[0] / q[2];
        double qy = q[1] / q[2];
        
        double dx = qx - rpts[i].p[0];
        double dy = qy - rpts[i].p[1];
        
        double dist = sqrt(dx * dx + dy * dy);
      
        if (dist <= thresh) 
        {
            count++;
            inliers.push_back(i);
        }
    }
    
    return count;
    
}
double align_horn(int n, v3_t *right_pts, v3_t *left_pts, 
                  double *R, double *T, 
                  double *Tout, double *scale, double *weight) {
    int i;
    v3_t right_centroid = v3_new(0.0, 0.0, 0.0);
    v3_t left_centroid = v3_new(0.0, 0.0, 0.0);
    double M[2][2] = { { 0.0, 0.0 }, 
        { 0.0, 0.0 } };
    double MT[2][2];
    double MTM[2][2];
    double eval[2], sqrteval[2];
    double evec[2][2];
    double S[2][2], Sinv[2][2], U[2][2];
    double Tcenter[3][3] = { { 1.0, 0.0, 0.0 },
        { 0.0, 1.0, 0.0 },
        { 0.0, 0.0, 1.0 } };
    
    double Ttmp[3][3];
    
    double sum_num, sum_den, RMS_sum;
    
#if 1
    double weight_sum = 0.0;
    
    if (weight == NULL) {
        weight_sum = n;
        
        for (i = 0; i < n; i++) {
            right_centroid = 
            v3_add(right_centroid, right_pts[i]);
            left_centroid = 
            v3_add(left_centroid, left_pts[i]);
        }
        
        right_centroid = v3_scale(1.0 / weight_sum, right_centroid);
        left_centroid = v3_scale(1.0 / weight_sum, left_centroid);        
    } else {
        /* Compute the weighted centroid of both point sets */
        for (i = 0; i < n; i++) {
            right_centroid = 
            v3_add(right_centroid, v3_scale(weight[i], right_pts[i]));
            left_centroid = 
            v3_add(left_centroid, v3_scale(weight[i], left_pts[i]));
            weight_sum += weight[i];
            
        }
        
        right_centroid = v3_scale(1.0 / weight_sum, right_centroid);
        left_centroid  = v3_scale(1.0 / weight_sum, left_centroid);
    
    }
#else
    /* Calculate the centroid of both sets of points */
    for (i = 0; i < n; i++) {
        right_centroid = v3_add(right_centroid, right_pts[i]);
        left_centroid = v3_add(left_centroid, left_pts[i]);
    }
    
    right_centroid = v3_scale(1.0 / n, right_centroid);
    left_centroid = v3_scale(1.0 / n, left_centroid);
#endif
    
    /* Compute the scale */
    sum_num = sum_den = 0.0;
    
    for (i = 0; i < n; i++) {
        v3_t r = v3_sub(right_centroid, right_pts[i]);
        v3_t l = v3_sub(left_centroid, left_pts[i]);
        
        sum_num = v3_magsq(r);
        sum_den = v3_magsq(l);
    }
    
    *scale = sqrt(sum_num / sum_den);
    
    /* Fill in the matrix M */
    for (i = 0; i < n; i++) 
    {
        v3_t r = v3_sub(right_centroid, right_pts[i]);
        v3_t l = v3_sub(left_centroid, left_pts[i]);
               
        if (weight != NULL) {
            M[0][0] += Vx(r) * Vx(l);
            M[0][1] += Vx(r) * Vy(l);
            M[1][0] += Vy(r) * Vx(l);
            M[1][1] += Vy(r) * Vy(l);
        } else {
            M[0][0] += Vx(r) * Vx(l);
            M[0][1] += Vx(r) * Vy(l);
            M[1][0] += Vy(r) * Vx(l);
            M[1][1] += Vy(r) * Vy(l);
        }
    }
    
    /* Compute MTM */
    matrix_transpose(2, 2, (double *)M, (double *)MT);
    matrix_product(2, 2, 2, 2, (double *)MT, (double *)M, (double *)MTM);
    
    /* Calculate Sinv, the inverse of the square root of MTM */
    dgeev_driver(2, (double *)MTM, (double *)evec, eval);
    
    /* MTM = eval[0] * evec[0]T * evec[0] + eval[1] * evec[1]T * evec[1] */
    /* S = sqrt(eval[0]) * evec[0]T * evec[0] + sqrt(eval[1]) * evec[1]T * evec[1] */
    sqrteval[0] = sqrt(eval[0]);
    sqrteval[1] = sqrt(eval[1]);
    
    S[0][0] = 
    (sqrteval[0]) * evec[0][0] * evec[0][0] +
    (sqrteval[1]) * evec[1][0] * evec[1][0];
    S[0][1] = 
    (sqrteval[0]) * evec[0][0] * evec[0][1] +
    (sqrteval[1]) * evec[1][0] * evec[1][1];
    S[1][0] = 
    (sqrteval[0]) * evec[0][1] * evec[0][0] +
    (sqrteval[1]) * evec[1][1] * evec[1][0];
    S[1][1] = 
    (sqrteval[0]) * evec[0][1] * evec[0][1] +
    (sqrteval[1]) * evec[1][1] * evec[1][1];
    
    Sinv[0][0] = 
    (1.0 / sqrteval[0]) * evec[0][0] * evec[0][0] +
    (1.0 / sqrteval[1]) * evec[1][0] * evec[1][0];
    Sinv[0][1] = 
    (1.0 / sqrteval[0]) * evec[0][0] * evec[0][1] +
    (1.0 / sqrteval[1]) * evec[1][0] * evec[1][1];
    Sinv[1][0] = 
    (1.0 / sqrteval[0]) * evec[0][1] * evec[0][0] +
    (1.0 / sqrteval[1]) * evec[1][1] * evec[1][0];
    Sinv[1][1] = 
    (1.0 / sqrteval[0]) * evec[0][1] * evec[0][1] +
    (1.0 / sqrteval[1]) * evec[1][1] * evec[1][1];
    
    // matrix_product(2, 2, 2, 2, (double *)S, (double *)Sinv, (double *)U);
    
    /* U = M * Sinv */
    matrix_product(2, 2, 2, 2, (double *)M, (double *)Sinv, (double *)U);
    
    /* Fill in the rotation matrix */
    R[0] = U[0][0]; R[1] = U[0][1]; R[2] = 0.0;
    R[3] = U[1][0], R[4] = U[1][1]; R[5] = 0.0;
    R[6] = 0.0;     R[7] = 0.0;     R[8] = 1.0;
    
    // memcpy(R, U, sizeof(double) * 4);
    
    /* Fill in the translation matrix */
    T[0] = T[4] = T[8] = 1.0;
    T[1] = T[3] = T[6] = T[7] = 0.0;
    T[2] = Vx(right_centroid);
    T[5] = Vy(right_centroid);
    
    Tcenter[0][0] = *scale;
    Tcenter[1][1] = *scale;
    Tcenter[0][2] = -*scale * Vx(left_centroid);
    Tcenter[1][2] = -*scale * Vy(left_centroid);
    
    matrix_product(3, 3, 3, 3, T, R, (double *)Ttmp);
    
#if 0
#if 0
    /* Do the scaling */
    Ttmp[0][0] *= *scale;
    Ttmp[0][1] *= *scale;
    Ttmp[0][2] *= *scale;
    Ttmp[1][0] *= *scale;
    Ttmp[1][1] *= *scale;
    Ttmp[1][2] *= *scale;
#else
    Tcenter[0][0] *= *scale;
    Tcenter[0][2] *= *scale;
    Tcenter[1][1] *= *scale;
    Tcenter[1][2] *= *scale;
#endif
#endif
    
    matrix_product(3, 3, 3, 3, (double *)Ttmp, (double *)Tcenter, Tout);
    
    T[2] = Vx(v3_sub(right_centroid, left_centroid));
    T[5] = Vy(v3_sub(right_centroid, left_centroid));
    
    
    /* Now compute the RMS error between the points */
    RMS_sum = 0.0;
    
    for (i = 0; i < n; i++) {
        v3_t r = v3_sub(right_centroid, right_pts[i]);
        v3_t l = v3_sub(left_centroid, left_pts[i]);
        v3_t resid;
        
        /* Rotate, scale l */
        v3_t Rl, SRl;
        
        Vx(Rl) = R[0] * Vx(l) + R[1] * Vy(l) + R[2] * Vz(l);
        Vy(Rl) = R[3] * Vx(l) + R[4] * Vy(l) + R[5] * Vz(l);
        Vz(Rl) = R[6] * Vx(l) + R[7] * Vy(l) + R[8] * Vz(l);
        
        SRl = v3_scale(*scale, Rl);
        
        resid = v3_sub(r, SRl);
        RMS_sum += v3_magsq(resid);
    }
    
    return sqrt(RMS_sum / n);
}









