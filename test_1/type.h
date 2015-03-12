//
//  type.h
//  test_1
//
//  Created by C-HChang on 2/25/14.
//  Copyright (c) 2014 C-HChang. All rights reserved.
//

#ifndef __type_h
#define __type_h
//#include <vector>
# include "vector.h"
using namespace std;

//typedef struct {
//    double p[3];
//} v3_t;
//
//typedef struct {
//    double p[2];
//} v2_t;




//inline v3_t v3_new(double x, double y, double z) {
//    v3_t v = { { x, y, z } };
//    return v;
//}

typedef struct
{
    vector<v3_t>R_pts;
    vector<v3_t>L_pts;
    
} matched;

#endif
