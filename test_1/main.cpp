//
//  main.cpp
//  test_1
//
//  Created by C-HChang on 2/11/14.
//  Copyright (c) 2014 C-HChang. All rights reserved.
//
//#include <iostream>                 // for std::cout
#include <utility>                    // for std::pair
#include <algorithm>                  // for std::for_each
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/numeric/ublas/matrix_sparse.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <vector>

#include "SURF_feature_matching.h"
#include "type.h"
#include <dirent.h>
#include <string.h>
#include <fstream>
#include <map>
#include "epipolar.h"
#include "vector.h"

using namespace std;
typedef struct {
    string node1;
    int node2;
} node_;



void pop_backpts_WI(v2_t*lpts, v2_t*rpts, matched *pts, int ith_pair);
void Store_feature2map (string &name1, string &name2 ,  int &i,  multimap<string , node_> &node);
void Plot_Graphstructure (vector<string> &vs, multimap<string , node_> &node);
void Split_string_function(vector<string>&vs, vector<string> &token);
pair<string, string> Find_first2frames(vector<string> token, multimap<string , node_> Feature_matching_Graph);
string Find_mostoverlapped_frame( string input, multimap<string , node_> Feature_matching_Graph);
void Generate_Graph_matrix (vector<string> token, multimap<string , node_> Feature_matching_Graph, vector<vector<int>> &Graph_matrix, vector<string> &pending_vector);
void PrimMST_driver(vector<vector<int>> Graph_matrix, vector<string> &pending_vector, vector<pair<int, int >> &MSTresult);
void PrimMST(vector<vector<int>> Graph_matrix, vector<vector<bool>> bool_matrix, vector<string> &pending_vector,vector<pair<int, int >> &MSTresult);
pair<int, int> Find_initialmaximumkeys(vector<vector<int>> Graph_matrix, vector<vector<bool>> &bool_matrix);
pair <int, int > Find_maximumkeys(pair <int, int > keypair, vector<vector<int>> Graph_matrix, vector<vector<bool>> &bool_matrix);
void set_boolmatrix (vector<vector<bool>> &bool_matrix,pair <int, int > keypair);
inline bool check_zero_colGraphmatrix(int col,vector<vector<int>> Graph_matrix) ; //check zero col ;trure = yes it zero col
void Plot_Prim_Graphstructure (vector<string> &vs,  vector<pair<int , int >> prim_result);
//#include <boost/graph/dijkstra_shortest_paths.hpp>
char *convert(const string & s);
using namespace boost;

int main(int,char*[])
{
    
    vector<string>  vs;
    ifstream ifs("/data.txt");
    string temp;
    while(getline( ifs, temp ))
        vs.push_back(temp);
    vector<char*>  vc;
    transform(vs.begin(), vs.end(), back_inserter(vc), convert);
    vector<string> token;
    Split_string_function(vs,token);
    // vector string for plotting graph //
    
    //
    uchar **imgdata;
    IplImage **img;
    int index = 0;
    
    
    multimap<string, node_> Feature_matching_Graph;
    
    
    img = (IplImage **)malloc((int)vs.size() * sizeof(IplImage *)); // Allocates memory to store just an IplImage pointer for each image loaded
    imgdata = (uchar **)malloc((int)vs.size() * sizeof(uchar *));
    
    for (int i=0;i<(int)vs.size();i++)
    {
        img[i] = cvLoadImage(vc[i], 1);
        
        if (!img[i]->imageData)
        {
            cout<<"can't open file"<<endl;
        }
        imgdata[i]=(uchar *)img[i]->imageData;
    }
    
    cout<<"total number of image "<<(int)vs.size()<<endl;
    for (int numimage_i=0;numimage_i<((int)vs.size())-1;numimage_i++)
    {
        cout<< "num_i "<<numimage_i <<endl;
        for (int numimage_j=numimage_i+1; numimage_j<((int)vs.size());numimage_j++)
        {
        
        cout<<"num_j "<<numimage_j<<endl;
        IplImage* IGray_r  = cvCreateImage(cvGetSize(img[numimage_j]), IPL_DEPTH_8U, 1);
        IplImage* IGray_l =  cvCreateImage(cvGetSize(img[numimage_i]), IPL_DEPTH_8U, 1);
        cvCvtColor( img[numimage_j],  IGray_r, CV_RGB2GRAY);
        cvCvtColor( img[numimage_i],  IGray_l, CV_RGB2GRAY);
        vector<int> featurematching_vector;
        matched * pts= new matched[1];
        featurematching_process(IGray_r,IGray_l,pts);
        int num_pts=(int)pts[0].R_pts.size();
        cout<<"total_# features "<<num_pts<<endl;
        if (num_pts>20)
        {
        //Store_feature2map(vs[numimage_i],vs[numimage_j], num_pts, Feature_matching_Graph);
        
            v2_t *lrefined_pt= new v2_t[num_pts];
            v2_t *rrefined_pt= new v2_t[num_pts];
            
            pop_backpts_WI(lrefined_pt,rrefined_pt,pts,0);
            double camera_1R[9];
            double camera_1t[3];
            double K2[9]= {732.0, 0.0,  0.0,
                0.000000e+00, 732.0, 0.0,
                0.000000e+00,  0.000000e+00,  1.000000e+00} ;
            double K1[9]= {732.0, 0.0,  0.0,
                0.000000e+00, 732.0,  0.0,
                0.000000e+00,  0.000000e+00,  1.000000e+00};
            double R[9];
            double t[3];
            double camera2t[3];
            double Homography_mat[6];
            
            
            
           //  EstimateTransform(lrefined_pt, rrefined_pt, MotionRigid , num_ofrefined_pts,  128  /*128 m_homography_rounds*/,
           //                   6/*6.0m_homography_threshold*/,Homography_mat, K1);
            
            
            for (int i=0;i<num_pts;i++)
            {
                v2_t p;
                v2_t q;
                p.p[0] = lrefined_pt[i].p[0]-400;
                p.p[1] = lrefined_pt[i].p[1]-300;
                q.p[0] = rrefined_pt[i].p[0]-400;
                q.p[1] = rrefined_pt[i].p[1]-300;
                
                lrefined_pt[i].p[0]=p.p[0];
                lrefined_pt[i].p[1]=p.p[1];
                rrefined_pt[i].p[0]=q.p[0];
                rrefined_pt[i].p[1]=q.p[1];
                //cout<< lrefined_pt[i].p[0]<<" "<< lrefined_pt[i].p[1]<<" "<< rrefined_pt[i].p[0]<<" "<<rrefined_pt[i].p[1]<<endl;
            }
            
            
            //cout<<"num_ofrefined_pts"<<num_ofrefined_pts<<endl;
            
            int num_inlier= EstimatePose5Point(lrefined_pt,rrefined_pt,num_pts,5000,1,K1, K2, R, t);
            
      
        Store_feature2map(token[numimage_i],token[numimage_j], num_pts, Feature_matching_Graph);
        }
        cvReleaseImage(&IGray_l);
        cvReleaseImage(&IGray_r);
        featurematching_vector.clear();
        
        }
    }
    cout<<"token_size "<<token.size()<<endl;
    
    vector<vector<int>> Graph_matrix;
    vector<string> pending_vector;
    vector<pair<int, int >> MSTresult;
    
    Generate_Graph_matrix (token,  Feature_matching_Graph, Graph_matrix, pending_vector);
    
    cout<<"pendingvec_size "<<pending_vector.size()<<endl;

    PrimMST_driver(Graph_matrix, pending_vector,MSTresult);
   
    cout<<"MST_structure << " <<MSTresult.size()<<endl;
    
    Plot_Graphstructure(token, Feature_matching_Graph);

    Plot_Prim_Graphstructure(token, MSTresult);
    
    string String_temp = token[0];
    
    
    pair<multimap<string, node_>::iterator, multimap<string, node_>::iterator> Gp1;

    Gp1 = Feature_matching_Graph.equal_range(String_temp);
    
    for (multimap<string, node_>::iterator it1 = Gp1.first;it1 != Gp1.second;++it1)
    
    {
        cout<<" [" <<(*it1).first << ", "<<(*it1).second.node1 <<", "<< (*it1).second.node2<<" ]"<<endl;
        
    }
    
    pair<string, string> first_pair ;
    first_pair = Find_first2frames(token, Feature_matching_Graph);
    cout<<"first_two_pairs of images"<<first_pair.first<<" "<<first_pair.second<<endl;
    string frame= Find_mostoverlapped_frame(first_pair.second,  Feature_matching_Graph);
    cout<<"3RD_ image_ "<<frame<<endl;

    free(img);
    free(imgdata);
    return 0;
}

char *convert(const string & s)
{
    char *pc = new char[s.size()+1];
    strcpy(pc, s.c_str());
    return pc;
}

void Store_feature2map (string &name1, string &name2 ,  int &i,  multimap<string , node_> &node)
{
    node_ node_vertex;
    node_vertex.node1= name2;
    node_vertex.node2=i;
    node.insert(pair<string, node_>(name1, node_vertex));
}
void Plot_Graphstructure (vector<string> &vs, multimap<string , node_> &Feature_matching_Graph)
{
   
    
        static char _header0 []= {"digraph structs{ "};
        static char _arrow []={"edge [arrowhead=none,arrowtail=dot];"};
        static char _header1 []= {"%s [margin=0 shape=box, style=filled, fillcolor=white, color=blue, label=<<TABLE border=\"0"};
        static char _header2 [] ={"\" cellborder=\"0\""};
        static char _header3 [] ={"> <TR><TD width=\"60\" "};
        static char _header4 [] ={"height=\"50\" "};
        static char _header5 [] ={"fixedsize=\"true\""};
        static char _header6 [] ={"><IMG SRC="};
        static char _header7 [] ={" scale=\"true\""};
        static char _header8 [] ={"/></TD><td><font point-size=\"10\">"};
        
        static char _header10[] ={"</font></td></TR></TABLE>>];"};
        int _num1=0;
        int _numimg=0;
        FILE *f = fopen("/Users/c-hchang/Desktop/te1_result.dot", "w");
    
    fprintf(f, _header0);
    fprintf(f, _arrow);
    fprintf(f,"\n");
    cout<<(int) vs.size()<<endl;
    for (int i=0;i< (int) vs.size();i++)
    {
        string String_temp = vs[i];
        
        if (f == NULL)
        {
            printf("Error opening file %s for writing\n", "ply_out");
            return;
        }
        
        /* Print the ply header */
        string str(vs[i]);
        
        fprintf(f, _header1,str.c_str());
        fprintf(f, _header2);
        fprintf(f, _header3);
        fprintf(f, _header4);
        fprintf(f, _header5);
        fprintf(f, _header6);
        //fprintf(f, "%s","\"/BD4.jpg\"");
       
        fprintf(f, "%s%s%s%s%s","\"","/",str.c_str(),".jpg","\"");
        fprintf(f, _header7);
        
        fprintf(f, _header8);
        fprintf(f, "%s",str.c_str());
        fprintf(f, _header10);
        fprintf(f,"\n");
    }
 
    
    for (int i=0;i<(int) vs.size();i++)
    {
      string String_temp_read;
      
      String_temp_read= vs[i];
      
      //string message ="/"+vs[i]+".jpg";
      
      string message =vs[i];
 
      pair<multimap<string, node_>::iterator, multimap<string, node_>::iterator> Gp1;
    
      Gp1 = Feature_matching_Graph.equal_range(message);
    
      for (multimap<string, node_>::iterator it1 = Gp1.first;it1 != Gp1.second;++it1)
      {
        //cout<<" [" <<(*it1).first << ", "<<(*it1).second.node1 <<", "<< (*it1).second.node2<<" ]"<<endl;
          string str_first((*it1).first);
          string str_second((*it1).second.node1);
          fprintf(f,"%s%s%s",str_first.c_str(),"->",str_second.c_str());
          fprintf(f,"\n");
      }

    }

      fprintf(f,"%s","}");
    
    
 fclose(f);

}
void Plot_Prim_Graphstructure (vector<string> &vs,  vector<pair<int , int >> prim_result)
{
    
    
    static char _header0 []= {"digraph structs{ "};
    static char _arrow []={"edge [arrowhead=none,arrowtail=dot];"};
    static char _header1 []= {"%s [margin=0 shape=box, style=filled, fillcolor=white, color=blue, label=<<TABLE border=\"0"};
    static char _header2 [] ={"\" cellborder=\"0\""};
    static char _header3 [] ={"> <TR><TD width=\"60\" "};
    static char _header4 [] ={"height=\"50\" "};
    static char _header5 [] ={"fixedsize=\"true\""};
    static char _header6 [] ={"><IMG SRC="};
    static char _header7 [] ={" scale=\"true\""};
    static char _header8 [] ={"/></TD><td><font point-size=\"10\">"};
    
    static char _header10[] ={"</font></td></TR></TABLE>>];"};
    static char _header11[] ={"  [color=blue,weight=1, penwidth=3];"};
    
    int _num1=0;
    int _numimg=0;
    FILE *f = fopen("/Users/c-hchang/Desktop/te1_prime_result.dot", "w");
    
    fprintf(f, _header0);
    fprintf(f, _arrow);
    fprintf(f,"\n");
    cout<<(int) vs.size()<<endl;
    for (int i=0;i< (int) vs.size();i++)
    {
        string String_temp = vs[i];
        
        if (f == NULL)
        {
            printf("Error opening file %s for writing\n", "ply_out");
            return;
        }
        
        /* Print the ply header */
        string str(vs[i]);
        
        fprintf(f, _header1,str.c_str());
        fprintf(f, _header2);
        fprintf(f, _header3);
        fprintf(f, _header4);
        fprintf(f, _header5);
        fprintf(f, _header6);
        //fprintf(f, "%s","\"/BD4.jpg\"");
        
        fprintf(f, "%s%s%s%s%s","\"","/",str.c_str(),".jpg","\"");
        fprintf(f, _header7);
        
        fprintf(f, _header8);
        fprintf(f, "%s",str.c_str());
        fprintf(f, _header10);
        fprintf(f,"\n");
    }
    
    
    for (int i=0;i<(int) prim_result.size();i++)
    {
        int _first  = prim_result[i].first;
        int _second = prim_result[i].second;
        
        string String_temp_read_1;
        string String_temp_read_2;
        
        String_temp_read_1= vs[_first];
        String_temp_read_2= vs[_second];
        
        //string message ="/"+vs[i]+".jpg";
        
        //string message =vs[i];
        
        //pair<multimap<string, node_>::iterator, multimap<string, node_>::iterator> Gp1;
        
        //Gp1 = Feature_matching_Graph.equal_range(message);
        
        //for (multimap<string, node_>::iterator it1 = Gp1.first;it1 != Gp1.second;++it1)
        //{
            //cout<<" [" <<(*it1).first << ", "<<(*it1).second.node1 <<", "<< (*it1).second.node2<<" ]"<<endl;
            //string str_first((*it1).first);
            //string str_second((*it1).second.node1);
            //string str = String_temp_read_1;
            const char *cstr_1 = String_temp_read_1.c_str();
            const char *cstr_2 = String_temp_read_2.c_str();
            fprintf(f,"%s%s%s",cstr_1,"->",cstr_2);
            fprintf(f,_header11);
            fprintf(f,"\n");
        //}
        
    }
    
    fprintf(f,"%s","}");
    
    
    fclose(f);
    
}

void Split_string_function(vector<string>&vs, vector<string> &token)
{

    for (int i=0;i< vs.size();i++)
    {
     string s = vs[i];
     string delimiter = "/";
     string delimiter2 = ".jpg";
     size_t pos = 0;
     string token_temp;
    while ((pos = s.find(delimiter)) != std::string::npos) {
    token_temp = s.substr(0, pos);
    s.erase(0, pos + delimiter.length());
    }
    while ((pos = s.find(delimiter2)) != std::string::npos) {
        token_temp = s.substr(0, pos);
        s.erase(0, pos + delimiter.length());
    }
    
    //std::cout << token_temp << std::endl;
    token.push_back(token_temp);
    }
}
pair <string, string > Find_first2frames(vector<string> token, multimap<string , node_> Feature_matching_Graph)
{
    int maximum_= -99999;
    pair<string,string> pair_temp;
    for(int i=0;i<(int)token.size();i++)
    {
        string temp = token[i];
        pair<multimap<string, node_>::iterator, multimap<string, node_>::iterator> Gp1;
        Gp1 = Feature_matching_Graph.equal_range(temp);
        
        for (multimap<string, node_>::iterator it1 = Gp1.first;it1 != Gp1.second;++it1)
        {
           if ((*it1).second.node2>maximum_)
           {
               pair_temp = std::make_pair (temp,(*it1).second.node1);
               maximum_=(*it1).second.node2;
           }
            
        }
       
     }
   
    return(pair_temp);
}
string Find_mostoverlapped_frame( string input, multimap<string , node_> Feature_matching_Graph)
{
    int maximum_ =-9999;
    string search_result;
    pair<multimap<string, node_>::iterator, multimap<string, node_>::iterator> Gp1;
    Gp1 = Feature_matching_Graph.equal_range(input);
    
    for (multimap<string, node_>::iterator it1 = Gp1.first;it1 != Gp1.second;++it1)
    {
        if ((*it1).second.node2>maximum_)
        {
            search_result =(*it1).second.node1;
            maximum_=(*it1).second.node2;
        }
        
    }
    
    return(search_result);
}
void Generate_Graph_matrix (vector<string> token, multimap<string , node_> Feature_matching_Graph, vector<vector<int>> &Graph_matrix, vector<string> &pending_vector)
{
   
    //   generate graph matrix based on the given feature matching resutl//
    //   convert the upp-triangular matrix to symmetric matrix         //
    
    vector<int>temp;
    
    
    for (int i=0; i<(int)token.size();i++)
    {
        for (int j=0;j<(int)token.size();j++)
        {
            temp.push_back(NULL);
        }
        Graph_matrix.push_back(temp);
    }

    pair<multimap<string, node_>::iterator, multimap<string, node_>::iterator> Gp1;

    //
    
    
    for(int i=0;i<(int)token.size();i++)
    {
        string temp_i= token[i];
        Gp1 = Feature_matching_Graph.equal_range(temp_i);
            
         for(int j=0;j<(int)token.size();j++)
          {
           string temp_j= token[j];
              for (multimap<string, node_>::iterator it1 = Gp1.first;it1 != Gp1.second;++it1)
                {
            
                      if ((*it1).second.node1== temp_j)
                      {
                      
                          Graph_matrix[i][j]=(*it1).second.node2;
                          Graph_matrix[j][i]=(*it1).second.node2;
                      
                      }
                    
                }
        }
     
    }
    
    
    // to get the number of useful images from graph matrix //
    // detect the single image or image without any feature matchign will be
    // inserted with zero coliums and rows//
    
    for(int i=0;i<(int)token.size();i++)
    {
        bool checker=check_zero_colGraphmatrix(i, Graph_matrix);  //check zero col ;trure = yes it zero col
        if (checker== false)
          pending_vector.push_back(token[i]);
    }
    
    temp.clear();
}

void PrimMST_driver(vector<vector<int>> Graph_matrix, vector<string> &pending_vector, vector<pair<int, int >> &MSTresult)
{
    int size_ = (int)Graph_matrix.size();
    vector<bool> temp;
    vector<vector<bool>> bool_matrix;
    for (int i=0; i<size_;i++)
    {
        for (int j=0;j<size_;j++)
        {
           
                temp.push_back(true);
        }
           bool_matrix.push_back(temp);
    }
    
    for (int i=0;i<size_;i++)
        for (int j=0;j<size_;j++)
        {
          if (Graph_matrix[i][j]==0)    // add false in boolmatrix
            bool_matrix[i][j]=false;    // the non-matching image

        }
    
    
    PrimMST( Graph_matrix,bool_matrix, pending_vector, MSTresult);

}
void PrimMST(vector<vector<int>> Graph_matrix, vector<vector<bool>> bool_matrix, vector<string>& pending_vector, vector<pair<int,int>> &MST_result)
{
 
    pair <int, int > keypair;
    pair <int, int > keypair2;
    //pair <int, int > keypair3;
    //pair <int, int > keypair4;
    //pair <int, int > keypair5;
    //MST_result
    keypair=Find_initialmaximumkeys( Graph_matrix,  bool_matrix);
    cout<<"Initial _pair "<<keypair.first+1<<" "<<keypair.second+1<<endl;
    MST_result.push_back(std::make_pair(keypair.first,keypair.second));
    for (int i=0;i< pending_vector.size()-2;i++)
    {
       if (i==0)
       {
        keypair2=Find_maximumkeys(keypair, Graph_matrix,bool_matrix);
        cout<<"Pair_ " << i <<" "<< keypair2.first+1<<" "<< keypair2.second+1<<endl;
        MST_result.push_back(std::make_pair(keypair2.first,keypair2.second));
       }
       else
       {
        keypair2=Find_maximumkeys(keypair2,Graph_matrix,bool_matrix);
        cout<<"Pair_ " << i <<" "<< keypair2.first+1<<" "<< keypair2.second+1<<endl;
        MST_result.push_back(std::make_pair(keypair2.first,keypair2.second));
       }
    
    }
    //    cout<<"Initial _pair "<<keypair.first<<" "<<keypair.second<<endl;
    //    keypair2=Find_maximumkeys(keypair, Graph_matrix,bool_matrix);
    //    cout<<"second _pair "<<keypair2.first<<" "<<keypair2.second<<endl;
    //    keypair3=Find_maximumkeys(keypair2, Graph_matrix,bool_matrix);
    //    cout<<"3 _pair "<<keypair3.first<<" "<<keypair3.second<<endl;
    //    keypair4=Find_maximumkeys(keypair3, Graph_matrix,bool_matrix);
    //    cout<<"4 _pair "<<keypair4.first<<" "<<keypair4.second<<endl;
    //    keypair5=Find_maximumkeys(keypair4, Graph_matrix,bool_matrix);
    //    cout<<"5 _pair "<<keypair5.first<<" "<<keypair5.second<<endl;
    
    

}
 pair <int, int > Find_initialmaximumkeys(vector<vector<int>> Graph_matrix, vector<vector<bool>> &bool_matrix)
{
    // Find the maximum image pair from graph matrix //
    // and return the key pair to Prim algorithm //
    
    int maximum_= -999999;
    int size_ = (int)Graph_matrix.size();
    pair<int, int> key_pair;
    
    //    for(int i=0;i<size_;i++)
    //    {
    //        for(int j=0;j<size_;j++)
    //        {
    //            cout<<bool_matrix[i][j]<<" ";
    //
    //        }
    //        cout<<endl;
    //    }

    
    for (int i=0; i<size_;i++)
    {
        for (int j=i+1;j<size_;j++)
        {
            if (bool_matrix[i][j]== true && Graph_matrix[i][j]>maximum_)
            {
              //cout<<i<<" "<<j<<endl;
              key_pair = std::make_pair (i,j);
              maximum_=Graph_matrix[i][j];
            
            }
        
        }
        
    }
    
   
    
    bool_matrix[key_pair.first][key_pair.second]= false;
    bool_matrix[key_pair.second][key_pair.first]= false;
    
    //    for(int i=0;i<size_;i++)
    //    {
   //        for(int j=0;j<size_;j++)
   //        {
   //           cout<<Graph_matrix[i][j]<<" ";
   //
   //        }
   //        cout<<endl;
   //    }
    
    
    return(key_pair);

}
pair <int, int > Find_maximumkeys(pair <int, int > keypair, vector<vector<int>> Graph_matrix, vector<vector<bool>> &bool_matrix)
{
    
    int maximum_first = -999999;
     int maximum_second = -999999;
    int size_ = (int)Graph_matrix.size();
    int first_idx, second_idx;
    pair<int, int> key_pair_result;
    
    //cout<<keypair.first<<" "<<keypair.second<<endl;
   
    for(int i=0;i<size_;i++)
    {
      if(Graph_matrix[keypair.first][i]>maximum_first && bool_matrix[keypair.first][i]== true)
      {
          first_idx= i;
          maximum_first=Graph_matrix[keypair.first][i];
      }
        
    }
    for(int i=0;i<size_;i++)
    {
        if(Graph_matrix[keypair.second][i]>maximum_second && bool_matrix[keypair.second][i]== true)
        {
            second_idx= i;
            maximum_second=Graph_matrix[keypair.second][i];
        }
        
    }
    if (maximum_first>maximum_second)
         key_pair_result=std::make_pair (keypair.first,first_idx);
    else
         key_pair_result=std::make_pair (keypair.second,second_idx);
    
    //cout<<key_pair_result.first<<" "<<key_pair_result.second<<endl;
    
    set_boolmatrix (bool_matrix,key_pair_result);    // set the key zero matrix to update the Bool matrix
    
    return(key_pair_result);
    
}
void set_boolmatrix (vector<vector<bool>> &bool_matrix,pair <int, int > keypair)
{
    int size_ = bool_matrix.size();
    for(int i=0;i<size_;i++)
     {
        bool_matrix[keypair.first][i]= false;
        bool_matrix[i][keypair.first]= false;
        //bool_matrix[keypair.second][i]= false;
        //bool_matrix[i][keypair.second]= false;
    }
}
 inline bool check_zero_colGraphmatrix(int col,vector<vector<int>> Graph_matrix)  //check zero col ;trure = yes it zero col
{
    bool checker= true;
    int size_ = (int) Graph_matrix[col].size();
    for (int idx=0;idx<size_;idx++)
    {
        if (Graph_matrix[col][idx]==0)
        {
            checker = (checker & true);
        }
        else
        {
            checker = (checker & false);
        }
    }
    return(checker);
}
// test_ 5 points
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