#include <stdio.h>
#include "graph.h"
#include <opencv2/opencv.hpp>
#include <math.h>
#include <iostream>
#include <algorithm>
// #include <opencv2/core/core.hpp>
// #include <opencv2/highgui/highgui.hpp>

#define GAMMA 100
// #define GAMMAT 23
#define BINSIZE 5
#define BINS 450/BINSIZE
// #define FACTOR 2.5
// #define FACTORINIT 2
#define FACTORINIT 2.5
#define M tan(45*3.14159265/180.0)
#define VEHICLE_FACTOR 1


// typedef Graph<int,int,int> GraphType;
typedef Graph<float,float,float> GraphType;

using namespace cv;
using namespace std;

double stddev = 0;
// Vec3i globalMean = Vec3i(0,0,0);

Vec3f meanCL[BINS] , meanCLBkgrnd[BINS] , varCL[BINS] , varCLBkgrnd[BINS];
vector<Vec2i> roadPixels[BINS] , bkgrndPixels[BINS];


inline int twoD_to_oneD(const int &y , const int &x , const int &width )
{
    return x + y*width;
}

inline Point oneD_to_twoD(const int &p , const int &width )
{
    return Point(p%width , p/width);
}




float getWeightCL(Vec3f meanCL[] , Vec3f varCL[] , Vec3b Ib)
{
    static const float inv_sqrt_2pi = 0.3989422804014327;
    
    Vec3f pixel = Vec3f(Ib);
    int normVal = int( sqrt(pixel[0]*pixel[0] + pixel[1]*pixel[1] + pixel[2]*pixel[2]) );
    
    int index = normVal/BINSIZE;
    float x;

    // cout<<"meanCL : : "<<meanCL[index]<<endl;
    // float weight = exp(-1*norm(pixel-meanCL[index])/GAMMA);

    if ( meanCL[index][0] < 0 )
    {
        return 1e-15;
    }
    else
    {
        Vec3f diff = (pixel - meanCL[index]);
        diff[0] /= sqrt(varCL[index][0]); 
        diff[1] /= sqrt(varCL[index][1]);
        diff[2] /= sqrt(varCL[index][2]);

        x = diff.dot(diff);
    }

    // cout<<x<<endl;

    float weight = inv_sqrt_2pi * exp(-0.5*x) / sqrt( varCL[index][0]*varCL[index][1]*varCL[index][2] );
    // cout<<"norm : : "<<norm(pixel - meanCL[index])<<endl;
    // cout<<"weight : : "<<weight<<endl;
    weight = (0.00001 + weight)/1.00001;

    // cout<<weight<<endl;

    return weight;
}


float getWeight(const Vec3b &Ib1 , const Vec3b &Ib2 )
{
    static const float inv_sqrt_2pi = 0.3989422804014327;

    Vec3f I1 = Vec3f(Ib1);
    Vec3f I2 = Vec3f(Ib2);

    float t1,t2;
    // t1 = I1[2];
    // t2 = I2[2];

    // I1[2] = 0;
    // I2[2] = 0;

    // I1 /= sqrt((I1).dot(I1));
    // I2 /= sqrt((I2).dot(I2));
 
    // I1[2] = 0;
    // I2[2] = 0;

    // float a = sqrt((I1-I2).dot(I1-I2))/GAMMA;
    // float weight = inv_sqrt_2pi * exp(-0.5*a*a )/GAMMA ;
    // weight = (0.00001 + weight)/1.00001;
    // // cout<<weight<<endl;
    // return weight;
    // // return 7e5*weight/(1+abs(t1-t2));

    float a = sqrt((I1-I2).dot(I1-I2));
    // float weight = inv_sqrt_2pi * exp(-0.5*a*a )/GAMMA ;
    // float weight = std::max(1e-5, 1.0 - a*M);
    // weight = (1e-5 + weight)/(1+1e-5);
    float p_road1 = getWeightCL(meanCL, varCL , Ib1);
    float p_road2 = getWeightCL(meanCL, varCL , Ib2);
    float p_bkgd1 = getWeightCL(meanCLBkgrnd, varCLBkgrnd , Ib1);
    float p_bkgd2 = getWeightCL(meanCLBkgrnd, varCLBkgrnd , Ib2);
    
    float normaLize = p_road1+p_bkgd1;
    p_road1 /= normaLize;
    p_bkgd1 /= normaLize;

    normaLize = p_road2+p_bkgd2;
    p_road2 /= normaLize;
    p_bkgd2 /= normaLize;

    float weight = p_road1*p_road2 + p_bkgd1*p_bkgd2;
    // cout<<weight<<endl;
    // return 10*weight;

    return 7*weight;
    
}



int main(int argc, char** argv)
{

    if( argc != 3)
    {
     cout <<" Usage: driverFunc image map" << endl;
     return -1;
    }

    Mat image , gray_image;

    // VideoCapture cap(argv[1]); // open the default camera

    
    // if(!cap.isOpened())  // check if we succeeded
    //     return -1;

    
    gray_image = imread(argv[1], CV_LOAD_IMAGE_COLOR);   // Read the file
    // cvtColor(gray_image,gray_image,CV_BGR2HSV);
    // gray_image = imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);   // Read the file

    Mat roimap = imread(argv[2], CV_LOAD_IMAGE_COLOR);
    // Mat roimap_hsv;
    // cvtColor(roimap,roimap_hsv,CV_BGR2HSV);
    // cout<<roimap_hsv.cols<<endl;
    
    cv::resize(gray_image,gray_image,roimap.size());

    if(! gray_image.data )                              // Check for invalid input
    {
        cout <<  "Could not open or find the image" << std::endl ;
        return -1;
    }

    const int width = gray_image.cols; 
    const int height = gray_image.rows; 

    cout<<gray_image.cols<<endl;
    cout<<gray_image.rows<<endl;
    
    int NumNodes = width*height;

    //deepL

    // cout<<roimap_hsv.rows<<endl;
    // cout<<roimap.at<Vec3b>(194,97)<<endl;

    Vec3b road = Vec3b(128,64,128);            
    Vec3b vehicle = Vec3b(128,0,64);
    int total_road = 0;            
    
    for (int i = 0; i < width; ++i)
    {
        for (int j = 0; j < height; ++j)
        {
            Vec3i pixel = (Vec3i)gray_image.at<Vec3b>(j,i);
            int norm = sqrt( pixel[0]*pixel[0] + pixel[1]*pixel[1] + pixel[2]*pixel[2] );
            // cout<<i<<"  "<<j<< "  "<<pixel<<endl;

            if ( roimap.at<Vec3b>(j,i) == road )
                {
                    roadPixels[ norm/BINSIZE ].push_back( Vec2i(i,j) );
                    total_road ++;
                }
            else
                bkgrndPixels[ norm/BINSIZE ].push_back( Vec2i(i,j) );
        }
    }

    // cout<<"Hello"<<endl;
    float FACTOR = FACTORINIT;
    float roadPercent = (float)total_road/(width*height);
    
    // if(roadPercent < 0.15)
    //     FACTOR = 1+2*roadPercent;
    
    // cout<<FACTOR<<endl;

    for (int i = 0; i < BINS; ++i)
    {
       if(roadPixels[i].size() > 0)
        {
            Vec3f meanVal = Vec3f(0,0,0) ;
            
            for (int j = 0; j < roadPixels[i].size(); ++j)
            {
                meanVal += (Vec3f)gray_image.at<Vec3b>( roadPixels[i][j][1] , roadPixels[i][j][0] );
            }

            meanCL[i][0] = meanVal[0] / roadPixels[i].size();            
            meanCL[i][1] = meanVal[1] / roadPixels[i].size();            
            meanCL[i][2] = meanVal[2] / roadPixels[i].size();            
        }
        else
            meanCL[i] = Vec3f(-1,-1,-1);

        if(bkgrndPixels[i].size() > 0)
        {
            Vec3f meanVal = Vec3f(0,0,0) ;
            
            for (int j = 0; j < bkgrndPixels[i].size(); ++j)
            {
                meanVal += (Vec3f)gray_image.at<Vec3b>( bkgrndPixels[i][j][1] , bkgrndPixels[i][j][0] );
            }

            meanCLBkgrnd[i][0] = meanVal[0] / bkgrndPixels[i].size();            
            meanCLBkgrnd[i][1] = meanVal[1] / bkgrndPixels[i].size();            
            meanCLBkgrnd[i][2] = meanVal[2] / bkgrndPixels[i].size();            
        }
        else
            meanCLBkgrnd[i] = Vec3f(-1,-1,-1);

        // cout<<meanCL[i]<<endl; 
    }
    

    for (int i = 0; i < BINS; ++i)
    {
       if(roadPixels[i].size() > 0)
        {
            Vec3f variance = Vec3f(1,1,1) ;
            
            for (int j = 0; j < roadPixels[i].size(); ++j)
            {
                Vec3f diff =  ( meanCL[i] - (Vec3f)gray_image.at<Vec3b>( roadPixels[i][j][1] , roadPixels[i][j][0] ) );
                variance += Vec3f(diff[0]*diff[0], diff[1]*diff[1] , diff[2]*diff[2]);
            }

            varCL[i][0] = (variance[0] / roadPixels[i].size())+1;            
            varCL[i][1] = (variance[1] / roadPixels[i].size())+1;            
            varCL[i][2] = (variance[2] / roadPixels[i].size())+1;            
        }
        else
            varCL[i] = Vec3f(-1,-1,-1);

        if(bkgrndPixels[i].size() > 0)
        {
            Vec3f variance = Vec3f(1,1,1) ;
            
            for (int j = 0; j < bkgrndPixels[i].size(); ++j)
            {
                Vec3f diff =  ( meanCLBkgrnd[i] - (Vec3f)gray_image.at<Vec3b>( bkgrndPixels[i][j][1] , bkgrndPixels[i][j][0] ) );
                variance += Vec3f(diff[0]*diff[0] , diff[1]*diff[1] , diff[2]*diff[2]);
            }

            varCLBkgrnd[i][0] = (variance[0] / bkgrndPixels[i].size())+1;            
            varCLBkgrnd[i][1] = (variance[1] / bkgrndPixels[i].size())+1;            
            varCLBkgrnd[i][2] = (variance[2] / bkgrndPixels[i].size())+1;            
        }
        else
            varCLBkgrnd[i] = Vec3f(-1,-1,-1);

        cout<<varCL[i]<<endl; 
    }



    // Mat canvas = Mat::zeros(gray_image.rows, gray_image.cols*2+10, gray_image.type());


    // VideoWriter outputVideo("../segment1.avi", CV_FOURCC('X','V','I','D'), cap.get(CV_CAP_PROP_FPS), S, true);
    // VideoWriter outputVideoCombined("../segment1Combined.avi", CV_FOURCC('X','V','I','D'), cap.get(CV_CAP_PROP_FPS), Size(2*gray_image.cols + 10 , gray_image.rows), true);
    // // outputVideo.open("../segment.mp4", CV_FOURCC('X','2','6','4'), cap.get(CV_CAP_PROP_FPS), S, true);


// while(1)
{

    // cap >> gray_image;
    // gray_image.copyTo(canvas(Range::all(), Range(0, gray_image.cols)));

    GraphType *g = new GraphType(/*estimated # of nodes*/ NumNodes, /*estimated # of edges*/ 4*NumNodes); 

    for (int i = 0; i < NumNodes; ++i)
    {
        g->add_node();
    }

    
    for (int i = 1; i < width; ++i)
    {
        int n1, n2;
        float weight;
        
        n1 = twoD_to_oneD(0,i,width);
        n2 = twoD_to_oneD(0,i-1,width);

        weight = getWeight( gray_image.at<Vec3b>(0,i) , gray_image.at<Vec3b>(0,i-1));
        
        g->add_edge( n1 , n2 , weight , weight );
        
        // weight  = getWeightTerminals(gray_image.at<Vec3b>(0,i),mean);
        // weight  = getWeightCL( meanCL , varCL , gray_image.at<Vec3b>(0,i) );
        float w1 = getWeightCL( meanCL , varCL , gray_image.at<Vec3b>(0,i) );
        float w2 = getWeightCL( meanCLBkgrnd , varCLBkgrnd , gray_image.at<Vec3b>(0,i) );
        
        if (roimap.at<Vec3b>(0,i) == road)
        {
            w1 = 1;
            w2 = 0;
        }
        else
        {
            w1 /= FACTORINIT;
        }

        // if (roimap.at<Vec3b>(0,i) == vehicle)
        // {
        //     w1 /= VEHICLE_FACTOR;
        //     w2 *= VEHICLE_FACTOR;
        // }

        float normaLize = w1+w2;
        g->add_tweights(n1 , w1/normaLize  , w2/normaLize);
    }


    for (int j = 1; j < height; ++j)
    {
        int n1, n2 ;
        float weight;
        n1 = twoD_to_oneD(j,0,width);
        n2 = twoD_to_oneD(j-1,0,width);

        // cout<<j<<endl;
        weight = getWeight( gray_image.at<Vec3b>(j,0) , gray_image.at<Vec3b>(j-1,0));
        
        g->add_edge( n1 , n2 , weight , weight );
        
        // weight  = getWeightTerminals(gray_image.at<Vec3b>(j,0),mean);
        float w1 = getWeightCL( meanCL , varCL , gray_image.at<Vec3b>(j,0));
        float w2 = getWeightCL( meanCLBkgrnd , varCLBkgrnd , gray_image.at<Vec3b>(j,0));
        if (roimap.at<Vec3b>(j,0) == road)
        {
            w1 = 1;
            w2 = 0;
        }
        else
        {
            w1 /= FACTORINIT;
        }

        // if (roimap.at<Vec3b>(j,0) == vehicle)
        // {
        //     w1 /= VEHICLE_FACTOR;
        //     w2 *= VEHICLE_FACTOR;
        // }

        float normaLize = w1+w2;
        g->add_tweights(n1 , w1/normaLize  , w2/normaLize);

    }

    // cout<<" Error "<<endl;    

    for (int i = 1; i < width; ++i)
    {
        for (int j = 1; j < height; ++j)
        {
            // cout << (int)gray_image.at<uchar>(j,i)<< "  ";
            int n1,n2 ;
            float weight;
            n1 = twoD_to_oneD(j,i,width);

            // cout<<j<< "  " << i<< "  "<<n1<<endl; 

            n2 = twoD_to_oneD(j-1,i,width);
            weight = getWeight( gray_image.at<Vec3b>(j,i) , gray_image.at<Vec3b>(j-1,i)); 
            g->add_edge( n1 , n2 , weight , weight );
        

            n2 = twoD_to_oneD(j,i-1,width);
            weight = getWeight( gray_image.at<Vec3b>(j,i) , gray_image.at<Vec3b>(j,i-1)); 
            g->add_edge( n1 , n2 , weight , weight );

            // weight  = getWeightTerminals(gray_image.at<Vec3b>(j,i) , mean);
            float w1  = getWeightCL( meanCL , varCL , gray_image.at<Vec3b>(j,i) );
            float w2  = getWeightCL( meanCLBkgrnd , varCLBkgrnd , gray_image.at<Vec3b>(j,i) );
            
            if (roimap.at<Vec3b>(j,i) == road)
            {
                w1 = 1;
                w2 = 0;
                // cout<<"Non Road"<<w1<<"  "<<w2<<endl;
                            
            }
            else
            {
                w1 /= FACTORINIT;
            }

            // else
                // cout<<"Non Road"<<w1<<"  "<<w2<<endl;

            // if (roimap.at<Vec3b>(j,i) == vehicle)
            // {
            //     w1 /= VEHICLE_FACTOR;
            //     w2 *= VEHICLE_FACTOR;

            //     // cout<<"asfsaf\n";
            // }

            // if (i==312 && j==289)
            // if (i==224 && j==222)
            // {
            //      cout<<i<<"  "<<j<<"  "<<weight<<endl;
            // }
            
            float normaLize = w1+w2;

                // cout<<weight<<endl;
            g->add_tweights(n1 , w1/normaLize , w2/normaLize);
            
        }
    }

    float flow = g -> maxflow();
    printf("Flow = %f\n", flow);

    for (int i = 0; i < width; ++i)
    {
        for (int j = 0; j < height; ++j)
        {
            int n1;
            n1 = twoD_to_oneD(j,i,width);
            
            if (g->what_segment(n1) == GraphType::SOURCE)
                {
                    gray_image.at<Vec3b>(j,i) = Vec3b(255,105,180);
                }
            else
                {
                    gray_image.at<Vec3b>(j,i) = Vec3b(0,0,0);
                }// cout<<"SINK"<<endl;
            
        }
    }
    imwrite( "data/segmentation_wi.png", gray_image );

    // gray_image.copyTo(canvas(Range::all(), Range(gray_image.cols+10, gray_image.cols*2+10)));
    // outputVideoCombined << canvas;



    // imshow("seg", gray_image);
    // imshow("seg", canvas);
    // waitKey(20);
    // if(waitKey(30) >= 0) break;
    
    delete g;

}
    // printf("Minimum cut:\n");
    // if (g->what_segment(0) == GraphType::SOURCE)
    //  printf("node0 is in the SOURCE set\n");
    // else
    //  printf("node0 is in the SINK set\n");
    // if (g->what_segment(1) == GraphType::SOURCE)
    //  printf("node1 is in the SOURCE set\n");
    // else
    //  printf("node1 is in the SINK set\n");


    return 0;
}
