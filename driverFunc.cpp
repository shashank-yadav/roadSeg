#include <stdio.h>
#include "graph.h"
#include <opencv2/opencv.hpp>
#include <math.h>
#include <iostream>
#include <algorithm>
// #include <opencv2/core/core.hpp>
// #include <opencv2/highgui/highgui.hpp>

#define GAMMA 22
#define GAMMAT 22
#define BINS 50
#define BINSIZE 10
#define FACTOR 1.2

// typedef Graph<int,int,int> GraphType;
typedef Graph<float,float,float> GraphType;

using namespace cv;
using namespace std;

double stddev = 0;
// Vec3i globalMean = Vec3i(0,0,0);


inline int twoD_to_oneD(const int &y , const int &x , const int &width )
{
    return x + y*width;
}

inline Point oneD_to_twoD(const int &p , const int &width )
{
    return Point(p%width , p/width);
}

float getWeight(const Vec3b &Ib1 , const Vec3b &Ib2 )
{
    static const float inv_sqrt_2pi = 0.3989422804014327;
    Vec3f I1 = Vec3f(Ib1);
    Vec3f I2 = Vec3f(Ib2);

    // I1 /= (I1[0] + I1[1] + I1[2]);
    // I2 /= (I2[0] + I2[1] + I2[2]);

    // // cout<<I1<<endl;
    // // cout<<I2<<endl;
    float a = sqrt((I1-I2).dot(I1-I2))/GAMMA;
    // float weight = ( inv_sqrt_2pi * exp(-1*norm(I1-I2)/GAMMA) );
    float weight = inv_sqrt_2pi * exp(-0.5*a*a )/GAMMA ;
    weight = (0.00001 + weight)/1.00001;

    // return Point2f( weight , (1 - weight) );
    // I1 = I1/(sqrt(I1.dot(I1)));
    // I2 = I2/(sqrt(I2.dot(I2)));
    // float weight = (I1.dot(I2));
    // cout<<weight<<endl;
    
    return 30*weight;

}



float getWeightCL(Vec3f meanCL[] , Vec3b &Ib)
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
        // Vec3f x1,x2;
        // vector<int> NonNegative;

        // for (int i = 0; i < BINS; ++i)
        // {
        //     if( meanCL[i][0] > 0 )
        //         NonNegative.push_back(i);
            
        // }

        // if (index > NonNegative[NonNegative.size() - 1])
        // {
        //     x1 = meanCL[NonNegative[NonNegative.size() - 2]];
        //     x2 = meanCL[NonNegative[NonNegative.size() - 1]];
        // }
        // else
        // {
        //     x1 = meanCL[NonNegative[0]];
        //     x2 = meanCL[NonNegative[1]];

        // }

        // x = norm((x1-pixel).cross(x2-pixel))/norm(x1-x2);
        // cout<<" X : "<<x<<endl;

        return 1e-10;
    }
    else
    {
        Vec3f diff = pixel - meanCL[index]; 
        x = diff.dot(diff);
    }

    float weight = inv_sqrt_2pi * exp(-0.5*x/(GAMMAT*GAMMAT)) / GAMMAT;
    // cout<<"norm : : "<<norm(pixel - meanCL[index])<<endl;
    // cout<<"weight : : "<<weight<<endl;
    weight = (0.00001 + weight)/1.00001;
    // cout<<weight<<endl;

    return weight;
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
    // gray_image = imread(argv[1], CV_LOAD_IMAGE_GRAYSCALE);   // Read the file

    if(! gray_image.data )                              // Check for invalid input
    {
        cout <<  "Could not open or find the image" << std::endl ;
        return -1;
    }

    const int width = gray_image.cols; 
    const int height = gray_image.rows; 

    cout<<gray_image.cols<<endl; // 1600
    cout<<gray_image.rows<<endl; // 1200
    
    int NumNodes = width*height;

    //deepL
    Vec3f meanCL[BINS] , meanCLBkgrnd[BINS];
    vector<Vec2i> roadPixels[BINS] , bkgrndPixels[BINS];

    Mat roimap = imread(argv[2], CV_LOAD_IMAGE_COLOR);
    Vec3b road = Vec3b(128,64,128);            
    
    for (int i = 0; i < width; ++i)
    {
        for (int j = 0; j < height; ++j)
        {
            Vec3i pixel = (Vec3i)gray_image.at<Vec3b>(j,i);
            int norm = sqrt(pixel[0]*pixel[0] + pixel[1]*pixel[1] + pixel[2]*pixel[2]);
            int index = norm/BINSIZE;

            if ( roimap.at<Vec3b>(j,i) == road )
                roadPixels[ index ].push_back( Vec2i(i,j) );
            else
                bkgrndPixels[ index ].push_back( Vec2i(i,j) );
        }
    }



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
    
    Mat canvas = Mat::zeros(gray_image.rows, gray_image.cols*2+10, gray_image.type());


    // VideoWriter outputVideo("../segment1.avi", CV_FOURCC('X','V','I','D'), cap.get(CV_CAP_PROP_FPS), S, true);
    // VideoWriter outputVideoCombined("../segment1Combined.avi", CV_FOURCC('X','V','I','D'), cap.get(CV_CAP_PROP_FPS), Size(2*gray_image.cols + 10 , gray_image.rows), true);
    // // outputVideo.open("../segment.mp4", CV_FOURCC('X','2','6','4'), cap.get(CV_CAP_PROP_FPS), S, true);


// while(1)
{

    // cap >> gray_image;
    gray_image.copyTo(canvas(Range::all(), Range(0, gray_image.cols)));

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
        float w1 = getWeightCL( meanCL , gray_image.at<Vec3b>(0,i) );
        float w2 = getWeightCL( meanCLBkgrnd , gray_image.at<Vec3b>(0,i) );
        if (roimap.at<Vec3b>(0,i) != road)
        {
            w1 /= FACTOR;
            w2 *= FACTOR;
        }
        g->add_tweights(n1 , w1  , w2);
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
        float w1 = getWeightCL( meanCL , gray_image.at<Vec3b>(j,0));
        float w2 = getWeightCL( meanCLBkgrnd , gray_image.at<Vec3b>(j,0));
        if (roimap.at<Vec3b>(j,0) != road)
        {
            w1 /= FACTOR;
            w2 *= FACTOR;
        }
    
        g->add_tweights(n1 , w1  , w2);

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
            float w1  = getWeightCL( meanCL ,  gray_image.at<Vec3b>(j,i) );
            float w2  = getWeightCL( meanCLBkgrnd ,  gray_image.at<Vec3b>(j,i) );
            
            if (roimap.at<Vec3b>(j,i) != road)
            {
                w1 /= FACTOR;
                w2 *= FACTOR;
            }
                // cout<<weight<<endl;
            g->add_tweights(n1 , w1 , w2);
            
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
                    gray_image.at<Vec3b>(j,i) = Vec3b(255,255,255);
                }
            else
                {
                    gray_image.at<Vec3b>(j,i) = Vec3b(0,0,0);
                }// cout<<"SINK"<<endl;
            
        }
    }
    imwrite( "data/segmentation_wi.jpg", gray_image );

    gray_image.copyTo(canvas(Range::all(), Range(gray_image.cols+10, gray_image.cols*2+10)));
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
