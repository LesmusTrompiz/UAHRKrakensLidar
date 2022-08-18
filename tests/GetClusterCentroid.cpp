#include <gtest/gtest.h>
#include "analize_scan_utils.hpp"
#include <math.h>
#define PI 3.14

inline float deg2rad(float deg)
{
    return deg * PI /180;
}

TEST(GetClusterCentroid, BasicUse){
    /**
        @test Test the basic use of the 
        get_cluster_contour_centroid.
    */
    cluster   scan;
    Point2d   centroid;


    // Fill the scan with one constant object
    // at 1 meter from the lidar
    for (int i=-5; i<=5; i++)
    {
        scan.emplace_back(Point2d(i,i));
    }


    // Call the tested function
    centroid = get_cluster_contour_centroid(scan);

    // The only object is the whole scan
    EXPECT_EQ(centroid, Point2d(0,0));   
}

TEST(GetClusterCentroid, EmptyInput){
    /**
        @test Test what happens if an 
        empty vector is given to the 
        get_cluster_contour_centroid.


    */
    cluster   scan;
    Point2d   centroid;


    // The only object is the whole scan
    EXPECT_DEBUG_DEATH(get_cluster_contour_centroid(scan),"");
}