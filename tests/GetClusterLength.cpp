#include <gtest/gtest.h>
#include "analize_scan_utils.hpp"
#include <math.h>
#define PI 3.14

inline float deg2rad(float deg)
{
    return deg * PI /180;
}


TEST(GetClusterLength, TwoPoints){
    /**
        @test Test the basic use of the 
        GetClusterSquareLength function
        with a 2 point cluster.
    */

    cluster tested_cluster;
    int x, y = 0;
    float length = 0;


    // Fill the cluster vector with a line
    for (int i=0; i<=1; i++)
    {
        x = 1;
        y = i;
        tested_cluster.emplace_back(Point2d(x,y));
    }
    // Call the tested function
    length = get_cluster_length(tested_cluster);

    // Float 
    EXPECT_FLOAT_EQ(length, 1);
}


TEST(GetClusterLength, BasicCaseOfUse){
    /**
        @test Test the basic use of the 
        GetClusterLength function
        with a 4 point cluster.
    */

    cluster tested_cluster;
    int x, y = 0;
    float length = 0;


    // Fill the cluster vector with a line
    for (int i=0; i<=3; i++)
    {
        x = 1;
        y = i;
        tested_cluster.emplace_back(Point2d(x,y));
    }
    // Call the tested function
    length = get_cluster_length(tested_cluster);

    // Float 
    EXPECT_FLOAT_EQ(length, 3);
}

TEST(GetClusterLength, ClusterIncrementedBy3){
    /**
        @test Test the basic use of the 
        GetClusterLength function
        with a 4 point cluster that
        between points is a distance
        of three.
    */

    cluster tested_cluster;
    int x, y = 0;
    float length = 0;


    // Fill the cluster vector with a line
    for (int i=0; i<=3; i++)
    {
        x = 1;
        y = i * 3;
        tested_cluster.emplace_back(Point2d(x,y));
    }
    // Call the tested function
    length = get_cluster_length(tested_cluster);

    // Float 
    EXPECT_FLOAT_EQ(length, 9);
}

TEST(GetClusterLength, ClusterIncrementedByRoot2){
    /**
        @test Test the basic use of the 
        GetClusterLength function
        with a 5 point cluster that
        between points is a distance
        of root(2).
    */

    cluster tested_cluster;
    int x, y = 0;
    float length = 0;


    // Fill the cluster vector with a line
    for (int i=0; i<=5; i++)
    {
        x = i;
        y = i;
        tested_cluster.emplace_back(Point2d(x,y));
    }
    // Call the tested function
    length = get_cluster_length(tested_cluster);

    // Float 
    EXPECT_FLOAT_EQ(length, sqrt(50));
}

TEST(GetClusterLength, EmptyVector){
    /**
        @test Test the  use of the 
        GetClusterLength function
        with an empty cluster.
    */

    cluster tested_cluster;
    int x, y = 0;
    float length = 0;


    // Fill the cluster vector with a line
    for (int i=0; i<=5; i++)
    {
        x = i;
        y = i;
        tested_cluster.emplace_back(Point2d(x,y));
    }
    // Call the tested function
    length = get_cluster_length(tested_cluster);

    // Float 
    EXPECT_FLOAT_EQ(length, sqrt(50));
}


TEST(GetClusterLength, ClusterIncrementedByMinus1){
    /**
        @test Test the basic use of the 
        GetClusterLength function
        with a 5 point cluster that
        between points is a distance
        of -1.
    */

    cluster tested_cluster;
    int x, y = 0;
    float length = 0;


    // Fill the cluster vector with a line
    for (int i=0; i<=5; i++)
    {
        x = -i;
        y = 1;
        tested_cluster.emplace_back(Point2d(x,y));
    }
    // Call the tested function
    length = get_cluster_length(tested_cluster);

    // Float 
    EXPECT_FLOAT_EQ(length, 5);
}

TEST(GetClusterLength, ClusterIncrementedByMinus3){
    /**
        @test Test the basic use of the 
        GetClusterLength function
        with a 5 point cluster that
        between points is a distance
        of -3.
    */

    cluster tested_cluster;
    int x, y = 0;
    float length = 0;


    // Fill the cluster vector with a line
    for (int i=0; i<=5; i++)
    {
        x = -i * 3;
        y = 1;
        tested_cluster.emplace_back(Point2d(x,y));
    }
    // Call the tested function
    length = get_cluster_length(tested_cluster);

    // Float 
    EXPECT_FLOAT_EQ(length, 15);
}

TEST(GetClusterLength, ClusterIncrementedByMinusRoot2){
    /**
        @test Test the basic use of the 
        GetClusterLength function
        with a 4 point cluster that
        between points is a distance
        of 3 * -root(2).
    */

    cluster tested_cluster;
    int x, y = 0;
    float length = 0;


    // Fill the cluster vector with a line
    for (int i=0; i<=3; i++)
    {
        x = -i * 3;
        y = -i * 3;
        tested_cluster.emplace_back(Point2d(x,y));
    }
    // Call the tested function
    length = get_cluster_length(tested_cluster);

    // Float 
    EXPECT_FLOAT_EQ(length,sqrt(162));
}


