#include <gtest/gtest.h>
#include "analize_scan_utils.hpp"
#include <math.h>
#define PI 3.14

inline float deg2rad(float deg)
{
    return deg * PI /180;
}


TEST(FilterClusterByLength, FilteredBy2and3){
    /**
        @test Test the basic use of the 
        FilterClusterByLength function.
        The input will be four clusters
        with a length of 1,2,3 and 4.
        The min length will be 2 and 
        the max length will be 3, so
        the vector of length 1 and 4
        will be filtered from the 
        output vector.

    */

    cluster cluster_length_1;
    cluster cluster_length_2;
    cluster cluster_length_3;
    cluster cluster_length_4;

    std::vector<cluster> clusters;
    std::vector<cluster> filtered_clusters;

    int x, y = 0;

    // Create all clusters
    for (int i=0; i<=1; i++)
    {
        x = 1;
        y = i;
        cluster_length_1.emplace_back(Point2d(x,y));
        cluster_length_2.emplace_back(Point2d(x,y*2));
        cluster_length_3.emplace_back(Point2d(x,y*3));
        cluster_length_4.emplace_back(Point2d(x,y*4));
    }

    clusters.push_back(cluster_length_1);
    clusters.push_back(cluster_length_2);
    clusters.push_back(cluster_length_3);
    clusters.push_back(cluster_length_4);

    // Call the tested function
    filter_clusters_by_length(clusters,2,3,filtered_clusters);
    
    // 
    ASSERT_EQ(filtered_clusters.size(), (long unsigned int) 2);
    EXPECT_EQ(filtered_clusters[0], cluster_length_2);
    EXPECT_EQ(filtered_clusters[1], cluster_length_3);
}


TEST(FilterClusterByLength, FilteredBy0and5){
    /**
        @test Test the basic use of the 
        FilterClusterByLength function.
        The input will be four clusters
        with a length of 1,2,3 and 4.
        The min length will be 0 and 
        the max length will be 5, so
        all the clusters will be on the 
        output vector.

    */

    cluster cluster_length_1;
    cluster cluster_length_2;
    cluster cluster_length_3;
    cluster cluster_length_4;

    std::vector<cluster> clusters;
    std::vector<cluster> filtered_clusters;


    int x, y = 0;

    // Create all clusters
    for (int i=0; i<=1; i++)
    {
        x = 1;
        y = i;
        cluster_length_1.emplace_back(Point2d(x,y));
        cluster_length_2.emplace_back(Point2d(x,y*2));
        cluster_length_3.emplace_back(Point2d(x,y*3));
        cluster_length_4.emplace_back(Point2d(x,y*4));
    }

    clusters.push_back(cluster_length_1);
    clusters.push_back(cluster_length_2);
    clusters.push_back(cluster_length_3);
    clusters.push_back(cluster_length_4);

    // Call the tested function
    filter_clusters_by_length(clusters,0,5,filtered_clusters);
    
    // 
    ASSERT_EQ(filtered_clusters.size(), (long unsigned int) 4);
    EXPECT_EQ(filtered_clusters[0], cluster_length_1);
    EXPECT_EQ(filtered_clusters[1], cluster_length_2);
    EXPECT_EQ(filtered_clusters[2], cluster_length_3);
    EXPECT_EQ(filtered_clusters[3], cluster_length_4);
}


TEST(FilterClusterByLength, FilteredBy5and8){
    /**
        @test Test the basic use of the 
        FilterClusterByLength function.
        The input will be four clusters
        with a length of 1,2,3 and 4.
        The min length will be 5 and 
        the max length will be 38 so
        the all cluster from the 
        output vector.

    */

    cluster cluster_length_1;
    cluster cluster_length_2;
    cluster cluster_length_3;
    cluster cluster_length_4;

    std::vector<cluster> clusters;
    std::vector<cluster> filtered_clusters;


    int x, y = 0;

    // Create all clusters
    for (int i=0; i<=1; i++)
    {
        x = 1;
        y = i;
        cluster_length_1.emplace_back(Point2d(x,y));
        cluster_length_2.emplace_back(Point2d(x,y*2));
        cluster_length_3.emplace_back(Point2d(x,y*3));
        cluster_length_4.emplace_back(Point2d(x,y*4));
    }

    clusters.push_back(cluster_length_1);
    clusters.push_back(cluster_length_2);
    clusters.push_back(cluster_length_3);
    clusters.push_back(cluster_length_4);

    // Call the tested function
    filter_clusters_by_length(clusters,5,8,filtered_clusters);
    
    // 
    ASSERT_EQ(filtered_clusters.size(), (long unsigned int) 0);
}

TEST(FilterClusterByLength, EmptyInputVector){
    /**
        @test Test the basic use of the 
        FilterClusterByLength function.
        The input will and empty vector
        so the output vector must be 
        empty.

    */

    std::vector<cluster> clusters;
    std::vector<cluster> filtered_clusters;


    // Call the tested function
    filter_clusters_by_length(clusters,0,5,filtered_clusters);
    
    // The output vector must be empty
    ASSERT_EQ(filtered_clusters.size(), (long unsigned int) 0);
}

TEST(FilterClusterByLength, NotEmptyOutputVector){
    /**
        @test Test the basic use of the 
        FilterClusterByLength function.
        The output vector given to 
        filter cluster by length will 
        be filled with random data
        that must be erased.

    */

    cluster cluster_length_1;
    cluster cluster_length_2;
    cluster cluster_length_3;
    cluster cluster_length_4;

    std::vector<cluster> clusters;
    std::vector<cluster> filtered_clusters;


    int x, y = 0;

    // Create all clusters
    for (int i=0; i<=1; i++)
    {
        x = 1;
        y = i;
        cluster_length_1.emplace_back(Point2d(x,y));
        cluster_length_2.emplace_back(Point2d(x,y*2));
        cluster_length_3.emplace_back(Point2d(x,y*3));
        cluster_length_4.emplace_back(Point2d(x,y*4));
    }

    clusters.push_back(cluster_length_1);
    clusters.push_back(cluster_length_2);
    clusters.push_back(cluster_length_3);
    clusters.push_back(cluster_length_4);

    // Fill the output vector with some data
    filtered_clusters.emplace_back(cluster());
    filtered_clusters.emplace_back(cluster());
    filtered_clusters.emplace_back(cluster());
    filtered_clusters.emplace_back(cluster());

    for (int i=0; i<=1; i++)
    {
        filtered_clusters[0].emplace_back(Point2d(i+3, i+3));
        filtered_clusters[1].emplace_back(Point2d(i*5, i*2));
        filtered_clusters[2].emplace_back(Point2d(i*3, i*4));
    }    


    // Call the tested function
    filter_clusters_by_length(clusters,2,3,filtered_clusters);
    
    // 
    ASSERT_EQ(filtered_clusters.size(), (long unsigned int) 2);
    EXPECT_EQ(filtered_clusters[0], cluster_length_2);
    EXPECT_EQ(filtered_clusters[1], cluster_length_3);
}


TEST(FilterClusterByLength, EmptyVector){
   /**
        @test Test the basic use of the 
        FilterClusterByLength function.
        The input will and empty vector
        and the output vector given to 
        the function will be filled with
        some data, the output vector must 
        be empty.

    */

    std::vector<cluster> clusters;
    std::vector<cluster> filtered_clusters;

    // Fill the output vector with some data
    filtered_clusters.emplace_back(cluster());
    filtered_clusters.emplace_back(cluster());
    filtered_clusters.emplace_back(cluster());
    filtered_clusters.emplace_back(cluster());

    for (int i=0; i<=1; i++)
    {
        filtered_clusters[0].emplace_back(Point2d(i+3, i+3));
        filtered_clusters[1].emplace_back(Point2d(i*5, i*2));
        filtered_clusters[2].emplace_back(Point2d(i*3, i*4));
    }    

    // Call the tested function
    filter_clusters_by_length(clusters,0,5,filtered_clusters);
    
    // The output vector must be empty
    ASSERT_EQ(filtered_clusters.size(), (long unsigned int) 0);
}


TEST(FilterClusterByLength, PositiveCoordsAndNegativeCoords){
    /**
        @test Test the basic use of the 
        FilterClusterByLength function
        with a set of negative and
        positive clusters
    */
    cluster cluster_length_1;
    cluster cluster_length_2;
    cluster cluster_length_3;
    cluster cluster_length_4;

    std::vector<cluster> clusters;
    std::vector<cluster> filtered_clusters;


    int x, y = 0;

    // Create all clusters
    for (int i=0; i<=3; i++)
    {
        x = 1;
        y = i;
        cluster_length_1.emplace_back(Point2d(x,y));
        cluster_length_2.emplace_back(Point2d(-x,y*2));
        cluster_length_3.emplace_back(Point2d(3*x,y*3));
        cluster_length_4.emplace_back(Point2d(-x,y*4));
    }

    clusters.push_back(cluster_length_1);
    clusters.push_back(cluster_length_2);
    clusters.push_back(cluster_length_3);
    clusters.push_back(cluster_length_4);

    // Call the tested function
    filter_clusters_by_length(clusters,5,13,filtered_clusters);
    
    // 
    ASSERT_EQ(filtered_clusters.size(), (long unsigned int) 3);
    EXPECT_EQ(filtered_clusters[0], cluster_length_2);
    EXPECT_EQ(filtered_clusters[1], cluster_length_3);
    EXPECT_EQ(filtered_clusters[2], cluster_length_4);
}

