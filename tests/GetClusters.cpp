#include <gtest/gtest.h>
#include "analize_scan_utils.hpp"
#include <math.h>
#define PI 3.14

inline float deg2rad(float deg)
{
    return deg * PI /180;
}

TEST(GetClusters, BasicObject){
    /**
        @test Test the basic use of the 
        get_cluster proccesure. It takes
        a scan with only one object.
    */
    std::vector<Point2d>   scan;
    std::vector<cluster>   clusters;


    // Fill the scan with one constant object
    // at 1 meter from the lidar
    for (int i=0; i<=360; i++)
    {
        scan.emplace_back(Point2d(1.0 * cos(deg2rad(i)), 1.0 * sin(deg2rad(i))));
    }


    // Call the tested function
    get_clusters(scan, clusters, 0.1);

    // There is only one object
    ASSERT_EQ(clusters.size(),(long unsigned int)1);

    // The only object is the whole scan
    EXPECT_EQ(scan, clusters[0]);   
}

TEST(GetClusters, InputVectorClear){
    /**
        @test Once the output vector
        is filled in the get cluster
        function, clearing the input
        vector does not affect the
        output vector.
    */
    std::vector<Point2d>   scan;
    std::vector<Point2d>   scan2;

    std::vector<cluster>   clusters;


    // Fill the scan with one constant object
    // at 1 meter from the lidar
    for (int i=0; i<=360; i++)
    {
        scan.emplace_back(Point2d(1.0 * cos(deg2rad(i)), 1.0 * sin(deg2rad(i))));
        scan2.emplace_back(Point2d(1.0 * cos(deg2rad(i)), 1.0 * sin(deg2rad(i))));

    }

    // Call the tested function
    get_clusters(scan, clusters, 0.1);

    // Clear the input vector
    scan.clear();

    // There is only one object and 
    // it has not been cleared.
    ASSERT_EQ(clusters.size(),(long unsigned int)1);
    EXPECT_EQ(scan2, clusters[0]);
}



TEST(GetClusters, NotInputVector){
    /**
        @test If the input vector
        is empty then the output
        should be empty.
    */
    std::vector<Point2d>   polarv;
    std::vector<cluster> clusters;

    // Call the tested function
    get_clusters(polarv, clusters, 1);

    // Check that the output 
    // cluster is empty.
    EXPECT_EQ(clusters.size(),(long unsigned int)0);
}


TEST(GetClusters, OnlyOneBeam){
    /**
        @test If the input vector
        only contains one beam
        there is only one object
        formed by this beam.
    */
    std::vector<Point2d>   scan;
    std::vector<cluster> clusters;


    // Fill the scan with only one beam
    scan.emplace_back(Point2d(10.0, 0));

    // Call the tested function
    get_clusters(scan, clusters, 1);
    
    // Check that the clusters 
    // vector only contains one
    // vector with only one scan.
    ASSERT_EQ(clusters.size()    ,(long unsigned int)1);
    EXPECT_EQ(clusters[0].size() ,(long unsigned int)1);
    EXPECT_EQ(clusters[0]        , scan);  
}

TEST(GetClusters, OneRisingObject){
    /**
        @test It takes a scan with 
        only one object but this 
        object is an a rising edge.
    */
    std::vector<Point2d>   scan;
    std::vector<cluster>   clusters;
    float m {1.0};


    // Fill the scan with one object that
    // is every beam 0.01 meters further
    for (int i=0; i<=360; i++)
    {
        scan.emplace_back(Point2d(m * cos(deg2rad(i)), m * sin(deg2rad(i))));
        m = m + 0.01;
    }


    // Call the tested function
    get_clusters(scan, clusters, 0.1);

    // There is only one object
    ASSERT_EQ(clusters.size(),(long unsigned int)1);

    // The only object is the whole scan
    EXPECT_EQ(scan, clusters[0]);   
}

TEST(GetClusters, OneDecreasingObject){
    /**
        @test It takes a scan with 
        only one object but this 
        object is an a decreasing
        edge.
    */
    std::vector<Point2d>   scan;
    std::vector<cluster>   clusters;
    float m {1.0};


    // Fill the scan with one object that
    // is every beam 0.01 meters nearer
    for (int i=0; i<=360; i++)
    {
        scan.emplace_back(Point2d(m * cos(deg2rad(i)), m * sin(deg2rad(i))));
        m = m - 0.01;
    }


    // Call the tested function
    get_clusters(scan, clusters, 0.1);

    // There is only one object
    ASSERT_EQ(clusters.size(),(long unsigned int)1);

    // The only object is the whole scan
    EXPECT_EQ(scan, clusters[0]);   
}

TEST(GetClusters, OneObjectWithRandomNoise){
    /**
        @test It takes a scan with 
        only one object but this 
        object has a random noise.
    */
    std::vector<Point2d>   scan;
    std::vector<cluster>   clusters;
    float m {1.0};
    float n {0.1};

    // Fill the start of the scan with 
    // one object that is every beam 
    // 0.01 meters further
    for (int i=0; i<180; i++)
    {
        m = 1.0 + n;
        scan.emplace_back(Point2d(m * cos(deg2rad(i)), m * sin(deg2rad(i))));
        
        // Generate a random number between 0 and 4
        n = rand() % 5;        
        // Change the range of the number to -2.5 and 1.5.
        n = n - 2.5;          
        // Tranfomr it in to a range of -0.025 and 0.015
        n = n / 100;
    }


    // Call the tested function
    get_clusters(scan, clusters, 0.1);

    // There is only one object
    ASSERT_EQ(clusters.size(),(long unsigned int)1);

    // The only object is the whole scan
    EXPECT_EQ(scan, clusters[0]);   
}

TEST(GetClusters, OneCornerObject){
    /**
        @test It takes a scan with 
        only one object but this 
        object starts in rising edge
        but ends in a decreasing edge
        making a corner.
    */
    std::vector<Point2d>   scan;
    std::vector<cluster>   clusters;
    float m {1.0};

    // Fill the start of the scan with 
    // one object that is every beam 
    // 0.01 meters further
    for (int i=0; i<180; i++)
    {
        scan.emplace_back(Point2d(m * cos(deg2rad(i)), m * sin(deg2rad(i))));
        m = m + 0.01;
    }

    // Fill the end of the scan with 
    // the same object but start 
    // decreasing its module
    for (int i=180; i<=360; i++)
    {
        scan.emplace_back(Point2d(m * cos(deg2rad(i)), m * sin(deg2rad(i))));
        m = m - 0.01;
    }

    // Call the tested function
    get_clusters(scan, clusters, 0.1);

    // There is only one object
    ASSERT_EQ(clusters.size(),(long unsigned int)1);

    // The only object is the whole scan
    EXPECT_EQ(scan, clusters[0]);   
}


TEST(GetClusters, TwoObjects){
    /**
        @test Test the basic use of the 
        get_cluster proccesure. It takes
        a scan with two objects.
    */
    std::vector<Point2d>   scan;
    std::vector<cluster>   clusters;
    std::vector<Point2d>   first_obj;
    std::vector<Point2d>   second_obj;

    float x{0};
    float y{0};


    // Fill the beggining of the scan with 
    // one constant object that is 1 meter 
    // from the lidar
    for (int i=0; i<180; i++){
        // Calculate X and Y 
        // cartesian coord
        x = 1.0 * cos(deg2rad(i));
        y = 1.0 * sin(deg2rad(i));

        // Store this coords in the 
        // scan vector and in an object
        // vector to improve readable
        scan.emplace_back(Point2d(x, y));
        first_obj.emplace_back(Point2d(x, y));
    }

    // Fill the end of the scan with 
    // one constant object that is 2 meter 
    // from the lidar
    for (int i=180; i<220; i++){
        // Calculate X and Y 
        // cartesian coord
        x = 2.0 * cos(deg2rad(i));
        y = 2.0 * sin(deg2rad(i));

        // Store this coords in the 
        // scan vector and in an object
        // vector to improve readable
        scan.emplace_back(Point2d(x, y));
        second_obj.emplace_back(Point2d(x, y));
    }
        
    // Call the tested function 
    get_clusters(scan, clusters, 0.1);

    // There are 2 clusters in the output 
    // cluster vector
    ASSERT_EQ(clusters.size(),(long unsigned int)2);

    // The first cluster corresponds to 
    // the first object.
    EXPECT_EQ(clusters[0], first_obj);
    EXPECT_EQ(clusters[1], second_obj);
}


TEST(GetClusters, TwoObjectsButOneAroundTheLidarEdge){
    /**
        @test Test that get_cluster can
        identify if one object is around
        the lidar edge.
    */
    std::vector<Point2d>   scan;
    std::vector<cluster>   clusters;
    std::vector<Point2d>   first_obj;
    std::vector<Point2d>   second_obj;

    float x{0};
    float y{0};


    // Fill the beggining of the scan with 
    // one constant object that is 1 meter 
    // from the lidar
    for (int i=0; i<180; i++){
        // Calculate X and Y 
        // cartesian coord
        x = 1.0 * cos(deg2rad(i));
        y = 1.0 * sin(deg2rad(i));

        // Store this coords in the 
        // scan vector and in an object
        // vector to improve readable
        scan.emplace_back(Point2d(x, y));
        first_obj.emplace_back(Point2d(x, y));
    }

    // Fill the intermediate of the scan with 
    // one constant object that is 2 meter 
    // from the lidar
    for (int i=180; i<220; i++){
        // Calculate X and Y 
        // cartesian coord
        x = 2.0 * cos(deg2rad(i));
        y = 2.0 * sin(deg2rad(i));

        // Store this coords in the 
        // scan vector and in an object
        // vector to improve readable
        scan.emplace_back(Point2d(x, y));
        second_obj.emplace_back(Point2d(x, y));
    }

    // Fill the end of the scan with 
    // same object that was in the
    // start of the scan.
    for (int i=220; i<=360; i++){
        // Calculate X and Y 
        // cartesian coord
        x = 1.0 * cos(deg2rad(i));
        y = 1.0 * sin(deg2rad(i));

        // Store this coords in the 
        // scan vector and in an object
        // vector to improve readable
        scan.emplace_back(Point2d(x, y));
        first_obj.emplace_back(Point2d(x, y));
    }
        
    // Call the tested function 
    get_clusters(scan, clusters, 0.1);

    // There are 2 clusters in the output 
    // cluster vector
    ASSERT_EQ(clusters.size(),(long unsigned int)2);

    // The first cluster corresponds to 
    // the first object.
    EXPECT_EQ(clusters[0], first_obj);
    EXPECT_EQ(clusters[1], second_obj);
}





TEST(GetClusters, ThreeObjects){
    /**
        @test Test the basic use of the 
        get_cluster proccesure. It takes
        a scan with three objects.
    */
    std::vector<Point2d>   scan;
    std::vector<cluster>   clusters;
    std::vector<Point2d>   first_obj;
    std::vector<Point2d>   second_obj;
    std::vector<Point2d>   third_obj;


    float x{0};
    float y{0};


    // Fill the beggining of the scan with 
    // one constant object that is 1 meter 
    // from the lidar
    for (int i=0; i<180; i++){
        // Calculate X and Y 
        // cartesian coord
        x = 1.0 * cos(deg2rad(i));
        y = 1.0 * sin(deg2rad(i));

        // Store this coords in the 
        // scan vector and in an object
        // vector to improve readable
        scan.emplace_back(Point2d(x, y));
        first_obj.emplace_back(Point2d(x, y));
    }

    // Fill the end of the scan with 
    // one constant object that is 2 meter 
    // from the lidar
    for (int i=180; i<220; i++){
        // Calculate X and Y 
        // cartesian coord
        x = 2.0 * cos(deg2rad(i));
        y = 2.0 * sin(deg2rad(i));

        // Store this coords in the 
        // scan vector and in an object
        // vector to improve readable
        scan.emplace_back(Point2d(x, y));
        second_obj.emplace_back(Point2d(x, y));
    }

    for (int i=180; i<220; i++){
        // Calculate X and Y 
        // cartesian coord
        x = 1.5 * cos(deg2rad(i));
        y = 1.5 * sin(deg2rad(i));

        // Store this coords in the 
        // scan vector and in an object
        // vector to improve readable
        scan.emplace_back(Point2d(x, y));
        third_obj.emplace_back(Point2d(x, y));
    }
        
    // Call the tested function 
    get_clusters(scan, clusters, 0.1);

    // There are 3 clusters in the output 
    // cluster vector
    ASSERT_EQ(clusters.size(),(long unsigned int)3);

    // The different clusters corresponds
    // to the different objs
    EXPECT_EQ(clusters[0], first_obj);
    EXPECT_EQ(clusters[1], second_obj);
    EXPECT_EQ(clusters[2], third_obj);
}

TEST(GetClusters, ThreeObjectsAndTwoVerySimilar){
    /**
        @test Test the basic use of the 
        get_cluster proccesure. It takes
        a scan with three objects but
        the first and the third are quite
        similar.
    */
    std::vector<Point2d>   scan;
    std::vector<cluster>   clusters;
    std::vector<Point2d>   first_obj;
    std::vector<Point2d>   second_obj;
    std::vector<Point2d>   third_obj;


    float x{0};
    float y{0};


    // Fill the beggining of the scan with 
    // one constant object that is 1 meter 
    // from the lidar
    for (int i=0; i<180; i++){
        // Calculate X and Y 
        // cartesian coord
        x = 1.0 * cos(deg2rad(i));
        y = 1.0 * sin(deg2rad(i));

        // Store this coords in the 
        // scan vector and in an object
        // vector to improve readable
        scan.emplace_back(Point2d(x, y));
        first_obj.emplace_back(Point2d(x, y));
    }

    // Fill the end of the scan with 
    // one constant object that is 2 meter 
    // from the lidar
    for (int i=180; i<220; i++){
        // Calculate X and Y 
        // cartesian coord
        x = 2.0 * cos(deg2rad(i));
        y = 2.0 * sin(deg2rad(i));

        // Store this coords in the 
        // scan vector and in an object
        // vector to improve readable
        scan.emplace_back(Point2d(x, y));
        second_obj.emplace_back(Point2d(x, y));
    }

    for (int i=180; i<220; i++){
        // Calculate X and Y 
        // cartesian coord
        x = 1.0 * cos(deg2rad(i));
        y = 1.0 * sin(deg2rad(i));

        // Store this coords in the 
        // scan vector and in an object
        // vector to improve readable
        scan.emplace_back(Point2d(x, y));
        third_obj.emplace_back(Point2d(x, y));
    }
        
    // Call the tested function 
    get_clusters(scan, clusters, 0.1);

    // There are 3 clusters in the output 
    // cluster vector
    ASSERT_EQ(clusters.size(),(long unsigned int)3);

    // The different clusters corresponds
    // to the different objs
    EXPECT_EQ(clusters[0], first_obj);
    EXPECT_EQ(clusters[1], second_obj);
    EXPECT_EQ(clusters[2], third_obj);
}



/*
TEST(GetClusters, TwoObjectsButOneVerySmallInTheMiddle){
    
        @test Test that get_cluster can
        identify one small in fron of a
        big one.
    
    std::vector<Point2d>   scan;
    std::vector<cluster>   clusters;
    std::vector<Point2d>   first_obj;
    std::vector<Point2d>   second_obj;

    float x{0};
    float y{0};


    // Fill the beggining of the scan with 
    // one constant object that is 1 meter 
    // from the lidar
    for (int i=0; i<180; i++){
        // Calculate X and Y 
        // cartesian coord
        x = 1.0 * cos(deg2rad(i));
        y = 1.0 * sin(deg2rad(i));

        // Store this coords in the 
        // scan vector and in an object
        // vector to improve readable
        scan.emplace_back(Point2d(x, y));
        first_obj.emplace_back(Point2d(x, y));
    }

    // Fill the intermediate of the scan with 
    // one constant object that is 2 meter 
    // from the lidar
    for (int i=0; i<2; i++){
        // Calculate X and Y 
        // cartesian coord
        x = 0.5 * cos(deg2rad(i));
        y = 0.5 * sin(deg2rad(i));

        // Store this coords in the 
        // scan vector and in an object
        // vector to improve readable
        scan.emplace_back(Point2d(x, y));
        second_obj.emplace_back(Point2d(x, y));
    }

    // Fill the end of the scan with 
    // same object that was in the
    // start of the scan.
    for (int i=182; i<=220; i++){
        // Calculate X and Y 
        // cartesian coord
        x = 1.0 * cos(deg2rad(i));
        y = 1.0 * sin(deg2rad(i));

        // Store this coords in the 
        // scan vector and in an object
        // vector to improve readable
        scan.emplace_back(Point2d(x, y));
        first_obj.emplace_back(Point2d(x, y));
    }
        
    // Call the tested function 
    get_clusters(scan, clusters, 0.1);

    // There are 2 clusters in the output 
    // cluster vector
    ASSERT_EQ(clusters.size(),(long unsigned int)2);

    // The first cluster corresponds to 
    // the first object.
    EXPECT_EQ(clusters[0], first_obj);
    EXPECT_EQ(clusters[1], second_obj);
}*/


