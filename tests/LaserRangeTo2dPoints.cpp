#include <gtest/gtest.h>
#include "analize_scan_utils.hpp"
#include <math.h>
#define PI 3.14

inline float deg2rad(float deg)
{
    return deg * PI /180;
}

TEST(LaserRangeTo2dPoints, BasicCaseOfUse){
    /**
        @test Test the basic use of the 
        LaserRangeTo2dPoints proccesure.
    */
    std::vector<float> ranges;
    std::vector<Point2d> points2D;
    std::vector<Point2d> expected_output;


    const float delta {0.1};
    float angle {0.0};
    float x {0.0};
    float y {0.0};

    // Fill the ranges vector with 
    // some numbers and calculate
    // the 2d points and stored
    // in an expected output
    for (int i=0; i<=15; i++)
    {
        ranges.push_back(i);
     
     
        x = i * cos(angle);
        y = i * sin(angle);

        angle = angle + delta;
        expected_output.emplace_back(Point2d(x,y));
    }

    // Call the tested function
    LaserRangeTo2dPoints(ranges, delta, points2D);

    // There are 16 points
    ASSERT_EQ(points2D.size(),(long unsigned int)16);

    // The output vector is the expected output
    EXPECT_EQ(points2D, expected_output);   
}


TEST(LaserRangeTo2dPoints, BasicCaseOfUseNegativeNumbers){
    /**
        @test Test the basic use of the 
        LaserRangeTo2dPoints proccesure
        with negative ranges.
    */
    std::vector<float> ranges;
    std::vector<Point2d> points2D;
    std::vector<Point2d> expected_output;


    const float delta {0.1};
    float angle {0.0};
    float x {0.0};
    float y {0.0};

    // Fill the ranges vector with 
    // some numbers and calculate
    // the 2d points and stored
    // in an expected output
    for (int i=0; i<=15; i++)
    {
        ranges.push_back(-i);
     
     
        x = -i * cos(angle);
        y = -i * sin(angle);

        angle = angle + delta;
        expected_output.emplace_back(Point2d(x,y));
    }

    // Call the tested function
    LaserRangeTo2dPoints(ranges, delta, points2D);

    // There are 16 points
    ASSERT_EQ(points2D.size(),(long unsigned int)16);

    // The output vector is the expected output
    EXPECT_EQ(points2D, expected_output);   
}



TEST(LaserScanToPolarV, BasicCaseOfUseDeltaZero){
    /**
        @test Test the basic use of the 
        LaserRangeTo2dPoints proccesure
        with a null delta increment.
    */
    std::vector<float> ranges;
    std::vector<Point2d> points2D;
    std::vector<Point2d> expected_output;


    const float delta {0.0};
    float angle {0.0};
    float x {0.0};
    float y {0.0};

    // Fill the ranges vector with 
    // some numbers and calculate
    // the 2d points and stored
    // in an expected output
    for (int i=0; i<=15; i++)
    {
        ranges.push_back(i);
     
     
        x = i * cos(angle);
        y = i * sin(angle);

        angle = angle + delta;
        expected_output.emplace_back(Point2d(x,y));
    }

    // Call the tested function
    LaserRangeTo2dPoints(ranges, delta, points2D);

    // There are 16 points
    ASSERT_EQ(points2D.size(),(long unsigned int)16);

    // The output vector is the expected output
    EXPECT_EQ(points2D, expected_output);   
}


TEST(LaserScanToPolarV, EmptyRange){
    /**
        @test Test the LaserRangeTo2dPoints
        proccesure if a empty vector of ranges
        is given.
    */
    std::vector<float> ranges;
    std::vector<Point2d> points2D;
    std::vector<Point2d> expected_output;


    const float delta {5.0};

    // Call the tested function
    LaserRangeTo2dPoints(ranges, delta, points2D);

    // There are 16 points
    ASSERT_EQ(points2D.size(),(long unsigned int)0);

}



TEST(LaserScanToPolarV, NotEmptyPolarV){
    /**
        @test Test the basic use of the 
        LaserRangeTo2dPoints proccesure
        given a not Empty output vector.
    */
    std::vector<float> ranges;
    std::vector<Point2d> points2D;
    std::vector<Point2d> expected_output;


    const float delta {0.0};
    float angle {0.0};
    float x {0.0};
    float y {0.0};

    // Fill the ranges vector with 
    // some numbers and calculate
    // the 2d points and stored
    // in an expected output
    for (int i=0; i<=15; i++)
    {
        ranges.push_back(i);
    
        x = i * cos(angle);
        y = i * sin(angle);

        angle = angle + delta;
        expected_output.emplace_back(Point2d(x,y));
    }

    // Fill the output vector with random numbers 
    // before the LaserRangeTo2dPoints
    for (int i=0; i<=35; i++)
    {
        points2D.emplace_back(Point2d(i,i*3));
    }

    // Call the tested function
    LaserRangeTo2dPoints(ranges, delta, points2D);

    // There are 16 points
    ASSERT_EQ(points2D.size(),(long unsigned int)16);

    // The output vector is the expected output
    EXPECT_EQ(points2D, expected_output);   
}