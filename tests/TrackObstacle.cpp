#include <gtest/gtest.h>
#include "analize_scan_utils.hpp"
#include <math.h>


TEST(TrackObstacle, EmptyTracked){
    /**
        @test Test the basic use of the 
        TrackObstacle proccesure.

        Given an empty tracked vector,
        the proccesure should fill the
        tracked vector with all the new
        detected points. 
    */
    std::vector<Point2d> centroids;
    std::vector<Obstacle> tracked_obstacles;
    std::vector<Obstacle> expected_output;


    // Generate some centroids
    for (int i=0; i<=2; i++)
    {
        centroids.emplace_back(i,i);
        expected_output.emplace_back(i,i);
    }

    // Call the tested function
    track_obstacles(centroids, tracked_obstacles);

    // There are 3 tracked obstacles
    ASSERT_EQ(tracked_obstacles.size(),(long unsigned int) 3);
    EXPECT_EQ(tracked_obstacles,expected_output);
}

TEST(TrackObstacle, EqualCentroids){
    /**
        @test Test the basic use of the 
        TrackObstacle proccesure.

        The centroids vector are the
        centroids of the tracked 
        vector. The output tracked
        vector should increase its
        track_count elements +1. 
    */
    std::vector<Point2d> centroids;
    std::vector<Obstacle> tracked_obstacles;
    std::vector<Obstacle> expected_output;


    // Generate some centroids
    for (int i=0; i<=5; i++)
    {
        centroids.emplace_back(i,i);
        tracked_obstacles.emplace_back(i,i,i);
        expected_output.emplace_back(i,i,i+1);
    }

    // Call the tested function
    track_obstacles(centroids, tracked_obstacles);

    // There are 3 tracked obstacles
    ASSERT_EQ(tracked_obstacles.size(),(long unsigned int) 6);
    EXPECT_EQ(tracked_obstacles,expected_output);
}

TEST(TrackObstacle, EmptyScan){
    /**
        @test Test the basic use of the 
        TrackObstacle proccesure.

        The centroids vector is
        empty so the tracked 
        obstacles should reduce 
        the tracked_count of its
        elements by one, in case
        that the tracked_count is
        lower than cero this
        element shoul be deleted.

    */
    std::vector<Point2d> centroids;
    std::vector<Obstacle> tracked_obstacles;
    std::vector<Obstacle> expected_output;


    // Generate some centroids
    for (int i=0; i<=5; i++)
    {
        tracked_obstacles.emplace_back(i,i,i);
        if(i != 0)
        {
            expected_output.emplace_back(i,i,i-1);
        }
    }

    // Call the tested function
    track_obstacles(centroids, tracked_obstacles);

    // There are 3 tracked obstacles
    ASSERT_EQ(tracked_obstacles.size(),(long unsigned int) 5);
    EXPECT_EQ(tracked_obstacles,expected_output);
}


/*
TEST(TrackObstacle, SimilarCentroids){
    /**
        @test Test the basic use of the 
        TrackObstacle proccesure.

        The centroids vector are 
        similar to the centroids
        of the tracked vector. 
        The output tracked vector 
        should increase its
         track_count elements +1. 
    */
/*
    std::vector<Point2d> centroids;
    std::vector<Obstacle> tracked_obstacles;
    std::vector<Obstacle> expected_output;


    // Generate some centroids
    for (int i=0; i<=5; i++)
    {
        centroids.emplace_back(i,i);
        tracked_obstacles.emplace_back(i,i,i);
        expected_output.emplace_back(i,i,i+1);
    }

    // Call the tested function
    track_obstacles(centroids, tracked_obstacles);

    // There are 3 tracked obstacles
    ASSERT_EQ(tracked_obstacles.size(),(long unsigned int) 6);
    EXPECT_EQ(tracked_obstacles,expected_output);
}
*/