#include <gtest/gtest.h>
#include "analize_scan_utils.hpp"


TEST(LaserScanToPolarV, BasicCaseOfUse){
    
    std::vector<float> ranges;
    std::vector<polar> polarv;

    float delta = 0.1;
    
    for (int i=0; i<=15; i++)
    {
        ranges.push_back(i);
    }

    
    laser_scan_to_polarv(ranges, delta, polarv);

    for (int i=0; i<=15; i++)
    {
        EXPECT_FLOAT_EQ(polarv[i].mod,  i);
        EXPECT_FLOAT_EQ(polarv[i].alfa, i*delta);
    }
}

TEST(LaserScanToPolarV, BasicCaseOfUseNegativeNumbers){
    
    std::vector<float> ranges;
    std::vector<polar> polarv;

    float delta = 0.1;
    
    for (int i=0; i<=10; i++)
    {
        ranges.push_back(-i);
    }

    
    laser_scan_to_polarv(ranges, delta, polarv);

    for (int i=0; i<=10; i++)
    {
        EXPECT_FLOAT_EQ(polarv[i].mod,  -i);
        EXPECT_FLOAT_EQ(polarv[i].alfa, i*delta);
    }
}


TEST(LaserScanToPolarV, BasicCaseOfUseDeltaZero){
    std::vector<float> ranges;
    std::vector<polar> polarv;
    float delta = 0;
    
    for (int i=0; i<=12; i++)
    {
        ranges.push_back(i);
    }

    laser_scan_to_polarv(ranges, delta, polarv);

    for (int i=0; i<=12; i++)
    {
        EXPECT_FLOAT_EQ(polarv[i].mod,  i);
        EXPECT_FLOAT_EQ(polarv[i].alfa, 0);
    }
}

TEST(LaserScanToPolarV, EmptyRange){
    
    std::vector<float> ranges;
    std::vector<polar> polarv;
    float delta = 0.1;
    laser_scan_to_polarv(ranges, delta, polarv);
    EXPECT_EQ(polarv.size(),(long unsigned int)0);
}

TEST(LaserScanToPolarV, NotEmptyPolarV){
    
    std::vector<float> ranges;
    std::vector<polar> polarv;

    float delta = 0.1;
    
    for (int i=0; i<=15; i++)
    {
        ranges.push_back(i);
        polarv.emplace_back(polar(500.0,500.0));
    }

    
    laser_scan_to_polarv(ranges, delta, polarv);

    for (int i=0; i<=15; i++)
    {
        EXPECT_FLOAT_EQ(polarv[i].mod,  i);
        EXPECT_FLOAT_EQ(polarv[i].alfa, i*delta);
    }
}

TEST(GetClusters, BasicCircleAroundTheObject){
    
    std::vector<polar>   polarv;
    std::vector<cluster> clusters;
    float delta = 0.1;

    for (int i=0; i<=15; i++)
    {
        polarv.emplace_back(polar(10.0, i*delta));
    }

    get_clusters(polarv, clusters, 1, 0.2);


    ASSERT_EQ(clusters.size(),(long unsigned int)1);

    
    for (int i=0; i<=15; i++)
    {
        EXPECT_FLOAT_EQ(clusters[0][i].mod,  10.0);
        EXPECT_FLOAT_EQ(clusters[0][i].alfa, i*delta);
    }
}


TEST(GetClusters, BasicCircleAroundTheObject2){
    
    std::vector<polar>   polarv;
    std::vector<cluster> clusters;
    float delta = 0.1;

    for (int i=0; i<=15; i++)
    {
        polarv.emplace_back(polar(10.0, i*delta));
    }

    get_clusters(polarv, clusters, 1, delta);

    ASSERT_EQ(clusters.size(),(long unsigned int)1);

    for (int i=0; i<=15; i++)
    {
        EXPECT_FLOAT_EQ(clusters[0][i].mod,  10);
        EXPECT_FLOAT_EQ(clusters[0][i].alfa, i*delta);
    }
}

TEST(GetClusters, VectorClear){
    
    std::vector<polar>   polarv;
    std::vector<cluster> clusters;
    float delta = 0.1;

    for (int i=0; i<=15; i++)
    {
        polarv.emplace_back(polar(10.0, i*delta));
    }

    get_clusters(polarv, clusters, 1, delta);

    polarv.clear();
    ASSERT_EQ(clusters.size(),(long unsigned int)1);

    
    for (int i=0; i<=15; i++)
    {
        EXPECT_FLOAT_EQ(clusters[0][i].mod,  10);
        EXPECT_FLOAT_EQ(clusters[0][i].alfa, i*delta);
    }
}

TEST(GetClusters, NotPolarRanges){
    
    std::vector<polar>   polarv;
    std::vector<cluster> clusters;
    float delta = 0.1;

    get_clusters(polarv, clusters, 1, delta);
    EXPECT_EQ(clusters.size(),(long unsigned int)0);
}

TEST(GetClusters, OnePolarRanges){
    
    std::vector<polar>   polarv;
    std::vector<cluster> clusters;
    float delta = 0.1;

    polarv.emplace_back(polar(10.0, 0));

    get_clusters(polarv, clusters, 1, delta);
    ASSERT_EQ(clusters.size(),(long unsigned int)1);
    EXPECT_FLOAT_EQ(clusters[0][0].mod,  10);
    EXPECT_FLOAT_EQ(clusters[0][0].alfa, 0);
    EXPECT_EQ(clusters[0].size(),(long unsigned int)1);
}

TEST(GetClusters, TwoObjects){
    std::vector<polar>   polarv;
    std::vector<cluster> clusters;
    float delta = 0.1;

    for (int i=0; i<=15; i++)
    {
        if (i < 7) polarv.emplace_back(polar(10.0, i*delta));
        else polarv.emplace_back(polar(200.0, i*delta));
    }

    get_clusters(polarv, clusters, 1, delta);

    polarv.clear();
    ASSERT_EQ(clusters.size(),(long unsigned int)2);
    EXPECT_EQ(clusters[0].size(),(long unsigned int)7);
    EXPECT_EQ(clusters[1].size(),(long unsigned int)9);
    
    for (int i=0; i<7; i++)
    {
        EXPECT_FLOAT_EQ(clusters[0][i].mod,  10);
        EXPECT_FLOAT_EQ(clusters[0][i].alfa, i*delta);        
    }

    for (int i=7; i<=15; i++)
    {
        EXPECT_FLOAT_EQ(clusters[1][i-7].mod,  200);
        EXPECT_FLOAT_EQ(clusters[1][i-7].alfa, i*delta);        
    }
}

TEST(GetClusters, TwoObjectsButOneAroundTheLidarEdge){
    
    std::vector<polar>   polarv;
    std::vector<cluster> clusters;
    float delta = 0.1;

    for (int i=0; i<=20; i++)
    {
        if      (i < 7)  polarv.emplace_back(polar(10.0, -179 + i*delta));
        else if (i < 15) polarv.emplace_back(polar(200.0, i*delta));
        else             polarv.emplace_back(polar(10.0, 178 + i*delta));
    }

    get_clusters(polarv, clusters, 1, 6);

    polarv.clear();
    ASSERT_EQ(clusters.size(),(long unsigned int)2);
    EXPECT_EQ(clusters[0].size(),(long unsigned int)13);
    EXPECT_EQ(clusters[1].size(),(long unsigned int)8);
    
    for (int i=0; i<=20; i++)
    {
        if (i < 7)
        {
            EXPECT_FLOAT_EQ(clusters[0][i].mod,  10.0);
            EXPECT_FLOAT_EQ(clusters[0][i].alfa, -179 + i*delta);
        }
        else if (i < 15)
        {
            EXPECT_FLOAT_EQ(clusters[1][i-7].mod,  200);
            EXPECT_FLOAT_EQ(clusters[1][i-7].alfa, i*delta);
        } 
        else
        {
            EXPECT_FLOAT_EQ(clusters[0][i-8].mod,  10.0);
            EXPECT_FLOAT_EQ(clusters[0][i-8].alfa, 178 + i*delta);
        }
    }
}


TEST(GetClusters, ThreeObjects){
    std::vector<polar>   polarv;
    std::vector<cluster> clusters;
    float delta = 0.1;

    for (int i=0; i<=20; i++)
    {
        if (i < 7) polarv.emplace_back(polar(10.0, i*delta));
        else if (i < 15) polarv.emplace_back(polar(200.0, i*delta));
        else polarv.emplace_back(polar(10.0, i*delta));
    }

    get_clusters(polarv, clusters, 1, delta);

    polarv.clear();
    ASSERT_EQ(clusters.size(),(long unsigned int)3);
    EXPECT_EQ(clusters[0].size(),(long unsigned int)7);
    EXPECT_EQ(clusters[1].size(),(long unsigned int)8);
    EXPECT_EQ(clusters[2].size(),(long unsigned int)6);

    
    for (int i=0; i<=20; i++)
    {
        if (i < 7)
        {
            EXPECT_FLOAT_EQ(clusters[0][i].mod,  10.0);
            EXPECT_FLOAT_EQ(clusters[0][i].alfa, i*delta);
        }
        else if (i < 15)
        {
            EXPECT_FLOAT_EQ(clusters[1][i-7].mod,  200);
            EXPECT_FLOAT_EQ(clusters[1][i-7].alfa, i*delta);
        } 
        else
        {
            EXPECT_FLOAT_EQ(clusters[2][i-15].mod,  10.0);
            EXPECT_FLOAT_EQ(clusters[2][i-15].alfa, i*delta);
        }
    }
}
