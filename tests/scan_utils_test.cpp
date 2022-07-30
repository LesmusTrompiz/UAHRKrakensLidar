#include <gtest/gtest.h>
#include "analize_scan_utils.hpp"


TEST(LaserScanToPolarV, BasicUse){
    
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

TEST(LaserScanToPolarV, BasicUseNegativeNumbers){
    
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


TEST(LaserScanToPolarV, BasicUseDeltaZero){
    
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

    EXPECT_EQ(polarv.size(),0);
}

