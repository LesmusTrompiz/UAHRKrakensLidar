#include "analize_scan_utils.hpp"


//void filter_beams_by_range(const std::vector<float> ranges, std::vector<polar> polars, const float min_range, const float max_range, const float min_angle, const float max_angle, const float delta_angle)
//{
//    /**
//     * @brief Filters a vector of beams by  
//     * 
//     * @param b Byte received
//     */
//
//
//    return;
//}


void laser_scan_to_polarv(const std::vector<float> &ranges, const float &delta_angle, std::vector<polar> &polar_vector){
    float angle = 0.0;


    for (auto r : ranges)
    {
        polar_vector.emplace_back(polar(r,angle));
        angle = angle + delta_angle;
    }


    return;
}


void get_clusters(std::vector<polar> &scan, std::vector<cluster> &clusters, float range_edge, float &angle_edge)
{
    /**
     * @brief Search the clusters in a scan and stores in clusters.   
     * 
     * @param b Byte received
    */



    return;
}


