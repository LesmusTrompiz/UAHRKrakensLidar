#include "analize_scan_utils.hpp"
#include <iostream>
extern "C"{
    #include <math.h>
}
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


inline float calculate_square_dist(const Point2d &p1, const Point2d &p2)
{
    return ((p2.x - p1.x) * (p2.x - p1.x)) + ((p2.y - p1.y) * (p2.y - p1.y));
}

inline bool same_cluster(Point2d p1, Point2d p2, float max_dist)
{
    return calculate_square_dist(p1, p2) <= (max_dist * max_dist);
}


void LaserRangeTo2dPoints(const std::vector<float> &ranges,const float &angle_increment, std::vector<Point2d> &out_points)
{
    float angle = 0.0;
    out_points.clear();
    for (auto &r : ranges)
    {
        out_points.emplace_back(r*cos(angle), r * sin(angle));
        angle = angle + angle_increment;
    }
    return;
}

float get_cluster_square_length(const std::vector<Point2d> &cluster) {
    float length = 0.0;
    auto  old_p   = cluster.begin();

    for (auto p = cluster.begin() +1; p < cluster.end(); p++ )
    {
        length = length + calculate_square_dist(*old_p,*p);
    }
    return length;
}

inline bool object_between_end_and_start(const std::vector<cluster> &clusters, const Point2d &end_polar, const float &dist_increment)
{
    // This special case can only occur when the output clusters size
    //  is not empty  

    return (clusters.size() && same_cluster(clusters[0][0],end_polar,dist_increment));
}


Point2d get_cluster_contour_centroid (const std::vector<Point2d> &cluster) {
    Point2d centroid;

    for (auto &p : cluster)
    {
        centroid.x = centroid.x + p.x;
        centroid.y = centroid.y + p.y;
    }
    centroid.x = centroid.x / cluster.size();
    centroid.y = centroid.y / cluster.size();
    return centroid;
}

void filter_cluster_by_length(const std::vector<std::vector<Point2d>> &clusters, const float &min_length, const float &max_length, std::vector<std::vector<Point2d>> &out_clusters) {
    
    float square_min_length     = min_length * min_length;
    float square_max_length     = max_length * max_length;
    float cluster_square_length = 0;

    out_clusters.clear();

    for(auto &c : clusters)
    {
        cluster_square_length = get_cluster_square_length(c);
        if ((square_min_length <= cluster_square_length) && (cluster_square_length <= square_max_length))
        {
            out_clusters.push_back(c);
        }
    }

    return;
}



void get_clusters(const std::vector<Point2d> &scan, std::vector<std::vector<Point2d>> &clusters, const float &dist_increment)
{
    /**
     * @brief Search the clusters in a polar vector and stores in the output clusters vector.   
     * The procces consists on comparing two secuencial polars of the vector and check if 
     * they are near enough to consider part of the same cluster.
     *  
     * @param scan     A vector of polar structs that may contain clusters.
     * @param clusters A vector of clusters that contain the cluster found in scan.
     * @param mod_increment The max module incremente between polars to consider part of
     * the same object.
     * @param alfa_increment The max angle incremente between polars to consider part of
     * the same object.
     * 
    */

    // Clean the old values of the output vector
    clusters.clear();    

    // If the scan object is empty dont do anything, it does not contain 
    // clusters and the procces of searching will make the program fail.
    if (scan.size())
    {

        // Create and auxiliar cluster, it will be used
        // to feed the output cluster vector.
        cluster aux_cluster;

        // Reserve a space of 20 polars to avoid moving procces
        aux_cluster.reserve(20);

        // old polar will store the value of the
        // last polar in the last iteration, it 
        // will be initialized with the head of the
        // scan.
        auto    old_p  = scan.begin();

        // Store the head in the aux cluster
        aux_cluster.push_back(*old_p);

        // Iterate over the tail of the scan
        for (auto p = scan.begin() + 1; p < scan.end(); p++)
        {

            // If the actual polar and the last one are 
            // part of the same cluster add it to the actual
            // cluster
            if (same_cluster(*p,*old_p,dist_increment))
            {
                aux_cluster.push_back(*p); 
            }
            else
            {
                // If they are from different clusters
                // store the old cluster in the output
                // vector
                clusters.push_back(aux_cluster);

                // Clear the content of the aux cluster
                aux_cluster.clear();

                // Store the new polar
                aux_cluster.push_back(*p);
            }
            // Store the value in the old polar variable for 
            // the next iteration
            old_p = p;
        }

        // Righ now there is a cluster in aux cluster that may be pushed
        // to the clusters output vector.But there is a special case
        // where the aux cluster is part of the first cluster stored in 
        // the output vector, this may happen with the objects that
        // are between the end scan of the lidar and the start of the scan. 

        // If the last cluster is part of the first one insert the containt of
        // aux_cluster in the first cluster
        if(object_between_end_and_start(clusters, *old_p, dist_increment)) 
            clusters[0].insert(clusters[0].end(),aux_cluster.begin(), aux_cluster.end());                
            
        // If it isnt the special case just push the aux_cluster 
        // to the output vector.
        else clusters.push_back(aux_cluster);
    }
    return;
}

Point2d nearest_centroid(const Point2d &center, const std::vector<Point2d> &neighbours)
{
    auto  nearest_neigh = neighbours.begin();
    float min_dist      = calculate_square_dist(center, *nearest_neigh);
    float dist          = 0.0;

    for (auto p = neighbours.begin() + 1; p < neighbours.end(); p++)
    {
        dist = calculate_square_dist(*p, center);
        if (dist < min_dist)
        {
            nearest_neigh = p;
            min_dist      = dist;
        }
    }
    return *nearest_neigh;
}




