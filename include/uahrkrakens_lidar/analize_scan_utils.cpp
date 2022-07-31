#include "analize_scan_utils.hpp"
#include <iostream>

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
    /**
        * @brief Given all the ranges of a
        * laser scan and the angle increment
        * between them, stores the values 
        * as a polar struct in the polar vector.
    
        * @param ranges ranges of a laser scan
        * @param delta_angle angle increment between ranges
        * @param polar_vector a vector that contains the 
        * ranges as polar form

    */

    // Clean the old values of the output vector
    polar_vector.clear();                                            

    // Create and auxiliar variable
    // that stores the current angle
    // of the beam
    float angle = 0.0;

    // Iterate over the range vector and 
    // store the values of the beams as 
    // polar struct
    for (auto & r : ranges)
    {
        polar_vector.emplace_back(polar(r,angle));
        angle = angle + delta_angle;
    }
    return;
}

inline bool same_cluster(const polar &p, const polar &edge,const float &mod_increment, const float &alfa_increment)
{
    return ((edge.mod - mod_increment  <= p.mod) && (edge.alfa - alfa_increment  <= p.alfa)
    &&
    (p.mod <= edge.mod + mod_increment) && (p.alfa <= edge.alfa + alfa_increment));
}

bool object_between_end_and_start(const std::vector<cluster> &clusters, const polar &end_polar, const float &mod_increment, const float &alfa_increment)
{
    // This special case can only occur when the output clusters size
    //  is not empty  
    if (clusters.size())
    {
        // The nearest polar to the polar in 
        // the scan end will be the first 
        // of the first cluster.
        auto p = clusters[0][0];

        // Before comparing them its neccesary
        // to add 360 degrees to the first scan
        // cause other whise it will be whole
        // circunference between them
        p.alfa = p.alfa + 360;
        return (same_cluster(p,end_polar,mod_increment,alfa_increment));
    }
    return false;
}

void get_clusters(const std::vector<polar> &scan, std::vector<cluster> &clusters, const float &mod_increment, const float &alfa_increment)
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
        // to feed the output cluester vector.
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
            if (same_cluster(*p,*old_p,mod_increment,alfa_increment))
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
        if(object_between_end_and_start(clusters, *old_p, mod_increment, alfa_increment)) 
            clusters[0].insert(clusters[0].end(),aux_cluster.begin(), aux_cluster.end());                
            
        // If it isnt the special case just push the aux_cluster 
        // to the output vector.
        else clusters.push_back(aux_cluster);
    }
    return;
}


