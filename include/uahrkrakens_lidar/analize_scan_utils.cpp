#include "analize_scan_utils.hpp"
#include <iostream>
extern "C"{
    #include <math.h>
    #include <cassert>
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


inline float calculate_square_dist(const Point2d &p1, const Point2d &p2)
{
    return ((p2.x - p1.x) * (p2.x - p1.x)) + ((p2.y - p1.y) * (p2.y - p1.y));
}

inline float calculate_dist(const Point2d &p1, const Point2d &p2)
{
    return sqrt(((p2.x - p1.x) * (p2.x - p1.x)) + ((p2.y - p1.y) * (p2.y - p1.y)));
}

inline bool same_cluster(Point2d p1, Point2d p2, float max_dist)
{
    return calculate_square_dist(p1, p2) <= (max_dist * max_dist);
}


inline bool object_between_end_and_start(const std::vector<cluster> &clusters, const Point2d &end_polar, const float &dist_increment)
{
    // This special case can only occur when the output clusters size
    // is not empty  
    return (clusters.size() && same_cluster(clusters[0][0],end_polar,dist_increment));
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

float get_cluster_length(const std::vector<Point2d> &cluster) {
    float length = 0.0;
    auto  old_p   = cluster.begin();

    for (auto p = cluster.begin() +1; p < cluster.end(); p++ )
    {
        length = length + calculate_dist(*old_p,*p);
        old_p = p;
    }
    return length;
}

void filter_clusters_by_length(const std::vector<std::vector<Point2d>> &clusters, const float &min_length, const float &max_length, std::vector<std::vector<Point2d>> &out_clusters) {
    
    float cluster_square_length = 0;

    out_clusters.clear();

    for(auto &c : clusters)
    {
        cluster_square_length = get_cluster_length(c);
        if ((min_length <= cluster_square_length) && (cluster_square_length <= max_length))
        {
            out_clusters.push_back(c);
        }
    }
    //std::copy_if(begin(clusters),end(clusters),std::back_inserter(out_clusters),)
    return;
}

Point2d get_cluster_contour_centroid (const cluster &cluster_) {
    Point2d centroid;
    
    // There is a filter before calling
    // this function that prevents 
    // the input of an empty cluster.
    
    // To prevent the execution of an
    // IF inside this loop in the optimized
    // version I am going to put here 
    // an assert that will check the no
    // entry of an empty vector in the
    // production code.
    assert(cluster_.size() != 0);

    for (auto &p : cluster_)
    {
        centroid.x = centroid.x + p.x;
        centroid.y = centroid.y + p.y;
    }
    centroid.x = centroid.x / cluster_.size();
    centroid.y = centroid.y / cluster_.size();
    return centroid;
}

void get_clusters_centroid(const std::vector<cluster> &clusters, std::vector<Point2d> &centroids)
{
    centroids.clear();

    for(auto &c : clusters)
    {
        centroids.emplace_back(get_cluster_contour_centroid(c));
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



bool same_centroid(Point2d c1,Point2d c2){
    int dist = 0.3;
    return same_cluster(c1, c2, 0.3);
}

bool is_noise(Point2d c1,Point2d c2){
    int dist = 0.3;
    return same_cluster(c1, c2, 0.1);
}

int get_nearest_centroid(Point2d center, std::vector<Point2d> centroids){
    return 0;
}

void track_obstacles(std::vector<Point2d> &detected_obstacles, std::vector<Obstacle> &tracked_obstacles)
{
    std::vector<int>  similar_centroids;
    std::vector<Point2d> aux_vector;
    int index;
    // Iterate over the tracked obstacles
    int itrac = 0; 

    if(tracked_obstacles.size() == 0)
    {
        for(auto &obs : detected_obstacles)
        {
            tracked_obstacles.emplace_back(obs);
        }
        return;
    }
    while(itrac < tracked_obstacles.size()){
        similar_centroids.clear();
        // Compare the new centroid with 
        // all the tracked obstacles
        for(int idet = 0; idet < detected_obstacles.size(); idet++){
            if (same_centroid(tracked_obstacles[itrac].centroid,detected_obstacles[idet]))   similar_centroids.push_back(idet); 
        }

        switch (similar_centroids.size()){
            case 0:
                if(tracked_obstacles[itrac].track_count > 0){
                    tracked_obstacles[itrac].track_count--;
                    ++itrac;
                }
                else{
                    tracked_obstacles.erase(tracked_obstacles.begin() + itrac);
                }

                break;
            case 1:
                tracked_obstacles[itrac].track_count++;
                // @todo detec if is static noise if(is_noise)...
                //tracked_obstacles[itrac].centroid = detected_obstacles[similar_centroids[0]];
                detected_obstacles.erase(detected_obstacles.begin()+ similar_centroids[0]);
                ++itrac;
                break;
            default:
                tracked_obstacles[itrac].track_count++;
                aux_vector.clear();
                for(int i = 0; i < similar_centroids.size(); i++){
                    aux_vector.emplace_back(detected_obstacles[similar_centroids[i]]);
                }
                index = get_nearest_centroid(tracked_obstacles[itrac].centroid, aux_vector);
                tracked_obstacles[itrac].centroid = detected_obstacles[similar_centroids[index]];
                std::cout << "El centroide similar es " << similar_centroids[index];
                
                detected_obstacles.erase(detected_obstacles.begin()+ similar_centroids[index]);
                ++itrac;
                break;
        }   
    }
    return;
}
