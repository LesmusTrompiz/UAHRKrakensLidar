
#include "point2d.hpp"

#ifndef ANALIZE_SCAN_UTILS_HPP
#define ANALIZE_SCAN_UTILS_HPP
    void LaserRangeTo2dPoints(const std::vector<float> &ranges,const float &angle_increment, std::vector<Point2d> &out_points);
    void get_clusters(const std::vector<Point2d> &scan, std::vector<std::vector<Point2d>> &clusters, const float &dist_increment);

#endif

