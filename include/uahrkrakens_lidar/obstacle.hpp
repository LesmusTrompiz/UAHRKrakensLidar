#ifndef obstacle_CONST

#define obstacle_CONST
#include <vector>
#include <cinttypes>
#include "point2d.hpp"

struct Obstacle
{
    Point2d centroid;
    uint8_t track_count;
    Obstacle()                                  : centroid{}         , track_count{0}      {};
    Obstacle(float x, float y  , uint8_t count) : centroid{x,y}      , track_count{count}  {};
    Obstacle(float x, float y                 ) : centroid{x,y}      , track_count{0}      {};
    Obstacle(Point2d _centroid , uint8_t count) : centroid{_centroid}, track_count{count}  {};
    Obstacle(Point2d &_centroid, uint8_t count) : centroid{_centroid}, track_count{count}  {};

    friend inline bool operator==(const Obstacle& lhs, const Obstacle& rhs) { 
        return (lhs.centroid == rhs.centroid && lhs.track_count == rhs.track_count); 
    }

    friend inline bool operator!=(const Obstacle& lhs, const Obstacle& rhs) {
        return !(lhs == rhs);
    }

};





#endif