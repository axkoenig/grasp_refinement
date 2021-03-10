#ifndef GRASP_QUALITY_H
#define GRASP_QUALITY_H

#include <vector>

#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>

class GraspQuality
{
private:
    float mu;
    int num_edges;
    double beta;

public:
    GraspQuality(float mu = 0.9, int num_edges = 4);

    // all args have to be in same coordinate system!
    float getEpsilon(const std::vector<tf2::Vector3> &contact_positions,
                     const std::vector<tf2::Vector3> &contact_normals,
                     const tf2::Vector3 &object_com_world,
                     bool verbose = false);
};

#endif