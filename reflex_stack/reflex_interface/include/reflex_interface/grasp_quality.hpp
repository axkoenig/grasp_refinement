#ifndef GRASP_QUALITY_H
#define GRASP_QUALITY_H

#include <vector>

#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>

class GraspQuality
{
private:
    float mu;
    int num_primitive_wrenches;

public:
    GraspQuality(float mu = 0.2, int num_primitive_wrenches = 4);
    
    float getEpsilon(std::vector<tf2::Transform> contact_frames_world,
                     tf2::Vector3 object_com_world);
};

#endif