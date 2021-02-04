#include "reflex_interface/grasp_quality.hpp"

GraspQuality::GraspQuality(float mu, int num_primitive_wrenches)
{
    this->mu = mu;
    this->num_primitive_wrenches = num_primitive_wrenches;
}

float GraspQuality::getEpsilon(std::vector<tf2::Transform> contact_frames_world,
                               tf2::Vector3 object_com_world)
{
    // we assume that coosy z axis is normal

    // actual grasp quality calculation
    // calculate wrench primitives from normal, beta and mu and num primitive wrenches
    // calculate convex hull of all wrench primitives
    // calc ball and return radius (qhull equations)

    return 0.0;
}
