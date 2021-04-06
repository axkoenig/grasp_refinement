#ifndef GRASP_QUALITY_H
#define GRASP_QUALITY_H

#include <vector>

#include <tf2/LinearMath/Vector3.h>

class GraspWrenchSpace
{
public:
    std::vector<tf2::Vector3> force_primitives;
    std::vector<tf2::Vector3> torque_primitives;
};

class GraspQuality
{
private:
    float mu;
    int num_edges;
    int num_contacts;
    double beta;
    GraspWrenchSpace gws = GraspWrenchSpace();
    std::vector<tf2::Vector3> contact_positions;
    std::vector<tf2::Vector3> contact_normals;
    tf2::Vector3 object_com_world;

public:
    GraspQuality(float mu = 1, int num_edges = 4);

    // all args have to be in same coordinate system!
    float getEpsilon(const std::vector<tf2::Vector3> &contact_positions,
                     const std::vector<tf2::Vector3> &contact_normals,
                     const tf2::Vector3 &object_com_world,
                     bool verbose = false);

    void fillEpsilonFTSeparate(const std::vector<tf2::Vector3> &contact_positions,
                               const std::vector<tf2::Vector3> &contact_normals,
                               const tf2::Vector3 &object_com_world,
                               float &epsilon_force,
                               float &epsilon_torque,
                               bool verbose = false);

    float getSlipMargin(std::vector<tf2::Vector3> &contact_normals,
                        const std::vector<tf2::Vector3> &contact_forces,
                        const std::vector<float> &contact_force_magnitudes,
                        const int &num_contacts,
                        bool verbose = false);

    void updateGraspWrenchSpace(bool verbose);
    bool isValidNumContacts();
};

#endif