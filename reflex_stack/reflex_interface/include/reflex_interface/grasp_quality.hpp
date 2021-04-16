#ifndef GRASP_QUALITY_H
#define GRASP_QUALITY_H

#include <vector>

#include <Eigen/Dense>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Transform.h>

#include "reflex_interface/task_polytope.hpp"

class GraspWrenchSpace
{
public:
    std::vector<tf2::Vector3> force_primitives;
    std::vector<tf2::Vector3> torque_primitives;
};

class GraspQuality
{
public:
    enum ContactModel
    {
        PointContactWithoutFriction,
        HardFinger,
        SoftFinger
    };

    GraspQuality(float mu_epsilon = 1, float mu_delta = 1, int num_edges = 4, ContactModel contact_model = HardFinger);

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

    template <class T>
    bool isSameSize(const int desired_size, const std::vector<T> &vector) { return vector.size() == desired_size; };
    void updateGraspWrenchSpace(bool verbose);
    bool isValidNumContacts();
    Eigen::MatrixXd getCrossProductMatrix(const tf2::Vector3 &r);
    Eigen::MatrixXd getPartialGraspMatrix(const tf2::Transform &contact_frame, const tf2::Vector3 &object_position);
    Eigen::MatrixXd getGraspMatrix(const std::vector<tf2::Transform> &contact_frames,
                                   const tf2::Vector3 &object_position,
                                   const int &num_contacts);
    float getSlipMarginWithTaskWrenches(std::vector<tf2::Vector3> &contact_forces,
                                        std::vector<tf2::Vector3> &contact_normals,
                                        const std::vector<tf2::Transform> &contact_frames,
                                        const tf2::Vector3 &object_position,
                                        const int &num_contacts);

private:
    float mu_epsilon;
    float mu_delta;
    int num_edges;
    int num_contacts;
    double beta;
    GraspWrenchSpace gws = GraspWrenchSpace();
    TaskPolytope tp = TaskPolytope();
    std::vector<tf2::Vector3> contact_positions;
    std::vector<tf2::Vector3> contact_normals;
    tf2::Vector3 object_com_world;
    ContactModel contact_model;
};

#endif