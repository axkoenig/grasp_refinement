#include <math.h>

#include <tf2/LinearMath/Quaternion.h>
// #include <libqhull/libqhull.h>

#include "reflex_interface/grasp_quality.hpp"

GraspQuality::GraspQuality(float mu, int num_edges)
{
    this->mu = mu;
    this->num_edges = num_edges;
}

float GraspQuality::getEpsilon(std::vector<tf2::Transform> contact_frames_world,
                               tf2::Vector3 object_com_world)
{
    int num_contacts = contact_frames_world.size();

    // wrench primitives of all contacts
    std::vector<tf2::Vector3> force_primitives = {};
    std::vector<tf2::Vector3> torque_primitives = {};

    // beta is opening angle of force cone (defines boundary where slip occurs)
    double beta = atan(mu);
    tf2::Quaternion q_beta;
    q_beta.setRPY(beta, 0, 0);
    tf2::Transform rot_beta = tf2::Transform(q_beta, tf2::Vector3{0, 0, 0});

    // phi is angle around contact normal between each force primitives
    double phi = 2 * M_PI / num_edges;
    tf2::Quaternion q_phi;
    for (int i = 0; i < num_contacts; i++)
    {
        // compute lever arm from object COM to contact position
        tf2::Vector3 r = contact_frames_world[i].getOrigin() - object_com_world;

        for (int j = 0; j < num_edges; j++)
        {
            q_phi.setRPY(0, 0, j * phi);
            tf2::Transform rot_phi = tf2::Transform(q_phi, tf2::Vector3{0, 0, 0});

            // turn contact normal by multiples of phi, then tilt around beta and get z axis
            tf2::Vector3 force_primitive = contact_frames_world[i] * rot_phi * rot_beta * tf2::Vector3{0, 0, 1};
            force_primitives.push_back(force_primitive);
            torque_primitives.push_back(r.cross(force_primitive));
        }
    }

    // wrench space is 6 dimensional (3 force, 3 torque)
    int dim = 6; 
    int num_primitive_wrenches = num_contacts * num_edges;
    int num_vertices = dim * num_primitive_wrenches;
    
    // vertices is a concatenation of all 6D vectors for each contact
    // format: { f_x_1, f_y_1, f_z_1, t_x_1, t_y_1, t_z_1,  ... 
    //           f_x_n, f_y_n, f_z_n, t_x_n, t_y_n, t_z_n }
    // where n is num_contacts
    // coordT vertices[num_vertices];

    // // iterate over primitives
    // for (int i = 0; i < num_primitive_wrenches; i++)
    // {   
    //     // iterate over x,y,z coordinates
    //     for (int j = 0; j < 3; j++)
    //     {
    //         vertices[6*i+j] = force_primitives[i][j];
    //         vertices[6*i+j+3] = torque_primitives[i][j];
    //     }
    // }

    // int i = qh_new_qhull(dim, num_contacts, vertices, false, NULL, NULL, NULL);

    // calculate convex hull of all wrench primitives

    // calc ball and return radius (qhull equations)

    return 0.0;
}
