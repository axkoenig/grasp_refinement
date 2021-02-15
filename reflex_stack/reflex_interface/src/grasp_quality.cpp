#include <math.h>

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>

extern "C"
{
#include <libqhull_r.h>
}

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

    // no contacts
    if (num_contacts == 0)
    {
        return -1.0;
    }

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
    int num_ft_primitives = num_contacts * num_edges;
    int num_points = dim * num_ft_primitives;

    // allocate memory for points
    coordT *points = (coordT *)calloc(num_points, sizeof(coordT));
    ;

    // debug
    ROS_INFO_STREAM("Size of force primitive is: " << force_primitives.size() << " I expect it to be " << num_ft_primitives);
    ROS_INFO_STREAM("Size of torque primitive is: " << torque_primitives.size() << " I expect it to be " << num_ft_primitives);

    // points is a concatenation of all 6D primitive wrenches for each contact
    // format: { f_x_11, f_y_11, f_z_11, t_x_11, t_y_11, t_z_11,  ...
    //           f_x_ne, f_y_ne, f_z_ne, t_x_ne, t_y_ne, t_z_ne }
    // where    n is num_contacts
    // and      e is num_edges

    // iterate over primitives
    for (int i = 0; i < num_ft_primitives; i++)
    {
        // iterate over x,y,z coordinates in Vector3
        for (int j = 0; j < 3; j++)
        {
            points[dim * i + j] = force_primitives[i][j];
            points[dim * i + j + 3] = torque_primitives[i][j];
        }
    }

    // Tv = verify result: structure, convexity, and point inclusion
    // Qt = triangulated output
    // s = print summary to stderr
    // n = print hyperplane normals with offsets
    char flags[] = "qhull Tv Qt s";
    qhT qh_qh;
    qhT *qh = &qh_qh;
    qh_zero(qh, NULL);
    int exitcode = qh_new_qhull(qh, dim, num_points, points, true, flags, NULL, NULL);

    if (exitcode != 0)
    {
        ROS_WARN("Convex hull creation failed. Returning -1.0.");
        qh_freeqhull(qh, !qh_ALL);
        int curlong, totlong;
        qh_memfreeshort(qh, &curlong, &totlong);
        return -1.0;
    }

    // find facet that is closest to origin
    coordT origin[dim] = {0, 0, 0, 0, 0, 0};
    boolT bestoutside, isoutside;
    realT bestdist;
    qh_findbestfacet(qh, origin, bestoutside, &bestdist, &isoutside);

    // free memory
    qh_freeqhull(qh, !qh_ALL);
    int curlong, totlong;
    qh_memfreeshort(qh, &curlong, &totlong);

    return bestdist;
}
