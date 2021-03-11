#include <math.h>

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>

extern "C"
{
#include <libqhull_r.h>
}

#include "reflex_interface/grasp_quality.hpp"

std::string vec2string(tf2::Vector3 vec)
{
    return std::to_string(vec[0]) + ", " + std::to_string(vec[1]) + ", " + std::to_string(vec[2]);
}

GraspQuality::GraspQuality(float mu, int num_edges)
{
    this->mu = mu;
    this->num_edges = num_edges;
    beta = atan(mu);
}

bool GraspQuality::isValidNumContacts()
{
    num_contacts = contact_positions.size();
    if (num_contacts != contact_normals.size())
    {
        ROS_ERROR("Number of contact positions and normals must be equal.");
        return false;
    }

    // require at least 2 contacts for epsilon
    if (num_contacts < 2)
    {
        return false;
    }
    return true;
}

void GraspQuality::updateGraspWrenchSpace(bool verbose)
{
    // clearing old gws results
    gws.force_primitives.clear();
    gws.torque_primitives.clear();

    // beta is opening angle of force cone (defines boundary where slip occurs)
    tf2::Quaternion q_beta;
    q_beta.setRPY(beta, 0, 0);

    // phi is angle around contact normal between each force primitive
    double phi = 2 * M_PI / num_edges;
    tf2::Quaternion q_phi;

    if (verbose)
    {
        ROS_INFO_STREAM("DEBUG: beta: " << beta);
        ROS_INFO_STREAM("DEBUG: num_contacts: " << num_contacts);
        ROS_INFO_STREAM("DEBUG: phi: " << phi);
        ROS_INFO_STREAM("DEBUG: object COM " << vec2string(object_com_world));
    }

    for (int i = 0; i < num_contacts; i++)
    {
        // compute lever arm from object COM to contact position
        tf2::Vector3 r = contact_positions[i] - object_com_world;

        if (verbose)
        {
            ROS_INFO_STREAM("DEBUG: contact_positions[i] " << i << " is " << vec2string(contact_positions[i]));
            ROS_INFO_STREAM("DEBUG: lever arm for contact " << i << " is " << vec2string(r));
        }

        for (int j = 0; j < num_edges; j++)
        {
            q_phi.setRPY(0, 0, j * phi);
            tf2::Quaternion q_tot = q_phi * q_beta;
            q_tot.normalize();

            // turn contact normal by multiples of phi, then tilt around beta and get z axis
            tf2::Vector3 force_primitive = tf2::quatRotate(q_tot, contact_normals[i]);

            gws.force_primitives.push_back(force_primitive);
            gws.torque_primitives.push_back(r.cross(force_primitive));

            if (verbose)
            {
                tf2::Vector3 vec = contact_normals[i];
                ROS_INFO_STREAM("DEBUG: contact normal for contact " << i << "is \t" << vec2string(contact_normals[i]));
                ROS_INFO_STREAM("DEBUG: contact normal for contact LEN " << i << "is \t" << sqrt(pow(vec[0], 2) + pow(vec[1], 2) + pow(vec[2], 2)));
                ROS_INFO_STREAM("DEBUG: force prim for contact " << i << " and edge " << j << " is  \t" << vec2string(force_primitive));
                ROS_INFO_STREAM("DEBUG: force prim for contact LEN " << i << "is \t" << sqrt(pow(force_primitive[0], 2) + pow(force_primitive[1], 2) + pow(force_primitive[2], 2)));
                ROS_INFO_STREAM("DEBUG: torque prim for contact " << i << " and edge " << j << " is  \t" << vec2string(r.cross(force_primitive)));
            }
        }
    }
}

float GraspQuality::getEpsilon(const std::vector<tf2::Vector3> &contact_positions,
                               const std::vector<tf2::Vector3> &contact_normals,
                               const tf2::Vector3 &object_com_world,
                               bool verbose)
{
    this->contact_positions = contact_positions;
    this->contact_normals = contact_normals;
    this->object_com_world = object_com_world;

    if (!isValidNumContacts())
    {
        return -1;
    }

    updateGraspWrenchSpace(verbose);

    // wrench space is 6 dimensional (3 force, 3 torque)
    int dim = 6;
    int num_ft_primitives = num_contacts * num_edges;
    int num_points = dim * num_ft_primitives;

    // allocate memory for points
    coordT *points = (coordT *)calloc(num_points, sizeof(coordT));

    if (verbose)
    {
        ROS_INFO_STREAM("DEBUG: num_points: " << num_points);
        ROS_INFO_STREAM("DEBUG: num_ft_primitives: " << num_ft_primitives);
        ROS_INFO_STREAM("Size of force primitive is: " << gws.force_primitives.size() << " I expect it to be " << num_ft_primitives);
        ROS_INFO_STREAM("Size of torque primitive is: " << gws.torque_primitives.size() << " I expect it to be " << num_ft_primitives);
    }

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
            points[dim * i + j] = gws.force_primitives[i][j];
            points[dim * i + j + 3] = gws.torque_primitives[i][j];

            if (verbose)
            {
                ROS_INFO_STREAM("dim * i + j = " << dim * i + j << " is assigned " << gws.force_primitives[i][j]);
                ROS_INFO_STREAM("dim * i + j + 3 = " << dim * i + j + 3 << " is assigned " << gws.torque_primitives[i][j]);
            }
        }
    }
    if (verbose)
    {
        for (int i = 0; i < num_points; i++)
        {
            ROS_INFO_STREAM("points entry " << i << ": " << points[i]);
        }
    }

    // qhull options
    // Tv = verify result: structure, convexity, and point inclusion
    // Qt = triangulated output
    // s = print summary to stderr
    // n = print hyperplane normals with offsets
    char flags[] = "qhull Qt";
    qhT qh_qh;
    qhT *qh = &qh_qh;
    qh_zero(qh, NULL);
    int exitcode = qh_new_qhull(qh, dim, num_ft_primitives, points, true, flags, NULL, NULL);

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

    if (verbose)
    {
        ROS_INFO_STREAM("bestoutside: " << bestoutside);
        ROS_INFO_STREAM("isoutside: " << isoutside);
        ROS_INFO_STREAM("==>bestdist: " << bestdist);
    }

    // free memory
    qh_freeqhull(qh, !qh_ALL);
    int curlong, totlong;
    qh_memfreeshort(qh, &curlong, &totlong);

    return abs(bestdist);
}
