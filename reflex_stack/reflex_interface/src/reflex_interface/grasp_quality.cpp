#include <math.h>

extern "C"
{
#include <libqhull_r.h>
}

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>

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
        ROS_INFO_STREAM("DEBUG: Beta: " << beta);
        ROS_INFO_STREAM("DEBUG: Phi: " << phi);
        ROS_INFO_STREAM("DEBUG: Num contacts: " << num_contacts);
        ROS_INFO_STREAM("DEBUG: Object COM " << vec2string(object_com_world));
    }

    for (int i = 0; i < num_contacts; i++)
    {
        // compute lever arm from object COM to contact position
        tf2::Vector3 r = contact_positions[i] - object_com_world;

        // compute rotation between world z and contact normal
        tf2::Vector3 world_z = tf2::Vector3(0, 0, 1);
        tf2::Quaternion rot_to_world_z = tf2::shortestArcQuatNormalize2(contact_normals[i], world_z);

        if (verbose)
        {
            ROS_INFO_STREAM("DEBUG: Contact[" << i << "] position: \t" << vec2string(contact_positions[i]));
            ROS_INFO_STREAM("DEBUG: Contact[" << i << "] normal: \t" << vec2string(contact_normals[i]));
            ROS_INFO_STREAM("DEBUG: Contact[" << i << "] normal length: \t" << contact_normals[i].length());
            ROS_INFO_STREAM("DEBUG: Contact[" << i << "] lever arm: \t" << vec2string(r));
        }

        for (int j = 0; j < num_edges; j++)
        {
            q_phi.setRPY(0, 0, j * phi);
            tf2::Quaternion q_tot = rot_to_world_z * q_phi * q_beta * rot_to_world_z.inverse();
            q_tot.normalize();

            // turn contact normal by multiples of phi, then tilt around beta
            tf2::Vector3 force_primitive = tf2::quatRotate(q_tot, contact_normals[i]);
            tf2::Vector3 torque_primitive = r.cross(force_primitive);

            gws.force_primitives.push_back(force_primitive);
            gws.torque_primitives.push_back(torque_primitive);

            if (verbose)
            {
                tf2::Vector3 vec = contact_normals[i];
                ROS_INFO_STREAM("DEBUG: Contact[" << i << "], Edge[" << j << "] force primitive: \t" << vec2string(force_primitive));
                ROS_INFO_STREAM("DEBUG: Contact[" << i << "], Edge[" << j << "] torque primitive: \t" << vec2string(torque_primitive));
                ROS_INFO_STREAM("DEBUG: Contact[" << i << "], Edge[" << j << "] force primitive length: \t" << force_primitive.length());
                ROS_INFO_STREAM("DEBUG: Contact[" << i << "], Edge[" << j << "] torque primitive length: \t" << torque_primitive.length());
            }
        }
    }
}

// TODO this should be a member method.
float calcRadiusLargestBall(int dim, int num_ft_primitives, coordT *points, int num_points, bool verbose)
{
    for (int i = 0; i < num_points; i++)
    {
        if (isnan(points[i]))
        {
            ROS_WARN("At least one of your qhull points is nan. Returning 0 for epsilon.");
            return 0;
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
        ROS_WARN("Convex hull creation failed. Returning 0 for epsilon.");
        qh_freeqhull(qh, !qh_ALL);
        int curlong, totlong;
        qh_memfreeshort(qh, &curlong, &totlong);
        return 0;
    }
    // find facet that is closest to origin
    coordT origin[dim] = {0, 0, 0};
    boolT bestoutside, isoutside;
    realT bestdist;

    // this qhull routine returns distance from point (here: origin) to nearest facet
    qh_findbestfacet(qh, origin, bestoutside, &bestdist, &isoutside);

    if (verbose)
    {
        ROS_INFO_STREAM("DEBUG: qhull 'bestoutside': \t" << bestoutside);
        ROS_INFO_STREAM("DEBUG: qhull 'isoutside': \t" << isoutside);
        ROS_INFO_STREAM("DEBUG: qhull 'bestdist': \t" << bestdist);
    }

    // free memory
    qh_freeqhull(qh, !qh_ALL);
    int curlong, totlong;
    qh_memfreeshort(qh, &curlong, &totlong);

    // origin must be within convex hull (force closure!), else return 0
    return isoutside ? 0 : abs(bestdist);
}

void GraspQuality::fillEpsilonFTSeparate(const std::vector<tf2::Vector3> &contact_positions,
                                         const std::vector<tf2::Vector3> &contact_normals,
                                         const tf2::Vector3 &object_com_world,
                                         float &epsilon_force,
                                         float &epsilon_torque,
                                         bool verbose)
{
    this->contact_positions = contact_positions;
    this->contact_normals = contact_normals;
    this->object_com_world = object_com_world;

    if (!isValidNumContacts())
    {
        return;
    }

    updateGraspWrenchSpace(verbose);
    // wrench space is 3 dimensional (3 force, 3 torque separately)
    int dim = 3;
    int num_ft_primitives = num_contacts * num_edges;
    int num_points = dim * num_ft_primitives;

    // allocate memory for points
    coordT *points_force = (coordT *)calloc(num_points, sizeof(coordT));
    coordT *points_torque = (coordT *)calloc(num_points, sizeof(coordT));

    // points_force is a concatenation of all 3D primitive forces for each contact
    // format: { f_x_11, f_y_11, f_z_11,  ...
    //           f_x_ne, f_y_ne, f_z_ne, }
    // where    n is num_contacts
    // and      e is num_edges
    // similar for torque

    // iterate over primitives
    for (int i = 0; i < num_ft_primitives; i++)
    {
        // iterate over x,y,z coordinates in Vector3
        for (int j = 0; j < 3; j++)
        {
            points_force[dim * i + j] = gws.force_primitives[i][j];
            points_torque[dim * i + j] = gws.torque_primitives[i][j];
        }
    }
    epsilon_force = calcRadiusLargestBall(dim, num_ft_primitives, points_force, num_points, verbose);
    epsilon_torque = calcRadiusLargestBall(dim, num_ft_primitives, points_torque, num_points, verbose);
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
        return 0;
    }

    updateGraspWrenchSpace(verbose);

    // wrench space is 6 dimensional (3 force + 3 torque)
    int dim = 6;
    int num_ft_primitives = num_contacts * num_edges;
    int num_points = dim * num_ft_primitives;

    // allocate memory for points
    coordT *points = (coordT *)calloc(num_points, sizeof(coordT));

    if (verbose)
    {
        ROS_INFO_STREAM("DEBUG: num_points: " << num_points);
        ROS_INFO_STREAM("DEBUG: num_ft_primitives: " << num_ft_primitives);
        ROS_INFO_STREAM("DEBUG: Size of force primitive is: " << gws.force_primitives.size() << " I expect it to be " << num_ft_primitives);
        ROS_INFO_STREAM("DEBUG: Size of torque primitive is: " << gws.torque_primitives.size() << " I expect it to be " << num_ft_primitives);
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
    return calcRadiusLargestBall(dim, num_ft_primitives, points, num_points, verbose);
}
