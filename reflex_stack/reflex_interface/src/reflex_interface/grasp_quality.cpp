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

GraspQuality::GraspQuality(float mu_epsilon, float mu_delta, int num_edges, ContactModel contact_model)
{
    this->mu_epsilon = mu_epsilon;
    this->mu_delta = mu_delta;
    this->num_edges = num_edges;
    this->contact_model = contact_model;
    beta = atan(mu_epsilon);

    // define task wrenches (how much total task wrench has to be applied from fingers to object)
    float obj_mass = 0.5;
    float obj_weight = obj_mass * 9.81; // grasp must resist only gravity for now (quasi-static assumption)
    tp.add_task_wrench(tf2::Vector3(0, 0, obj_weight), tf2::Vector3(0, 0, 0));
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

Eigen::MatrixXd GraspQuality::getCrossProductMatrix(const tf2::Vector3 &r)
{
    Eigen::MatrixXd m(3, 3);
    m << 0, -r[2], r[1],
        r[2], 0, -r[0],
        -r[1], r[0], 0;
    return m;
}

Eigen::MatrixXd GraspQuality::getPartialGraspMatrix(const tf2::Transform &contact_frame, const tf2::Vector3 &object_position)
{
    // dist from world frame to i_th contact point
    tf2::Vector3 dist_to_contact_i = contact_frame.getOrigin() - object_position;

    // compute cross product matrix
    Eigen::MatrixXd S_i = getCrossProductMatrix(dist_to_contact_i);

    // assemble P_i
    Eigen::MatrixXd P_i(6, 6);
    P_i.topLeftCorner(3, 3) = Eigen::MatrixXd::Identity(3, 3);
    P_i.topRightCorner(3, 3) = Eigen::MatrixXd::Zero(3, 3);
    P_i.bottomLeftCorner(3, 3) = S_i;
    P_i.bottomRightCorner(3, 3) = Eigen::MatrixXd::Identity(3, 3);

    // compute R_i (which is the basis of the contact frame)
    tf2::Vector3 n_i = contact_frame * tf2::Vector3(1, 0, 0);
    tf2::Vector3 t_i = contact_frame * tf2::Vector3(0, 1, 0);
    tf2::Vector3 o_i = contact_frame * tf2::Vector3(0, 0, 1);
    Eigen::MatrixXd R_i(3, 3);
    R_i << n_i[0], t_i[0], o_i[0],
        n_i[1], t_i[1], o_i[1],
        n_i[2], t_i[2], o_i[2];

    // assemble R_i_bar
    Eigen::MatrixXd R_i_bar(6, 6);
    R_i_bar.topLeftCorner(3, 3) = R_i;
    R_i_bar.topRightCorner(3, 3) = Eigen::MatrixXd::Zero(3, 3);
    R_i_bar.bottomLeftCorner(3, 3) = Eigen::MatrixXd::Zero(3, 3);
    R_i_bar.bottomRightCorner(3, 3) = R_i;

    // compute partial grasp matrix G_i
    Eigen::MatrixXd G_i(6, 6);
    G_i = P_i * R_i_bar;
    return G_i;
}

Eigen::MatrixXd GraspQuality::getGraspMatrix(const std::vector<tf2::Transform> &contact_frames,
                                             const tf2::Vector3 &object_position,
                                             const int &num_contacts)
{
    // NOTE: for all calculations regarding the grasp matrix, we follow the notation in "Springer Handbook
    // of Robotics, Chapter 38 - Grasping, Second Edition, by Prattichizzo and Trinkle"

    // define grasp matrix G and partial grasp matrix G_i
    Eigen::MatrixXd G(6, num_contacts * 6);
    Eigen::MatrixXd G_i(6, 6);

    for (int i = 0; i < num_contacts; i++)
    {
        G_i = getPartialGraspMatrix(contact_frames[i], object_position);
        // insert G_i into G at correct column index
        G.block<6, 6>(0, i * 6) = G_i;
    }
    return G;
}

float GraspQuality::getSlipMarginWithTaskWrenches(std::vector<tf2::Vector3> &contact_forces,
                                                  std::vector<tf2::Vector3> &contact_normals,
                                                  const std::vector<tf2::Transform> &contact_frames,
                                                  const tf2::Vector3 &object_position,
                                                  const int &num_contacts)
{
    if (!num_contacts)
    {
        return 0;
    }

    if (!(isSameSize(num_contacts, contact_forces) && isSameSize(num_contacts, contact_normals) && isSameSize(num_contacts, contact_frames)))
    {
        ROS_ERROR("Number of contact forces/normals/frames does not match number of contacts. Returning 0 for slip margin.");
        return 0;
    }

    // compute grasp matrix (nxm)
    Eigen::MatrixXd G(6, num_contacts * 6);
    G = getGraspMatrix(contact_frames, object_position, num_contacts);

    // TODO
    // assemble selection matrix
    // calculate effective grasp matrix

    // compute grasp matrix pseudoinverse (mxn)
    Eigen::MatrixXd G_pinv = G.completeOrthogonalDecomposition().pseudoInverse();

    float lowest_slip_margin = 0;
    std::vector<float> contact_force_magnitudes;

    // for each task wrench compute the slip margin
    for (int i = 0; i < tp.num_task_wrenches; i++)
    {
        contact_force_magnitudes.clear();

        Eigen::MatrixXd task_wrench_i(6, 1);
        task_wrench_i << tp.task_forces[i][0], tp.task_forces[i][1], tp.task_forces[i][2],
            tp.task_torques[i][0], tp.task_torques[i][1], tp.task_torques[i][2];

        // map task wrench onto contact frames and obtain wrench intensity vector lambda expressed in contact frames
        Eigen::MatrixXd lambda(6 * num_contacts, 1);
        lambda = G_pinv * task_wrench_i;

        // iterate over each contact
        for (int j = 0; j < num_contacts; j++)
        {
            // lambda is [fx0,fy0,fz0,tx0,ty0,tz0,fx1,fy1,fz1,tx2 ... ]. lambda_j_start_idx points to fxj
            int lambda_j_start_idx = i * 6;

            // hard contact model: only pull out contact forces from lambda
            tf2::Vector3 task_force_j = tf2::Vector3(lambda(lambda_j_start_idx, 0),
                                                     lambda(lambda_j_start_idx + 1, 0),
                                                     lambda(lambda_j_start_idx + 2, 0));

            // transform contact task force from contact to world frame
            task_force_j = contact_frames[j].inverse() * task_force_j;

            // add contact task forces to currently measured contact forces
            contact_forces[j] += task_force_j;
            contact_force_magnitudes.push_back(contact_forces[j].length());
        }

        // compute slip margin with this task force
        float slip_margin = getSlipMargin(contact_normals, contact_forces, contact_force_magnitudes, num_contacts);

        // save first or new lowest slip_margin
        if (i == 0 || slip_margin < lowest_slip_margin)
        {
            lowest_slip_margin = slip_margin;
        }
    }
    return lowest_slip_margin;
}

float GraspQuality::getSlipMargin(std::vector<tf2::Vector3> &contact_normals,
                                  const std::vector<tf2::Vector3> &contact_forces,
                                  const std::vector<float> &contact_force_magnitudes,
                                  const int &num_contacts,
                                  bool verbose)
{

    if (!(isSameSize(num_contacts, contact_forces) && isSameSize(num_contacts, contact_normals) && isSameSize(num_contacts, contact_force_magnitudes)))
    {
        ROS_ERROR("Number of contact forces/normals/force_magnitudes does not match number of contacts. Returning 0 for slip margin.");
        return 0;
    }

    float min_delta = 0;

    for (int i = 0; i < num_contacts; i++)
    {
        // calc measured normal force by scalar projection
        float f_norm_m = abs(contact_forces[i].dot(contact_normals[i].normalize()));

        // calc measured tangential force
        float f_tang_m = sqrt(pow(contact_force_magnitudes[i], 2) - pow(f_norm_m, 2));

        // calc allowed tangential force using Coulomb
        float f_tang_allow = mu_delta * f_norm_m;

        // calc margin to slip boundary
        float f_tang_margin = f_tang_allow - f_tang_m;

        // save first or new lowest f_tang_margin
        if (i == 0 || f_tang_margin < min_delta)
        {
            min_delta = f_tang_margin;
        }

        if (verbose)
        {
            ROS_INFO_STREAM(">> CONTACT[" << i << "] of [" << num_contacts << "]");
            ROS_INFO_STREAM("DEBUG: contact_forces[" << i << "]: \t" << vec2string(contact_forces[i]));
            ROS_INFO_STREAM("DEBUG: contact_normals[" << i << "]: \t" << vec2string(contact_normals[i]));
            ROS_INFO_STREAM("DEBUG: contact_force_magnitudes[" << i << "]: \t" << contact_force_magnitudes[i]);
            ROS_INFO_STREAM("f_norm_m is " << f_norm_m);
            ROS_INFO_STREAM("f_tang_m is " << f_tang_m);
            ROS_INFO_STREAM("f_tang_allow is " << f_tang_allow);
            ROS_INFO_STREAM("f_tang_margin is " << f_tang_margin);
        }
    }
    if (verbose)
    {
        ROS_INFO_STREAM("==> final min_delta is " << min_delta);
    }
    return min_delta;
}