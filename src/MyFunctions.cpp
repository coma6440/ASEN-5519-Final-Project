#include "MyFunctions.h"
#include <fstream>
#include <iostream>
#include <fcl/narrowphase/collision.h>

void DefineProblem(ob::StateSpacePtr& stateSpace, ob::StateSpacePtr& goalSpace)
    {
    // State space setup
    auto se3_state(std::make_shared<ob::SE3StateSpace>());
    auto se3_goal(std::make_shared<ob::SE3StateSpace>());
    auto vel_state(std::make_shared<ob::RealVectorStateSpace>(1));
    auto vel_goal(std::make_shared<ob::RealVectorStateSpace>(1));

    // Make the compound state space
    stateSpace = se3_state + vel_state;
    goalSpace = se3_goal + vel_goal;

    // Create the bounds for the SE3 space
    ob::RealVectorBounds se3_bounds_state(3);
    ob::RealVectorBounds se3_bounds_goal(3);
    ob::RealVectorBounds vel_bounds_state(1);
    ob::RealVectorBounds vel_bounds_goal(1);

    // State space bounds
    se3_bounds_state.setLow(0, 0);
    se3_bounds_state.setHigh(0, 15);
    se3_bounds_state.setLow(1, -10);
    se3_bounds_state.setHigh(1, 10);
    se3_bounds_state.setLow(2, -10);
    se3_bounds_state.setHigh(2, 10);

    // Goal space bounds
    se3_bounds_goal.setLow(0, 14);
    se3_bounds_goal.setHigh(0, 15);
    se3_bounds_goal.setLow(1, -1);
    se3_bounds_goal.setHigh(1, 1);
    se3_bounds_goal.setLow(2, -1);
    se3_bounds_goal.setHigh(2, 1);

    // Velocity space bounds
    vel_bounds_state.setLow(0.0);
    vel_bounds_state.setHigh(0.5);
    vel_bounds_goal.setLow(0.0);
    vel_bounds_goal.setHigh(0.01);

    // Set the bounds for the spaces
    se3_state->setBounds(se3_bounds_state);
    se3_goal->setBounds(se3_bounds_goal);
    vel_state->setBounds(vel_bounds_state);
    vel_goal->setBounds(vel_bounds_goal);
    return;
    }

void GetEnvironment(std::string ws_name, std::vector<std::shared_ptr<fcl::CollisionObjectf>>& obstacles, std::shared_ptr<fcl::CollisionObjectf>& robot)
    {
    std::cout << "Planning with: " << ws_name << std::endl;
    YAML::Node config = YAML::LoadFile("../configs/" + ws_name + ".yaml");
    YAML::Node robot_node = config["robot"];
    YAML::Node obs_node = config["obstacles"];

    // Robot Definition
    std::vector<float> rob_size = robot_node["size"].as<std::vector<float>>();
    std::vector<float> start_pos = robot_node["start_position"].as<std::vector<float>>();

    fcl::Vector3f rob_translation = fcl::Vector3f(start_pos[0], start_pos[1], start_pos[2]);
    robot = CollisionBox(rob_size[0], rob_size[1], rob_size[2], rob_translation);

    // Obstacles definition
    if (obs_node)
        {
        YAML::Node obs;
        std::vector<float> obs_size;
        std::vector<float> obs_pos;
        std::vector<float> obs_orient;
        // Iterate over all obstacles in yaml file
        for (YAML::const_iterator it = obs_node.begin();it != obs_node.end();++it)
            {
            obs = obs_node[it->first.as<std::string>()];
            obs_pos = obs["position"].as<std::vector<float>>();

            obs_orient = obs["orientation"].as<std::vector<float>>();
            obs_size = obs["size"].as<std::vector<float>>();
            fcl::Vector3f obs_translation = fcl::Vector3f(obs_pos[0], obs_pos[1], obs_pos[2]);
            fcl::Quaternionf obs_rotation = fcl::Quaternionf(obs_orient[3], obs_orient[0], obs_orient[1], obs_orient[2]);
            obstacles.push_back(CollisionBox(obs_size[0], obs_size[1], obs_size[2], obs_translation, obs_rotation));
            }
        }
    }

std::shared_ptr<fcl::CollisionObjectf> CollisionBox(float l, float w, float h, fcl::Vector3f translation, fcl::Quaternionf rotation)
    {
    std::shared_ptr<fcl::CollisionGeometryf> geometry(new fcl::Boxf(l, w, h));
    fcl::CollisionObjectf collision_object(geometry);
    collision_object.setTransform(rotation, translation);
    auto obj = std::make_shared<fcl::CollisionObjectf>(collision_object);
    return obj;
    }

bool isStateValid(ob::SpaceInformation* si, const ob::State* state, std::vector<std::shared_ptr<fcl::CollisionObjectf>> obstacles, std::shared_ptr<fcl::CollisionObjectf> robot)
    {
    const auto* se3state = state->as<ob::CompoundStateSpace::StateType>()->as<ob::SE3StateSpace::StateType>(0);
    const auto* pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
    const auto* rot = se3state->as<ob::SO3StateSpace::StateType>(1);
    const auto* vel = state->as<ob::CompoundStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(1);

    fcl::Vector3f translation(pos->values[0], pos->values[1], pos->values[2]);
    fcl::Quaternionf rotation(rot->w, rot->x, rot->y, rot->z);
    robot.get()->setTransform(rotation, translation);
    // Checks that there are no collisions
    for (std::shared_ptr<fcl::CollisionObjectf> obs : obstacles)
        {
        fcl::CollisionRequestf request;
        // result will be returned via the collision result structure
        fcl::CollisionResultf result;
        // perform collision test
        if (fcl::collide(robot.get(), obs.get(), request, result))
            {
            return false;
            }
        }
    // Checks if state is in valid bounds
    return si->satisfiesBounds(state);
    }

bool isStateValid(oc::SpaceInformation* si, const ob::State* state, std::vector<std::shared_ptr<fcl::CollisionObjectf>> obstacles, std::shared_ptr<fcl::CollisionObjectf> robot)
    {
    const auto* se3state = state->as<ob::CompoundStateSpace::StateType>()->as<ob::SE3StateSpace::StateType>(0);
    const auto* pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
    const auto* rot = se3state->as<ob::SO3StateSpace::StateType>(1);
    const auto* vel = state->as<ob::CompoundStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(1);

    fcl::Vector3f translation(pos->values[0], pos->values[1], pos->values[2]);
    fcl::Quaternionf rotation(rot->w, rot->x, rot->y, rot->z);
    robot->setTransform(rotation, translation);
    // Checks that there are no collisions
    for (std::shared_ptr<fcl::CollisionObjectf> obs : obstacles)
        {
        fcl::CollisionRequestf request;
        // result will be returned via the collision result structure
        fcl::CollisionResultf result;
        // perform collision test
        if (fcl::collide(robot.get(), obs.get(), request, result))
            {
            return false;
            }
        }

    // Checks if state is in valid bounds
    return si->satisfiesBounds(state);
    }

void saveGeometricPath(og::SimpleSetup ss, std::string fname)
    {
    static char name[50];
    time_t now = time(0);
    std::string s = "../solutions/geometric/" + fname + "_%Y%m%d%H%M%S.txt";
    strftime(name, sizeof(name), s.c_str(), localtime(&now));
    std::ofstream file(name);
    og::PathGeometric path = ss.getSolutionPath();
    path.interpolate();
    path.printAsMatrix(file);
    file.close();
    }

void saveControlPath(oc::PathControl path, std::string fname)
    {
    static char name[60];
    time_t now = time(0);
    std::string s = "../solutions/kinodynamic/" + fname + "_%Y%m%d%H%M%S.txt";
    strftime(name, sizeof(name), s.c_str(), localtime(&now));
    std::ofstream file(name);
    path.interpolate();
    path.printAsMatrix(file);
    file.close();
    }


oc::PathControl makePath(const ob::SpaceInformationPtr& si, oc::PathControl prevPath, const unsigned int start, const unsigned int end)
    {
    oc::PathControl path(si);
    for (unsigned int i = start; i < end; i++)
        {
        path.append(prevPath.getState(i), prevPath.getControl(i), prevPath.getControlDuration(i));

        }
    return path;
    }

void DynamicsODE(const oc::ODESolver::StateType& x, const oc::Control* control, oc::ODESolver::StateType& xdot)
    {
    const double* u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    const Eigen::Quaternionf orientation(x[6], x[3], x[4], x[5]);
    const Eigen::Quaternionf ang_vel(0, u[1], u[2], u[3]);
    const Eigen::Vector3f body_velocity(x[7], 0, 0);
    const Eigen::Vector3f vel = orientation * body_velocity; // This returns the same value as q*v*q'
    const Eigen::Quaternionf qdot = ang_vel * orientation;
    // Zero out qdot
    xdot.resize(x.size(), 0);
    xdot[0] = vel[0]; // X Velocity
    xdot[1] = vel[1]; // Y Velocity
    xdot[2] = vel[2]; // Z Velocity
    xdot[3] = 0.5 * qdot.x(); // q_dot x
    xdot[4] = 0.5 * qdot.y(); // q_dot y
    xdot[5] = 0.5 * qdot.z(); // q_dot z
    xdot[6] = 0.5 * qdot.w(); // q_dot w
    xdot[7] = u[0]; // Acceleration
    }

void PostIntegration(const ob::State* /*state*/, const oc::Control* /*control*/, const double /*duration*/, ob::State* result)
    {
    // Normalize orientation between 0 and 2*pi
    ob::SO3StateSpace SO3;
    SO3.enforceBounds(result->as<ob::CompoundStateSpace::StateType>()->as<ob::SE3StateSpace::StateType>(0)->as<ob::SO3StateSpace::StateType>(1));
    }

ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si)
    {
    ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
    obj->setCostThreshold(ob::Cost(std::numeric_limits<double>::max()));
    return obj;
    }

unsigned int findPlanTime(oc::PathControl pathSegment, double plan_time, double& actual_time)
    {
    actual_time = 0.0;
    unsigned int i;
    for (i = 0; i < pathSegment.getStateCount(); i++)
        {
        actual_time += pathSegment.getControlDuration(i);
        if (actual_time >= plan_time)
            {
            break;
            }
        }
    return i;
    }

oc::PathControl getSegment(const ob::SpaceInformationPtr& si, oc::PathControl path, unsigned int start, unsigned int end)
    {
    oc::PathControl newPath(si);
    if (end > path.getStateCount())
        {
        std::cout << "Reducing segment size to " << path.getStateCount() - 1 << std::endl;
        end = path.getStateCount() - 1;
        }
    for (unsigned int i = start; i < end; i++)
        {
        if (i == start)
            {
            newPath.append(path.getState(i));
            }
        else
            {
            newPath.append(path.getState(i), path.getControl(i - 1), path.getControlDuration(i - 1));
            }

        }
    return newPath;
    }

void saveCost(std::string dir, std::vector<double> segmentCosts, std::vector<double> initCosts, std::vector<double> finalCosts)
    {
    std::ofstream file1(dir + "segment_costs.txt");
    for (unsigned int i = 0; i < segmentCosts.size(); i++)
        {
        file1 << i << "," << segmentCosts[i] << std::endl;
        }
    file1.close();
    std::ofstream file2(dir + "opt_costs.txt");
    for (unsigned int i = 0; i < initCosts.size(); i++)
        {
        file2 << i << "," << initCosts[i] << "," << finalCosts[i] << std::endl;
        }
    file2.close();
    }