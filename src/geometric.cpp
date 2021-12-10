/* Standard Libraries */
#include <iostream>
#include <valarray>
#include <limits>
#include <fstream>
#include <time.h>
#include <random>

/* OMPL */
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>
#include <ompl/base/goals/GoalSpace.h>
/* FCL */
#include <fcl/fcl.h>

/* YAML */
#include <yaml-cpp/yaml.h>

#include "MyFunctions.h"

/* Namespaces */
namespace ob = ompl::base;
namespace og = ompl::geometric;

void planWithSimpleSetup(std::vector<std::shared_ptr<fcl::CollisionObjectf>> obstacles, std::shared_ptr<fcl::CollisionObjectf> robot, std::string ws)
    {

    // Make the compound state space
    ob::StateSpacePtr stateSpace;
    ob::StateSpacePtr goalSpace;
    DefineProblem(stateSpace, goalSpace);

    // Create a simple setup object
    og::SimpleSetup ss(stateSpace);

    // Declaring start state
    ob::ScopedState<> start(stateSpace);
    start->as<ob::CompoundState>()->as<ob::SE3StateSpace::StateType>(0)->setXYZ(0.0, 0.0, 0.0);
    start->as<ob::CompoundState>()->as<ob::SE3StateSpace::StateType>(0)->rotation().setIdentity();

    // Creating the goal space
    ob::GoalSpace goal(ss.getSpaceInformation());
    goal.setSpace(goalSpace);
    ob::GoalPtr goalPtr(&goal);

    // Set start state and goal space
    ss.setStartState(start);
    ss.setGoal(goalPtr);

    // set state validity checking for this space, geometric
    ob::SpaceInformation* si = ss.getSpaceInformation().get();
    ss.setStateValidityChecker(
        [si, obstacles, robot](const ob::State* state) { return isStateValid(si, state, obstacles, robot); });

    // Complete the setup
    ss.setup();

    // Solve the planning problem
    ob::PlannerStatus solved = ss.solve(5);
    if (solved)
        {
        saveGeometricPath(ss, ws);
        }
    return;
    // ss.simplifySolution();
    }

int main(int argc, char* argv[])
    {

    if (argc == 2)
        {
        std::string fname = argv[1];
        std::istringstream iss(fname);
        std::string ws;
        std::getline(iss, ws, '.');
        std::cout << "Planning with: " << fname << std::endl;
        YAML::Node config = YAML::LoadFile("../configs/" + fname);
        YAML::Node robot_node = config["robot"];
        YAML::Node obs_node = config["obstacles"];

        std::vector<std::shared_ptr<fcl::CollisionObjectf>> obstacles;
        std::shared_ptr<fcl::CollisionObjectf> robot;

        // Robot Definition
        std::vector<float> rob_size = robot_node["size"].as<std::vector<float>>();
        std::vector<float> start_pos = robot_node["start_position"].as<std::vector<float>>();
        std::vector<float> goal_pos = robot_node["goal_position"].as<std::vector<float>>();

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
                fcl::Quaternionf obs_rotation = fcl::Quaternionf(obs_orient[0], obs_orient[1], obs_orient[2], obs_orient[3]);
                obstacles.push_back(CollisionBox(obs_size[0], obs_size[1], obs_size[2], obs_translation, obs_rotation));
                }
            }
        //TODO: Getting a segfault on exit here
        planWithSimpleSetup(obstacles, robot, ws);
        }
    else if (argc > 2)
        {
        printf("Too many arguments supplied.\n");
        }
    else if (argc < 2)
        {
        printf("Expected workspace as argument.\n");
        }
    return 0;
    }
