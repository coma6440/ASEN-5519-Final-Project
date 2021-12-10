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

// TODO: https://www.codeproject.com/Articles/1078771/Techniques-for-Avoiding-Code-Duplication#example2

auto CollisionBox(float l, float h, float w, fcl::Vector3f translation = fcl::Vector3f(0, 0, 0), fcl::Quaternionf rotation = fcl::Quaternionf(0, 0, 0, 1))
    {
    std::shared_ptr<fcl::CollisionGeometryf> geometry(new fcl::Boxf(l, w, h));
    fcl::CollisionObjectf collision_object(geometry, fcl::Transform3f());
    auto obj = std::make_shared<fcl::CollisionObjectf>(collision_object);
    obj->setTransform(rotation, translation);
    return obj;
    }

template <typename T>
bool isStateValid(T si, const ob::State* state, std::vector<std::shared_ptr<fcl::CollisionObjectf>> obstacles, std::shared_ptr<fcl::CollisionObjectf> robot)
    {
    const auto* se3state = state->as<ob::CompoundStateSpace::StateType>()->as<ob::SE3StateSpace::StateType>(0);
    const auto* pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
    const auto* rot = se3state->as<ob::SO3StateSpace::StateType>(1);
    const auto* vel = state->as<ob::CompoundStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(1);
    // std::cout << "X = " << se3state->getX() << ", Y = " << se3state->getY() << ", Z = " << se3state->getZ() << std::endl;
    fcl::Vector3f translation(pos->values[0], pos->values[1], pos->values[2]);
    fcl::Quaternionf rotation(rot->x, rot->y, rot->z, rot->w);
    fcl::CollisionRequestf requestType(1, false, 1, false);
    fcl::CollisionResultf collisionResult;
    robot->setTransform(rotation, translation);
    // Checks that there are no collisions
    for (std::shared_ptr<fcl::CollisionObjectf> obs : obstacles)
        {
        if (fcl::collide(robot.get(), obs.get(), requestType, collisionResult))
            {
            return false;
            }
        }
    // Checks if state is in valid bounds
    return si->satisfiesBounds(state);
    }

template <typename T>
void savePath(T ss, ob::PlannerStatus solved, std::string fname)
    {
    static char name[50];
    time_t now = time(0);


    if (solved)
        {

        std::string s = "../solutions/geometric/" + fname + "_%Y%m%d%H%M%S.txt";
        strftime(name, sizeof(name), s.c_str(), localtime(&now));

        std::ofstream file(name);
        ss.getSolutionPath().printAsMatrix(file);
        file.close();
        }
    else
        {
        std::cout << "No solution found" << std::endl;
        }
    }


class MyGoalRegion : public ob::GoalSampleableRegion
    {
    public:
        MyGoalRegion(const ob::SpaceInformationPtr& si) : ob::GoalSampleableRegion(si)
            {
            }
        bool isSatisfied(const ob::State* st) const
            {
            const auto* se3state = st->as<ob::CompoundStateSpace::StateType>()->as<ob::SE3StateSpace::StateType>(0);
            const auto* vel = st->as<ob::CompoundStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(1);
            bool inGoal = false;
            if (distanceGoal(st) < 2.0)
                {
                inGoal = true;
                }
            return inGoal;
            }

        bool isSatisfied(const ob::State* st, double* distance) const
            {
            bool result = isSatisfied(st);

            if (distance != NULL)
                {
                *distance = distanceGoal(st);
                }
            return result;
            }
        double distanceGoal(const ob::State* st) const override
            {
            const auto* se3state = st->as<ob::CompoundStateSpace::StateType>()->as<ob::SE3StateSpace::StateType>(0);
            const auto* vel = st->as<ob::CompoundStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(1);
            double d = fabs((se3state->getX() - goal_x)) + fabs(se3state->getY() - goal_y) + fabs(se3state->getZ() - goal_z);
            return d;
            }
        void sampleGoal(ob::State* st) const override
            {
            auto* se3state = st->as<ob::CompoundStateSpace::StateType>()->as<ob::SE3StateSpace::StateType>(0);
            auto* vel = st->as<ob::CompoundStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(1);
            se3state->rotation().setIdentity();
            vel->values[0] = 0.0;
            se3state->setXYZ(15, 0, 0);
            }
        unsigned int maxSampleCount() const override
            {
            return 1000;
            }

    private:
        float vel_bound = 0.1;
        float goal_x = 15.0;
        float goal_y = 0.0;
        float goal_z = 0.0;
        // std::default_random_engine generator;
        // std::uniform_real_distribution<double> x_goal_dist(14.0, 15.0);
        // std::uniform_real_distribution<double> y_goal_dist(-1.0, 1.0);

    };


void planWithSimpleSetup(const std::string planType, std::vector<std::shared_ptr<fcl::CollisionObjectf>> obstacles, std::shared_ptr<fcl::CollisionObjectf> robot, std::string ws)
    {
    // TODO: Obstacle setup

    // State space setup
    auto SE3(std::make_shared<ob::SE3StateSpace>());
    auto velocity(std::make_shared<ob::RealVectorStateSpace>(1));
    // Make the compound state space
    ob::StateSpacePtr stateSpace = SE3 + velocity;

    // Create the bounds for the SE3 space
    ob::RealVectorBounds bounds(3);
    // X Position
    bounds.setLow(0, 0);
    bounds.setHigh(0, 15);
    // Y Position
    bounds.setLow(1, -10);
    bounds.setHigh(1, 10);
    // Z Position
    bounds.setLow(2, -10);
    bounds.setHigh(2, 10);

    // Create the bounds for the velocity space
    ob::RealVectorBounds velocityBound(1);
    velocityBound.setLow(0.0);
    velocityBound.setHigh(0.5);

    // Set the bounds for the spaces
    SE3->setBounds(bounds);
    velocity->setBounds(velocityBound);

    // Declaring start and goal states
    // For compound state spaces: https://ompl.kavrakilab.org/HybridSystemPlanning_8cpp_source.html
    ob::ScopedState<> start(stateSpace);
    start->as<ob::CompoundState>()->as<ob::SE3StateSpace::StateType>(0)->setXYZ(0.0, 0.0, 0.0);
    start->as<ob::CompoundState>()->as<ob::SE3StateSpace::StateType>(0)->rotation().setIdentity();

    ob::ScopedState<> goal(stateSpace);
    goal->as<ob::CompoundState>()->as<ob::SE3StateSpace::StateType>(0)->setXYZ(15.0, 0.0, 0.0);
    goal->as<ob::CompoundState>()->as<ob::SE3StateSpace::StateType>(0)->rotation().setIdentity();


    ob::PlannerStatus solved;


    og::SimpleSetup ss(stateSpace); // For geometric problem
    // set state validity checking for this space, geometric
    ob::SpaceInformation* si = ss.getSpaceInformation().get();
    ss.setStateValidityChecker(
        [si, obstacles, robot](const ob::State* state) { return isStateValid(si, state, obstacles, robot); });

    // For goal regions visit: https://ompl.kavrakilab.org/RigidBodyPlanningWithIK_8cpp_source.html
    ss.setStartAndGoalStates(start, goal);
    ss.setup();

    // Time to find a solution
    const float solve_time = 5;

    // Solve the planning problem
    solved = ss.solve(solve_time);
    savePath(ss, solved, ws);
    // ss.simplifySolution();


    }

int main(int argc, char* argv[])
    {

    if (argc == 3)
        {
        std::string fname = argv[2];
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
        planWithSimpleSetup(argv[1], obstacles, robot, ws);
        }
    else if (argc > 3)
        {
        printf("Too many arguments supplied.\n");
        }
    else if (argc < 3)
        {
        printf("Two argument expected.\n");
        }
    return 0;
    }
