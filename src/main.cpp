/* Standard Libraries */
#include <iostream>
#include <valarray>
#include <limits>
#include <fstream>
#include <time.h>
#include <random>

/* OMPL */
#include <ompl/base/SpaceInformation.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/control/planners/sst/SST.h>
// #include "MyPlanner.h"
#include <ompl/config.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/terminationconditions/CostConvergenceTerminationCondition.h>
#include <ompl/base/goals/GoalSpace.h>
/* FCL */
#include <fcl/fcl.h>

/* YAML */
#include <yaml-cpp/yaml.h>

/* Namespaces */
namespace ob = ompl::base;
namespace oc = ompl::control;
namespace og = ompl::geometric;


unsigned int findPlanTime(std::vector<double> durations, unsigned int n_states, double plan_time, double& actual_time)
    {
    for (unsigned int i = 0; i < n_states; i++)
        {
        actual_time += durations[i];
        if (actual_time >= plan_time)
            {
            return i;
            }
        }
    return n_states - 1;

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


// TODO: https://www.codeproject.com/Articles/1078771/Techniques-for-Avoiding-Code-Duplication#example2

auto CollisionBox(float l, float h, float w, fcl::Vector3f translation = fcl::Vector3f(0, 0, 0), fcl::Quaternionf rotation = fcl::Quaternionf(0, 0, 0, 1))
    {
    std::shared_ptr<fcl::CollisionGeometryf> geometry(new fcl::Boxf(l, w, h));
    fcl::CollisionObjectf collision_object(geometry, fcl::Transform3f());
    auto obj = std::make_shared<fcl::CollisionObjectf>(collision_object);
    obj->setTransform(rotation, translation);
    return obj;
    }

// Definition of the ODE for the kinematic car.
void DynamicsODE(const oc::ODESolver::StateType& x, const oc::Control* control, oc::ODESolver::StateType& xdot)
    {
    // TODO: Update to get proper dynamics, use quaternion math if possible
    const double* u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    const Eigen::Quaternionf orientation(x[6], x[3], x[4], x[5]);
    const Eigen::Quaternionf ang_vel(0, u[1], u[2], u[3]);
    const Eigen::Vector3f body_velocity(x[7], 0, 0);
    const Eigen::Vector3f vel = orientation * body_velocity;
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

// This is a callback method invoked after numerical integration.
void PostIntegration(const ob::State* /*state*/, const oc::Control* /*control*/, const double /*duration*/, ob::State* result)
    {
    // Normalize orientation between 0 and 2*pi
    ob::SO3StateSpace SO3;
    SO3.enforceBounds(result->as<ob::CompoundStateSpace::StateType>()->as<ob::SE3StateSpace::StateType>(0)->as<ob::SO3StateSpace::StateType>(1));
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

void saveControlPath(oc::PathControl path, std::string fname, unsigned int count)
    {
    // path.interpolate();
    static char name[50];
    time_t now = time(0);
    std::string s = "../solutions/kinodynamic/" + fname + "_" + std::to_string(count) + "_%Y%m%d%H%M%S.txt";
    strftime(name, sizeof(name), s.c_str(), localtime(&now));
    std::ofstream file(name);
    path.printAsMatrix(file);
    file.close();
    }

template <typename T>
void savePath(T ss, ob::PlannerStatus solved, std::string planType, std::string fname)
    {
    static char name[50];
    time_t now = time(0);


    if (solved)
        {
        if (planType == "g")
            {
            std::string s = "../solutions/geometric/" + fname + "_%Y%m%d%H%M%S.txt";
            strftime(name, sizeof(name), s.c_str(), localtime(&now));
            }
        else if (planType == "k")
            {
            std::string s = "../solutions/kinodynamic/" + fname + "_%Y%m%d%H%M%S.txt";
            std::cout << "Found a solution of length " << ss.getSolutionPath().length() << std::endl;
            strftime(name, sizeof(name), s.c_str(), localtime(&now));
            }

        std::ofstream file(name);
        ss.getSolutionPath().printAsMatrix(file);
        file.close();
        }
    else
        {
        std::cout << "No solution found" << std::endl;
        }
    }

ob::Cost getSegmentCost(oc::PathControl path, ob::OptimizationObjectivePtr opt, unsigned int start, unsigned int end)
    {
    og::PathGeometric geo_path = path.asGeometric();


    if (end != geo_path.getStateCount())
        {
        ob::State* end_state = geo_path.getState(end + 1);
        geo_path.keepBefore(end_state);
        }
    if (start != 0)
        {
        ob::State* start_state = geo_path.getState(start - 1);
        geo_path.keepAfter(start_state);
        }
    return geo_path.cost(opt);
    }

void saveCost(std::string fname, unsigned int count, ob::Cost segmentCost, ob::Cost totalCost)
    {

    std::ofstream file(fname);
    std::cout << count << ": segment = " << segmentCost.value() << ", total = " << totalCost.value() << std::endl;
    file.close();
    }

ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si)
    {
    ob::OptimizationObjectivePtr obj(new ob::PathLengthOptimizationObjective(si));
    obj->setCostThreshold(ob::Cost(std::numeric_limits<double>::max()));
    return obj;
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

    // create a control space
    auto cspace(std::make_shared<oc::RealVectorControlSpace>(stateSpace, 4));

    // set the bounds for the control space
    ob::RealVectorBounds cbounds(4);
    // Bounds for u1
    cbounds.setLow(0, -1);
    cbounds.setHigh(0, 1);
    // Bounds for u2
    cbounds.setLow(1, -0.2);
    cbounds.setHigh(1, 0.2);
    // Bounds for u2
    cbounds.setLow(2, -0.2);
    cbounds.setHigh(2, 0.2);
    // Bounds for u4
    cbounds.setLow(3, -0.2);
    cbounds.setHigh(3, 0.2);

    // Set the control bounds
    cspace->setBounds(cbounds);

    // Declaring start and goal states
    // For compound state spaces: https://ompl.kavrakilab.org/HybridSystemPlanning_8cpp_source.html
    ob::ScopedState<> start(stateSpace);
    start->as<ob::CompoundState>()->as<ob::SE3StateSpace::StateType>(0)->setXYZ(0.0, 0.0, 0.0);
    start->as<ob::CompoundState>()->as<ob::SE3StateSpace::StateType>(0)->rotation().setIdentity();

    ob::ScopedState<> goal(stateSpace);
    goal->as<ob::CompoundState>()->as<ob::SE3StateSpace::StateType>(0)->setXYZ(15.0, 0.0, 0.0);
    goal->as<ob::CompoundState>()->as<ob::SE3StateSpace::StateType>(0)->rotation().setIdentity();


    ob::PlannerStatus solved;
    if (planType == "k")
        {
        oc::SimpleSetup ss(cspace); // For controls problem
        // set state validity checking for this space, kinodynamic
        oc::SpaceInformation* si = ss.getSpaceInformation().get();
        ss.setStateValidityChecker([si, obstacles, robot](const ob::State* state) { return isStateValid(si, state, obstacles, robot); });
        // si->setStateValidityCheckingResolution(0.1);

        // Use the ODESolver to propagate the system. Call PostIntegration
        // when integration has finished to normalize the orientation values.
        auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(ss.getSpaceInformation(), &DynamicsODE));
        ss.setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, &PostIntegration));

        // Set the planner
        // ob::PlannerPtr planner(new oc::SST(ss.getSpaceInformation()));
        // ss.setPlanner(planner);

        ss.setOptimizationObjective(getThresholdPathLengthObj(ss.getSpaceInformation()));

        // For goal regions visit: https://ompl.kavrakilab.org/RigidBodyPlanningWithIK_8cpp_source.html
        // ss.getSpaceInformation()->setPropagationStepSize(.5);
        // ss.getSpaceInformation()->setMinMaxControlDuration(1, 2);
        // ss.setStartAndGoalStates(start, goal);

        // Goal region
        auto goal(std::make_shared<MyGoalRegion>(ss.getSpaceInformation()));
        ss.setStartState(start);
        ss.setGoal(goal);

        // Time to find a solution
        ob::PlannerTerminationCondition ptc_time = ob::timedPlannerTerminationCondition(120);
        ob::PlannerTerminationCondition ptc_sol = ob::CostConvergenceTerminationCondition(ss.getProblemDefinition(), 1, 1);
        ob::PlannerTerminationCondition ptc = ob::plannerOrTerminationCondition(ptc_time, ptc_sol);

        // Solve the planning problem
        solved = ss.solve(ptc);
        // ss.simplifySolution();
        unsigned int idx = 1;
        if (ss.haveExactSolutionPath())
            {
            oc::PathControl path = ss.getSolutionPath();
            path.interpolate();

            std::vector<double> durations = path.getControlDurations();
            ob::OptimizationObjectivePtr opt = getThresholdPathLengthObj(ss.getSpaceInformation());
            ob::Cost initialCost = path.asGeometric().cost(opt);
            unsigned int n_states = durations.size();
            double initialDuration = path.length();
            std::cout << "Initial Path: n = " << n_states << ", t = " << initialDuration << ", c = " << initialCost.value() << std::endl;

            // Set an optimizing planner
            ob::PlannerPtr planner(new oc::SST(ss.getSpaceInformation()));
            ss.setPlanner(planner);

            // Initialize some variables used in the loop
            ob::State* st = path.getState(0);
            double dt = 10.0;
            double actual_time = 0.0;
            double accum_time = 0.0;
            double planTime = 0.0;
            unsigned int count = 0;
            unsigned int prevIdx = 0;

            // Get first path segment
            unsigned int idx = findPlanTime(durations, n_states, actual_time + dt, actual_time);
            planTime = actual_time - accum_time;
            accum_time += actual_time;
            ob::Cost segmentCost = getSegmentCost(path, opt, 0, idx);
            // Save results of initial path
            saveControlPath(path, ws, count);
            saveCost("/solutions/kinodynamic/cost.txt", count, segmentCost, initialCost);

            // Define new start state
            st = path.getState(idx);

            while (!goal.get()->isSatisfied(st))
                {
                count++;
                // std::cout << "Using idx " << idx << " gives planning time of " << planTime << std::endl;
                // Copy and display new starting state
                si->copyState(start->as<ob::CompoundStateSpace::StateType>(), st);

                // Change the start state
                ss.getProblemDefinition()->clearStartStates();
                ss.setStartState(start);
                ss.getProblemDefinition()->clearSolutionPaths();
                // oc::PathControl newSolnPath = makePath(si, path, idx, n_states);
                // newSolnPath.print(std::cout);
                // ss.getProblemDefinition()->addSolutionPath(path);
                ob::PlannerTerminationCondition ptc_time = ob::timedPlannerTerminationCondition(planTime);
                ss.getPlanner().get()->clear();
                prevIdx = idx;
                solved = ss.solve(ptc_time);
                if (solved && ss.haveExactSolutionPath())
                    {

                    path = ss.getSolutionPath();
                    path.interpolate();
                    std::cout << "Found optimization" << std::endl;
                    saveControlPath(path, ws, count);
                    accum_time = 0.0;
                    durations = path.getControlDurations();
                    segmentCost = path.asGeometric().cost(getThresholdPathLengthObj(ss.getSpaceInformation()));
                    n_states = durations.size();
                    //std::cout << "n states remaining = " << n_states << std::endl;
                    prevIdx = 0;
                    }

                actual_time = 0.0;
                // Get first path segment
                idx = findPlanTime(durations, n_states, accum_time + dt, actual_time);
                planTime = actual_time - accum_time;
                accum_time += actual_time;
                // std::cout << "Act: " << actual_time << ", P: " << planTime << ", Acc: " << accum_time << std::endl;

                ob::Cost segmentCost = getSegmentCost(path, opt, prevIdx, idx);
                saveCost("/solutions/kinodynamic/cost.txt", count, segmentCost, path.asGeometric().cost(opt));

                // Define new start state
                st = path.getState(idx);
                }
            }
        else
            {
            // ob::PlannerTerminationCondition ptc_time = ob::timedPlannerTerminationCondition(10);
            // ob::PlannerTerminationCondition ptc_sol = ob::CostConvergenceTerminationCondition(ss.getProblemDefinition(), 1, 1);
            // ob::PlannerTerminationCondition ptc = ob::plannerOrTerminationCondition(ptc_time, ptc_sol);
            // ss.solve(ptc);
            }
        }
    else if (planType == "g")
        {
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
        savePath(ss, solved, planType, ws);
        // ss.simplifySolution();
        }

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