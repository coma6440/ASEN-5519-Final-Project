#include "MyFunctions.h"
#include <ompl/geometric/planners/kpiece/KPIECE1.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>

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


    // std::cout << ss.getStateValidityChecker()->isValid(start.get()) << std::endl;
    // stateSpace->registerDefaultProjection(ob::ProjectionEvaluatorPtr(new ob::SubspaceProjectionEvaluator(stateSpace.get(), 0)));

    ob::PlannerPtr planner(new og::RRTConnect(ss.getSpaceInformation()));
    ss.setPlanner(planner);


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
        std::istringstream iss(argv[1]);
        std::string ws;
        std::getline(iss, ws, '.');
        std::vector<std::shared_ptr<fcl::CollisionObjectf>> obstacles;
        std::shared_ptr<fcl::CollisionObjectf> robot;
        GetEnvironment(ws, obstacles, robot);
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
