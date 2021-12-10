/* OMPL */
#include "ompl/tools/benchmark/Benchmark.h"
#include <ompl/control/planners/sst/SST.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/est/EST.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/base/ProjectionEvaluator.h>

/* Static Library */
#include "MyFunctions.h"

/* Namespaces */
namespace ob = ompl::base;
namespace oc = ompl::control;
namespace og = ompl::geometric;
namespace ot = ompl::tools;


void planWithSimpleSetup(std::vector<std::shared_ptr<fcl::CollisionObjectf>> obstacles, std::shared_ptr<fcl::CollisionObjectf> robot, std::string ws)
    {
    // Make the compound state space and goal space
    ob::StateSpacePtr stateSpace;
    ob::StateSpacePtr goalSpace;
    DefineProblem(stateSpace, goalSpace);

    // Control space setup
    auto cspace(std::make_shared<oc::RealVectorControlSpace>(stateSpace, 4));

    // Create the bounds for the control space
    ob::RealVectorBounds cbounds(4);

    //  Control space bounds
    cbounds.setLow(0, -1);
    cbounds.setHigh(0, 1);
    cbounds.setLow(1, -0.2);
    cbounds.setHigh(1, 0.2);
    cbounds.setLow(2, -0.2);
    cbounds.setHigh(2, 0.2);
    cbounds.setLow(3, -0.2);
    cbounds.setHigh(3, 0.2);

    // Set the control bounds
    cspace->setBounds(cbounds);

    // Create a simple setup object
    oc::SimpleSetup ss(cspace);

    // Declaring start and goal states
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

    // Set state validity checking for this space
    oc::SpaceInformation* si = ss.getSpaceInformation().get();
    ss.setStateValidityChecker([si, obstacles, robot](const ob::State* state) { return isStateValid(si, state, obstacles, robot); });
    // si->setStateValidityCheckingResolution(0.001); // 1%

    // Use the ODESolver to propagate the system. Call PostIntegration when done
    auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(ss.getSpaceInformation(), &DynamicsODE));
    ss.setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, &PostIntegration));

    stateSpace->registerDefaultProjection(ob::ProjectionEvaluatorPtr(new ob::SubspaceProjectionEvaluator(stateSpace.get(), 0)));

    // Complete the setup
    ss.setup();

    // Benchmarking
    ot::Benchmark b(ss, "kinodynamic_" + ws);

    b.addPlanner(ob::PlannerPtr(new oc::SST(ss.getSpaceInformation())));
    b.addPlanner(ob::PlannerPtr(new oc::RRT(ss.getSpaceInformation())));
    b.addPlanner(ob::PlannerPtr(new oc::KPIECE1(ss.getSpaceInformation())));
    b.addPlanner(ob::PlannerPtr(new oc::EST(ss.getSpaceInformation())));
    // Solve the planning problem
    ot::Benchmark::Request req;
    req.maxTime = 1.0;
    req.maxMem = 1000.0;
    req.runCount = 10;
    req.displayProgress = true;
    b.benchmark(req);

    // This will generate a file of the form ompl_host_time.log
    b.saveResultsToFile();
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
