/* OMPL */
#include <ompl/control/planners/sst/SST.h>
#include <ompl/control/planners/rrt/RRT.h>
// #include "MyPlanner.h"

/* Static Library */
#include "MyFunctions.h"

/* Namespaces */
namespace ob = ompl::base;
namespace oc = ompl::control;
namespace og = ompl::geometric;


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

    // Use the ODESolver to propagate the system. Call PostIntegration when done
    auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(ss.getSpaceInformation(), &DynamicsODE));
    ss.setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, &PostIntegration));

    // Set the planner
    ob::PlannerPtr planner(new oc::RRT(ss.getSpaceInformation()));
    ss.setPlanner(planner);

    // Planner optimization objectives
    ob::OptimizationObjectivePtr opt = getThresholdPathLengthObj(ss.getSpaceInformation());
    ss.setOptimizationObjective(opt);

    // Planner termination conditions
    ob::PlannerTerminationCondition ptc_time = ob::timedPlannerTerminationCondition(60);
    ob::PlannerTerminationCondition ptc_sol = ob::CostConvergenceTerminationCondition(ss.getProblemDefinition(), 1, 1);
    ob::PlannerTerminationCondition ptc = ob::plannerOrTerminationCondition(ptc_time, ptc_sol);

    ss.setup();

    // Solve the planning problem
    ob::PlannerStatus solved = ss.solve(ptc);


    if (ss.haveExactSolutionPath())
        {
        unsigned int idx = 1;
        oc::PathControl initialPath = ss.getSolutionPath();
        initialPath.interpolate();
        double initialCost = initialPath.asGeometric().cost(opt).value();
        std::cout << "Initial Path: n = " << initialPath.getStateCount() << ", t = " << initialPath.length() << ", c = " << initialCost << std::endl;

        // Set an optimizing planner
        ob::PlannerPtr planner(new oc::SST(ss.getSpaceInformation()));
        ss.setPlanner(planner);

        // Initialize some variables used in the loop
        ob::State* st = initialPath.getState(0);
        double dt = 20.0;
        double actual_time = 0.0;
        double accum_time = 0.0;
        double planTime = 0.0;
        unsigned int count = 0;
        double finalCost = 0.0;
        std::vector<double> init_costs;
        std::vector<double> final_costs;
        std::vector<double> segment_costs;

        // Get first path segment
        idx = findPlanTime(initialPath, dt, actual_time);
        planTime = actual_time;
        accum_time += actual_time;
        oc::PathControl currentSegment = getSegment(ss.getSpaceInformation(), initialPath, 0, idx + 1);
        oc::PathControl remainingSegment = getSegment(ss.getSpaceInformation(), initialPath, idx, initialPath.getStateCount());
        // Save results of initial path
        saveControlPath(initialPath, ws + "_init");
        saveControlPath(currentSegment, ws + "_c_" + std::to_string(count));
        finalCost += currentSegment.asGeometric().cost(opt).value();
        segment_costs.push_back(currentSegment.asGeometric().cost(opt).value());


        while (remainingSegment.getStateCount() > 0)
            {
            count++;
            init_costs.push_back(remainingSegment.asGeometric().cost(opt).value());
            st = remainingSegment.getState(0);
            si->copyState(start->as<ob::CompoundStateSpace::StateType>(), st);
            // Change the start state
            ss.getProblemDefinition()->clearStartStates();
            ss.setStartState(start);
            // Clear solutions and define new planner
            ss.getProblemDefinition()->clearSolutionPaths();
            ob::PlannerTerminationCondition ptc_time = ob::timedPlannerTerminationCondition(planTime);
            ss.getPlanner().get()->clear();
            // Solve new planner
            solved = ss.solve(ptc_time);
            if (solved && ss.haveExactSolutionPath())
                {
                ob::Cost currentRemainingCost = remainingSegment.asGeometric().cost(opt);
                ob::Cost futureCost = ss.getSolutionPath().asGeometric().cost(opt);
                if (futureCost.value() < currentRemainingCost.value())
                    {
                    std::cout << "Found optimization from " << currentRemainingCost.value() << " to " << futureCost.value() << std::endl;
                    remainingSegment = ss.getSolutionPath();
                    remainingSegment.interpolate();
                    }
                }
            saveControlPath(remainingSegment, ws + "_r_" + std::to_string(count));
            final_costs.push_back(remainingSegment.asGeometric().cost(opt).value());
            idx = findPlanTime(remainingSegment, dt, actual_time);
            planTime = actual_time;
            accum_time += actual_time;
            // Get next path segment
            std::cout << "Plan time = " << planTime << ", idx = " << idx << ", n = " << remainingSegment.getStateCount() << std::endl;
            currentSegment = getSegment(ss.getSpaceInformation(), remainingSegment, 0, idx + 1);
            saveControlPath(currentSegment, ws + "_c_" + std::to_string(count));
            remainingSegment = getSegment(ss.getSpaceInformation(), remainingSegment, idx, remainingSegment.getStateCount());
            finalCost += currentSegment.asGeometric().cost(opt).value();
            segment_costs.push_back(currentSegment.asGeometric().cost(opt).value());
            std::cout << "Remaining states: " << remainingSegment.getStateCount() << std::endl;
            }
        saveCost("../solutions/", segment_costs, init_costs, final_costs);
        std::cout << "Initial cost = " << initialCost << ", Final cost = " << finalCost << std::endl;
        }
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
