/* OMPL */
#include <ompl/control/planners/sst/SST.h>
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
    // ob::PlannerPtr planner(new oc::SST(ss.getSpaceInformation()));
    // ss.setPlanner(planner);

    // Planner optimization objectives
    ss.setOptimizationObjective(getThresholdPathLengthObj(ss.getSpaceInformation()));

    // Planner termination conditions
    ob::PlannerTerminationCondition ptc_time = ob::timedPlannerTerminationCondition(120);
    ob::PlannerTerminationCondition ptc_sol = ob::CostConvergenceTerminationCondition(ss.getProblemDefinition(), 1, 1);
    ob::PlannerTerminationCondition ptc = ob::plannerOrTerminationCondition(ptc_time, ptc_sol);

    // Solve the planning problem
    ob::PlannerStatus solved = ss.solve(ptc);

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
        saveControlPath(path, ws);
        saveCost("/solutions/kinodynamic/cost.txt", count, segmentCost, initialCost);

        // Define new start state
        st = path.getState(idx);

        while (!goal.isSatisfied(st))
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
                // TODO: Find out why this is sometimes repeating endlessly
                path = ss.getSolutionPath();
                path.interpolate();
                std::cout << "Found optimization" << std::endl;
                saveControlPath(path, ws);
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
