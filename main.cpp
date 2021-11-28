/* Standard Libraries */
#include <iostream>
#include <valarray>
#include <limits>
#include <fstream>

/* OMPL */
#include <ompl/base/SpaceInformation.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/control/planners/sst/SST.h>
#include <ompl/config.h>

/* FCL */
#include <fcl/fcl.h>

/* Namespaces */
namespace ob = ompl::base;
namespace oc = ompl::control;
namespace og = ompl::geometric;

/* Global Variables and Constants */
// Definition of single robot and obstacle, should remove global definition eventually
const std::shared_ptr<fcl::CollisionGeometryf> robot_body(new fcl::Boxf(2.0, 1.0, 1.0));
const std::shared_ptr<fcl::CollisionGeometryf> obstacle_body(new fcl::Boxf(5.0, 5.0, 5.0));
const fcl::CollisionObjectf robot_body_object(robot_body, fcl::Transform3f());
const fcl::CollisionObjectf obstacle_body_object(obstacle_body, fcl::Transform3f());
const auto robot_collision_object = std::make_shared<fcl::CollisionObjectf>(robot_body_object);
const auto obstacle_collision_object = std::make_shared<fcl::CollisionObjectf>(obstacle_body_object);

auto CollisionBox(float l, float h, float w)
    {
    std::shared_ptr<fcl::CollisionGeometryf> geometry(new fcl::Boxf(l, w, h));
    fcl::CollisionObjectf collision_object(geometry, fcl::Transform3f());
    return std::make_shared<fcl::CollisionObjectf>(collision_object);
    }

auto CollisionBox(float l, float h, float w, fcl::Vector3f translation, fcl::Quaternionf rotation)
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
    const Eigen::Quaternionf orientation(x[3], x[4], x[5], x[6]);
    // std::cout << x[7] << ", " << u[0] << std::endl;
    const Eigen::Vector3f body_velocity(x[7], 0, 0);
    const Eigen::Vector3f vel = -(orientation * body_velocity);
    // std::cout << vel << std::endl;
    // Zero out qdot
    xdot.resize(x.size(), 0);
    //xdot[0] = 1;
    // Something wrong with velocity, needs to be negative in order to find solution? 
    xdot[0] = vel[0]; // X Velocity
    xdot[1] = vel[1]; // Y Velocity
    xdot[2] = vel[2]; // Z Velocity
    xdot[3] = 0.5 * (u[1] * x[6] + u[2] * x[5] - u[3] * x[4]); // q_dot x
    xdot[4] = 0.5 * (u[2] * x[6] + u[3] * x[3] - u[1] * x[5]); // q_dot y
    xdot[5] = 0.5 * (u[3] * x[6] + u[1] * x[4] - u[2] * x[3]); // q_dot z
    xdot[6] = -0.5 * (u[1] * x[3] + u[2] * x[4] + u[3] * x[5]); // q_dot w
    xdot[7] = u[0]; // Acceleration
    }

// This is a callback method invoked after numerical integration.
void PostIntegration(const ob::State* /*state*/, const oc::Control* /*control*/, const double /*duration*/, ob::State* result)
    {
    // Normalize orientation between 0 and 2*pi
    ob::SO3StateSpace SO3;
    SO3.enforceBounds(result->as<ob::CompoundStateSpace::StateType>()->as<ob::SE3StateSpace::StateType>(0)->as<ob::SO3StateSpace::StateType>(1));
    }

// Uses control space information
bool isStateValid(const oc::SpaceInformation* si, const ob::State* state)
    {
    const auto* se3state = state->as<ob::CompoundStateSpace::StateType>()->as<ob::SE3StateSpace::StateType>(0);
    const auto* pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
    const auto* rot = se3state->as<ob::SO3StateSpace::StateType>(1);
    const auto* val = state->as<ob::CompoundStateSpace::StateType>()->as<ob::RealVectorStateSpace::StateType>(1);
    // std::cout << "X = " << se3state->getX() << ", Y = " << se3state->getY() << ", Z = " << se3state->getZ() << std::endl;
    fcl::Vector3f translation(pos->values[0], pos->values[1], pos->values[2]);
    fcl::Quaternionf rotation(rot->x, rot->y, rot->z, rot->w);
    fcl::CollisionRequestf requestType(1, false, 1, false);
    fcl::CollisionResultf collisionResult;
    robot_collision_object->setTransform(rotation, translation);
    const bool collided = fcl::collide(robot_collision_object.get(), obstacle_collision_object.get(), requestType, collisionResult);
    const bool bounded = si->satisfiesBounds(state);
    // Checks if state is in valid bounds and that there is no collision
    // std::cout << "X = " << se3state->getX() << ", Y = " << se3state->getY() << ", Z = " << se3state->getZ() << ", V = " << val->values[0] << ", bounded = " << bounded << std::endl;
    return (bounded && !collisionResult.isCollision());
    }

// Uses base state information
// bool isStateValid(const ob::SpaceInformation* si, const ob::State* state)
//     {
//     // TODO: Determine the difference between control and base state information
//     const auto* se3state = state->as<ob::CompoundStateSpace::StateType>()->as<ob::SE3StateSpace::StateType>(0);
//     const auto* pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
//     const auto* rot = se3state->as<ob::SO3StateSpace::StateType>(1);
//     fcl::Vector3f translation(pos->values[0], pos->values[1], pos->values[2]);
//     fcl::Quaternionf rotation(rot->x, rot->y, rot->z, rot->w);
//     fcl::CollisionRequestf requestType(1, false, 1, false);
//     fcl::CollisionResultf collisionResult;
//     robot_collision_object->setTransform(rotation, translation);
//     fcl::collide(robot_collision_object.get(), obstacle_collision_object.get(), requestType, collisionResult);
//     // Checks if state is in valid bounds and that there is no collision
//     return (si->satisfiesBounds(state) && !collisionResult.isCollision());
//     }

void planWithSimpleSetup()
    {
    // TODO: Create easy switch between geometric and kinodynamic planning 
    // Obstacle setup


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
    velocityBound.setLow(0);
    velocityBound.setHigh(0.5);

    // Set the bounds for the spaces
    SE3->setBounds(bounds);
    velocity->setBounds(velocityBound);

    // create a control space
    auto cspace(std::make_shared<oc::RealVectorControlSpace>(stateSpace, 4));

    // set the bounds for the control space
    ob::RealVectorBounds cbounds(4);
    // Bounds for u1
    cbounds.setLow(0, -0.05);
    cbounds.setHigh(0, 0.05);
    // Bounds for u2
    cbounds.setLow(1, -0.1);
    cbounds.setHigh(1, 0.1);
    // Bounds for u2
    cbounds.setLow(2, -0.1);
    cbounds.setHigh(2, 0.1);
    // Bounds for u4
    cbounds.setLow(3, -0.1);
    cbounds.setHigh(3, 0.1);

    // Set the control bounds
    cspace->setBounds(cbounds);

    // define a simple setup class
    oc::SimpleSetup ss(cspace); // For controls problem
    // og::SimpleSetup ss(stateSpace); // For geometric problem

    // set state validity checking for this space, kinodynamic
    oc::SpaceInformation* si = ss.getSpaceInformation().get();
    ss.setStateValidityChecker(
        [si](const ob::State* state) { return isStateValid(si, state); });
    // si->setStateValidityCheckingResolution(0.1);

    // set state validity checking for this space, geometric
    // ob::SpaceInformation* si = ss.getSpaceInformation().get();
    // ss.setStateValidityChecker(
    //     [si](const ob::State* state) { return isStateValid(si, state); });

    // Use the ODESolver to propagate the system. Call PostIntegration
    // when integration has finished to normalize the orientation values.
    // Comment this out for geometric planning
    auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(ss.getSpaceInformation(), &DynamicsODE));
    ss.setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, &PostIntegration));

    // For compound state spaces: https://ompl.kavrakilab.org/HybridSystemPlanning_8cpp_source.html
    ob::ScopedState<> start(stateSpace);
    start->as<ob::CompoundState>()->as<ob::SE3StateSpace::StateType>(0)->setXYZ(0.0, 0.0, 0.0);
    start->as<ob::CompoundState>()->as<ob::SE3StateSpace::StateType>(0)->rotation().setIdentity();

    ob::ScopedState<> goal(stateSpace);
    goal->as<ob::CompoundState>()->as<ob::SE3StateSpace::StateType>(0)->setXYZ(15.0, 0.0, 0.0);
    goal->as<ob::CompoundState>()->as<ob::SE3StateSpace::StateType>(0)->rotation().setIdentity();

    // For goal regions visit: https://ompl.kavrakilab.org/RigidBodyPlanningWithIK_8cpp_source.html
    ss.setStartAndGoalStates(start, goal, 0.1);

    // Set the planner
    ob::PlannerPtr planner(new oc::SST(ss.getSpaceInformation()));
    ss.setPlanner(planner);

    ss.setup();


    // Time to find a solution
    const float solve_time = 60;

    // Solve the planning problem
    ob::PlannerStatus solved = ss.solve(solve_time);
    // ss.simplifySolution();

    if (solved)
        {
        std::cout << "Found solution!" << std::endl;
        std::ofstream file("../solutions/solution.txt");
        ss.getSolutionPath().printAsMatrix(file);
        file.close();
        }
    else
        std::cout << "No solution found" << std::endl;
    }

int main(int /*argc*/, char** /*argv*/)
    {
    fcl::Vector3f obs_translation(7.5, 0.0, 0.0);
    fcl::Quaternionf obs_rotation(0, 0, 0, 1);
    obstacle_collision_object->setTransform(obs_rotation, obs_translation);
    //const auto box = CollisionBox(2.0, 1.0, 1.0);
    planWithSimpleSetup();

    return 0;
    }
