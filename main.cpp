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
void DynamicsODE(const oc::ODESolver::StateType& q, const oc::Control* control, oc::ODESolver::StateType& qdot)
    {
    // TODO: Update to get proper dynamics, use quaternion math if possible
    const double* u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
    const double theta = q[2];
    double carLength = 0.2;

    // Zero out qdot
    qdot.resize(q.size(), 0);

    qdot[0] = u[0] * cos(theta);
    qdot[1] = u[0] * sin(theta);
    qdot[2] = u[0] * tan(u[1]) / carLength;
    }

// This is a callback method invoked after numerical integration.
void PostIntegration(const ob::State* /*state*/, const oc::Control* /*control*/, const double /*duration*/, ob::State* result)
    {
    // Normalize orientation between 0 and 2*pi
    ob::SO3StateSpace SO3;
    SO3.enforceBounds(result->as<ob::SE3StateSpace::StateType>()->as<ob::SO3StateSpace::StateType>(1));
    }

// Uses control space information
bool isStateValid(const oc::SpaceInformation* si, const ob::State* state)
    {
    const auto* se3state = state->as<ob::SE3StateSpace::StateType>();
    const auto* pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
    const auto* rot = se3state->as<ob::SO3StateSpace::StateType>(1);
    fcl::Vector3f translation(pos->values[0], pos->values[1], pos->values[2]);
    fcl::Quaternionf rotation(rot->x, rot->y, rot->z, rot->w);
    fcl::CollisionRequestf requestType(1, false, 1, false);
    fcl::CollisionResultf collisionResult;
    robot_collision_object->setTransform(rotation, translation);
    fcl::collide(robot_collision_object.get(), obstacle_collision_object.get(), requestType, collisionResult);
    // Checks if state is in valid bounds and that there is no collision
    return (si->satisfiesBounds(state) && !collisionResult.isCollision());
    }

// Uses base state information
bool isStateValid(const ob::SpaceInformation* si, const ob::State* state)
    {
    // TODO: Determine the difference between control and base state information
    const auto* se3state = state->as<ob::SE3StateSpace::StateType>();
    const auto* pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);
    const auto* rot = se3state->as<ob::SO3StateSpace::StateType>(1);
    fcl::Vector3f translation(pos->values[0], pos->values[1], pos->values[2]);
    fcl::Quaternionf rotation(rot->x, rot->y, rot->z, rot->w);
    fcl::CollisionRequestf requestType(1, false, 1, false);
    fcl::CollisionResultf collisionResult;
    robot_collision_object->setTransform(rotation, translation);
    fcl::collide(robot_collision_object.get(), obstacle_collision_object.get(), requestType, collisionResult);
    // Checks if state is in valid bounds and that there is no collision
    return (si->satisfiesBounds(state) && !collisionResult.isCollision());
    }

void planWithSimpleSetup()
    {
    // TODO: Create easy switch between geometric and kinodynamic planning 
    // Obstacle setup


    // State space setup
    auto space(std::make_shared<ob::SE3StateSpace>());

    ob::RealVectorBounds bounds(3);
    // Set the bounds for the state space
    bounds.setLow(0, 0);
    bounds.setHigh(0, 15);
    bounds.setLow(1, -10);
    bounds.setHigh(1, 10);
    bounds.setLow(2, -10);
    bounds.setHigh(1, 10);

    space->setBounds(bounds);

    // create a control space
    auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 2));

    // set the bounds for the control space
    ob::RealVectorBounds cbounds(2);
    // Bounds for u1
    cbounds.setLow(0, -0.3);
    cbounds.setHigh(0, 0.3);
    // Bounds for u2
    cbounds.setLow(1, -0.5);
    cbounds.setHigh(1, 0.5);

    // Set the control bounds
    cspace->setBounds(cbounds);

    // define a simple setup class
    // oc::SimpleSetup ss(cspace); // For controls problem
    og::SimpleSetup ss(space); // For geometric problem

    // set state validity checking for this space, kinodynamic
    // oc::SpaceInformation* si = ss.getSpaceInformation().get();
    // ss.setStateValidityChecker(
    //     [si](const ob::State* state) { return isStateValid(si, state); });

    // set state validity checking for this space, geometric
    ob::SpaceInformation* si = ss.getSpaceInformation().get();
    ss.setStateValidityChecker(
        [si](const ob::State* state) { return isStateValid(si, state); });

    // Use the ODESolver to propagate the system. Call PostIntegration
    // when integration has finished to normalize the orientation values.
    // Comment this out for geometric planning
    // auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(ss.getSpaceInformation(), &DynamicsODE));
    // ss.setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, &PostIntegration));

    ob::ScopedState<ob::SE3StateSpace> start(space);
    start->setXYZ(0.0, 0.0, 0.0);
    start->rotation().setIdentity();

    ob::ScopedState<ob::SE3StateSpace> goal(space);
    goal->setXYZ(15.0, 0.5, 0.0);
    goal->rotation().setIdentity();

    ss.setStartAndGoalStates(start, goal, 0.05);

    ss.setup();

    // Time to find a solution
    const float solve_time = 5;

    // Solve the planning problem
    ob::PlannerStatus solved = ss.solve(solve_time);

    if (solved)
        {
        std::cout << "Found solution!" << std::endl;
        std::ofstream file("../solution.txt");
        ss.getSolutionPath().printAsMatrix(file);
        file.close();
        }
    else
        std::cout << "No solution found" << std::endl;
    }

int main(int /*argc*/, char** /*argv*/)
    {
    fcl::Vector3f obs_translation(5.0, 0.0, 0.0);
    fcl::Quaternionf obs_rotation(0, 0, 0, 1);
    obstacle_collision_object->setTransform(obs_rotation, obs_translation);
    //const auto box = CollisionBox(2.0, 1.0, 1.0);
    planWithSimpleSetup();

    return 0;
    }
