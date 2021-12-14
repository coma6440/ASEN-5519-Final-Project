/* Standard Libraries */
#include <iostream>
#include <valarray>
#include <limits>
#include <fstream>
#include <time.h>
#include <random>

/* OMPL General */
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/config.h>
#include <ompl/geometric/SimpleSetup.h>

/* OMPL Geometric */
#include <ompl/base/goals/GoalSpace.h>

/* OMPL Control */
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/terminationconditions/CostConvergenceTerminationCondition.h>

/* FCL */
#include <fcl/fcl.h>

/* YAML */
#include <yaml-cpp/yaml.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace oc = ompl::control;

void DefineProblem(ob::StateSpacePtr& stateSpace, ob::StateSpacePtr& goalSpace);

void GetEnvironment(std::string ws_name, std::vector<std::shared_ptr<fcl::CollisionObjectf>>& obstacles, std::shared_ptr<fcl::CollisionObjectf>& robot);

std::shared_ptr<fcl::CollisionObjectf> CollisionBox(float l, float w, float h, fcl::Vector3f translation = fcl::Vector3f(0, 0, 0), fcl::Quaternionf rotation = fcl::Quaternionf(0, 0, 0, 1));

bool isStateValid(ob::SpaceInformation* si, const ob::State* state, std::vector<std::shared_ptr<fcl::CollisionObjectf>> obstacles, std::shared_ptr<fcl::CollisionObjectf> robot);

bool isStateValid(oc::SpaceInformation* si, const ob::State* state, std::vector<std::shared_ptr<fcl::CollisionObjectf>> obstacles, std::shared_ptr<fcl::CollisionObjectf> robot);

void saveGeometricPath(og::SimpleSetup ss, std::string fname);

void saveControlPath(oc::PathControl path, std::string fname);

oc::PathControl makePath(const ob::SpaceInformationPtr& si, oc::PathControl prevPath, const unsigned int start, const unsigned int end);

void DynamicsODE(const oc::ODESolver::StateType& x, const oc::Control* control, oc::ODESolver::StateType& xdot);

void PostIntegration(const ob::State* /*state*/, const oc::Control* /*control*/, const double /*duration*/, ob::State* result);

ob::OptimizationObjectivePtr getThresholdPathLengthObj(const ob::SpaceInformationPtr& si);

unsigned int findPlanTime(oc::PathControl pathSegment, double plan_time, double& actual_time);

oc::PathControl getSegment(const ob::SpaceInformationPtr& si, oc::PathControl path, unsigned int start, unsigned int end);

void saveCost(std::string fname, unsigned int count, ob::Cost segmentCost, ob::Cost totalCost);