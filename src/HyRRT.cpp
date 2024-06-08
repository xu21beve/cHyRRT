// Adapted from OMPL Planner template:
// https://ompl.kavrakilab.org/newPlanner.html

#include "../HyRRT.h"
#include "ompl/base/Planner.h"
#include "ompl/base/goals/GoalState.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/tools/config/SelfConfig.h"
#include <list>

namespace base = ompl::base;
namespace tools = ompl::tools;

using namespace std::chrono;

ompl::geometric::HyRRT::HyRRT(const base::SpaceInformationPtr &si_)
    : base::Planner(si_, "HyRRT") {
  specs_.approximateSolutions = false;
  specs_.directed = true;
}

ompl::geometric::HyRRT::~HyRRT() {
  // free any allocated memory
  freeMemory();
}

void ompl::geometric::HyRRT::initTree(void) {
  // get input states with PlannerInputStates helper, pis_
  while (const base::State *st = pis_.nextStart()) {
    auto *motion = new Motion(si_);
    si_->copyState(motion->state, st);
    motion->root = motion->state;
    // Add start motion to the tree
    nn_->add(motion);
  }
}

void ompl::geometric::HyRRT::randomSample(Motion *randomMotion,
                                          std::mt19937 gen) {
  base::State *rstate = randomMotion->state;

  base::StateSamplerPtr sampler = si_->allocStateSampler();
  // Replace later with the ompl sampler, for now leave as custom
  sampler->sampleUniform(rstate);
}

base::PlannerStatus
ompl::geometric::HyRRT::solve(const base::PlannerTerminationCondition &ptc) {
  // Start the timer for measuring the total duration
  auto start = high_resolution_clock::now();

  // ===== Initialization =====
  // Make sure the planner is configured correctly
  // Ensures that there is at least one input state and a goal object specified
  checkValidity();
  checkAllParametersSet();

  // Get the goal object
  base::Goal *goal = pdef_->getGoal().get();

  // Initialize variables for telemetry
  double totalCollisionTime = 0.0;
  int totalCollisions = 0;
  const unsigned int TF_INDEX = si_->getStateDimension() - 2;
  const unsigned int TJ_INDEX = si_->getStateDimension() - 1;

  // Add start motions to the tree
  while (const base::State *st = pis_.nextStart()) {
    auto *motion = new Motion(si_);
    si_->copyState(motion->state, st);
    motion->root = motion->state; // This state is the root of a tree
    nn_->add(motion);
  }

  initTree();

  // ===== Random State Generation =====
  // Create a random generator (by default, uniform)
  std::random_device rd;
  std::mt19937 gen(rd());

  // Vectors for storing flow and jump inputs
  std::vector<double> flowInputs;
  std::vector<double> jumpInputs;

  // Allocate memory for a random motion
  auto *randomMotion = new Motion(si_);

  // ===== Main Planning Loop =====
  // Periodically check if the termination condition is met
  // If it is, terminate planning
  int iterations = 0;

nextIteration:
  while (!ptc()) {
    iterations++;
    randomSample(randomMotion, gen);

    auto *newMotion = new Motion(si_);
    newMotion->parent = nn_->nearest(randomMotion);

    double random = rand();
    double randomFlowTimeMax = random / RAND_MAX * tM_;
    double tFlow = 0;
    bool collision = false;
    bool in_jump = jumpSet_(newMotion->state);
    bool in_flow = flowSet_(newMotion->state);
    bool priority = in_jump && in_flow
                        ? random / RAND_MAX > 0.5
                        : in_jump; // If both are true, equal chance of being in
                                   // flow or jump set.

    // ===== Sample and Instantiate Parent Vertex =====
    base::State *previousState = si_->allocState();
    auto *parentMotion = nn_->nearest(randomMotion);
    si_->copyState(previousState, parentMotion->state);
    auto *collisionParentMotion = nn_->nearest(randomMotion);

    // ===== Instantiate Intermediate States and Edge =====
    std::vector<std::vector<double>> *propStepStates =
        new std::vector<std::vector<double>>;
    pushBackState(propStepStates, previousState);
    std::vector<base::State *> *intermediateStates =
        new std::vector<base::State *>;

    // ===== Run Flow or Jump Propagation =====
    switch (priority) {
    case false: // Flow
      flowInputs = sampleFlowInputs_();
      while (tFlow < randomFlowTimeMax && flowSet_(newMotion->state)) {
        // ===== Allocate and Add Intermediate State =====
        base::State *intermediateState = si_->allocState();
        si_->copyState(intermediateState, previousState);
        intermediateStates->push_back(intermediateState);

        tFlow += flowStepDuration_;

        // ===== Find New State with Flow Propagation =====
        base::State *newState = si_->allocState();
        newState = this->continuousSimulator_(flowInputs, previousState,
                                              flowStepDuration_, newState);
        newState->as<base::RealVectorStateSpace::StateType>()
            ->values[TF_INDEX] += flowStepDuration_;

        if (unsafeSet_(newState))
          goto nextIteration;

        pushBackState(propStepStates, newState);

        std::vector<double> startPoint = stateToVector(parentMotion->state);
        std::vector<double> endPoint = stateToVector(newState);

        double ts = startPoint[TF_INDEX];
        double tf = endPoint[TF_INDEX];

        si_->copyState(previousState, newState);

        // ===== Collision Checking =====
        auto collision_checking_start_time =
            high_resolution_clock::now(); // for planner statistics only
        collision = collisionChecker_(propStepStates, jumpSet_, ts, tf,
                                      newState, TF_INDEX);
        auto collision_checking_end_time = high_resolution_clock::now();
        totalCollisionTime +=
            duration_cast<microseconds>(collision_checking_end_time -
                                        collision_checking_start_time)
                .count();

        si_->copyState(newMotion->state,
                       newState); // Set parent state for next iterations

        // ===== Add Motion to Tree or Handle Collision/Goal =====
        bool inGoalSet =
            distanceFunc_(
                newMotion->state,
                pdef_->getGoal()->as<base::GoalState>()->getState()) <=
            tolerance_;

        if (tFlow >= randomFlowTimeMax || collision || inGoalSet) {
          auto *motion = new Motion(si_);
          si_->copyState(motion->state, newState);
          motion->parent = parentMotion;

          si_->freeState(newState);
          collisionParentMotion = motion;

          motion->edge = intermediateStates;

          if (inGoalSet) {
            newMotion->edge = intermediateStates;
            goto escape;
          } else if (collision) {
            totalCollisions++;
            goto jump;
          } else {
            nn_->add(motion);
          }
          break;
        }
      }
      break;

    case true: // Jump
      collisionParentMotion = parentMotion;
    jump:
      jumpInputs = sampleJumpInputs_();
      base::State *newState = si_->allocState();
      newState = this->discreteSimulator_(
          newMotion->state, jumpInputs,
          newState); // changed from previousState to newMotion->state
      newState->as<base::RealVectorStateSpace::StateType>()->values[TJ_INDEX]++;

      if (unsafeSet_(newState))
        goto nextIteration;

      si_->copyState(newMotion->state, newState);

      auto *motion = new Motion(si_);
      si_->copyState(motion->state, newState);
      motion->parent = collisionParentMotion;

      nn_->add(motion);
      nn_->add(collisionParentMotion);
      si_->freeState(newState);

      break;
    }

  escape:
    si_->freeState(previousState);
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::system_clock::now() - start);
    if (distanceFunc_(newMotion->state,
                      pdef_->getGoal()->as<base::GoalState>()->getState()) <=
        tolerance_) {
      // ===== Report Statistics and Construct Path =====
      std::cout << "total duration (microseconds): " << duration.count()
                << endl;
      std::cout << "total number of iterations (attempted propagation steps): "
                << iterations << std::endl;
      std::cout << "total collision checking duration (microseconds): "
                << totalCollisionTime << std::endl;
      return constructPath(newMotion);
    }
  }

  // ===== Path Generation Failed =====
  return base::PlannerStatus::INFEASIBLE; // If failed to find a path within the
                                          // specified max number of iterations,
                                          // then path generation has failed
}

base::PlannerStatus ompl::geometric::HyRRT::constructPath(Motion *last_motion) {
  vector<Motion *> trajectory;
  nn_->list(trajectory);
  std::vector<Motion *> mpath;

  double finalDistance =
      distanceFunc_(trajectory.back()->state,
                    pdef_->getGoal()->as<base::GoalState>()->getState());
  Motion *solution = last_motion;

  int pathSize = 0;

  // Construct the path from the goal to the start by following the parent
  // pointers
  while (solution != nullptr) {
    mpath.push_back(solution);
    if (solution->edge != nullptr) // A jump motion does not contain an edge
      pathSize += solution->edge->size() + 1; // +1 for the end state

    solution = solution->parent;
  }

  // Create a new path object to store the solution path
  auto path(std::make_shared<PathGeometric>(si_));
  trajectoryMatrix_ = {};

  // Reserve space for the path states
  path->getStates().reserve(pathSize);

  // Add the states to the path in reverse order (from start to goal)
  for (int i = mpath.size() - 1; i >= 0; --i) {
    // Append all intermediate states to the path, including starting state,
    // excluding end vertex
    if (mpath[i]->edge != nullptr) { // A jump motion does not contain an edge
      for (auto state : *mpath[i]->edge) {
        path->append(
            state); // Need to make a new motion to append to trajectory matrix
        trajectoryMatrix_.push_back(state);
      }
    }
  }
  path->append(mpath[0]->state); // append goal state to the path
  trajectoryMatrix_.push_back(mpath[0]->state);

  // Add the solution path to the problem definition
  pdef_->addSolutionPath(path, finalDistance > 0.0, finalDistance, getName());
  pdef_->getSolutionPath()->as<ompl::geometric::PathGeometric>()->printAsMatrix(
      std::cout);

  // Print the number of vertices in the trajectory
  std::cout << "total number of vertices: " << mpath.size() << endl;

  // Return a status indicating that an exact solution has been found
  if (finalDistance > 0.0)
    return base::PlannerStatus::APPROXIMATE_SOLUTION;
  else
    return base::PlannerStatus::EXACT_SOLUTION;
}

void ompl::geometric::HyRRT::clear() {
  Planner::clear();
  // clear the data structures here
}

// optional, if additional setup/configuration is needed, the setup() method can
// be implemented
void ompl::geometric::HyRRT::setup() {
  Planner::setup();
  tools::SelfConfig sc(si_, getName());
  if (!nn_)
    nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
  nn_->setDistanceFunction([this](const Motion *a, const Motion *b) {
    return ompl::geometric::HyRRT::distanceFunc_(a->state, b->state);
  });
}

void ompl::geometric::HyRRT::freeMemory(void) {
  if (nn_) {
    std::vector<Motion *> motions;
    nn_->list(motions);
    for (auto &motion : motions) {
      if (motion->state != nullptr)
        si_->freeState(motion->state);
      delete motion;
    }
  }
}

void ompl::geometric::HyRRT::getPlannerData(base::PlannerData &data) const {
  Planner::getPlannerData(data);

  vector<Motion *> motions;
  if (nn_)
    nn_->list(motions);

  if (lastGoalMotion_ != nullptr)
    data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

  for (auto &motion : motions) {
    if (motion->parent == nullptr)
      data.addStartVertex(base::PlannerDataVertex(motion->state));
    else
      data.addEdge(base::PlannerDataVertex(motion->parent->state),
                   base::PlannerDataVertex(motion->state));
  }
}