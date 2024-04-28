// Adapted from OMPL Planner template: https://ompl.kavrakilab.org/newPlanner.html

#include <ompl/base/Planner.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/base/GoalTypes.h>
#include <ompl/control/ODESolver.h>
#include "include/hyRRT.h"
#include "ompl/control/spaces/RealVectorControlSpace.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include <ompl/control/SimpleSetup.h>
#include <ompl/base/goals/GoalState.h>
// #include "CommonMath/Trajectory.hpp"
// #include "include/polyFit.hpp"
// #include "quartic.cpp"
// #include "CommonMath/RectPrism.hpp"
// #include "CommonMath/ConvexObj.hpp"
#include <list>

namespace base = ompl::base;
namespace tools = ompl::tools;

using namespace std::chrono;

ompl::geometric::hyRRT::hyRRT(const base::SpaceInformationPtr &si_)
    : base::Planner(si_, "hyRRT")
{
    specs_.approximateSolutions = false;
    specs_.directed = true;
}

ompl::geometric::hyRRT::~hyRRT(void)
{
    // free any allocated memory
    freeMemory();
}

void ompl::geometric::hyRRT::init_tree()
{
    // get input states with PlannerInputStates helper, pis_
    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        motion->root = motion->state;
        // Add start motion to the tree
        nn_->add(motion);
    }
}

void ompl::geometric::hyRRT::random_sample(Motion *random_motion, std::mt19937 gen)
{
    base::State *rstate = random_motion->state;

    base::StateSamplerPtr sampler = si_->allocStateSampler();
    // Replace later with the ompl sampler, for now leave as custom
    sampler->sampleUniform(rstate);
}

bool checkBounds(base::State *state)
{
    std::vector<double> x_cur = {state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0], state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1], state->as<ompl::base::RealVectorStateSpace::StateType>()->values[2], state->as<ompl::base::RealVectorStateSpace::StateType>()->values[3]};
    if (x_cur[0] < 0.5 || x_cur[0] > 6 || x_cur[1] < 0 || x_cur[1] > 7)
        return false;
    return true;
}

void push_back_state(std::vector<std::vector<double>> *states_list, base::State *state)
{
    ompl::base::RealVectorStateSpace::StateType *state_vec = state->as<ompl::base::RealVectorStateSpace::StateType>();
    std::vector<double> new_state;
    new_state.push_back(state_vec->values[0]);
    new_state.push_back(state_vec->values[1]);
    new_state.push_back(0.0);
    new_state.push_back(state_vec->values[6]);
    states_list->push_back(new_state);
}

base::PlannerStatus ompl::geometric::hyRRT::solve(const base::PlannerTerminationCondition &ptc)
{
    auto start = high_resolution_clock::now();

    // make sure the planner is configured correctly; ompl::base::Planner::checkValidity
    // ensures that there is at least one input state and a ompl::base::Goal object specified
    checkValidity();
    checkAllParametersSet();

    base::Goal *goal = pdef_->getGoal().get();

    double totalCollisionTime = 0.0;
    double totalCollisionPtTime = 0.0;
    int polyTime = 0;
    double tsCollision = 0.0;
    int totalCollisions = 0;
    double totalIntTime = 0.0;
    unsigned int tFIndex = si_->getStateDimension() - 4;
    unsigned int tJIndex = si_->getStateDimension() - 3;

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        motion->root = motion->state; // this state is the root of a tree
        // Add start motion to the tree
        nn_->add(motion);
    }

    // Create a vector of x input values
    std::vector<double> inputValues1;
    for (double i = -0.5; i <= 1; i += 1.5 / 20)
    {
        inputValues1.push_back(i);
    }

    // Create a vector of y input values
    std::vector<double> inputValues2;
    for (double i = -1.0; i <= 1; i += 2.0 / 20.0)
    {
        inputValues2.push_back(i);
    }

    init_tree();

    // Create random generator (by default, uniform)
    std::random_device rd;
    std::mt19937 gen(rd());

    auto *random_motion = new Motion(si_);

    // periodically check if ptc() returns true.
    // if it does, terminate planning.
    int i = 0;
    while (!ptc())
    {
        random_sample(random_motion, gen);

        auto *new_motion = new Motion(si_);
        new_motion->parent = nn_->nearest(random_motion);

        bool priority = jumpSet_(new_motion->state);
        double tFlow = 0;

        double random = rand();
        double randomFlowTimeMax = random / RAND_MAX * Tm_;

        // Create a uniform distribution to select a random index, return the index of a random value between specified interval
        // TODO: Change this to OMPL sampler, and use the specified sampler --- determine the number of samplers using the number of input ranges specified by user
        std::uniform_real_distribution<> dis1(0, inputValues1.size() - 1);
        std::uniform_real_distribution<> dis2(0, inputValues2.size() - 1);

        // Select a random value from the vector
        double u1 = inputValues1[dis1(gen)];
        double u2 = inputValues2[dis2(gen)];

        base::State *previous_state = si_->allocState();
        auto *parent_motion = nn_->nearest(random_motion);
        si_->copyState(previous_state, parent_motion->state);
        auto *collision_parent_motion = nn_->nearest(random_motion);

        bool collision = false;
        std::vector<std::vector<double>> *propStepStates = new std::vector<std::vector<double>>;
        push_back_state(propStepStates, previous_state);

        // Run either flow or jump propagation until propagation step is completed or collision occurs  // TODO: Add other case for both, then choose randomly, using user-defined threshold for relative probabilities
        switch (priority)
        {
        case false: // Flow
            while (tFlow < randomFlowTimeMax && flowSet_(new_motion->state))
            {
                collision = false;
                tFlow += flowStepLength_;

                // Find new state with flow propagation
                base::State *new_state = si_->allocState();

                new_state = this->flowPropagation_(std::vector<double>{u1, u2}, previous_state, flowStepLength_, new_state);
                new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[tFIndex] += flowStepLength_;

                push_back_state(propStepStates, new_state);

                std::vector<double> startPoint = motion_to_vector(parent_motion->state);
                std::vector<double> endPoint = motion_to_vector(new_state);

                double ts = startPoint[tFIndex];
                double tf = endPoint[tFIndex];

                // Place collision checker here

                si_->copyState(previous_state, new_state);

                if (!checkBounds(new_state))
                {
                    goto escape;
                }

                collision = collisionChecker_(propStepStates, jumpSet_, ts, tf, new_state, tFIndex);

                // If the motion encounters no collision with jump sets, then add the successful motion to the temporary trees
                if (!collision)
                {
                    si_->copyState(new_motion->state, new_state); // Set parent state for next iterations
                }
                else
                {
                    totalCollisions++;
                    si_->copyState(new_motion->state, new_state); // get collision state
                }

                if (tFlow >= randomFlowTimeMax || collision)
                {
                    auto *motion = new Motion(si_);
                    si_->copyState(motion->state, new_state);
                    motion->parent = parent_motion;

                    si_->freeState(new_state);
                    collision_parent_motion = motion;

                    if (collision)
                        goto jump;
                    else
                        nn_->add(motion);
                    break;
                }
                pdef_->getGoal()->as<ompl::base::GoalState>()->getState();
                // Check if goal is reached. If so, escape to solution checking TODO: Consider moving out to after the full propagation for computational efficiency
                if (distanceFunc_(new_motion->state, pdef_->getGoal()->as<ompl::base::GoalState>()->getState()) <= tolerance_)
                {
                    goto escape;
                }
            }
            break;

        case true: // Jump (use jumpPropagation)
            collision_parent_motion = parent_motion;
        jump:
            base::State *new_state = si_->allocState();
            new_state = this->jumpPropagation_(new_motion->state, 0, new_state); // changed from previous_state to new_motion->state
            new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[tJIndex]++;

            if (!checkBounds(new_state))
            {
                goto escape;
            }

            si_->copyState(new_motion->state, new_state);

            auto *motion = new Motion(si_);
            si_->copyState(motion->state, new_state);
            motion->parent = collision_parent_motion;

            nn_->add(motion);
            nn_->add(collision_parent_motion);
            si_->freeState(new_state);

            break;
        }

    escape:
        si_->freeState(previous_state);
        if (distanceFunc_(new_motion->state, pdef_->getGoal()->as<ompl::base::GoalState>()->getState()) <= 0.1) // finalDistance <= 0.01 || TODO: Add back in later
        {
            vector<Motion *> trajectory;
            nn_->list(trajectory);
            std::vector<Motion *> mpath;

            double finalDistance = distanceFunc_(trajectory.back()->state, pdef_->getGoal()->as<ompl::base::GoalState>()->getState());
            Motion *solution = new_motion;

            // Construct the path from the goal to the start by following the parent pointers
            while (solution != nullptr)
            {
                mpath.push_back(solution);
                solution = solution->parent;
            }

            // Create a new path object to store the solution path
            auto path(std::make_shared<PathGeometric>(si_));
            trajectoryMatrix_ = {};

            // Reserve space for the path states
            path->getStates().reserve(mpath.size());

            // Add the states to the path in reverse order (from start to goal)
            for (int i = mpath.size() - 1; i >= 0; --i) {
                path->append(mpath[i]->state);
                trajectoryMatrix_.push_back(mpath[i]);
            }

            // Add the solution path to the problem definition
            pdef_->addSolutionPath(path, finalDistance > 0.0, finalDistance, getName());
            pdef_->getSolutionPath()->as<ompl::geometric::PathGeometric>()->printAsMatrix(std::cout);

            // To get the value of duration use the count()
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - start);
            cout << "total duration (microseconds): " << duration.count() << endl;

            // Return a status indicating that an exact solution has been found
            if (finalDistance > 0.0)
                return base::PlannerStatus::APPROXIMATE_SOLUTION;
            else
                return base::PlannerStatus::EXACT_SOLUTION;
        }
    }
    return base::PlannerStatus::INFEASIBLE; // if failed to find a path within specified max number of iteratioins, then path generation has failed
}

void ompl::geometric::hyRRT::clear(void)
{
    Planner::clear();
    // clear the data structures here
}

// optional, if additional setup/configuration is needed, the setup() method can be implemented
void ompl::geometric::hyRRT::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);
    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b)
                             { return distanceFunction(a, b); });
}

void ompl::geometric::hyRRT::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion *> motions;
        nn_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }
    }
}

void ompl::geometric::hyRRT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    vector<Motion *> motions;
    if (nn_)
        nn_->list(motions);

    if (lastGoalMotion_ != nullptr)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motion->state));
        else
            data.addEdge(base::PlannerDataVertex(motion->parent->state), base::PlannerDataVertex(motion->state));
    }
}