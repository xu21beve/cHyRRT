// Adapted from OMPL Planner template: https://ompl.kavrakilab.org/newPlanner.html

#include <ompl/base/MotionValidator.h>
#include <ompl/base/Planner.h>
#include <ompl/util/RandomNumbers.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/base/GoalTypes.h>
#include <ompl/control/ODESolver.h>
#include "include/hyRRT.h"
#include "ompl/control/spaces/RealVectorControlSpace.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include <ompl/control/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/base/spaces/TimeStateSpace.h>
#include <ompl/base/goals/GoalState.h>

namespace base = ompl::base;
namespace tools = ompl::tools;

ompl::geometric::hyRRT::hyRRT(const base::SpaceInformationPtr &si)
    : base::Planner(si, "hyRRT")
{
    specs_.approximateSolutions = false;
    specs_.directed = true;
}

ompl::geometric::hyRRT::~hyRRT(void)
{
    // free any allocated memory
    freeMemory();
}

double distanceFunc(base::State *state1, base::State *state2)
{
    double dist = 0;
    dist = sqrt(pow(state1->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] - state2->as<ompl::base::RealVectorStateSpace::StateType>()->values[0], 2) + pow(state1->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] - state2->as<ompl::base::RealVectorStateSpace::StateType>()->values[1], 2));
    return fabs(dist);
}

void ompl::geometric::hyRRT::defineModel(std::function<base::State(base::State *, double, double)> *jumpFunc,
                                                        std::function<base::State(double, base::State *, double)> *flowFunc,
                                                        std::function<bool(base::State *, base::State *)> collisionFunc,
                                                        std::function<double(base::State *, base::State *)> distanceFunc
    ) {
        // jumpPropagation = jumpFunc;
        // flowPropagation = flowFunc;
        // collisionChecker = collisionFunc;
        // distanceFunc = distanceFunc;
    }

base::PlannerStatus ompl::geometric::hyRRT::solve(const base::PlannerTerminationCondition &ptc)
{
    // make sure the planner is configured correctly; ompl::base::Planner::checkValidity
    // ensures that there is at least one input state and a ompl::base::Goal object specified
    checkValidity();

    base::Goal *goal = pdef_->getGoal().get();

    double jumpTimeInterval = 0.01;   // TODO: Add in definition
    double flowTimeMax = 1;           // TODO: Add in in definition
    double maxTotalPathDistance = 20; // TODO: Add in in definition
    double minStepDistance = 1e-6;    // TODO: Add in in definition
    double flowTimeSteps = 0.001;     // TODO: Add in in definition
    double dist = 0.1;

    // Create a vector of input values
    std::vector<double> inputValues;
    for (double i = 0.0; i <= 5; i += 0.1)
    {
        inputValues.push_back(i);
    }

    // get input states with PlannerInputStates helper, pis_
    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        motion->root = motion->state;
        // Add start motion to the tree
        nn_->add(motion);
        // cout << "line 73 called" << endl;
    }

    srand((unsigned)std::time(NULL));

    // periodically check if ptc() returns true.
    // if it does, terminate planning.
    int i = 0;
    while (ptc() == false) //  && i < 2000              //  && tFlowTracking < 80 && tJumpTracking < 100
    {
        i++;
        auto *randommotion = new Motion(si_);
        base::State *rstate = randommotion->state;

        // Calculate random flow time max, only used when flowing
        double random = rand();
        double randomFlowTimeMax = random / RAND_MAX * flowTimeMax;

        base::StateSamplerPtr sampler = si_->allocStateSampler();
        // Replace later with the ompl sampler, for now leave as custom
        sampler->sampleUniform(rstate); // TODO: Replace with custom samplers? Or try to set bounds for each level.

        auto *new_motion = new Motion(si_);
        auto *new_new_motion = new Motion(si_);
        new_motion->parent = nn_->nearest(randommotion);
        new_new_motion->parent = nn_->nearest(randommotion);
        // find the pointer address of new_new_motion->parent's duplicate in nn_printing_ and set new_new_motion->parent to that pointer
        for (int i = 0; i < nn_printing_vector_.size(); i++)
        {
            base::State *nn_state = nn_printing_vector_.at(i)->state;
            base::State *new_new_state = new_new_motion->parent->state;

            // Check if all values of the state are the same
            if (nn_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] == new_new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] && nn_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] == new_new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] && nn_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[2] == new_new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[2] && nn_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[3] == new_new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[3] && nn_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[4] == new_new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[4] && nn_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[5] == new_new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[5])
            {
                new_new_motion->parent = nn_printing_vector_.at(i);
                break;
            }
        }

        std::vector<Motion *> motion_tree;
        std::vector<Motion *> motion_tree_printable;
        std::vector<Motion *> traj;
        // std::vector<base::State *> state_tree(int(randomFlowTimeMax / flowTimeSteps)); // Maximum new states is all flows

        bool priority = checkPriority(new_motion->parent->state);
        // checkPriority(new_new_motion->parent->state);
        double tFlow = 0;
        int flows = 0;

        // Track number of steps, alternative to motion_tree.size()
        int steps = 0;

        // Create a random number generator //TODO: (1) change to interval uniform sampling for n dimensions, getting the intervals for each dimension from the user. (2) Get a true uniform continuous sample
        std::random_device rd;
        std::mt19937 gen(rd());

        // Create a uniform distribution to select a random index, return the index of a random value between specified interval
        std::uniform_int_distribution<> dis(0, inputValues.size() - 1);

        // Select a random value from the vector
        double u = inputValues[dis(gen)];

        // Run either flow or jump propagation until propagation step is completed or collision occurs  // TODO: Add other case for both, then choose randomly, using user-defined threshold for relative probabilities
        switch (priority)
        {
        case false: // Flow
            while (tFlow < randomFlowTimeMax && checkPriority(new_motion->parent->state) == false)
            {
                Motion *new_motion_ptr = new Motion(si_);

                // Find new state with flow propagation
                base::State *new_state = flowPropagation(-9.81, new_motion->parent->state, flowTimeSteps);

                // Update both nearest neighbor and printing tree with new state
                si_->copyState(new_motion->state, new_state);
                si_->copyState(new_new_motion->state, new_state);

                // Use ptr to modify the new_motion->state directly if collision occurs
                bool collision = collisionChecker(new_motion->parent->state, new_motion->state);

                // If the motion encounters no collision with jump sets, then add the successful motion to the temporary trees
                if (!collision)                                                                  
                {                                                                                
                    motion_tree.push_back(new_motion);

                    si_->copyState(new_motion->parent->state, motion_tree.at(flows)->state); // Set parent state ofr next iterations
                    si_->copyState(new_new_motion->parent->state, motion_tree.at(flows)->state);

                    // Initialize new motion pointer to add to tree
                    new_motion_ptr = new_new_motion;
                    motion_tree_printable.push_back(new_motion_ptr);

                    // Add new motion to printing tree
                    nn_printing_vector_.push_back(new_motion_ptr);
                    new_new_motion = new Motion(si_);

                    // set new parent motion of temporary motion to previously added motion
                    new_new_motion->parent = motion_tree_printable.at(steps);
                    si_->copyState(new_new_motion->parent->state, motion_tree_printable.at(steps)->state); // Set parent state ofr next iterations
                    si_->copyState(new_new_motion->state, new_state);
                }
                else
                {
                    // push back final motion with collision as last point
                    motion_tree.push_back(new_motion);

                    // set parent motions for next iteration to last motion in tree
                    si_->copyState(new_motion->parent->state, motion_tree.at(flows)->state);
                    si_->copyState(new_new_motion->parent->state, motion_tree.at(flows)->state);

                    // Initialize new motion pointer to add to tree
                    new_motion_ptr = new_new_motion;
                    motion_tree_printable.push_back(new_motion_ptr);
                    new_new_motion = new Motion(si_);

                    // set new parent motion of temporary motion to previously added motion
                    new_new_motion->parent = motion_tree_printable.at(steps);

                    // set parent motions for next iteration to last motion in tree
                    si_->copyState(new_new_motion->parent->state, motion_tree_printable.at(flows)->parent->state);
                    si_->copyState(new_new_motion->state, new_state);
                }

                // Check if goal is reached. If so, escape to solution checking TODO: Consider moving out to after the full propagation for computational efficiency
                if (distanceFunc(nn_printing_vector_.back()->state, pdef_->getGoal()->as<ompl::base::GoalState>()->getState()) <= 0.1)
                {
                    goto escape;
                }
                tFlow += flowTimeSteps;
                flows++;
                steps++;
                printMotion(new_motion_ptr);    //TODO: Remove later, for testing
            }

            // If collision occurs, then continue on to next iteration
            if (collisionChecker(new_motion->parent->state, new_motion->state))
            {
                // Update parent state of new_motion to the last state in the motion tree
                si_->copyState(new_motion->parent->state, motion_tree.back()->state);

                // Create new state with jump propagation
                base::State *new_state = jumpPropagation(new_motion->parent->state, jumpTimeInterval, u);

                // Update new_motion state to new state
                si_->copyState(new_motion->state, new_state);
                si_->copyState(new_new_motion->state, new_state);
                motion_tree.push_back(new_motion);
                
                // Update parent for next iteration
                si_->copyState(new_motion->parent->state, motion_tree.back()->state);
                si_->copyState(new_new_motion->parent->state, motion_tree.back()->state);

                // Initialize intermediary motion pointer to add to tree
                Motion *new_motion_ptr = new Motion(si_);

                // Initialize new motion pointer to add to tree
                new_motion_ptr = new_new_motion;
                motion_tree_printable.push_back(new_motion_ptr);

                nn_printing_vector_.push_back(new_motion_ptr);
                new_new_motion = new Motion(si_);

                // set new parent motion of temporary motion to previously added motion
                new_new_motion->parent = motion_tree_printable.back();

                // si_->copyState(new_new_motion->parent->state, motion_tree_printable.at(steps)->state); // Set parent state ofr next iterations
                si_->copyState(new_new_motion->state, new_state);

                // set new parent motion of temporary motion to previously added motion
                priority = true;
                steps++;
                printMotion(new_motion_ptr);    //TODO: Remove later, for testing
                continue;
            }
            nn_printing_->list(traj);
            break;

        case true: // Jump (use jumpPropagation)
            base::State *new_state = jumpPropagation(new_motion->parent->state, jumpTimeInterval, u);

            si_->copyState(new_motion->state, new_state);
            motion_tree.push_back(new_motion);

            // Update parent for next iteration
            si_->copyState(new_motion->parent->state, motion_tree.at(flows)->state); // Set parent state ofr next iterations
            si_->copyState(new_new_motion->parent->state, motion_tree.at(flows)->state); // Set parent state ofr next iterations

            // Initialize intermediary motion pointer to add to tree
            si_->copyState(new_new_motion->state, new_state);

            // Initialize new motion pointer to add to tree
            Motion *new_motion_ptr = new Motion(si_);
            new_motion_ptr = new_new_motion;
            motion_tree_printable.push_back(new_motion_ptr);
            nn_printing_vector_.push_back(new_motion_ptr);
            new_new_motion = new Motion(si_);

            // set new parent motion of temporary motion to previously added motion
            new_new_motion->parent = motion_tree_printable.at(steps);
            si_->copyState(new_new_motion->parent->state, motion_tree_printable.at(steps)->state); // Set parent state ofr next iterations
            si_->copyState(new_new_motion->state, new_state);
            steps++; // set new parent motion of temporary motion to previously added motion
            printMotion(new_motion_ptr);    //TODO: Remove later, for testing
            break;
        }

    escape:
        //TODO: Move to individual propagation steps
        // if (distanceFunc(motion_tree.front()->parent->state, motion_tree.back()->state) > minStepDistance) // removed: checkPriority(new_motion->state) == priority && TODO: Revise this to make sure it works for cases other than bouncing ball
        // {                                                                                                  // check to make sure new state is not in unsafe set (TODO: Add later) and at least a minimum distance away from root state
        nn_->add(motion_tree); // TODO: Add back in distance function verification
 
        // After done propagating, append the pointers in nn_printing_vector_ to nn_printing_

        // Berkeley Collision Checker
        // No collision checker for now
        // If collision, then continue on to next iteration

        // print nn_printing_ as a matrix
        if (distanceFunc(nn_printing_vector_.back()->state, pdef_->getGoal()->as<ompl::base::GoalState>()->getState()) <= 0.1) // finalDistance <= 0.01 || TODO: Add back in later
        {

            nn_printing_->add(nn_printing_vector_);
            nn_printing_->add(nn_printing_vector_.back());

            vector<Motion *> trajectory;
            nn_printing_->list(trajectory);
            std::vector<Motion *> mpath;

            double finalDistance = distanceFunc(trajectory.back()->state, pdef_->getGoal()->as<ompl::base::GoalState>()->getState());

            Motion *solution = nn_printing_vector_.back();

            // Construct the path from the goal to the start by following the parent pointers
            while (solution != nullptr)
            {
                mpath.push_back(solution);
                solution = solution->parent;
            }

            // Create a new path object to store the solution path
            auto path(std::make_shared<PathGeometric>(si_));

            // Reserve space for the path states
            path->getStates().reserve(mpath.size());

            // Add the states to the path in reverse order (from start to goal)
            for (int i = mpath.size() - 1; i >= 0; --i)
                path->append(mpath[i]->state);

            // Add the solution path to the problem definition
            pdef_->addSolutionPath(path, finalDistance > 0.0, finalDistance, getName());
            pdef_->getSolutionPath()->as<ompl::geometric::PathGeometric>()->printAsMatrix(std::cout);

            // Return a status indicating that an exact solution has been found
            if (finalDistance > 0.0)
                return base::PlannerStatus::APPROXIMATE_SOLUTION;
            else
                return base::PlannerStatus::EXACT_SOLUTION;
        }
    }
    return base::PlannerStatus::INFEASIBLE; // if failed to find a path within specified max number of iteratioins, then path generation has failed
}

double sampleInJump()
{ // TODO: Add in later
    double random = rand();
    return random / RAND_MAX * 5; // for speed
}

bool ompl::geometric::hyRRT::checkPriority(base::State *state)
{
    double velocity = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];
    double acceleration_cur = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[2];
    double pos_cur = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];

    if (pos_cur <= 0 && velocity <= 0)
    {
        return true;
    }
    else if (pos_cur >= -0.1)
    {
        return false;
    }
    else
    {
        return false;
    }
} // Random selection if in intersection --- 0 if jump, 1 if flow

base::State *ompl::geometric::hyRRT::flowPropagation(double input_accel, base::State *x_cur, double tFlow) const
{
    double velocity = x_cur->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];
    double acceleration_cur = x_cur->as<ompl::base::RealVectorStateSpace::StateType>()->values[2];
    double pos_cur = x_cur->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
    double tFlow_cur = x_cur->as<ompl::base::RealVectorStateSpace::StateType>()->values[3];
    double tJump_cur = x_cur->as<ompl::base::RealVectorStateSpace::StateType>()->values[4];

    double v = velocity + (input_accel)*tFlow;                               // v = v0 + at
    double x = pos_cur + velocity * tFlow + input_accel * pow(tFlow, 2) / 2; // x = v0 * t + 1/2(at^2)
    double tFlow_new = tFlow_cur + tFlow;

    base::State *new_state = si_->allocState();
    new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = x;
    new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = v;
    new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[2] = acceleration_cur; // Flow should not be increasing the acceleration, TODO: schange this later for clarity (make starting acceleration -9.81)
    new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[3] = tFlow_cur + tFlow;
    new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[4] = x_cur->as<ompl::base::RealVectorStateSpace::StateType>()->values[4]; // No change in jump time
    new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[5] = 0.0;                                                                 // Set control input to 0 for flow
    return new_state;
}

std::vector<double> solveTimeWhenXIsNegativeOne(double x0, double v0, double a)
{
    std::vector<double> solutions;

    double discriminant = v0 * v0 - 4 * 0.5 * a * (x0 + 1);

    // Check if the discriminant is greater than or equal to 0
    // If it is less than 0, the equation has no real solutions
    if (discriminant >= 0)
    {
        double t1 = (-v0 + std::sqrt(discriminant)) / (a);
        double t2 = (-v0 - std::sqrt(discriminant)) / (a);

        // Only consider the solutions where time is positive
        if (t1 >= 0)
            solutions.push_back(t1);
        if (t2 >= 0)
            solutions.push_back(t2);
    }

    return solutions;
}

bool ompl::geometric::hyRRT::collisionChecker(base::State *parent, base::State *x_cur)
{
    base::State *collisionState = nullptr;
    double time;
    std::vector<double> state = {x_cur->as<ompl::base::RealVectorStateSpace::StateType>()->values[0], x_cur->as<ompl::base::RealVectorStateSpace::StateType>()->values[1], x_cur->as<ompl::base::RealVectorStateSpace::StateType>()->values[2]};

    if (state[0] <= 0)
    /**  || TODO: Add back?
        (parent->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] < 0 &&
        state[1] > 0)*/
    {
        double v0 = parent->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];
        double a = -9.81;
        double x0 = parent->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
        // Solve for time of collision using the quadratic formula, from the kinematic equation: x = x0 + v0t + 1/2at^2
        time = solveTimeWhenXIsNegativeOne(x0, v0, a)[0];
        x_cur->as<ompl::base::RealVectorStateSpace::StateType>()->values[2] = a;                                       // Leave acceleration changes to jumpPropagation
        x_cur->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = v0;                                      // Leave velocity changes to jumpPropagation
        x_cur->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = -std::numeric_limits<double>::epsilon(); // TODO: Bug here, need to fix (can't just set to -0.1)
        return true;
    }
    else
        return false;
}

base::State *ompl::geometric::hyRRT::jumpPropagation(base::State *x_cur, double tJump, double u) const
{
    base::State *new_state = si_->allocState();

    double velocity = -0.8 * x_cur->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];
    double acceleration = x_cur->as<ompl::base::RealVectorStateSpace::StateType>()->values[2];
    double pos_cur = x_cur->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
    double tFlow_cur = x_cur->as<ompl::base::RealVectorStateSpace::StateType>()->values[3];
    double tJump_cur = x_cur->as<ompl::base::RealVectorStateSpace::StateType>()->values[4];

    double v = velocity;
    double x = pos_cur;
    double tJump_new = tJump_cur + tJump;

    new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = x;
    new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = v + u;
    new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[2] = acceleration;
    new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[3] = tFlow_cur; // No change in flow time
    new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[4] = tJump_new; // No change in jump time
    new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[5] = u;         // Append control input to state

    return new_state;
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

    if (!nn_printing_)
        nn_printing_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_printing_->setDistanceFunction([this](const Motion *a, const Motion *b)
                                      { return distanceFunction(a, b); });
}

void ompl::geometric::hyRRT::freeMemory()
{
    if (nn_printing_)
    {
        std::vector<Motion *> motions;
        nn_printing_->list(motions);
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

int main()
{
    // Construct the robot state space in which we're planning. We're
    // planning in [0,1]x[0,1], a subset of R^2.
    base::StateSpacePtr space(new base::RealVectorStateSpace(6)); // Adding in third dimension for jump time (change to 3 later)

    // Set the bounds of space to be in [0,1].
    space->as<base::RealVectorStateSpace>()->setBounds(-100, 100);

    // Construct a space information instance for this state space
    base::SpaceInformationPtr si(new base::SpaceInformation(space));

    // Set the object used to check which states in the space are valid
    // si->setStateValidityChecker(base::StateValidityCheckerPtr(new ValidityChecker(si))); //TODO: Add later

    si->setup();

    // Set our robot's starting state to be the bottom-left corner of
    // the environment, or (0,0).
    base::ScopedState<> start(space);
    start->as<base::RealVectorStateSpace::StateType>()->values[0] = 14;
    start->as<base::RealVectorStateSpace::StateType>()->values[1] = 0.0;
    start->as<base::RealVectorStateSpace::StateType>()->values[2] = -9.81;
    start->as<base::RealVectorStateSpace::StateType>()->values[3] = 0.0;
    start->as<base::RealVectorStateSpace::StateType>()->values[4] = 0.0;
    start->as<base::RealVectorStateSpace::StateType>()->values[5] = 0;

    base::ScopedState<> secondStart(space);
    secondStart->as<base::RealVectorStateSpace::StateType>()->values[0] = 1;
    secondStart->as<base::RealVectorStateSpace::StateType>()->values[1] = 0.0;
    secondStart->as<base::RealVectorStateSpace::StateType>()->values[2] = -9.81;
    secondStart->as<base::RealVectorStateSpace::StateType>()->values[3] = 0.0;
    secondStart->as<base::RealVectorStateSpace::StateType>()->values[4] = 0.0;
    secondStart->as<base::RealVectorStateSpace::StateType>()->values[5] = 0;

    // Set our robot's goal state to be the top-right corner of the
    // environment, or (1,1).
    base::ScopedState<> goal(space);
    goal->as<base::RealVectorStateSpace::StateType>()->values[0] = 10;
    goal->as<base::RealVectorStateSpace::StateType>()->values[1] = 0.0;
    goal->as<base::RealVectorStateSpace::StateType>()->values[2] = -9.81;

    // Create a problem instance
    base::ProblemDefinitionPtr pdef(new base::ProblemDefinition(si));

    // Set the start and goal states
    pdef->setStartAndGoalStates(start, goal, 0.1);
    // pdef->addStartState(secondStart);
    // Construct our optimizing planner using the RRTstar algorithm.
    ompl::geometric::hyRRT optimizingPlanner(si);

    // Set the problem instance for our planner to solve
    optimizingPlanner.setProblemDefinition(pdef);
    optimizingPlanner.setup();

    // attempt to solve the planning problem within one second of
    // planning time
    base::PlannerStatus solved = optimizingPlanner.solve(base::timedPlannerTerminationCondition(20));
    cout << "solution status: " << solved << endl;
}