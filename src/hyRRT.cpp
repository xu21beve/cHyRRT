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
#include "CommonMath/Trajectory.hpp"
#include "include/polyFit.hpp"
#include "quartic.cpp"
#include "CommonMath/RectPrism.hpp"
#include "CommonMath/ConvexObj.hpp"
#include <list>

namespace base = ompl::base;
namespace tools = ompl::tools;

using namespace std::chrono;
    double functionName(double x)
    {
        return 2.0;
    }

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

double distanceFunc(base::State *state1, base::State *state2)
{
    double dist = 0;
    dist = sqrt(pow(state1->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] - state2->as<ompl::base::RealVectorStateSpace::StateType>()->values[0], 2) + pow(state1->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] - state2->as<ompl::base::RealVectorStateSpace::StateType>()->values[1], 2));
    return fabs(dist);
}

// TODO: Only for multicopter example, need to remove and place in example folder
bool collisionregion1(double x1, double x2, double v1, double v2)
{ // True if collision
    return x1 <= 4.5 && x1 >= 0.5 && x2 >= 1 && x2 <= 1.5;
}

bool collisionregion2(double x1, double x2, double v1, double v2)
{ // True if collision
    return x1 <= 4.5 && x1 >= 0.5 && x2 >= 1.4 && x2 <= 1.5;
}

bool collisionregion3(double x1, double x2, double v1, double v2)
{ // True if collision
    return x1 <= 4.5 && x1 >= 0.5 && x2 >= 2.5 && x2 <= 3.0;
}

bool collisionregion4(double x1, double x2, double v1, double v2)
{ // True if collision
    return x1 <= 4.5 && x1 >= 0.5 && x2 >= 2.9 && x2 <= 3;
}

bool collisionregion5(double x1, double x2, double v1, double v2)
{ // True if collision
    return x1 <= 0.5 && x1 >= 0.0 && x2 >= 1.5 && x2 <= 2.5;
}

bool collisionregion6(double x1, double x2, double v1, double v2)
{ // True if collision
    return x1 <= 4.5 && x1 >= 4.4 && x2 >= 1 && x2 <= 1.5;
}

bool collisionregion7(double x1, double x2, double v1, double v2)
{ // True if collision
    return x1 <= 4.5 && x1 >= 4.4 && x2 >= 2.5 && x2 <= 3;
}

// void ompl::geometric::hyRRT::defineModel(std::function<base::State *(base::State *, double, base::State *)> jumpFunc,
//                                          std::function<base::State *(double, double, base::State *, double, base::State *)> flowFunc,
//                                          std::function<double(base::State *, base::State *)> distanceFunc)
// {
//     this->jumpPropagation = jumpFunc;
//     this->flowPropagation = flowFunc;
//     this->distanceFunc = distanceFunc;
// }

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

using namespace CommonMath;

enum CollisionResult
{
    NoCollision = 0,
    Collision = 1,
    CollisionIndeterminable = 2,
};

struct DetailedCollisionResult
{
    CollisionResult collisionType;
    double collisionTime;
};


// TODO: Need to move to example folder
CommonMath::RectPrism leftRect = CommonMath::RectPrism(Vec3(0.25, 2, 0), Vec3(0.5, 2, 2));
CommonMath::RectPrism topRect = CommonMath::RectPrism(Vec3(2.5, 2.75, 0), Vec3(4, 0.5, 2));
CommonMath::RectPrism bottomRect = CommonMath::RectPrism(Vec3(2.5, 1.25, 0), Vec3(4, 0.5, 2));

std::shared_ptr<CommonMath::RectPrism> leftRectPrism = std::make_shared<CommonMath::RectPrism>((leftRect));
std::shared_ptr<CommonMath::RectPrism> topRectPrism = std::make_shared<CommonMath::RectPrism>((topRect));
std::shared_ptr<CommonMath::RectPrism> bottomRectPrism = std::make_shared<CommonMath::RectPrism>((bottomRect));

/**
 * Collision Checker for polynomial equations --- refactored to work with OMPL
 */
DetailedCollisionResult polyCollisionChecker(
    double ts, double tf, std::shared_ptr<CommonMath::ConvexObj> obstacle,
    double minTimeSection, unsigned iterationNumber, CommonMath::Trajectory _traj)
{

    DetailedCollisionResult testResult;

    // First find the position halfway between the start and end time of this segment
    double midTime = (ts + tf) / 2.0;
    Vec3 midpoint = _traj.GetValue(midTime);

    if (obstacle->IsPointInside(midpoint))
    {
        if ((tf - ts) <= minTimeSection)
        {
            // If the time section is small enough, consider it a collision
            testResult.collisionType = Collision;
            testResult.collisionTime = ts;
            return testResult;
        }
        else
        {
            // Recursively check both halves of the trajectory
            unsigned nextIterationNumber = iterationNumber + 1;
            DetailedCollisionResult firstHalfResult = polyCollisionChecker(ts, midTime, obstacle, minTimeSection, nextIterationNumber, _traj);
            DetailedCollisionResult secondHalfResult = polyCollisionChecker(midTime, tf, obstacle, minTimeSection, nextIterationNumber, _traj);

            // Merge or prioritize collision results from both halves as needed
            // For simplicity, this example prioritizes collisions in the first half
            if (firstHalfResult.collisionType != NoCollision)
            {
                return firstHalfResult;
            }
            else
            {
                return secondHalfResult;
            }
        }
    }

    if (tf - ts < minTimeSection)
    {
        // Our time resolution is too small, just give up (trajectory is likely tangent to obstacle surface)
        testResult.collisionType = CollisionIndeterminable;
        testResult.collisionTime = ts;
        return testResult;
    }

    Vec3 endPoint = _traj.GetValue(tf);
    if (obstacle->IsPointInside(endPoint))
    {
        // Recursively check both halves of the trajectory based on collision in the first half
        DetailedCollisionResult firstHalfResult;
        unsigned nextIterationNumber = iterationNumber + 1;
        firstHalfResult = polyCollisionChecker(ts, midTime, obstacle,
                                               minTimeSection,
                                               nextIterationNumber, _traj);
        if (firstHalfResult.collisionType != NoCollision)
        {
            return firstHalfResult;
        }
        else
        {
            // Recursively check the second half of the trajectory
            unsigned nextIterationNumber = iterationNumber + 1;
            return polyCollisionChecker(midTime, tf, obstacle, minTimeSection,
                                        nextIterationNumber, _traj);
        }
    }

    // Get the plane separating the midpoint and the obstacle
    CommonMath::Boundary tangentPlane = obstacle->GetTangentPlane(midpoint);

    // Take the dot product of the trajectory with the unit normal of the separating plane.
    // This gives the distance of the trajectory from the plane as a function of time
    double c[5] = {0, 0, 0, 0, 0};
    std::vector<Vec3> trajDerivativeCoeffs = _traj.GetDerivativeCoeffs();
    for (unsigned dim = 0; dim < 3; dim++)
    {
        c[0] += tangentPlane.normal[dim] * trajDerivativeCoeffs[0][dim]; // t**4
        c[1] += tangentPlane.normal[dim] * trajDerivativeCoeffs[1][dim]; // t**3
        c[2] += tangentPlane.normal[dim] * trajDerivativeCoeffs[2][dim]; // t**2
        c[3] += tangentPlane.normal[dim] * trajDerivativeCoeffs[3][dim]; // t
        c[4] += tangentPlane.normal[dim] * trajDerivativeCoeffs[4][dim]; // 1
    }

    // Solve the roots
    double roots[4];
    unsigned rootCount;
    if (fabs(c[0]) > double(1e-6))
    {
        rootCount = Quartic::solve_quartic(c[1] / c[0], c[2] / c[0], c[3] / c[0],
                                           c[4] / c[0], roots);
    }
    else
    {
        rootCount = Quartic::solveP3(c[2] / c[1], c[3] / c[1], c[4] / c[1], roots);
    }
    // The first "rootCount" entries of roots are now the unordered roots
    std::sort(roots, roots + rootCount);
    // The first "rootCount" entries of roots are now the roots in ascending order

    // Get both lists of points to check (in ascending order)
    std::vector<double> testPointsLow;
    std::vector<double> testPointsHigh;
    testPointsLow.reserve(6);
    testPointsHigh.reserve(6);
    testPointsLow.push_back(ts);       // ts is always the first critical point in testPointsLow
    testPointsHigh.push_back(midTime); // midTime is always the first critical point in testPointsHigh
    for (int i = 0; i < rootCount; i++)
    {
        if (roots[i] <= ts)
        {
            // Skip root if it's before ts
            continue;
        }
        else if (roots[i] < midTime)
        {
            // Root is between ts and midTime
            testPointsLow.push_back(roots[i]);
        }
        else if (roots[i] < tf)
        {
            // Root is between midTime and tf
            testPointsHigh.push_back(roots[i]);
        }
        else
        {
            // Because the roots are in ascending order, there are no more roots are on (ts,tf)
            break;
        }
    }
    testPointsLow.push_back(midTime); // midTime is always the last critical point in testPointsLow
    testPointsHigh.push_back(tf);     // tf is always the last critical point in testPointsHigh

    // Check testPointsLow first. If the collision already takes place in first half, we can ignore the second half and return the collision time.
    for (typename std::vector<double>::reverse_iterator it =
             testPointsLow.rbegin() + 1;
         it != testPointsLow.rend(); it++)
    {
        // Check whether the critical point occurs on the obstacle side of the plane
        if ((_traj.GetValue(*it) - tangentPlane.point).Dot(tangentPlane.normal) <= 0)
        {
            // This critical point is on the obstacle side of the plane, so we must
            // keep searching over the rest of the trajectory starting at the
            // previous critical point and ending at ts (recall we are searching
            // backwards in time).

            DetailedCollisionResult lowTestPointsResult;
            unsigned nextIterationNumber = iterationNumber + 1;
            lowTestPointsResult = polyCollisionChecker(ts, *(it - 1),
                                                       obstacle,
                                                       minTimeSection,
                                                       iterationNumber, _traj);
            if (lowTestPointsResult.collisionType == NoCollision)
            {
                break;
            }
            else
            {
                return lowTestPointsResult;
            }
        }
    }
    for (typename std::vector<double>::iterator it = testPointsHigh.begin() + 1;
         it != testPointsHigh.end(); it++)
    {
        // Check whether the critical point occurs on the obstacle side of the plane
        if ((_traj.GetValue(*it) - tangentPlane.point).Dot(tangentPlane.normal) <= 0)
        {
            // This critical point is on the obstacle side of the plane, so we must
            // keep searching over the rest of the trajectory starting at the
            // previous critical point and ending at tf.
            DetailedCollisionResult highTestPointsResult;
            unsigned nextIterationNumber = iterationNumber + 1;
            highTestPointsResult = polyCollisionChecker(*(it - 1), tf,
                                                        obstacle,
                                                        minTimeSection,
                                                        nextIterationNumber, _traj);
            if (highTestPointsResult.collisionType == NoCollision)
            {
                // The section from the previous critical point until tf was feasible, meaning that all of the
                // trajectory from midTime to tf does not collide with the obstacle
                break;
            }
            else
            {
                // Either a collision was detected between the previous critical point and tf, or the recursion
                // became too deep (i.e. the time resolution too small) and the collision was indeterminable.
                return highTestPointsResult;
            }
        }
    }
    // Both segments are free of collision

    testResult.collisionType = NoCollision;
    testResult.collisionTime = 100000; // A very large time to indicate no collision;
    return testResult;
}

double actualPolyfitTime = 0.0;

Trajectory polyFit3D(std::vector<std::vector<double>> states)
{ // state is a matrix of however many rows, but four columns (x, y, z, tF)
    std::vector<double> tValues;
    Eigen::VectorXd yValues(states.size());
    Eigen::VectorXd xValues(states.size());
    // Eigen::VectorXd zVlaues(states.size());
    Eigen::VectorXd xCoeffs(5);
    Eigen::VectorXd yCoeffs(5);
    Eigen::VectorXd zCoeffs(5);

    for (unsigned rows = 0; rows < states.size(); rows++)
    {
        xValues[rows] = states[rows][0];
        yValues[rows] = states[rows][1];
        // zValues[rows] = states[rows][2];
        tValues.push_back(states[rows][3]);
    }

    xCoeffs = polyfit_Eigen(tValues, xValues, 5);
    yCoeffs = polyfit_Eigen(tValues, yValues, 5);
    // zCoeffs = polyfit_Eigen(tValues, zValues, 5);

    std::vector<Vec3> coeffs;
    // Change order of coefficients from 0->n to n->0
    for (int i = 5; i >= 0; i--)
    {
        coeffs.push_back(Vec3(xCoeffs.coeffRef(i), yCoeffs.coeffRef(i), zCoeffs.coeffRef(i)));
    }

    Trajectory traj(coeffs, tValues.front(), tValues.back());
    return traj;
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

bool collisionChecker(std::vector<std::vector<double>> *propStepStates, double ts, double tf, base::State *new_state, int tFIndex) {
                Trajectory _traj = polyFit3D(*propStepStates);

                DetailedCollisionResult leftCollisionResult = polyCollisionChecker(ts, tf, leftRectPrism, 1e-03, 0, _traj);
                DetailedCollisionResult topCollisionResult = polyCollisionChecker(ts, tf, topRectPrism, 1e-03, 0, _traj);
                DetailedCollisionResult bottomCollisionResult = polyCollisionChecker(ts, tf, bottomRectPrism, 1e-03, 0, _traj);

                DetailedCollisionResult trueCollisionResult;
                trueCollisionResult.collisionType == NoCollision;
                bool run = true;

                if (leftCollisionResult.collisionType == Collision)
                {
                    trueCollisionResult = leftCollisionResult;
                }
                else if (topCollisionResult.collisionType == Collision)
                {
                    trueCollisionResult = topCollisionResult;
                }
                else if (bottomCollisionResult.collisionType == Collision)
                {
                    trueCollisionResult = bottomCollisionResult;
                }
                else
                {
                    trueCollisionResult.collisionType == NoCollision;
                    run = false;
                }

                // No entiendo result below
                bool collision = run && trueCollisionResult.collisionType == Collision;
                std::vector<double> collision_point;

                if(collision) {
                Vec3 collision_point = _traj.GetValue(trueCollisionResult.collisionTime);
                    std::vector<Vec3> collision_vel_coeffs = _traj.GetDerivativeCoeffs();
                    collision_vel_coeffs.insert(collision_vel_coeffs.begin(), Vec3(0.0, 0.0, 0.0)); // FIXME: Expensive --- should use std::list instead    Inserts 0.0, 0.0, 0.0 as the fifth degree coefficient bcs only five terms currently
                    Trajectory _deriv_traj(collision_vel_coeffs, ts, tf);
                    Vec3 vel_collision_point = _deriv_traj.GetValue(trueCollisionResult.collisionTime);

                    // push back final motion with collision as last point
                    new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = collision_point[0];
                    new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = collision_point[1];
                    new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[2] = vel_collision_point[0];
                    new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[3] = vel_collision_point[1];
                    new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[tFIndex] = trueCollisionResult.collisionTime;
                }
}

base::PlannerStatus ompl::geometric::hyRRT::solve(const base::PlannerTerminationCondition &ptc)
{
    auto start = high_resolution_clock::now();

    // make sure the planner is configured correctly; ompl::base::Planner::checkValidity
    // ensures that there is at least one input state and a ompl::base::Goal object specified
    checkAllParametersSet();
    checkValidity();

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

        bool priority = checkPriority(new_motion->state);
        double tFlow = 0;
        int flows = 0;

        double random = rand();
        double randomFlowTimeMax = random / RAND_MAX * Tm_;

        // Track number of steps, alternative to motion_tree.size()
        int steps = 0;

        // Create a uniform distribution to select a random index, return the index of a random value between specified interval
        std::uniform_int_distribution<> dis1(0, inputValues1.size() - 1);
        std::uniform_int_distribution<> dis2(0, inputValues2.size() - 1);

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
            while (tFlow < randomFlowTimeMax && !checkPriority(new_motion->state))  // FIXME: Berkeley collision checker resolution is not good enough, so some collisions are not detected
            {
                collision = false;
                tFlow += flowStepLength_;

                // Find new state with flow propagation
                base::State *new_state = si_->allocState();

                new_state = this->flowPropagation_(u1, u2, previous_state, flowStepLength_, new_state);
                new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[tFIndex] += flowStepLength_;

                push_back_state(propStepStates, new_state);

                std::vector<double> startPoint = motion_to_vector(parent_motion->state);
                std::vector<double> endPoint = motion_to_vector(new_state);

                double ts = startPoint[tFIndex];
                double tf = endPoint[tFIndex];

                //Place collision checker here

                si_->copyState(previous_state, new_state);

                if (!checkBounds(new_state))
                {
                    goto escape;
                }

                collision = collisionChecker(propStepStates, ts, tf, new_state, tFIndex);

                // If the motion encounters no collision with jump sets, then add the successful motion to the temporary trees
                if (!collision)
                {
                    si_->copyState(new_motion->state, new_state); // Set parent state for next iterations
                }
                else
                {
                    totalCollisions++;
                    // set parent motions for next iteration to last motion in tree
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

                // Check if goal is reached. If so, escape to solution checking TODO: Consider moving out to after the full propagation for computational efficiency
                if (distanceFunc(new_motion->state, pdef_->getGoal()->as<ompl::base::GoalState>()->getState()) <= 0.1)
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
        if (distanceFunc(new_motion->state, pdef_->getGoal()->as<ompl::base::GoalState>()->getState()) <= 0.1) // finalDistance <= 0.01 || TODO: Add back in later
        {
            vector<Motion *> trajectory;
            nn_->list(trajectory);
            std::vector<Motion *> mpath;

            double finalDistance = distanceFunc(trajectory.back()->state, pdef_->getGoal()->as<ompl::base::GoalState>()->getState());
            Motion *solution = new_motion;

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

            // To get the value of duration use the count()
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now() - start);
            std::cout << "total duration (microseconds): " << duration.count() << endl;

            // Return a status indicating that an exact solution has been found
            if (finalDistance > 0.0)
                return base::PlannerStatus::APPROXIMATE_SOLUTION;
            else
                return base::PlannerStatus::EXACT_SOLUTION;
        }
    }
    return base::PlannerStatus::INFEASIBLE; // if failed to find a path within specified max number of iteratioins, then path generation has failed
}

bool ompl::geometric::hyRRT::checkPriority(base::State *state)
{
    // double velocity = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];
    // double acceleration_cur = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[2];
    // double pos_cur = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];

    // if (pos_cur <= 0 && velocity <= 0)
    // {
    //     return true;
    // }
    // else if (pos_cur >= -0.1)
    // {
    //     return false;
    // }
    // else
    // {
    //     return false;
    // }
    base::State *collisionState = nullptr;
    double time;
    std::vector<double> x_cur = {state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0], state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1], state->as<ompl::base::RealVectorStateSpace::StateType>()->values[2], state->as<ompl::base::RealVectorStateSpace::StateType>()->values[3]};
    double x1 = x_cur[0];
    double x2 = x_cur[1];
    double v1 = x_cur[2];
    double v2 = x_cur[3];
    bool value = false;

    // Jump state
    if (Xu(x1, x2))
    {
        value = true;

        // std::cout << "collision with jump" << std::endl;
    }

    return value;
} // Random selection if in intersection --- 0 if jump, 1 if flow
const base::State *constFlowProp(double u1, double u2, base::State *x_cur, double tFlow, base::State *new_state)
{}
base::State *defaultFlowPropagation(double u1, double u2, base::State *x_cur, double tFlow, base::State *new_state)
{
    // double y_pos = x_cur->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];
    // double acceleration_cur = x_cur->as<ompl::base::RealVectorStateSpace::StateType>()->values[2];
    // double x_pos = x_cur->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
    // double tFlow_cur = x_cur->as<ompl::base::RealVectorStateSpace::StateType>()->values[3];
    // double tJump_cur = x_cur->as<ompl::base::RealVectorStateSpace::StateType>()->values[4];
    double x1 = x_cur->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
    double x2 = x_cur->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];
    double x3 = x_cur->as<ompl::base::RealVectorStateSpace::StateType>()->values[2];
    double x4 = x_cur->as<ompl::base::RealVectorStateSpace::StateType>()->values[3];
    double x5 = x_cur->as<ompl::base::RealVectorStateSpace::StateType>()->values[4];
    double x6 = x_cur->as<ompl::base::RealVectorStateSpace::StateType>()->values[5];
    double x7 = x_cur->as<ompl::base::RealVectorStateSpace::StateType>()->values[6];
    double x8 = x_cur->as<ompl::base::RealVectorStateSpace::StateType>()->values[7];

    x1 = x1 + x3 * tFlow + x5 * pow(tFlow, 2) / 2; // x = v0 * t + 1/2(at^2)
    x2 = x2 + x4 * tFlow + x6 * pow(tFlow, 2) / 2; // x = v0 * t + 1/2(at^2)
    // double tFlow_new = x7 + tFlow;
    x3 = x3 + (x5)*tFlow; // v = v0 + at
    x4 = x4 + (x6)*tFlow;
    x5 += u1;
    x6 += u2;

    // new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = x;
    // new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = v;
    // new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[2] = acceleration_cur; // Flow should not be increasing the acceleration, TODO: schange this later for clarity (make starting acceleration -9.81)
    // new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[3] = tFlow_cur + tFlow;
    // new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[4] = x_cur->as<ompl::base::RealVectorStateSpace::StateType>()->values[4]; // No change in jump time
    // new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[5] = 0.0;                                                                 // Set control input to 0 for flow

    new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = x1;
    new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = x2;
    new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[2] = x3;
    new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[3] = x4;
    new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[4] = x5;
    new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[5] = x6;
    new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[6] = x7;
    new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[7] = x8; // No change in jump time
    new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[8] = u1; // Append control input to state
    new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[9] = u2; // Append control input to state
    return new_state;
}

bool Xu(double x1, double x2)
{
    if (x1 <= 4.4 && x1 >= 0 && x2 >= 1.1 && x2 <= 1.4)
    {
        return true;
    }
    if (x1 <= 0.4 && x1 >= 0 && x2 >= 1.1 && x2 <= 2.9)
    {
        return true;
    }
    if (x1 <= 4.4 && x1 >= 0 && x2 >= 2.6 && x2 <= 2.9)
    {
        return true;
    }
    return false;
}

// bool collisionChecker(base::State *parent, base::State *x_cur)
// {
//     base::State *collisionState = nullptr;
//     double time;
//     std::vector<double> state = {x_cur->as<ompl::base::RealVectorStateSpace::StateType>()->values[0], x_cur->as<ompl::base::RealVectorStateSpace::StateType>()->values[1], x_cur->as<ompl::base::RealVectorStateSpace::StateType>()->values[2], x_cur->as<ompl::base::RealVectorStateSpace::StateType>()->values[3]};
//     double x1 = state[0];
//     double x2 = state[1];
//     double v1 = state[2];
//     double v2 = state[3];
//     bool value = false;
//     // Jump state
//     if (collisionregion1(x1, x2, v1, v2) || collisionregion2(x1, x2, v1, v2) ||
//         collisionregion3(x1, x2, v1, v2) || collisionregion4(x1, x2, v1, v2) ||
//         collisionregion5(x1, x2, v1, v2) || collisionregion6(x1, x2, v1, v2) ||
//         collisionregion7(x1, x2, v1, v2))
//     {
//         value = true;

//         // std::cout << "collision with jump" << std::endl;
//     }

//     // collision state
//     if (x1 >= -0.1 && x1 <= 6 && x2 >= -0.1 && x2 <= 5 && !Xu(x1, x2))
//     {
//         value = false;
//         // std::cout << "in flow set" << std::endl;
//     }
//     return value;
// }

base::State *defaultJumpPropagation(base::State *x_cur, double u, base::State *new_state)
{
    double x1 = x_cur->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
    double x2 = x_cur->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];
    double x3 = x_cur->as<ompl::base::RealVectorStateSpace::StateType>()->values[2];
    double x4 = x_cur->as<ompl::base::RealVectorStateSpace::StateType>()->values[3];
    double x7 = x_cur->as<ompl::base::RealVectorStateSpace::StateType>()->values[6];
    double x8 = x_cur->as<ompl::base::RealVectorStateSpace::StateType>()->values[7];

    double e = 0.43;
    double kappa = 0.20;
    double vn, vt, vnplus, vtplus;

    if (collisionregion1(x1, x2, x3, x4) || collisionregion2(x1, x2, x3, x4) ||
        collisionregion3(x1, x2, x3, x4) || collisionregion4(x1, x2, x3, x4))
    {
        vn = x3;
        vt = x4;
        vnplus = -e * vn;
        vtplus = vt + kappa * (-e - 1) * std::atan(vt / vn) * vn;
        x3 = vnplus;
        x4 = vtplus;
    }
    else
    {
        vn = x4;
        vt = x3;
        vnplus = -e * vn;
        vtplus = vt + kappa * (-e - 1) * std::atan(vt / vn) * vn;
        x3 = vtplus;
        x4 = vnplus;
    }

    new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = x1;
    new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = x2;
    new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[2] = x3;
    new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[3] = x4; // No change in flow time
    new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[4] = 0;
    new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[5] = 0;
    new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[6] = x7;
    new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[7] = x8; // No change in jump time
    new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[8] = 0;         // Append control input to state
    new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[9] = 0;         // Append control input to state
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
    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b)
                             { return distanceFunction(a, b); });

    // if (!nn_printing_)
    //     nn_printing_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    // nn_printing_->setDistanceFunction([this](const Motion *a, const Motion *b)
    //                                   { return distanceFunction(a, b); });
}

// void integrateStates(const base::SpaceInformationPtr &si, control::ODESolver::ODE kinematics, base::StateSpace &setup)
// {
//     // Use the ODESolver to propagate the system.
// }

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