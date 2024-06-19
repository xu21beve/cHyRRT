#include <ompl/base/Planner.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/base/GoalTypes.h>
#include "../HyRRT.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include <ompl/control/SimpleSetup.h>
#include <ompl/base/goals/GoalState.h>
#include "../CommonMath/Trajectory.hpp"
#include "../CommonMath/RectPrism.hpp"
#include "../polyFit.h"
#include "../src/quartic.cpp"
#include "../CommonMath/RectPrism.hpp"
#include "../CommonMath/ConvexObj.hpp"
#include <list>
#include "../HyRRT.h"

using namespace CommonMath;


double distanceFunc(ompl::base::State *state1, ompl::base::State *state2)
{
    double dist = 0;
    dist = sqrt(pow(state1->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] - state2->as<ompl::base::RealVectorStateSpace::StateType>()->values[0], 2) + pow(state1->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] - state2->as<ompl::base::RealVectorStateSpace::StateType>()->values[1], 2));
    return fabs(dist);
}

bool jumpSet(ompl::base::State *state)
{
    double velocity = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];
    double acceleration_cur = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[2];
    double pos_cur = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];

    if (pos_cur <= 0 && velocity <= 0)
    {
        return true;
    }
    else
    {
        return false;
    }
} // Random selection if in intersection --- 0 if jump, 1 if flow

bool flowSet(ompl::base::State *state)
{
    return !jumpSet(state);
}

bool unsafeSet(ompl::base::State *state)
{
    std::vector<double> x_cur = {state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0], state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1], state->as<ompl::base::RealVectorStateSpace::StateType>()->values[2], state->as<ompl::base::RealVectorStateSpace::StateType>()->values[3]};
    if (x_cur[0] > 10)
        return true;
    return false;
}

ompl::base::State *flowPropagation(std::vector<double> inputs, ompl::base::State *x_cur, double tFlow, ompl::base::State *new_state)
{
    double velocity = x_cur->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];
    double acceleration_cur = x_cur->as<ompl::base::RealVectorStateSpace::StateType>()->values[2];
    double pos_cur = x_cur->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
    double tFlow_cur = x_cur->as<ompl::base::RealVectorStateSpace::StateType>()->values[3];
    double tJump_cur = x_cur->as<ompl::base::RealVectorStateSpace::StateType>()->values[4];

    double v = velocity + (acceleration_cur)*tFlow;                               // v = v0 + at
    double x = pos_cur + velocity * tFlow + (acceleration_cur) * pow(tFlow, 2) / 2; // x = v0 * t + 1/2(at^2)
    double tFlow_new = tFlow_cur + tFlow;

    new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = x;
    new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = v;
    new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[2] = acceleration_cur; // Flow should not be increasing the acceleration, TODO: schange this later for clarity (make starting acceleration -9.81)
    new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[3] = tFlow_cur;
    new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[4] = x_cur->as<ompl::base::RealVectorStateSpace::StateType>()->values[4];  // No change in jump time
    return new_state;
}

ompl::base::State *jumpPropagation(ompl::base::State *x_cur, std::vector<double> u, ompl::base::State *new_state)
{
    double velocity = -0.8 * x_cur->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];
    double acceleration = x_cur->as<ompl::base::RealVectorStateSpace::StateType>()->values[2];
    double pos_cur = x_cur->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
    double tFlow_cur = x_cur->as<ompl::base::RealVectorStateSpace::StateType>()->values[3];
    double tJump_cur = x_cur->as<ompl::base::RealVectorStateSpace::StateType>()->values[4];

    double v = velocity; // v = v0 + at  // TODO: Add in floor actuation as acceleration here by adding to acceleration (removed  + (acceleration) * tJump)
    double x = pos_cur;  // x = v0 * t + 1/2(at^2) ------ removed acceleration:  + acceleration * pow(tJump, 2) / 2

    new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = x;
    new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = v - u[0];
    new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[2] = acceleration;
    new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[3] = tFlow_cur; // No change in flow time
    new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[4] = tJump_cur;  // No change in jump time
    return new_state;
}

int main()
{
    // Set the bounds of space
    ompl::base::RealVectorStateSpace *statespace = new ompl::base::RealVectorStateSpace(0);
    statespace->addDimension(-10, 10);
    statespace->addDimension(-100, 100);
    statespace->addDimension(-10, 10);
    statespace->addDimension(0, std::numeric_limits<double>::epsilon());
    statespace->addDimension(0, std::numeric_limits<double>::epsilon());

    ompl::base::StateSpacePtr space(statespace); // Adding in third dimension for jump time (change to 3 later)

    // Construct a space information instance for this state space
    ompl::base::SpaceInformationPtr si(new ompl::base::SpaceInformation(space));

    si->setup();

    // Set our robot's starting state to be the bottom-left corner of
    // the environment, or (0,0).
    ompl::base::ScopedState<> start(space);
    start->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = 1;
    start->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = 0;
    start->as<ompl::base::RealVectorStateSpace::StateType>()->values[2] = -9.81;
    start->as<ompl::base::RealVectorStateSpace::StateType>()->values[3] = 0;
    start->as<ompl::base::RealVectorStateSpace::StateType>()->values[4] = 0;

    // Set our robot's goal state to be the top-right corner of the
    // environment, or (1,1).
    ompl::base::ScopedState<> goal(space);
    goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = 0;
    goal->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = 0;

    // Create a problem instance
    ompl::base::ProblemDefinitionPtr pdef(new ompl::base::ProblemDefinition(si));

    // Set the start and goal states
    pdef->setStartAndGoalStates(start, goal, 0.1);

    ompl::geometric::HyRRT cHyRRT(si);

    // Set the problem instance for our planner to solve
    cHyRRT.setProblemDefinition(pdef);
    cHyRRT.setup();

    cHyRRT.setContinuousSimulator(flowPropagation);
    cHyRRT.setDiscreteSimulator(jumpPropagation);
    cHyRRT.setDistanceFunction(distanceFunc);
    cHyRRT.setFlowSet(flowSet);
    cHyRRT.setJumpSet(jumpSet);
    cHyRRT.setTm(0.5);
    cHyRRT.setFlowStepDuration(0.001);
    cHyRRT.setFlowInputRange(std::vector<double>{-9.81}, std::vector<double>{-9.81});   // If input is a single value, only that value will every be used
    cHyRRT.setJumpInputRange(std::vector<double>{0}, std::vector<double>{0});
    cHyRRT.setUnsafeSet(unsafeSet);

    // attempt to solve the planning problem within one second of
    // planning time
    ompl::base::PlannerStatus solved = cHyRRT.solve(ompl::base::timedPlannerTerminationCondition(10000));
    cout << "solution status: " << solved << endl;

    // // How to access the solution path as a C++ vector
    // std::vector<ompl::geometric::HyRRT::Motion *> trajectory = cHyRRT.getTrajectoryMatrix();
    // for(auto &m : trajectory) {
    //     // Use m->as<ompl::base::RealVectorStateSpace::StateType>()->values[** desired index **]
    // }
}