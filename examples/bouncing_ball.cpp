#include "../HyRRT.h"

/** \brief Function computes the Pythagorean distance between two states. */
double distanceFunc(ompl::base::State *state1, ompl::base::State *state2)
{
    double dist = 0;
    dist = sqrt(pow(state1->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] - state2->as<ompl::base::RealVectorStateSpace::StateType>()->values[0], 2) + pow(state1->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] - state2->as<ompl::base::RealVectorStateSpace::StateType>()->values[1], 2));
    return fabs(dist);
}

/** \brief Jump set is whenever the ball is on or below the surface and has a downwards velocity. */
bool jumpSet(ompl::base::State *state)
{
    double velocity = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];
    double pos_cur = state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];

    if (pos_cur <= 0 && velocity <= 0)
        return true;
    else
        return false;
}

/** \brief Flow set is whenever the ball is above the surface or has an upwards velocity. */
bool flowSet(ompl::base::State *state)
{
    return !jumpSet(state);
}

/** \brief Unsafe set is whenever the ball is above 10 units from the ground, to reduce time spent planning. */
bool unsafeSet(ompl::base::State *state)
{
    if (state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] > 10)
        return true;
    return false;
}

/** \brief Simulates the dynamics of the ball when in flow regime, with no nonnegligble forces other than gravity. */
ompl::base::State *continuousSimulator(std::vector<double> inputs, ompl::base::State *x_cur, double tFlow, ompl::base::State *new_state)
{
    double velocity = x_cur->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];
    double acceleration_cur = x_cur->as<ompl::base::RealVectorStateSpace::StateType>()->values[2];
    double pos_cur = x_cur->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];

    double v = velocity + (acceleration_cur) * tFlow;                               // v = v0 + at
    double x = pos_cur + velocity * tFlow + (acceleration_cur) * pow(tFlow, 2) / 2; // x = v0 * t + 1/2(at^2)

    new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = x;
    new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = v;
    new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[2] = acceleration_cur; 
    new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[3] = x_cur->as<ompl::base::RealVectorStateSpace::StateType>()->values[3];   // Dynamics simulation functions does not touch flow time or jumps
    new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[4] = x_cur->as<ompl::base::RealVectorStateSpace::StateType>()->values[4];
    return new_state;
}

/** \brief Simulates the dynamics of the ball when in jump regime, with input from the surface. */
ompl::base::State *discreteSimulator(ompl::base::State *x_cur, std::vector<double> u, ompl::base::State *new_state)
{
    double velocity = -0.8 * x_cur->as<ompl::base::RealVectorStateSpace::StateType>()->values[1];

    new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[0] = x_cur->as<ompl::base::RealVectorStateSpace::StateType>()->values[0];
    new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[1] = velocity - u[0];
    new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[2] = x_cur->as<ompl::base::RealVectorStateSpace::StateType>()->values[2];
    new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[3] = x_cur->as<ompl::base::RealVectorStateSpace::StateType>()->values[3];   // Dynamics simulation functions does not touch flow time or jumps
    new_state->as<ompl::base::RealVectorStateSpace::StateType>()->values[4] = x_cur->as<ompl::base::RealVectorStateSpace::StateType>()->values[4];
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

    cHyRRT.setContinuousSimulator(continuousSimulator);
    cHyRRT.setDiscreteSimulator(discreteSimulator);
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

    // // How to access the solution path as a vector
    // std::vector<ompl::geometric::HyRRT::Motion *> trajectory = cHyRRT.getTrajectoryMatrix();
    // for(auto &m : trajectory) {
    //     // Use m->as<ompl::base::RealVectorStateSpace::StateType>()->values[** desired index **]
    // }
}