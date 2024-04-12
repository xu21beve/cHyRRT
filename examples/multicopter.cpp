#include "../src/hyRRT.cpp"
// #include "ompl/geometric/planners/rrt/hyRRT.h"

int main()
{
    // Construct the robot state space in which we're planning. We're
    // planning in [0,1]x[0,1], a subset of R^2.

    // Set the bounds of space
    base::RealVectorStateSpace *statespace = new base::RealVectorStateSpace(0);
    statespace->addDimension(0.5, 6.0);
    statespace->addDimension(0, 5);
    statespace->addDimension(-3, 8);
    statespace->addDimension(-3, 100000);
    statespace->addDimension(-3, 3);
    statespace->addDimension(-3, 3);
    statespace->addDimension(0, std::numeric_limits<double>::epsilon());
    statespace->addDimension(0, std::numeric_limits<double>::epsilon());
    statespace->addDimension(0, std::numeric_limits<double>::epsilon());
    statespace->addDimension(0, std::numeric_limits<double>::epsilon());

    base::StateSpacePtr space(statespace); // Adding in third dimension for jump time (change to 3 later)

    // Construct a space information instance for this state space
    base::SpaceInformationPtr si(new base::SpaceInformation(space));

    // Set the object used to check which states in the space are valid
    // si->setStateValidityChecker(base::StateValidityCheckerPtr(new ValidityChecker(si))); //TODO: Add later

    si->setup();

    // Set our robot's starting state to be the bottom-left corner of
    // the environment, or (0,0).
    base::ScopedState<> start(space);
    start->as<base::RealVectorStateSpace::StateType>()->values[0] = 1;
    start->as<base::RealVectorStateSpace::StateType>()->values[1] = 2;
    start->as<base::RealVectorStateSpace::StateType>()->values[2] = 0;
    start->as<base::RealVectorStateSpace::StateType>()->values[3] = 0;
    start->as<base::RealVectorStateSpace::StateType>()->values[4] = 0;
    start->as<base::RealVectorStateSpace::StateType>()->values[5] = 0;
    start->as<base::RealVectorStateSpace::StateType>()->values[6] = 0;
    start->as<base::RealVectorStateSpace::StateType>()->values[7] = 0;
    start->as<base::RealVectorStateSpace::StateType>()->values[8] = 0;
    start->as<base::RealVectorStateSpace::StateType>()->values[9] = 0;

    // Set our robot's goal state to be the top-right corner of the
    // environment, or (1,1).
    base::ScopedState<> goal(space);
    goal->as<base::RealVectorStateSpace::StateType>()->values[0] = 5;
    goal->as<base::RealVectorStateSpace::StateType>()->values[1] = 4;

    // Create a problem instance
    base::ProblemDefinitionPtr pdef(new base::ProblemDefinition(si));

    // Set the start and goal states
    pdef->setStartAndGoalStates(start, goal, 0.1);

    ompl::geometric::hyRRT cHyRRT(si);

    // Set the problem instance for our planner to solve
    cHyRRT.setProblemDefinition(pdef);
    cHyRRT.setup();
    // cHyRRT.defineModel(defaultJumpPropagation, defaultFlowPropagation, distanceFunc);
    cHyRRT.setFlowPropagationFunction(defaultFlowPropagation);
    cHyRRT.setJumpPropagationFunction(defaultJumpPropagation);
    cHyRRT.setDistanceFunction(distanceFunc);

    // auto parameters = cHyRRT.params();
    // std::cout << "Setting your flow propagation function succeeded: " << parameters.setParam("flowPropagation", "defaultFlowPropagation") << std::endl;
    // std::cout << "Setting your jump propagation function succeeded: " << parameters.setParam("jumpPropagation", "defaultJumpPropagation") << std::endl;
    // std::cout << "Setting your distance function succeeded: " << parameters.setParam("distanceFunction", "defaultDistance") << std::endl;

    // attempt to solve the planning problem within one second of
    // planning time
    base::PlannerStatus solved = cHyRRT.solve(base::timedPlannerTerminationCondition(10000));
    std::cout << "solution status: " << solved << endl;
}