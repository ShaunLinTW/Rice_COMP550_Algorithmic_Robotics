///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Jason Zhang(jz118), Shaun Lin(hl116)
//////////////////////////////////////

#include <iostream>
#include <fstream>

#include <ompl/base/ProjectionEvaluator.h>

#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>

// The collision checker routines
#include "CollisionChecking.h"

// Your implementation of RG-RRT
#include "RG-RRT.h"

#include <ompl/control/SpaceInformation.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include "ompl/control/planners/kpiece/KPIECE1.h"
#include <ompl/config.h>
#include <valarray>
#include <limits>


namespace ob = ompl::base;
namespace oc = ompl::control;
namespace og = ompl::geometric;
double PI = 3.1415926;

// Your projection for the car
class CarProjection : public ompl::base::ProjectionEvaluator
{
public:
    CarProjection(const ompl::base::StateSpace *space) : ProjectionEvaluator(space)
    {
    }

    unsigned int getDimension() const override
    {
        // TODO: The dimension of your projection for the car
        return 2;
    }
    virtual void defaultCellSizes(void) override
    {
        cellSizes_.resize(2);
        cellSizes_[0] = 0.5;
        cellSizes_[1] = 0.5;
    }

    void project(const ompl::base::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        // TODO: Your projection for the car
        const auto *carState = state->as<ob::SE2StateSpace::StateType>();
        projection(0) = carState->getX();
        projection(1) = carState->getY();
    }
};

void carODE(const ompl::control::ODESolver::StateType &q, const ompl::control::Control *control,
            ompl::control::ODESolver::StateType &qdot)
{
    // TODO: Fill in the ODE for the car's dynamics
    const double *u = control->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
    const double theta = q[2];
    const double v = q[3];
    qdot.resize(q.size(), 0);

    // Current rotational state
    const double omega = u[0];
    const double velocity = u[1];

    qdot[0] = v * cos(theta);
    qdot[1] = v * sin(theta);
    qdot[2] = omega; // angular velocity
    qdot[3] = velocity; // forward acceleration of the vehicle
}

void carPostIntegration(const ob::State *state, const oc::Control *control, const double duration, ob::State *result)
{
    // Ensure orientation lies between 0 and 2*pi
    // ob::SO2StateSpace SO2;
    // auto s = result->as<ob::CompoundState>()->as<ob::SE2StateSpace::StateType>(0);
    // SO2.enforceBounds(s->as<ob::SO2StateSpace::StateType>(1));
    ob::SO2StateSpace SO2;
    SO2.enforceBounds(result->as<ob::CompoundState>()->as<ob::SE2StateSpace::StateType>(0));
}

bool isStateValid(const oc::SpaceInformation *si, const std::vector<Rectangle> &obstacles, const ob::State *state)
{
    // Cast the state to an appropriate type
    const auto *se2state = state->as<ob::SE2StateSpace::StateType>();

    // Extract the robot's (x,y) position from its state
    const auto *pos = se2state->as<ob::RealVectorStateSpace::StateType>(0);

    return si->satisfiesBounds(se2state) && isValidStateSquare(pos, 0.25, obstacles);
}

void makeStreet(std::vector<Rectangle> &obstacles)
{
    // TODO: Fill in the vector of rectangles with your street environment.
    Rectangle rect;
    rect.x = -11;
    rect.y = -6;
    rect.width = 8;
    rect.height = 10;
    obstacles.push_back(rect);

    rect.x = 2;
    rect.y = -6;
    rect.width = 9;
    rect.height = 10;
    obstacles.push_back(rect);

    rect.x = -7;
    rect.y = 8;
    rect.width = 14;
    rect.height = 3;
    obstacles.push_back(rect);
}

ompl::control::SimpleSetupPtr createCar(std::vector<Rectangle> &obstacles)
{
    // TODO: Create and setup the car's state space, control space, validity checker, everything you need for planning.
    
    // Create the state space for x, y
    auto s1 = std::make_shared<ob::SE2StateSpace>();

    // set the bounds for the R^2
    ob::RealVectorBounds s1_bounds(2);
    s1_bounds.setLow(-10);
    s1_bounds.setHigh(10);
    s1->setBounds(s1_bounds);

    // Create the state space for theta
    auto s2 = std::make_shared<ob::SO2StateSpace>();

    // Create the state space for forward velocity
    auto r1 = std::make_shared<ob::RealVectorStateSpace>(1);

    // Set the bounds for the R^1
    ob::RealVectorBounds r1_bounds(1);
    r1_bounds.setLow(-10);
    r1_bounds.setHigh(10);
    r1->setBounds(r1_bounds);

    // Create the state space
    ob::StateSpacePtr space;
    space = s1 + s2 + r1;

    // Create street environment
    makeStreet(obstacles);

    // Create control space
    auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 2));

    // Set lower and upper bounds
    ob::RealVectorBounds cbounds(2);
    cbounds.setLow(-10);
    cbounds.setHigh(10);
    cspace->setBounds(cbounds);

    // Create a simple setup class
    oc::SimpleSetupPtr ss(new oc::SimpleSetup(cspace));

    // Create a control space instance 
    auto odeSolver(std::make_shared<oc::ODEBasicSolver<>>(ss->getSpaceInformation(), &carODE));
    ss->setStatePropagator(oc::ODESolver::getStatePropagator(odeSolver, &carPostIntegration));
    ss->getSpaceInformation()->setPropagationStepSize(0.05);

    // Set space information
    oc::SpaceInformation *si = ss->getSpaceInformation().get();

    // Set state validity checker
    ss->setStateValidityChecker(
        [si, obstacles](const ob::State *state) { 
            return isStateValid(si, obstacles, state); 
        }
    );

    // Set projection evaluator
    ss->getStateSpace()->registerDefaultProjection(
        std::make_shared<CarProjection>(ss->getStateSpace().get())
    );

    // Create start state
    ob::ScopedState<> start(space);
    start[0] = -8;
    start[1] = -8;
    start[2] = 0;
    start[3] = 0;

    // Create goal state
    ob::ScopedState<> goal(space);
    goal[0] = 9;
    goal[1] = 9;
    goal[2] = PI/2;
    goal[3] = 0;

    // Set start and goal states
    ss->setStartAndGoalStates(start, goal, 0.05);

    return ss;
}

void planCar(ompl::control::SimpleSetupPtr &ss, int choice)
{
    // TODO: Do some motion planning for the car
    // choice is what planner to use.

    if (choice == 1) {
        // set RTP as planner
        auto planner(std::make_shared<oc::RRT>(ss->getSpaceInformation()));
        ss->setPlanner(planner);
    }
    else if (choice == 2) {
        // set KPIECE as planner
    }
    else if (choice == 3) {
        // set RG-RRT as planner
    }

    // Attempt to solve the problem within 60 second of planning time
    ss->setup();
    ob::PlannerStatus solved = ss->solve(60.0);

    // If planning is successful
    if (solved)
    {
        // Print the path to screen
        std::cout << "Found solution:" << std::endl;
        ss->getSolutionPath().asGeometric().printAsMatrix(std::cout);

        // save path to a txt file
        std::ofstream fout;
        fout.open("./src/visualization/car_RRT.txt");
        ss->getSolutionPath().asGeometric().printAsMatrix(fout);
        fout.close();
    }
    else
        std::cout << "No solution found" << std::endl;
}

void benchmarkCar(ompl::control::SimpleSetupPtr &/* ss */)
{
    // TODO: Do some benchmarking for the car
}

int main(int argc, char **argv)
{
    std::vector<Rectangle> obstacles;
    makeStreet(obstacles);

    int choice;
    do
    {
        std::cout << "Plan or Benchmark? " << std::endl;
        std::cout << " (1) Plan" << std::endl;
        std::cout << " (2) Benchmark" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    ompl::control::SimpleSetupPtr ss = createCar(obstacles);

    // Planning
    if (choice == 1)
    {
        int planner;
        do
        {
            std::cout << "What Planner? " << std::endl;
            std::cout << " (1) RRT" << std::endl;
            std::cout << " (2) KPIECE1" << std::endl;
            std::cout << " (3) RG-RRT" << std::endl;

            std::cin >> planner;
        } while (planner < 1 || planner > 3);

        planCar(ss, planner);
    }
    // Benchmarking
    else if (choice == 2)
        benchmarkCar(ss);

    else
        std::cerr << "How did you get here? Invalid choice." << std::endl;

    return 0;
}
