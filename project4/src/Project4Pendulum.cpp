///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Jason Zhang(jz118), Shaun Lin(hl116)
//////////////////////////////////////
#include <fstream>
#include <iostream>
#include <math.h>
#include <ompl/base/ProjectionEvaluator.h>

#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>

// Your implementation of RG-RRT
#include "RG-RRT.h"

// #define PI 3.1415926;

namespace ob = ompl::base;
namespace oc = ompl::control;
namespace og = ompl::geometric;
double PI = 3.1415926;

// Your projection for the pendulum
class PendulumProjection : public ompl::base::ProjectionEvaluator
{
public:
    PendulumProjection(const ompl::base::StateSpacePtr& space) : ProjectionEvaluator(space)
    {
    }

    unsigned int getDimension() const override
    {
        // TODO: The dimension of your projection for the pendulum
        return 2;
    }

    void project(const ompl::base::State *state, Eigen::Ref<Eigen::VectorXd> projection) const override
    {
        // TODO: Your projection for the pendulum
        auto compound_state = state->as<ob::CompoundState>();
        auto s1 = compound_state->as<ob::SO2StateSpace::StateType>(0);
        auto r1 = compound_state->as<ob::RealVectorStateSpace::StateType>(1);

        projection(0) = s1->value;
        projection(1) = r1->values[0];
    }
};

void pendulumODE(const oc::ODESolver::StateType &q, const oc::Control *c,
                 oc::ODESolver::StateType &qdot)
{
    // Retrieve Control values: T(torque)
    const double *u = c->as<oc::RealVectorControlSpace::ControlType>()->values;
    const double torque = u[0];

    // Current rotational state
    const double theta = q[0];
    const double omega = q[1];

    // Ensure qdot is the same size as q. TODO: is this necessary?
    qdot.resize(q.size(), 0);
    const int g = 9.81;
    qdot[0] = omega; // angular velocity
    qdot[1] = -g*cos(theta) + torque; // angular acceleration
}

void pendulumPostIntegration(const ob::State* /*state*/, const oc::Control* /*constol*/, const double /*duration*/, ob::State* result) {
    // ob::SO2StateSpace SO2;
    // ensure orientation lies between 0 and 2*pi
    // auto s = *result->as<ob::SO2StateSpace::StateType>(0);
    // TODO:enforce the values
    // SO2.enforceBounds(s->values[0]);
    // SO2.enforceBounds (result->as<ob::CompoundStateSpace::StateType>()->as<ob::SO2StateSpace::StateType>(0));
}

bool isStateValid(const oc::SpaceInformation *si, const ob::State *state) {
    //TODO: what is the reason we need to implement this function?
    const auto *singleState = state->as<ob::RealVectorStateSpace::StateType>();
    // std::cout << "isStateValid: " << singleState->values[0] << " " << singleState->values[1] << std::endl;
    // std::cout << si->satisfiesBounds(singleState) << std::endl;
    return si->satisfiesBounds(singleState);
}

ompl::control::SimpleSetupPtr createPendulum(double t)
{
    // create workspace
    // auto space = std::make_shared<ob::RealVectorStateSpace>(2);
    // ob::RealVectorBounds bounds(2);
    // bounds.setHigh(0, 10);
    // bounds.setLow(0, -10);
    // space->setBounds(bounds);

    // construct the state space for theta
    auto s1 = std::make_shared<ob::SO2StateSpace>();
    
    // construct the state space for rotational velocity
    auto r1 = std::make_shared<ob::RealVectorStateSpace>(1);
    
    // set the bounds for the R^1
    ob::RealVectorBounds r1_bounds(1);
    r1_bounds.setLow(-10);
    r1_bounds.setHigh(10);
    r1->setBounds(r1_bounds);
    
    ob::StateSpacePtr space;
    space = s1+r1;

    space->registerProjection("PendProj", ob::ProjectionEvaluatorPtr(new PendulumProjection(space)));

    // create control space
    auto cspace = std::make_shared<oc::RealVectorControlSpace>(space, 1);
    ob::RealVectorBounds cbounds(1);
    cbounds.setLow(-t);
    cbounds.setHigh(t);
    cspace->setBounds(cbounds);

    // define setup class
    oc::SimpleSetupPtr ss(new oc::SimpleSetup(cspace));

    // set state validity check
    oc::SpaceInformation *si = ss->getSpaceInformation().get();
    ss->setStateValidityChecker(
        [si](const ob::State *state) { 
            return isStateValid(si, state); 
        }
    );
    auto odesolver(std::make_shared<oc::ODEBasicSolver<>>(ss->getSpaceInformation(), &pendulumODE));
    ss->setStatePropagator(oc::ODESolver::getStatePropagator(odesolver, &pendulumPostIntegration));
    ss->getSpaceInformation()->setPropagationStepSize(0.05);

    ob::ScopedState<ob::RealVectorStateSpace> start(space);
    start[0] = -PI/2;
    start[1] = 0.0;

    ob::ScopedState<ob::RealVectorStateSpace> goal(space);
    goal[0] = PI/2;
    goal[1] = 0.0;
    
    ss->setStartAndGoalStates(start, goal, 0.05);
    return ss;
}

void planPendulum(ompl::control::SimpleSetupPtr &ss, int choice)
{
    std::string plannerName = "";
    if (choice == 1) {
        // set RTP as planner
        auto planner(std::make_shared<oc::RRT>(ss->getSpaceInformation()));
        ss->setPlanner(planner);
        plannerName = "RRT";
    }
    else if (choice == 2) {
        // set KPIECE as planner
        auto planner(std::make_shared<oc::KPIECE1>(ss->getSpaceInformation()));
        planner->as<oc::KPIECE1>()->setProjectionEvaluator("PendProj");
        ss->setPlanner(planner);
        plannerName = "KPIECE";
    }
    else if (choice == 3) {
        // set RG-RRT as planner
    }
    

    // attempt to solve the problem within 1 min of planning time
    ss->setup();
    ob::PlannerStatus solved = ss->solve(60.0);

    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        // if ss is a ompl::geometric::SimpleSetup object
        ss->getSolutionPath().asGeometric().printAsMatrix(std::cout);

        //save data to a txt file
        std::ofstream fout;
        // fout.open("./src/exercise2_visualization/box_env1.txt");
        std::string filename = "./src/pendulum_visualization/" + plannerName + ".txt";
        std::cout << "Wrote to " << filename << std::endl;
        fout.open(filename);
        ss->getSolutionPath().asGeometric().printAsMatrix(fout);
        fout.close();
    }
    else
        std::cout << "No solution found" << std::endl;
}

void benchmarkPendulum(ompl::control::SimpleSetupPtr &/* ss */)
{
    // TODO: Do some benchmarking for the pendulum
}

int main(int /* argc */, char ** /* argv */)
{
    int choice;
    do
    {
        std::cout << "Plan or Benchmark? " << std::endl;
        std::cout << " (1) Plan" << std::endl;
        std::cout << " (2) Benchmark" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    int which;
    do
    {
        std::cout << "Torque? " << std::endl;
        std::cout << " (1)  3" << std::endl;
        std::cout << " (2)  5" << std::endl;
        std::cout << " (3) 10" << std::endl;

        std::cin >> which;
    } while (which < 1 || which > 3);

    double torques[] = {3., 5., 10.};
    double torque = torques[which - 1];

    ompl::control::SimpleSetupPtr ss = createPendulum(torque);

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

        planPendulum(ss, planner);
    }
    // Benchmarking
    else if (choice == 2)
        benchmarkPendulum(ss);

    else
        std::cerr << "How did you get here? Invalid choice." << std::endl;

    return 0;
}
