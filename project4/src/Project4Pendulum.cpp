///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Jason Zhang(jz118), Shaun Lin(hl116)
//////////////////////////////////////

#include <iostream>
#include <math.h>
#include <ompl/base/ProjectionEvaluator.h>

#include <ompl/control/SimpleSetup.h>
#include <ompl/control/ODESolver.h>

// Your implementation of RG-RRT
#include "RG-RRT.h"

namespace ob = ompl::base;
namespace oc = ompl::control;

// Your projection for the pendulum
class PendulumProjection : public ompl::base::ProjectionEvaluator
{
public:
    PendulumProjection(const ompl::base::StateSpace *space) : ProjectionEvaluator(space)
    {
    }

    unsigned int getDimension() const override
    {
        // TODO: The dimension of your projection for the pendulum
        return 0;
    }

    void project(const ompl::base::State */* state */, Eigen::Ref<Eigen::VectorXd> /* projection */) const override
    {
        // TODO: Your projection for the pendulum
    }
};

void pendulumODE(const ompl::control::ODESolver::StateType &q, const ompl::control::Control *c,
                 ompl::control::ODESolver::StateType &qdot)
{
    // Retrieve Control values: T(torque)
    const double *u = c->as<ompl::control::RealVectorControlSpace::ControlType>()->values;
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

void pendulumPostIntegration(const bsse::State* /*state*/, const Control* /*constol*/, const double /*duration*/, ob::State* result) {
    ompl::base::SO2StateSpace SO2;
    // ensure orientation lies between 0 and 2*pi
    ob::RealVectorStateSpace& s = *result->as<ob::RealVectorStateSpace>(2);
    SO2.enforceBounds(s[0]);
}

bool isStateValid(const oc::SpaceInformation *si, const ob::State *state) {
    //TODO: what is the reason we need to implement this function?
    const auto *singleState = state->as<ob::RealVectorStateSpace::StateType>();
    return si->satisfiesBounds(singleState);
}

ompl::control::SimpleSetupPtr createPendulum(double t)
{
    // create workspace
    auto space(std::make_shared<ob::RealVectorStateSpace>(2));
    ob::RealVectorBounds bounds(2);
    bounds.setHigh(0, 10);
    bounds.setLow(0, -10);

    space->setBounds(bounds);

    // create control space
    auto cspace(std::make_shared<ob::RealVectorStateSpace>(1));
    ob::RealVectorBounds bounds(1);
    bounds.setLow(-t);
    bounds.setHigh(t);

    cspace->setBounds(bounds);

    // define setup class
    oc::SimpleSetup ss(cspace);

    // set state validity check
    oc::SpaceInformation *si = ss.getSpaceInformation().get();
    ss.setStateValidityChecker(
        [si](const ob::State *state) { return isStateValid(si, state); };
    )
    auto odesolver(std::make_shared<oc::ODEBasicSolver<>>(ss.getSpaceInformation(), &pendulumODE));
    ss.setStatePropagator(oc::ODESolver::getStatePropagator(odesolver, &pendulumPostIntegration));
}

void planPendulum(ompl::control::SimpleSetupPtr &/* ss */, int /* choice */)
{
    // TODO: Do some motion planning for the pendulum
    // choice is what planner to use.
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
