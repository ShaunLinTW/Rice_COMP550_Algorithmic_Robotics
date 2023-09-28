///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 3
// Authors: Jason Zhang(jz118), Shaun Lin(hl116)
//////////////////////////////////////

#include <iostream>
#include <fstream>

// The collision checker routines
#include "CollisionChecking.h"

// Your random tree planner
#include "RTP.h"

//State Space generation
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

namespace ob = ompl::base;
namespace og = ompl::geometric;


void planPoint(const std::vector<Rectangle> &obstacles)
{
    // construct the state space we are planning in
    auto space(std::make_shared<ob::RealVectorStateSpace>(2));

    // set the bounds for the R^2
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-5);
    bounds.setHigh(5);
    space->setBounds(bounds);

    // define a simple setup class
    og::SimpleSetup ss(space);
    ss.setStateValidityChecker(std::bind(isValidStatePoint, std::placeholders::_1, obstacles));

    // Set start and goal state
    ob::ScopedState<ob::RealVectorStateSpace> start(space);
    start[0] = -4.5;
    start[1] = -4.5;

    ob::ScopedState<ob::RealVectorStateSpace> goal(space);
    goal[0] = 4.0;
    goal[1] = 3.0;
    ss.setStartAndGoalStates(start, goal, 0.05);

    // set RTP as planner
    auto planner(std::make_shared<og::RTP>(ss.getSpaceInformation()));
    ss.setPlanner(planner);

    // attempt to solve the problem within twenty seconds of planning time
    ob::PlannerStatus solved = ss.solve(20.0);

    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        // if ss is a ompl::geometric::SimpleSetup object
        ss.getSolutionPath().printAsMatrix(std::cout);

        //save data to a txt file
        std::ofstream fout;
        fout.open("./src/exercise2_visualization/point.txt");
        ss.getSolutionPath().printAsMatrix(fout);
        fout.close();
    }
    else
        std::cout << "No solution found" << std::endl;
}
    
void planBox(const std::vector<Rectangle> &obstacles)
{
        // construct the state space we are planning in
    auto space(std::make_shared<ob::SE2StateSpace>());

    // set the bounds for the R^2 part of SE(2)
    ob::RealVectorBounds bounds(2);
    bounds.setLow(-5);
    bounds.setHigh(5);
    space->setBounds(bounds);

    // define a simple setup class
    og::SimpleSetup ss(space);
    ss.setStateValidityChecker(std::bind(isValidStateSquare, std::placeholders::_1, 0.25, obstacles));

    // Set start and goal state
    ob::ScopedState<ob::SE2StateSpace> start(space);
    start->setX(-4.5);
    start->setY(-4.5);
    start->setYaw(0.785);

    ob::ScopedState<ob::SE2StateSpace> goal(space);
    goal->setX(4.0);
    goal->setY(3.0);
    goal->setYaw(2.355);
    ss.setStartAndGoalStates(start, goal, 0.05);

    // set RTP as planner
    auto planner(std::make_shared<og::RTP>(ss.getSpaceInformation()));
    ss.setPlanner(planner);

    // attempt to solve the problem within 1 min of planning time
    ob::PlannerStatus solved = ss.solve(60.0);

    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        // if ss is a ompl::geometric::SimpleSetup object
        ss.getSolutionPath().printAsMatrix(std::cout);

        //save data to a txt file
        std::ofstream fout;
        fout.open("./src/exercise2_visualization/box.txt");
        ss.getSolutionPath().printAsMatrix(fout);
        fout.close();
    }
    else
        std::cout << "No solution found" << std::endl;
}

void makeEnvironment1(std::vector<Rectangle> &obstacles){
    Rectangle rect;
    rect.x = -3;
    rect.y = -2;
    rect.width = 4;
    rect.height = 1;
    obstacles.push_back(rect);

    rect.x = 3;
    rect.y = 0.5;
    rect.width = 2;
    rect.height = 1;
    obstacles.push_back(rect);

    rect.x = 1.5;
    rect.y = 1;
    rect.width = 1;
    rect.height = 2;
    obstacles.push_back(rect);

    rect.x = -2;
    rect.y = 0;
    rect.width = 2;
    rect.height = 3;
    obstacles.push_back(rect);
}

void makeEnvironment2(std::vector<Rectangle> &obstacles)
{
    Rectangle rect;
    rect.x = 1;
    rect.y = -4;
    rect.width = 1;
    rect.height = 4;
    obstacles.push_back(rect);

    rect.x = -2;
    rect.y = -5;
    rect.width = 1;
    rect.height = 3;
    obstacles.push_back(rect);

    rect.x = 1;
    rect.y = 1;
    rect.width = 1;
    rect.height = 3;
    obstacles.push_back(rect);

    rect.x = 1;
    rect.y = 0;
    rect.width = 5;
    rect.height = 1;
    obstacles.push_back(rect);

    rect.x = -3;
    rect.y = 2;
    rect.width = 4;
    rect.height = 1;
    obstacles.push_back(rect);

    rect.x = -5;
    rect.y = -0.5;
    rect.width = 4.5;
    rect.height = 1;
    obstacles.push_back(rect);
}

int main(int /* argc */, char ** /* argv */)
{
    int robot, choice;
    std::vector<Rectangle> obstacles;

    do
    {
        std::cout << "Plan for: " << std::endl;
        std::cout << " (1) A point in 2D" << std::endl;
        std::cout << " (2) A rigid box in 2D" << std::endl;

        std::cin >> robot;
    } while (robot < 1 || robot > 2);

    do
    {
        std::cout << "In Environment: " << std::endl;
        std::cout << " (1) Environment 1" << std::endl;
        std::cout << " (2) Environment 2" << std::endl;

        std::cin >> choice;
    } while (choice < 1 || choice > 2);

    switch (choice)
    {
        case 1:
            makeEnvironment1(obstacles);
            break;
        case 2:
            makeEnvironment2(obstacles);
            break;
        default:
            std::cerr << "Invalid Environment Number!" << std::endl;
            break;
    }

    switch (robot)
    {
        case 1:
            planPoint(obstacles);
            break;
        case 2:
            planBox(obstacles);
            break;
        default:
            std::cerr << "Invalid Robot Type!" << std::endl;
            break;
    }

    return 0;
}
