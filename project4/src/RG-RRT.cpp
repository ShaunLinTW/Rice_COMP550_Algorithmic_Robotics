///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 4
// Authors: Jason Zhang(jz118), Shaun Lin(hl116)
//////////////////////////////////////

#include "RG-RRT.h"
#include "ompl/base/spaces/SE2StateSpace.h"
#include "ompl/control/spaces/RealVectorControlSpace.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"
#include <limits>

namespace ob = ompl::base;
namespace oc = ompl::control;

// TODO: Implement RGRRT as described

ompl::control::RGRRT::RGRRT(const SpaceInformationPtr &si) : base::Planner(si, "RGRRT")
{
    specs_.approximateSolutions = true;
    siC_ = si.get();

    Planner::declareParam<double>("goal_bias", this, &RGRRT::setGoalBias, &RGRRT::getGoalBias, "0.:.05:1.");
    Planner::declareParam<bool>("intermediate_states", this, &RGRRT::setIntermediateStates, &RGRRT::getIntermediateStates,
                                "0,1");
}

ompl::control::RGRRT::~RGRRT()
{
    freeMemory();
}

void ompl::control::RGRRT::setup()
{
    base::Planner::setup();
    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
}

void ompl::control::RGRRT::clear()
{
    Planner::clear();
    sampler_.reset();
    controlSampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    lastGoalMotion_ = nullptr;
}

void ompl::control::RGRRT::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion *> motions;
        nn_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state)
                si_->freeState(motion->state);
            if (motion->control)
                siC_->freeControl(motion->control);
            delete motion;
        }
    }
}

// setup reachable set for a motion
void ompl::control::RGRRT::setupReachableSet(Motion* const m) {
    // Retrieve the bounds of the control space
    auto bounds = siC_->getControlSpace()->as<RealVectorControlSpace>()->getBounds();
    
    // Extract the lower bounds and the difference in the first control dimension
    std::vector<double> low_bound = bounds.low;
    double difference = bounds.high[0] - low_bound[0];
    
    // Calculate the step size for the control values
    double step = difference / 10.0;
    
    // Initialize the control vector
    double control_value = low_bound[0];
    
    // Iterate 11 times to generate the reachable set
    for (size_t i = 0; i < 11; i++) 
    {
        // Create a new motion
        Motion *new_motion = new Motion(siC_);
        
        new_motion->control->as<RealVectorControlSpace::ControlType>()->values[0] = control_value;
        new_motion->control->as<RealVectorControlSpace::ControlType>()->values[1] = 0.0;  // Second control dimension is set to 0
        
        ob::State *newState = siC_->allocState();

        // Propagate the state
        siC_->propagateWhileValid(m->state, new_motion->control, 1, newState);
        
        // Add the resulting state to the reachable set
        m->reachableSet.push_back(new_motion);
        
        // Update the control value for the next iteration
        control_value += step;
    }
}


// select a reachable motion from the reachable set of qnear that is closest to qrand
int ompl::control::RGRRT::selectReachableMotion(const Motion* qnear, const Motion* qrand) {
    // Initialize variables to track the minimum distance and corresponding index
    double min_D = std::numeric_limits<double>::infinity();
    int minIndex = -1;

    // Get the reachable set from qnear
    const auto& reachable = qnear->reachableSet;

    // Iterate through the reachable set of qnear
    for (int i = 0; i < reachable.size(); ++i) {
        // Calculate the distance between the current reachable motion and qrand
        double curr_D = si_->distance(reachable[i]->state, qrand->state);

        // If this distance is smaller than the current minimum, update minD and minIndex
        if (curr_D < min_D) {
            min_D = curr_D;
            minIndex = i; // update the index of the motion in the reachable set that is closest to qrand
        }
    }

    // Return the index of the motion in the reachable set that is closest to qrand
    // If no motion is closer than any other, -1 will be returned
    return minIndex;
}

ompl::base::PlannerStatus ompl::control::RGRRT::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(siC_);
        si_->copyState(motion->state, st);
        siC_->nullControl(motion->control);
        setupReachableSet(motion); // setup reachable set for the motion
        nn_->add(motion);
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();
    if (!controlSampler_)
        controlSampler_ = siC_->allocDirectedControlSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());

    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();

    auto *rmotion = new Motion(siC_);
    base::State *rstate = rmotion->state;
    Control *rctrl = rmotion->control;
    base::State *xstate = si_->allocState();

    while (ptc == false)
    {
        /* sample random state (with goal biasing) */
        if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
            goal_s->sampleGoal(rstate);
        else
            sampler_->sampleUniform(rstate);

        /* find closest state in the tree */
        Motion *nmotion = nn_->nearest(rmotion);

        // if the qrand is too far from the qnear, select a reachable motion from the reachable set of qnear that is closest to qrand
        while (selectReachableMotion(nmotion, rmotion)) {
            // select a reachable motion from the reachable set of qnear that is closest to qrand
            int id = selectReachableMotion(nmotion, rmotion);

            // get the reachable motion
            Motion* reachable_motion = nmotion->reachableSet[id];

            // create a new motion
            auto *motion = new Motion(siC_);
            si_->copyState(motion->state, reachable_motion->state);
            siC_->copyControl(motion->control, reachable_motion->control);
            motion->steps = 1;
            motion->parent = nmotion;

            // add the motion to the tree
            nn_->add(motion);

            // update the nearest motion
            nmotion = motion;
        }

        /* sample a random control that attempts to go towards the random state, and also sample a control duration */
        unsigned int cd = controlSampler_->sampleTo(rctrl, nmotion->control, nmotion->state, rmotion->state);

        if (addIntermediateStates_)
        {
            // this code is contributed by Jennifer Barry
            std::vector<base::State *> pstates;
            cd = siC_->propagateWhileValid(nmotion->state, rctrl, cd, pstates, true);

            if (cd >= siC_->getMinControlDuration())
            {
                Motion *lastmotion = nmotion;
                bool solved = false;
                size_t p = 0;
                for (; p < pstates.size(); ++p)
                {
                    /* create a motion */
                    auto *motion = new Motion();
                    motion->state = pstates[p];
                    // we need multiple copies of rctrl
                    motion->control = siC_->allocControl();
                    siC_->copyControl(motion->control, rctrl);
                    motion->steps = 1;
                    motion->parent = lastmotion;
                    lastmotion = motion;
                    setupReachableSet(motion); // setup reachable set for the motion
                    nn_->add(motion);
                    double dist = 0.0;
                    solved = goal->isSatisfied(motion->state, &dist);
                    if (solved)
                    {
                        approxdif = dist;
                        solution = motion;
                        break;
                    }
                    if (dist < approxdif)
                    {
                        approxdif = dist;
                        approxsol = motion;
                    }
                }

                // free any states after we hit the goal
                while (++p < pstates.size())
                    si_->freeState(pstates[p]);
                if (solved)
                    break;
            }
            else
                for (auto &pstate : pstates)
                    si_->freeState(pstate);
        }
        else
        {
            if (cd >= siC_->getMinControlDuration())
            {
                /* create a motion */
                auto *motion = new Motion(siC_);
                si_->copyState(motion->state, rmotion->state);
                siC_->copyControl(motion->control, rctrl);
                motion->steps = cd;
                motion->parent = nmotion;

                setupReachableSet(motion); // setup reachable set for the motion
                nn_->add(motion);
                double dist = 0.0;
                bool solv = goal->isSatisfied(motion->state, &dist);
                if (solv)
                {
                    approxdif = dist;
                    solution = motion;
                    break;
                }
                if (dist < approxdif)
                {
                    approxdif = dist;
                    approxsol = motion;
                }
            }
        }
    }

    bool solved = false;
    bool approximate = false;
    if (solution == nullptr)
    {
        solution = approxsol;
        approximate = true;
    }

    if (solution != nullptr)
    {
        lastGoalMotion_ = solution;

        /* construct the solution path */
        std::vector<Motion *> mpath;
        while (solution != nullptr)
        {
            mpath.push_back(solution);
            solution = solution->parent;
        }

        /* set the solution path */
        auto path(std::make_shared<PathControl>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            if (mpath[i]->parent)
                path->append(mpath[i]->state, mpath[i]->control, mpath[i]->steps * siC_->getPropagationStepSize());
            else
                path->append(mpath[i]->state);
        solved = true;
        pdef_->addSolutionPath(path, approximate, approxdif, getName());
    }

    if (rmotion->state)
        si_->freeState(rmotion->state);
    if (rmotion->control)
        siC_->freeControl(rmotion->control);
    delete rmotion;
    si_->freeState(xstate);

    OMPL_INFORM("%s: Created %u states", getName().c_str(), nn_->size());

    return {solved, approximate};
}

void ompl::control::RGRRT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (nn_)
        nn_->list(motions);

    double delta = siC_->getPropagationStepSize();

    if (lastGoalMotion_)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (auto m : motions)
    {
        if (m->parent)
        {
            if (data.hasControls())
                data.addEdge(base::PlannerDataVertex(m->parent->state), base::PlannerDataVertex(m->state),
                             control::PlannerDataEdgeControl(m->control, m->steps * delta));
            else
                data.addEdge(base::PlannerDataVertex(m->parent->state), base::PlannerDataVertex(m->state));
        }
        else
            data.addStartVertex(base::PlannerDataVertex(m->state));
    }
}
