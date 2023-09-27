///////////////////////////////////////
// COMP/ELEC/MECH 450/550
// Project 3
// Authors: Jason Zhang(jz118), Shaun Lin(hl116)
//////////////////////////////////////

#include "RTP.h"
#include <limits>
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/tools/config/SelfConfig.h"

// TODO: Implement RTP as described
ompl::geometric::RTP::RTP(const base::SpaceInformationPtr &si)
  : base::Planner(si, "RTP")
{
    specs_.approximateSolutions = true;
    specs_.directed = true;

    Planner::declareParam<double>("range", this, &RTP::setRange, &RTP::getRange, "0.:1.:10000.");
    Planner::declareParam<double>("goal_bias", this, &RTP::setGoalBias, &RTP::getGoalBias, "0.:.05:1.");
    // Planner::declareParam<bool>("intermediate_states", this, &RTP::setIntermediateStates, &RTP::getIntermediateStates,
    //                             "0,1");
}

ompl::geometric::RTP::~RTP()
{
    freeMemory();
}

void ompl::geometric::RTP::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    
    lastGoalMotion_ = nullptr;
}

void ompl::geometric::RTP::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    // if (!nn_)
    //     nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    // nn_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
}

void ompl::geometric::RTP::freeMemory()
{
    // if (nn_)
    // {
    //     std::vector<Motion *> motions;
    //     nn_->list(motions);
    //     for (auto &motion : motions)
    //     {
    //         if (motion->state != nullptr)
    //             si_->freeState(motion->state);
    //         delete motion;
    //     }
    // }
    for (auto &motion : RTPtree)
    {
        if (motion->state != nullptr)
            si_->freeState(motion->state);
        delete motion;
    }
}

ompl::base::PlannerStatus ompl::geometric::RTP::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        // nn_->add(motion);
    }

    // if (nn_->size() == 0)
    // {
    //     OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
    //     return base::PlannerStatus::INVALID_START;
    // }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), RTPtree.size());

    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();

    auto *rmotion = new Motion(si_);
    base::State *rstate = rmotion->state;

    base::State *xstate = si_->allocState();

    
    while (!ptc)
    {
        /* STEP1: sample random state Qa from the exisitng tree */
        Motion *nmotion = RTPtree[rng_.uniformInt(0, RTPtree.size()-1)];
        std::cout << 
        /* STEP2: sample random state Qb (with goal biasing) */
        if ((goal_s != nullptr) && rng_.uniform01() < goalBias_ && goal_s->canSample()) {
            goal_s->sampleGoal(rstate);
        } else {
            sampler_->sampleUniform(rstate);
        }
        
        /* STEP3: check if there is collision */
        if (si_->checkMotion(nmotion->state, rstate))
        {
            // motion is the new node
            auto *motion = new Motion(si_);
            si_->copyState(motion->state, rstate);
            motion->parent = nmotion;
            RTPtree.push_back(motion);

            nmotion = motion;

            double dist = 0.0;
            bool sat = goal->isSatisfied(nmotion->state, &dist);
            if (sat)
            {
                approxdif = dist;
                solution = nmotion;
                break;
            }
            if (dist < approxdif)
            {
                approxdif = dist;
                approxsol = nmotion;
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
        auto path(std::make_shared<PathGeometric>(si_));
        for (int i = mpath.size() - 1; i >= 0; --i)
            path->append(mpath[i]->state);
        
        pdef_->addSolutionPath(path, approximate, approxdif, getName());
        solved = true;
    }

    si_->freeState(xstate);
    if (rmotion->state != nullptr)
        si_->freeState(rmotion->state);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states", getName().c_str(), RTPtree.size());

    return {solved, approximate};
}

void ompl::geometric::RTP::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    if (lastGoalMotion_ != nullptr)
        data.addGoalVertex(base::PlannerDataVertex(lastGoalMotion_->state));

    for (auto &motion : RTPtree)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motion->state));
        else
            data.addEdge(base::PlannerDataVertex(motion->parent->state), base::PlannerDataVertex(motion->state));
    }
}
