#ifndef FLATCAT_REWARD_HPP
#define FLATCAT_REWARD_HPP


#include <learning/reward.h>
#include <learning/learning_machine_interface.h>

namespace learning {

class flatcat_reward_space : public reward_base
{
public:
    flatcat_reward_space(  )
    : reward_base(2)
    {
        rewards.emplace_back( "intrinsic basic", []() { assert(false); return 0; } );
        rewards.emplace_back( "intrinsic super", []() { assert(false); return 0; } );
    }

    void add_intrinsic_rewards( learning::Learning_Machine_Interface const& basic_learner
                              , learning::Learning_Machine_Interface const& super_learner )
    {
        rewards.at(0) = { "intrinsic basic", [&basic_learner]() { return 1000*basic_learner.get_learning_progress(); } };
        rewards.at(1) = { "intrinsic super", [&super_learner]() { return 1000*super_learner.get_learning_progress(); } };
    }
};

} /* namespace learning */

#endif /* FLATCAT_REWARD_HPP */
