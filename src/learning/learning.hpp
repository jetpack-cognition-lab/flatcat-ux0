#ifndef FLATCAT_LEARNING_HPP
#define FLATCAT_LEARNING_HPP

#include <common/timer.h>

#include <learning/gmes.h>
#include <learning/sarsa.h>
#include <learning/reward.h>
#include <learning/payload.h>
#include <learning/eigenzeit.h>
#include <learning/action_selection.h>
#include <learning/epsilon_greedy.h>

#include "gmes_joint_group.hpp"
#include "reward.hpp"
#include "actions.hpp"

namespace supreme {

class FlatcatLearning
{
public:
	FlatcatActions                  actions;
	learning::flatcat_reward_space  reward;
	learning::GMES_Joint_Group      gmes_joint_group;
	learning::GMES_Layer            super_layer;

	learning::Epsilon_Greedy        epsilon_greedy;
	SARSA                           agent;
	Policy_Selector                 policy_selector;
	learning::Eigenzeit             eigenzeit;

	SimpleTimer                     timer_surprise,
	                                timer_boredom;

	bool enabled = true;

	/* experimental */
	float learning_progress_avg = 0.f;
	float learning_progress_var = 0.f;

	const float eta = 0.0001; // iir lowpass filter coeff

	FlatcatLearning( FlatcatRobot const& robot
                   , FlatcatControl& control
                   , FlatcatSettings const& settings)
	: actions(control, settings)
	, reward()
	, gmes_joint_group( robot.get_joints()
	                  , settings.gmes.number_of_experts
	                  , settings.gmes.joint_gmes_learning_rate
	                  , settings.gmes.local_learning_rate
	                  , settings.gmes.experience_size
	                  )
	, super_layer( 16
	             , gmes_joint_group.get_activations()
	             , actions
	             , reward.get_number_of_policies()
	             , /*initialQ=*/ .1
	             , 10.0  // settings.joint_gmes_learning_rate
	             , 0.0005 // settings.local_learning_rate
	             , 1    // settings.experience_size
	             )
	/* reinforcement learning */
	, epsilon_greedy(super_layer.payload, actions, settings.epsilon_exploration)
	, agent(super_layer.payload, reward, epsilon_greedy, actions.get_number_of_actions(), settings.sarsa_learning_rates)
	, policy_selector(agent, reward.get_number_of_policies(), /*random_policy_mode = */true, settings.trial_time_s)
	, eigenzeit(super_layer.gmes, settings.eigenzeit_steps)
	, timer_surprise(constants::us_per_sec*settings.time_of_surprise_s, /*enable=*/true)
	, timer_boredom (constants::us_per_sec*settings.time_of_boredom_s , /*enable=*/true)
	{
		reward.add_intrinsic_rewards(gmes_joint_group, super_layer);
	}

	void execute_cycle(void)
	{
		gmes_joint_group.execute_cycle();
		super_layer     .execute_cycle();
		eigenzeit       .execute_cycle();
		reward          .execute_cycle();
		policy_selector .execute_cycle();

		if (enabled and eigenzeit.has_progressed()) {
			agent.execute_cycle(super_layer.gmes.get_winner());
			actions.execute_cycle(agent); //note: must be processed after agent's step.
		}

		if (eigenzeit.has_progressed())
			reward.clear_aggregations();

		learning_progress_avg = (1.f - eta) * learning_progress_avg
		                      + eta  * super_layer.get_learning_progress();

		learning_progress_var = (1.f - eta) * learning_progress_var
		                      + eta * std::abs(super_layer.get_learning_progress() - learning_progress_avg);
	}

	bool is_surprised(void) const {
		return (learning_progress_avg+learning_progress_var) < super_layer.get_learning_progress();
	}

	void save(std::string f) {
		gmes_joint_group.save(f);
		super_layer.save(f);
	}

	void load(std::string f) {
		gmes_joint_group.load(f);
		super_layer.load(f);
	}

};

} /* namespace supreme */

#endif /* FLATCAT_LEARNING_HPP */
