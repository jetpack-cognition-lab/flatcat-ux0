#ifndef FLATCAT_ACTIONS_HPP
#define FLATCAT_ACTIONS_HPP

#include <learning/action_module.h>
#include <learning/reinforcement_learning.h>

#include <system/settings.hpp>
#include <system/control.hpp>

namespace supreme {

class FlatcatActions : public Action_Module_Interface
{

	FlatcatControl& control;
	FlatcatSettings const& settings;

	unsigned applied_policy = 0;
	unsigned applied_action = 0;
	unsigned applied_state  = 0;

	struct CSL_params { float head, body, tail; };


	std::vector<CSL_params> modes = { {1.0, 1.0, 1.0} // 0
	                                , {0.2, 0.6, 1.0} // 1
	                                , {0.4, 1.0, 0.4} // 2
	                                , {1.0, 0.6, 0.2} // 3
	                                , {0.2, 0.2, 0.2} // 4
	                                };

public:

	FlatcatActions( FlatcatControl& control
	              , FlatcatSettings const& settings )
	: control(control), settings(settings) {}

	std::size_t get_number_of_actions(void) const { return modes.size(); }
	std::size_t get_number_of_actions_available(void) const { return modes.size(); }
	bool exists(const std::size_t /*action_index*/) const {return true; }

	void execute_cycle(learning::RL_Interface const& learner)
	{
		/* update and check state + action from learner */
		applied_policy = learner.get_current_policy();
		applied_action = learner.get_current_action();
		applied_state  = learner.get_current_state();
		if (settings.terminal_diag) {
			sts_add("policy=%u, action=%u, state=%u",applied_policy,applied_action,applied_state);
			sts_msg("%3.1f %3.1f %3.1f ", modes.at(applied_action).head
			                            , modes.at(applied_action).body
			                            , modes.at(applied_action).tail );
		}
		control.set_modes( modes.at(applied_action).head
		                 , modes.at(applied_action).body
		                 , modes.at(applied_action).tail );
	}

};

} /* namespace supreme */

#endif /* FLATCAT_ACTIONS_HPP */
