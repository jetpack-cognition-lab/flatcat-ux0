#ifndef FLATCAT_CONTROL_HPP
#define FLATCAT_CONTROL_HPP

#include <common/timer.h>

#include <control/jointcontrol.h>
#include <control/controlmixer.h>
#include <control/control_vector.h>

#include <controller/csl_control.hpp>
#include <controller/pid_control.hpp>

#include "system/robot.hpp"
#include "system/tones.hpp"
#include "system/settings.hpp"
#include "system/neural_osc.hpp"

namespace supreme {

typedef std::array<float, constants::num_joints> TargetPosition_t;

enum class ControlMode_t : uint8_t
{
	none,
	position,
	csl_hold,
	behavior,
	END_ControlMode_t
};

namespace constants {

	const float Kp = 3.0f;
	const float Ki = 0.01f;
	const float Kd = 0.0f;

	const std::array<const char*, (unsigned) ControlMode_t::END_ControlMode_t> mode_str = { "NONE", "POS", "HOLD", "BEHV" };
	const float csl_release_mode = 0.0f;

} /* constants */



class FlatcatControl {
public:

	bool paused_by_user = false;
	bool voicemode      = true;

	bool contraction_only = true;

	/* sleep requested by user */
	bool deep_sleep_user_req = false;

	FlatcatRobot&             robot;
	FlatcatSettings const&    settings;
	TargetPosition_t          usr_params;     // for testing and demo

	ControlMode_t             cur_mode, tar_mode;

	std::vector<float> csl_tar_mode = {0.0f, 0.0f, 0.0f};
	std::vector<float> csl_cur_mode = {0.0f, 0.0f, 0.0f};

	struct csl_settings_t {
		float gf,gi;
	} csl_settings;

	std::vector<float> motor_tones;

	FlatcatControl(FlatcatRobot& robot, FlatcatSettings const& settings)
	: robot(robot)
	, settings(settings)
	, usr_params()
	, cur_mode(ControlMode_t::none)
	, tar_mode(ControlMode_t::none)
	, csl_settings()
	, motor_tones()
	{
		/* B-flat blues scale */
		unsigned bt = 2; // base tone Bb (B flat)
		motor_tones.emplace_back(tonetable[bt + 0]); // Bb base tone
		motor_tones.emplace_back(tonetable[bt + 3]); // Db minor third
		motor_tones.emplace_back(tonetable[bt + 5]); // Eb perfect forth
		motor_tones.emplace_back(tonetable[bt + 6]); // E  diminished fifth
		motor_tones.emplace_back(tonetable[bt + 7]); // F  perfect fifth
		motor_tones.emplace_back(tonetable[bt +10]); // Ab minor seventh

		/* CSL settings */
		for (unsigned i = 0; i < robot.get_number_of_joints(); ++i) {
			robot.motorcord[i].set_voltage_limit(settings.motor_voltage_limit);
			robot.motorcord[i].set_pwm_frequency(settings.motor_pwm_frequency);
			robot.motorcord[i].set_disable_position_limits(-0.9,0.9); //=off, range is +/-0.6
			robot.motorcord[i].set_csl_limits(-0.20,-0.40,+0.35,+0.60);
			robot.motorcord[i].set_csl_noise_level(0.001);
			robot.motorcord[i].set_target_csl_fb(settings.motor_csl_param_gf);
			robot.motorcord[i].set_target_csl_gain(settings.motor_csl_param_gi);
			robot.motorcord[i].set_target_csl_mode(constants::csl_release_mode);
			robot.motorcord[i].set_stall_trig_vel(settings.motor_csl_stalltrigvel);
			robot.motorcord[i].set_controller_type(supreme::sensorimotor::Controller_t::csl);
		}

		/* set battery active level to maximum */
		robot.battery.set_voltage_limit(1.0);

		csl_settings.gf = settings.motor_csl_param_gf;
		csl_settings.gi = settings.motor_csl_param_gi;
	}

	void execute_cycle(void) { /*not implemented yet*/ }

	void set_modes(float head, float body, float tail)
	{
		csl_tar_mode[0] = head;
		csl_tar_mode[1] = body;
		csl_tar_mode[2] = tail;
	}

	void disable_motors(void) {
		for (std::size_t i = 0; i < robot.get_number_of_joints(); ++i)
			robot.motorcord[i].disable();
	}

	void set_resting_mode(void) {
		for (std::size_t i = 0; i < robot.get_number_of_joints(); ++i) {
			robot.motorcord[i].set_controller_type(supreme::sensorimotor::Controller_t::csl);
			float mode = robot.motorcord[i].get_csl().get_mode();
			mode += 0.01*(constants::csl_release_mode - mode);
			csl_cur_mode[i] = mode;
			robot.motorcord[i].set_target_csl_mode(mode); // release mode
		}
	}

	void set_active_mode(void) {
		for (std::size_t i = 0; i < robot.get_number_of_joints(); ++i) {
			robot.motorcord[i].set_controller_type(supreme::sensorimotor::Controller_t::csl);
			float mode = robot.motorcord[i].get_csl().get_mode();
			mode += 0.01*((contraction_only ? 1.0 : csl_tar_mode[i]) - mode);
			csl_cur_mode[i] = mode;
			robot.motorcord[i].set_target_csl_mode(mode); // contraction mode
		}
	}

	void set_motor_voice(void) {
		/* set tones for all motors */
		for (std::size_t i = 0; i < robot.get_number_of_joints(); ++i)
		{
			auto& mot = robot.motorcord[i];
			//printf("%u ",i);
			if (mot.is_active()) {
				auto const& data = mot.get_data();
				if (voicemode and csl_cur_mode[i] > .5
					and data.output_voltage > settings.vm_beg
					and data.output_voltage < settings.vm_end )
					mot.set_pwm_frequency(motor_tones.at(2*i));
				else if (voicemode and csl_cur_mode[i] > .5
						and data.output_voltage < -settings.vm_beg
						and data.output_voltage > -settings.vm_end )
						mot.set_pwm_frequency(motor_tones.at(2*i+1));
				else
					mot.set_pwm_frequency(settings.motor_pwm_frequency-i*1000);
			}
		}
	}

};

} /* namespace supreme */

#endif /* FLATCAT_CONTROL_HPP */
