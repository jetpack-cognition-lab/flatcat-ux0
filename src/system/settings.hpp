#ifndef FLATCAT_SETTINGS_HPP
#define FLATCAT_SETTINGS_HPP

#include <string.h>

#include <common/settings.h>
#include <common/vector_n.h>


namespace supreme {

namespace defaults {
	const std::string settings_filename = "flatcat.dat";

	const VectorN joint_offsets = { .0f /* HEAD 0 */
	                              , .0f /* BODY 1 */
	                              , .0f /* TAIL 2 */
	                              };

	const unsigned tcp_port = 7332;

	const float voltage_disable_motors = 4.70f; // h-bridges disable themselves at approx. 4V5
	const float voltage_regenerate_lo  = 4.80f;
	const float voltage_regenerate_hi  = 5.20f;
	const float voltage_emergency_shutdown = 2.5f;

	const float motor_voltage_limit = 0.5f; // in %
	const float motor_csl_param_gf = 1.00f; // contraction deactivated, must be enabled via setting
	const float motor_csl_param_gi = 1.5f;
	const float motor_csl_stalltrigvel = 0.05f;
	const unsigned motor_pwm_frequency = 22500;

	const unsigned motor_temperature_off = 55; // °C
	const unsigned motor_temperature_on  = 51; //

	const unsigned time_to_shutdown_min = 15; // 15 minutes
	const unsigned time_of_surprise_s = 5;
	const unsigned time_of_boredom_s = 30;

	const unsigned voicemode = 1;
}

namespace constants {
	const unsigned us_per_sec = 1000000;
	const float max_bus_voltage = 5.92f;
	const float voltage_norm_V = 6.0f;
}

class FlatcatSettings : public Settings_Base
{
public:

	/* logging */
	unsigned cycles_log      = 100;
	unsigned cycles_nextfile = 360000;

	/* communication */
	unsigned tcp_port;
	unsigned udp_port = 1900;
	std::string group = "224.0.0.1";//"239.255.255.252";

	const char* serial_devicename = "ttyUSB0";
	const float update_rate_Hz = 100.0f;
	const float normed_active_bus_voltage = 1.0;   // =6V
	const float normed_resting_bus_voltage = 0.4; // =5V

	float voltage_disable_motors;
	float voltage_regenerate_lo;
	float voltage_regenerate_hi;
	float voltage_emergency_shutdown;

	/* timer settings */
	unsigned time_to_shutdown_min;
	unsigned time_of_surprise_s;
	unsigned time_of_boredom_s;

	/* CSL motor control */
	float motor_voltage_limit;
	float motor_csl_param_gf;
	float motor_csl_param_gi;
	float motor_csl_stalltrigvel;
	unsigned motor_pwm_frequency;

	unsigned motor_temperature_off;
	unsigned motor_temperature_on;

	VectorN joint_offsets;

	bool voicemode = true;
	const float vm_beg = 0.005;
	const float vm_end = 0.025;

	VectorN sarsa_learning_rates = {0.05, 0.05, 0.005, 0.005};
	uint64_t trial_time_s = 60;
	uint64_t eigenzeit_steps = 1000; // 10 seconds max.

	float epsilon_exploration = 0.1;

	unsigned learning_save_cycles = 12000;
	std::string save_state_name;
	std::string save_folder = "./data/";
	bool clear_state;

	bool initial_pause = false;
	bool terminal_diag = false;

	/* gmes */
	struct GMES_Settings_t {
		unsigned number_of_experts = 64;
		unsigned experience_size = 1;
		float joint_gmes_learning_rate = 100.0;
		float local_learning_rate = 0.001;
	} gmes;

	FlatcatSettings(int argc, char **argv)
	: Settings_Base         (argc, argv                          , defaults::settings_filename.c_str())
	, tcp_port              (read_uint ("tcp_port"               , defaults::tcp_port                ))
	, voltage_disable_motors(read_float("voltage_disable_motors" , defaults::voltage_disable_motors  ))
	, voltage_regenerate_lo (read_float("voltage_regenerate_lo"  , defaults::voltage_regenerate_lo   ))
	, voltage_regenerate_hi (read_float("voltage_regenerate_hi"  , defaults::voltage_regenerate_hi   ))
	, time_to_shutdown_min  (read_uint ("time_to_shutdown_min"   , defaults::time_to_shutdown_min    ))
	, time_of_surprise_s    (read_uint ("time_of_surprise_s"     , defaults::time_of_surprise_s      ))
	, time_of_boredom_s     (read_uint ("time_of_boredom_s"      , defaults::time_of_boredom_s       ))
	, motor_voltage_limit   (read_float("motor_voltage_limit"    , defaults::motor_voltage_limit     ))
	, motor_csl_param_gf    (read_float("motor_csl_param_gf"     , defaults::motor_csl_param_gf      ))
	, motor_csl_param_gi    (read_float("motor_csl_param_gi"     , defaults::motor_csl_param_gi      ))
	, motor_csl_stalltrigvel(read_float("motor_csl_stalltrigvel" , defaults::motor_csl_stalltrigvel  ))
	, motor_pwm_frequency   (read_uint ("motor_pwm_frequency"    , defaults::motor_pwm_frequency     ))
	, motor_temperature_off (read_uint ("motor_temperature_off"  , defaults::motor_temperature_off   ))
	, motor_temperature_on  (read_uint ("motor_temperature_on"   , defaults::motor_temperature_on    ))
	, joint_offsets         (read_vec  ("joint_offsets"          , defaults::joint_offsets           ))
	, voicemode             (read_uint ("voicemode"              , defaults::voicemode               ))
	, save_state_name       (read_string_option(argc, argv, "-n", "--name", "default"                ))
	, clear_state           (read_option_flag  (argc, argv, "-c", "--clear"                          ))
	, initial_pause         (read_option_flag  (argc, argv, "-p", "--pause"                          ))
	, terminal_diag         (read_option_flag  (argc, argv, "-d", "--diag"                           ))
	{
		save_folder += save_state_name + "/";

		sts_msg("Done loading flatcat settings");

		if (motor_temperature_off < motor_temperature_on)
			wrn_msg("Motor temperature protection is OFF");
	}
};

} /* namespace supreme */

#endif /* FLATCAT_SETTINGS_HPP */
