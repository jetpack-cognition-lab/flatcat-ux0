/*---------------------------------+
 | Matthias Kubisch                |
 | kubisch@informatik.hu-berlin.de |
 | Flatcat2                        |
 | Juli 7th 2021                   |
 +---------------------------------*/

/* standard library */
#include <unistd.h>
#include <signal.h>
#include <bitset>
#include <experimental/filesystem>

/* framework */
#include <common/log_messages.h>
#include <common/basic.h>
#include <common/globalflag.h>
#include <common/modules.h>
#include <common/stopwatch.h>
#include <common/timer.h>
#include <common/datalog.h>

/* libsensorimotor */
#include <motorcord.hpp>
#include <sensorimotor.hpp>

/* flatcat */
#include "system/robot.hpp"
#include "system/control.hpp"
#include "system/settings.hpp"
#include "system/actions.hpp"
#include "system/logging.hpp"
#include "system/communication.hpp"

#include "learning/learning.hpp"


namespace supreme {

	// commandline option parsing
	char* getCmdOption(char ** begin, char ** end, const std::string & option) {
		char ** itr = std::find(begin, end, option);
		if (itr != end && ++itr != end)
			return *itr;
		return 0;
	}

	// commandline option exist
	bool cmdOptionExists(char** begin, char** end, const std::string& option) {
		return std::find(begin, end, option) != end;
	}


class ButtonPauseStatus {
	bool state = false;
	bool pressed = false;
	//bool paused = false;
public:

	bool execute_cycle(bool s, bool paused)
	{
		bool state = s;
		if (pressed and !state)
			paused = !paused;
		pressed = state;
		return paused;
	}

	//bool is_paused(void) const { return paused; }
};

class MainApplication
{
public:
	MainApplication(int argc, char** argv, GlobalFlag& exitflag)
	: settings(argc, argv)
	, exitflag(exitflag)
	, robot(settings)
	, control(robot, settings)
	, timer_mainloop( static_cast<uint64_t>(constants::us_per_sec/settings.update_rate_Hz), /*enable=*/true )
	, timer_shutdown(constants::us_per_sec*60*settings.time_to_shutdown_min)
	, motors_log(robot.motorcord)
	, logger(argc, argv)
	, learning(robot, control, settings)
	, com(robot, control, settings, exitflag, learning)
	, watch()
	, button()
	{

		if (std::experimental::filesystem::exists(settings.save_folder)) {
			if (settings.clear_state) {
				wrn_msg("Overriding state: %s", settings.save_state_name.c_str());
				save(settings.save_folder);

			} else
				load(settings.save_folder);
		}
		else {
			basic::make_directory(settings.save_folder.c_str());
			save(settings.save_folder);
		}

		if (logger.is_enabled())
			logger.log("Time_ms_____ Statusflags_____ TL S Ubat_ Ubus_%s", motors_log.header().c_str());

		control.paused_by_user = settings.initial_pause;
		control.voicemode = (settings.voicemode > 0);
	}

	void finish() {
		exitflag.enable();
		sts_msg("bb flat");
	};

	bool execute_cycle();
	void learning_cycle(void);

	void save(std::string f, bool log_status = true) {
		if (log_status) sts_msg("Saving state: %s", settings.save_state_name.c_str());
		learning.save(f);
	}

	void load(std::string f) {
		sts_msg("Loading state: %s", settings.save_state_name.c_str());
		learning.load(f);
	}

private:
	FlatcatSettings             settings;
	GlobalFlag&                 exitflag;

	/* robot baseline */
	FlatcatRobot                robot;
	FlatcatControl              control;

	SimpleTimer                 timer_mainloop,
	                            timer_shutdown;

	Motor_Log                   motors_log;
	Datalog                     logger;
	FlatcatLearning             learning;
	FlatcatCommunication        com;

	Stopwatch                   watch;
	ButtonPauseStatus           button;


	unsigned long cycles = 0;
	unsigned remaining_time_us = 0;

};

} /* namespace supreme */
