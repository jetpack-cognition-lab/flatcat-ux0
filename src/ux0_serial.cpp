#include "./ux0_serial.h"

static GlobalFlag exitflag;

void
signal_terminate_handler(int signum)
{
	sts_msg("\nReceived signal %d. Exiting.\n", signum);
	exitflag.enable();
}


/* TODO: features
	+ adjust CSL
	+ find bug in voice mode
	+ set voltage level lower according to battery life
	+ buzz for button press (in booted mode)
	+ can we detect the fur?
	+ increase button shutdown duration by 1 sec
*/

namespace supreme {

bool
MainApplication::execute_cycle(void)
{
	robot.execute_cycle();
	auto const& status = robot.get_status();

	/* pause flag can also be set/cleared by the user via web ui */
	control.paused_by_user = button.execute_cycle(robot.is_button_pressed(), control.paused_by_user);

	/* print on terminal each second */
	if (cycles % 100 == 0)
		sts_msg("Ub%4.2f Uc%4.2f Um%4.2f I%5.1f X%02u S=%u E%u T%u F=%s %s C=%3.1f %u"
		       , status.Ubat, status.Ubus, status.Umot, status.Ilim, status.ttlive, status.state
		       , status.motorcord.errors, status.motorcord.timeouts
		       , status.flag_str.c_str(), robot.is_resting()? "RES" : "ACT"
		       , control.csl_cur_mode[0], remaining_time_us);

	/* set tones for all motors */
	control.set_motor_voice();

	/* check charger connected or paused -> resting with standby bus power */
	if (robot.is_charger_connected() or control.paused_by_user) {
		robot.set_bus_voltage(settings.normed_resting_bus_voltage);
		control.disable_motors();
		robot.state = FlatcatRobot::FlatcatState_t::resting;
		learning.enabled = false;
		if (not com.is_connected())
			timer_shutdown.start();
		else {
			timer_shutdown.stop(); // don't shutdown if tcp client is still connected
			timer_shutdown.reset();
		}
	}
	else {
		robot.set_bus_voltage(settings.normed_active_bus_voltage);
		robot.state = FlatcatRobot::FlatcatState_t::active;
		learning.enabled = true;
		timer_shutdown.stop();
		timer_shutdown.reset();
	}

	/* active vs. resting, check motors's voltage */
	if (status.Umot < settings.voltage_disable_motors)
		robot.inhibit();
	else if (status.Umot < settings.voltage_regenerate_lo or robot.is_resting()) {
		control.set_resting_mode();
	}
	else if (status.Umot > settings.voltage_regenerate_hi) {
		control.set_active_mode();
	}

	learning.execute_cycle();
	com.execute_cycle(cycles);

	++cycles;

	/* data logging */
	if ((cycles % settings.cycles_log == 0) and logger.is_enabled())
		logger.log("%12llu %s %02u %u %5.3f %5.3f%s"
		          , watch.get_current_time_ms()
		          , status.flag_str.c_str(), status.ttlive, status.state
		          , status.Ubat, status.Ubus, motors_log.log());


	if (cycles % settings.cycles_nextfile == 0) // each hour
		logger.next();

	if (cycles % settings.learning_save_cycles == 0)
		save(settings.save_folder); // save learning data

	if (timer_shutdown.is_timed_out()) {
		sts_msg("Exceeded standby time of %u minutes.\n"
		        "Sending shutdown request to Energy Module."
		       , settings.time_to_shutdown_min);
		robot.battery.disable();
	}

	/* shutdown */
	const bool is_shutdown_signaled = robot.is_shutdown_signaled();
	if (is_shutdown_signaled or robot.is_powerfail()) {
		robot.state = FlatcatRobot::FlatcatState_t::halting;
		sts_msg("Shutdown flatcat due to %s", is_shutdown_signaled ? "BMS notified shutdown" : "unexpected power fail");
		sts_msg("Last measurements: Ubat=%4.2f Ubus=%4.2f T=%02u F=%s"
		       , status.Ubat, status.Ubus, status.ttlive, status.flag_str.c_str());
		control.disable_motors();
		robot.motorcord.execute_cycle(); // update motors before quitting
		return false;
	}

	remaining_time_us = 0;
	while(!timer_mainloop.check_if_timed_out_and_restart()) {
		usleep(50);
		remaining_time_us += 50; //TODO log this?
	}

	return true;
}

} /* namespace supreme */

#define XSTR(x) #x
#define STR(x) XSTR(x)

int main(int argc, char* argv[])
{
	sts_msg("Initializing flatcat <3");
	srand((unsigned) time(NULL));
	signal(SIGINT, signal_terminate_handler);

	/* handle version switch before initializing app */
	if(supreme::cmdOptionExists(argv, argv+argc, "-v")) {
		sts_msg("%s version: %s", argv[0], STR(__COMPILETIMESTAMP__));
		return EXIT_SUCCESS;
	}

	supreme::MainApplication app(argc, argv, exitflag);

	sts_msg("Starting main loop.");
	while(!exitflag.status())
	{
		if (!app.execute_cycle()) {
			app.finish();
			sts_add("Synchronizing filesystem...");
			sync();
			sts_msg("DONE.");
			sts_msg("________\nSHUTDOWN");
			if (0 != system("sudo shutdown -h now"))
				wrn_msg("Error while calling shutdown command.");
			return EXIT_FAILURE;
		}
	}

	/* terminate application w/o shutdown */
	app.finish();
	sts_msg("____\nDONE.");
	return EXIT_SUCCESS;
}
