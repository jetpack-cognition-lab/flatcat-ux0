#include "./ux0_serial.h"


static GlobalFlag exitflag;

void
signal_terminate_handler(int signum)
{
	sts_msg("\nReceived signal %d. Exiting.\n", signum);
	exitflag.enable();
}


/* TODO list
    + set voltage level lower according to battery life
*/

namespace supreme {

bool
MainApplication::execute_cycle(void)
{
	robot.execute_cycle();
	auto const& status = robot.get_status();


	paused = button.execute_cycle(robot.is_button_pressed());

	/* print on terminal each second */
	if (cycles % 100 == 0)
		sts_msg("Ub=%4.2f Uc=%4.2f Um=%4.2f I=%5.1f T=%02u S=%s E=%u T=%u F=%s %s C=%3.1f"
		       , status.Ubat, status.Ubus, status.Umot, status.Ilim, status.ttlive, status.state_str.c_str()
               , status.motorcord.errors, status.motorcord.timeouts
		       , status.flag_str.c_str(), inhibited? "INH" : "ACT"
                , control.csl_cur_mode[0]);

	/* set tones for all motors */
    control.set_motor_voice();

	/* check charger connected or paused -> inhibited with standby bus power */
	if (robot.is_charger_connected() or paused) {
        control.disable();
		inhibited = true;
        learning.enabled = false;
	}
	else {
		robot.battery.set_controller_type(supreme::sensorimotor::Controller_t::voltage);
		robot.battery.set_target_voltage(settings.normed_active_bus_voltage);
		inhibited = false;
        learning.enabled = true;
	}

	/* active vs. resting, check motors's voltage */
	if (status.Umot < settings.voltage_disable_motors) robot.motorcord.disable_all();
	else if (status.Umot < settings.voltage_regenerate_lo or paused) {
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


	/* emergency shutdown */
    bool is_shutdown_signaled = status.ttlive < 10;
	if (is_shutdown_signaled or status.Ubat < 2.7) {
		sts_msg("Shutdown flatcat due to %s", is_shutdown_signaled ? "BMS notified shutdown" : "unexpected power fail");
		sts_msg("Last measurements: Ubat=%4.2f Ubus=%4.2f T=%02u F=%s"
		       , status.Ubat, status.Ubus, status.ttlive, status.flag_str.c_str());
		robot.motorcord.disable_all();
		robot.motorcord.execute_cycle(); // update motors before quitting
		return false;
	}

	unsigned c = 0;
	while(!timer.check_if_timed_out_and_restart()) {
		usleep(100);
		++c; //TODO log this?
	}

	return true;
}



} /* namespace supreme */

int main(int argc, char* argv[])
{
	sts_msg("Initializing flatcat <3");
	srand((unsigned) time(NULL));
	signal(SIGINT, signal_terminate_handler);

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
			system("sudo shutdown -h now");
			return EXIT_FAILURE;
		}
	}

	/* terminate application w/o shutdown */
	app.finish();
	sts_msg("____\nDONE.");
	return EXIT_SUCCESS;
}
