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


class ButtonPauseStatus {
    bool state = false;
    bool pressed = false;
    bool paused = false;
public:

    bool execute_cycle(bool s)
    {
        bool state = s;
	    if (pressed and !state) {
		    if (paused) {
			    paused = false;
		    } else {
			    paused = true;
		    }
	    }
	    pressed = state;
        return paused;
    }

    bool is_paused(void) const { return paused; }
};

class MainApplication
{
public:
    MainApplication(int argc, char** argv, GlobalFlag& exitflag)
    : settings(argc, argv)
    , exitflag(exitflag)
    , robot(settings)
    , control(robot, settings)
    , timer( static_cast<uint64_t>(constants::us_per_sec/settings.update_rate_Hz), /*enable=*/true )
    , motors_log(robot.motorcord)
    , logger(argc, argv)
    , learning(robot, control, settings)
    , com(robot, settings, exitflag, learning)
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

    }

    void finish() {
        exitflag.enable();
        sts_msg("bb flat");
    };

    bool execute_cycle();
    void learning_cycle(void);

    void save(std::string f) {
        sts_msg("Saving state: %s", settings.save_state_name.c_str());
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
    SimpleTimer                 timer;
    Motor_Log                   motors_log;
    Datalog                     logger;
    FlatcatLearning             learning;
    FlatcatCommunication        com;

    Stopwatch                   watch;
    ButtonPauseStatus           button;


    unsigned long cycles = 0;

    bool inhibited = false;
    bool paused = false;


};

} /* namespace supreme */

