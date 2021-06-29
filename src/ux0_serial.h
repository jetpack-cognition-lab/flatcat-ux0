/*---------------------------------+
 | Matthias Kubisch                |
 | kubisch@informatik.hu-berlin.de |
 | Flatcat2                        |
 | Feb 2021                        |
 +---------------------------------*/

#include <signal.h>
#include <bitset>

#include <common/log_messages.h>
#include <common/basic.h>
#include <common/globalflag.h>
#include <common/modules.h>
#include <common/stopwatch.h>
#include <common/timer.h>
#include <common/datalog.h>

#include <motorcord.hpp>
#include <sensorimotor.hpp>



class Motor_Log : public Loggable<768> {

    supreme::motorcord const& motors;

    std::string replace_index(std::string str, uint8_t idx) {
       for (auto &s : str) {
           if (s == '?') s = std::to_string(idx).at(0);
       }
       return str;
    }

public:
    std::string header_template = " vol?__ pos?__ cur?__ sup? tmp?";

    Motor_Log(supreme::motorcord const& motors) : motors(motors) {}

    std::string header(void) {
        std::string result = "";
        for (std::size_t i = 0; i < motors.size() -1; ++i) //TODO w/o battery module
            result.append(replace_index(header_template, i));
        return result;
    }

    const char* log()
    {
        for (std::size_t i = 0; i < motors.size() -1 /* "-1"=TODO*/; ++i) {
            auto const& m = motors[i];
            append( " %+5.3f %+5.3f %+5.3f %4.2f %04.1f"
                  , m.get_data().output_voltage
                  , m.get_data().position
                  , m.get_data().current
                  , m.get_data().voltage_supply
                  , m.get_data().temperature );
        }
        return done();
    }
};

enum StatusBits : uint8_t
{
	button_pressed      =  0,
	charger_connected   =  1,
	battery_charging    =  2,
	limiter_fault       =  3,
	vbus_charging       =  4,
	no_battery_inserted =  5,
//	reserved_6          =  6,
	shutdown_initiated  =  7,
	vbat_ov_warning     =  8,
	vbat_uv_warning     =  9,
	vbat_uv_error       = 10,
	vbus_ov_warning     = 11,
	vbus_uv_warning     = 12,
//	reserved_13         = 13,
//	reserved_14         = 14,
//	reserved_15         = 15,
};

namespace constants {
    const float update_rate_Hz = 100.0f;
    const unsigned us_per_sec = 1000000;
}

class MainApplication
{
public:
    MainApplication(int argc, char** argv)
    : motors(4, constants::update_rate_Hz, false)
    , battery(motors[3])
    , timer( static_cast<uint64_t>(constants::us_per_sec/constants::update_rate_Hz), /*enable=*/true )
    , motors_log(motors)
    , logger(argc, argv)
    {
        for (unsigned i = 0; i < 3; ++i) {
            motors[i].set_voltage_limit(0.30);
            motors[i].set_disable_position_limits(-0.9,0.9);
            motors[i].set_csl_limits(-0.9,0.9);

            motors[i].set_target_csl_fb(1.017);
            motors[i].set_target_csl_gain(2.2);
            motors[i].set_target_csl_mode(1.0);
            motors[i].set_controller_type(supreme::sensorimotor::Controller_t::csl);

        }
        battery.set_voltage_limit(1.0);

        if (logger.is_enabled())
            logger.log("Statusflags_____ TL S Ubat_ Ubus_%s", motors_log.header().c_str());
    }

    bool execute_cycle();
    void finish() { sts_msg("bb flat"); };

private:

    supreme::motorcord      motors;
    supreme::sensorimotor&  battery;
    SimpleTimer             timer;
    Motor_Log               motors_log;
    Datalog                 logger;

    unsigned long cycles = 0;

    float avg_Umot = 0.0;
    float avg_Ubus = 0.0;
    float csl_mode = 0.5;

    bool inhibited = false;
    bool user_pause = false;
    bool user_button_pressed = false;
    bool user_button_released = false;
};

