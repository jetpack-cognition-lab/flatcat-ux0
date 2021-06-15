/*---------------------------------+
 | Matthias Kubisch                |
 | kubisch@informatik.hu-berlin.de |
 | Flatcat2                        |
 | Feb 2021                        |
 +---------------------------------*/

#include <signal.h>

#include <common/log_messages.h>
#include <common/basic.h>
#include <common/globalflag.h>
#include <common/modules.h>
#include <common/stopwatch.h>

#include <motorcord.hpp>
#include <sensorimotor.hpp>


/*
class Motor_Log : public Loggable<768> {

    supreme::motorcord0 const& motors;

public:
    Motor_Log(supreme::motorcord0 const& motors) : motors(motors) {}

    const char* log()
    {
        for (std::size_t i = 0; i < motors.size(); ++i) {
            auto const& m = motors[i];
            append( " %+e %+e %+e %+e %+e %+e %+e"
                  , m.get_data().output_voltage
                  , m.get_data().position
                  , m.get_data().velocity
                  , m.get_data().current
                  //, m.get_data().voltage_backemf
                  , m.get_data().voltage_supply
                  , m.get_data().temperature );
        }
        return done();
    }
};*/

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

class MainApplication
{
public:
    MainApplication(int /*argc*/, char** /*argv*/)
    : motors(4, 100.0f, false)
    , battery(motors[3])
//    , motors_log(motors)
    {
        //motors[2].set_phi_disable(1.0);
        //motors[0].set_voltage_limit(1.0);
        //motors[0].set_phi_disable(1.0);

        // settings for blue motor
        for (unsigned i = 0; i < 3; ++i) {
            motors[i].set_voltage_limit(0.34);
            //motors[i].set_disable_position_limits(-0.7,0.7);
            //motors[i].set_csl_limits(-0.45,0.45);

            motors[i].set_disable_position_limits(-0.9,0.9);
            motors[i].set_csl_limits(-0.9,0.9);

            motors[i].set_target_csl_fb(1.006);
            motors[i].set_target_csl_gain(2.0);
            motors[i].set_target_csl_mode(1.0); // TODO enable csl, set mode to 1
            motors[i].set_controller_type(supreme::sensorimotor::Controller_t::csl);

        }
        //motors[0].set_loop_gain(3.0);
        //motors[1].set_loop_gain(3.0);

        battery.set_voltage_limit(1.0);
    }

    bool execute_cycle();
    void finish() { sts_msg("bb flat"); };

private:
    supreme::motorcord   motors;
    supreme::sensorimotor& battery;

    /* Logging */
    //Motor_Log             motors_log;

    unsigned long cycles = 0;

    float avg_Umot = 0.0;
    float avg_Ubus = 0.0;
    bool user_enable = false;
};

