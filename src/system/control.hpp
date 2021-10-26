#ifndef FLATCAT_CONTROL_HPP
#define FLATCAT_CONTROL_HPP

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

    bool enabled = false;

    FlatcatRobot&             robot;
    FlatcatSettings const&    settings;
    TargetPosition_t          usr_params;     // for testing and demo

    ControlMode_t             cur_mode, tar_mode;

    std::vector<float> csl_tar_mode = {0.0f, 0.0f, 0.0f};
    std::vector<float> csl_cur_mode = {0.0f, 0.0f, 0.0f};


    //std::vector<jcl::NeuralOscillator> so2;

    FlatcatControl(FlatcatRobot& robot, FlatcatSettings const& settings)
    : robot(robot)
    , settings(settings)
    , usr_params()
    , cur_mode(ControlMode_t::none)
    , tar_mode(ControlMode_t::none)
    //, so2(3)
    {

        /* CSL settings */
        for (unsigned i = 0; i < 3; ++i) {
            robot.motorcord[i].set_voltage_limit(settings.motor_voltage_limit);
            robot.motorcord[i].set_pwm_frequency(settings.motor_pwm_frequency);
            robot.motorcord[i].set_disable_position_limits(-0.9,0.9);
            robot.motorcord[i].set_csl_limits(-0.9,0.9);

            robot.motorcord[i].set_target_csl_fb(settings.motor_csl_param_gf);
            robot.motorcord[i].set_target_csl_gain(settings.motor_csl_param_gi);
            robot.motorcord[i].set_target_csl_mode(constants::csl_release_mode);
            robot.motorcord[i].set_controller_type(supreme::sensorimotor::Controller_t::csl);

        }

        /* set battery active level to maximum */
        robot.battery.set_voltage_limit(1.0);

        /*unsigned i = 0;
        for (auto& o: so2) {
            o.set_frequency(0.001*i++);
            o.restart();
        }*/
    }

    void execute_cycle(void)
    {

        if (enabled)
        {
            //TODO:
        }
    }

    void set_modes(float head, float body, float tail)
    {
        csl_tar_mode[0] = head;
        csl_tar_mode[1] = body;
        csl_tar_mode[2] = tail;
    }

    void disable(void) {
        robot.motorcord.disable_all();
        enabled = false;
    }

    void set_resting_mode(void) {
        for (std::size_t i = 0; i < robot.motorcord.size()-1; ++i) {
            robot.motorcord[i].set_controller_type(supreme::sensorimotor::Controller_t::csl);
            float mode = robot.motorcord[i].get_csl().get_mode();
		    mode += 0.01*(constants::csl_release_mode - mode);
            csl_cur_mode[i] = mode;
		    robot.motorcord[i].set_target_csl_mode(mode); // release mode
        }
    }

    void set_active_mode(void) {
        for (std::size_t i = 0; i < robot.motorcord.size()-1; ++i) {
			robot.motorcord[i].set_controller_type(supreme::sensorimotor::Controller_t::csl);
            float mode = robot.motorcord[i].get_csl().get_mode();
            mode += 0.01*(1.0/*csl_tar_mode[i]*/ - mode);
            csl_cur_mode[i] = mode;
			robot.motorcord[i].set_target_csl_mode(mode); // contraction mode
		}
    }

    void set_motor_voice(void) {
        /* set tones for all motors */
	    for (std::size_t i = 0; i < robot.motorcord.size()-1; ++i)
        if (robot.motorcord[i].is_active())
	    {
		    auto& m = robot.motorcord[i];
		    auto const& data = m.get_data();


		    if (csl_cur_mode[i] > .9
		    and fabs(data.output_voltage) > 0.01
		    and fabs(data.output_voltage) < 0.02)
			    m.set_pwm_frequency(tonetable[3*i+2]);
		    else {
			    m.set_pwm_frequency(settings.motor_pwm_frequency-i*1000);
            }
	    }
    }

    /*
    void welcome(void) {
        for (auto& o: so2)
            o.step();

        for (std::size_t i = 0; i < robot.motorcord.size()-1; ++i) {
            auto& m = robot.motorcord[i];
            m.set_pwm_frequency(tonetable[3*i+1]);
			//m.set_controller_type(supreme::sensorimotor::Controller_t::csl);
            m.set_controller_type(supreme::sensorimotor::Controller_t::voltage);
            so2[i].step();
			m.set_target_voltage( 0.04 * fabs(so2.at(i).x1));
//			m.set_target_csl_mode(fabs(so2.at(i).x1));
		}
    }*/

};

} /* namespace supreme */

#endif /* FLATCAT_CONTROL_HPP */

