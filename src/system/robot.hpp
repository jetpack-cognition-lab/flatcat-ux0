#ifndef FLATCAT_ROBOT_HPP
#define FLATCAT_ROBOT_HPP

#include <cassert>
#include <array>

#include <common/log_messages.h>
#include <robots/robot.h>
#include <robots/joint.h>
#include <robots/accel.h>

#include <communication_ctrl.hpp>
#include <motorcord.hpp>
#include <sensorimotor.hpp>

#include "system/settings.hpp"

namespace supreme {

namespace constants {
    const unsigned num_joints = 3;

    const std::array<int16_t, num_joints> dir = { +1, +1, +1};
    const float position_scale = 270.0/360.0;

} /* namespace constants */

class FlatcatRobot : public robots::Robot_Interface
{
public: //TODO undo "all public"
    supreme::motorcord     motorcord;
    supreme::sensorimotor& battery;

    std::size_t number_of_joints;
    std::size_t number_of_joints_sym;
    std::size_t number_of_accels;

    robots::Jointvector_t joints;
    robots::Accelvector_t accels;

    bool enabled = false;

    struct FlatcatStatus_t
    {
        float Umot = 5.0f;
        float Ubus = 5.0f;
        float Ubat = 3.6f;
        float Ilim = 0.0f;
        float Imot = 0.0f;
        uint8_t ttlive = 10;
        uint8_t state = 0;
        uint16_t flags = 0;
        std::string flag_str = "";
        std::string state_str = "";
        struct MotorcordStatus_t {
            unsigned errors = 0;
            unsigned timeouts = 0;
        } motorcord;
    } status;

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


    enum class FlatcatState_t
    {
        initializing = 0,
        paused       = 1,
        active       = 2,
        halting      = 3,
    } state ;


    /*TODO: enum class FlatcatState_t {

    }*/

public:


    FlatcatRobot(FlatcatSettings const& settings)
    : motorcord(constants::num_joints+1/*+bat*/, settings.update_rate_Hz, false)
    , battery(motorcord[3])
    , number_of_joints(constants::num_joints)
    , number_of_joints_sym(/* will be counted */)
    , number_of_accels(1)
    , joints()
    , accels()
    , status()
    , state(FlatcatState_t::initializing)
    {

        assert(motorcord.size() == constants::num_joints+1); // incl. bat
        sts_msg("Creating Flatcat Robot Interface");
        joints.reserve(number_of_joints);

        /* define joints */
        joints.emplace_back( 0, robots::Joint_Type_Normal, 0, "HEAD" , -0.75, +0.75, .0 );
        joints.emplace_back( 1, robots::Joint_Type_Normal, 1, "MIDL" , -0.75, +0.75, .0 );
        joints.emplace_back( 2, robots::Joint_Type_Normal, 2, "TAIL" , -0.75, +0.75, .0 );

        unsigned i = 0;
        for (auto const& j : joints) {
            assert(j.joint_id == i++);
            if (j.is_symmetric()) ++number_of_joints_sym; // count symmetric joints
        }

        accels.emplace_back();

        /* configure joints */
        assertion(settings.joint_offsets.size() == joints.size(), "%u =!= %u", settings.joint_offsets.size(), joints.size());
        for (auto const& j : joints) {
            unsigned i = j.joint_id;
            assert(i < constants::num_joints);
            auto& m  = motorcord[i];
            m.set_direction  (constants::dir.at(i)        );
            m.set_scalefactor(constants::position_scale   );
            m.set_offset     (settings.joint_offsets.at(i));
        }
    }

    std::size_t get_number_of_joints           (void) const { return number_of_joints;     }
    std::size_t get_number_of_symmetric_joints (void) const { return number_of_joints_sym; }
    std::size_t get_number_of_accel_sensors    (void) const { return number_of_accels;     }

    const robots::Jointvector_t& get_joints(void) const { return joints; }
          robots::Jointvector_t& set_joints(void)       { return joints; }

    const robots::Accelvector_t& get_accels(void) const { return accels; }
          robots::Accelvector_t& set_accels(void)       { return accels; }

    bool execute_cycle(void)
    {
        //TODO: write_motorcord();          /* write motors     */
        motorcord.execute_cycle();  /* read motor cord  */

        status.motorcord.errors   = motorcord.get_errors();
        status.motorcord.timeouts = motorcord.get_timeouts();

        read_motorcord();           /* read sensors     */
        return true;
    }

    double get_normalized_mechanical_power(void) const
    {
        float power = .0;
        for (unsigned i = 0; i < motorcord.size(); ++i) {
            auto const& m = motorcord[i];
            power += m.get_data().voltage_supply * m.get_data().current;
        }
        return power;
    }

    void read_motorcord(void)
    {
        for (auto& j : joints) {
            auto const& m = motorcord[j.joint_id];
            j.s_ang = m.get_data().position;
            j.s_vel = m.get_data().velocity_lpf;
            j.s_vol = m.get_data().voltage_supply/constants::voltage_norm_V; // normalized to 1=6V
            //TODO: current, temp,
        }

        /**TODO accelerometer
        auto const& r0 = spinalcord.status_data.motors[0];
        accels[0].a.x = +r0.acceleration.x;
        accels[0].a.y = -r0.acceleration.z;
        accels[0].a.z = +r0.acceleration.y; */

        update_status_data();
    }

    void write_motorcord(void) {
        for (auto& j : joints) {
            auto& m = motorcord[j.joint_id];
            if (enabled) {
                m.set_target_csl_mode( j.motor.get() );
            }
            j.motor.transfer();
            j.motor = .0f;
        }
    }

    void set_enable(bool value) { enabled = value; }

    void disable_motors(void) {
		for (auto& j : joints)
            motorcord[j.joint_id].set_target_csl_mode(.0);
	}

    /* non-robot interface member function */
    //SpinalCord::TimingStats const& get_motorcord_timing(void) const { return spinalcord.get_timing(); }
    void reset_motor_statistics(void) { /**TODO spinalcord.reset_statistics();*/ }

    supreme::motorcord const& get_motors(void) const { return motorcord; }
    supreme::motorcord      & set_motors(void)       { return motorcord; }


    FlatcatStatus_t const& get_status(void) const { return status; }

    bool is_charger_connected(void) const { return status.flags & (1 << StatusBits::charger_connected); }
    bool is_button_pressed(void) const { return status.flags & (1 << StatusBits::button_pressed); }

private:

    void update_status_data(void) {
        /* motors' voltages */
        float Umot = std::min( motorcord[0].get_data().voltage_supply,
                     std::min( motorcord[1].get_data().voltage_supply
                             , motorcord[2].get_data().voltage_supply ));

        /* get energymodule's data */
        auto const& dat = battery.get_data().raw_recv;

        /* battery/charger voltage */
        float Ubat = (dat[0]*256 + dat[1]) * 0.0001f; //.1 mV to V

        /* bus voltage measured by energymodule */
        float Ubus = (dat[2]*256 + dat[3]) * 0.0001f; //.1 mV to V

        float Ilim = (dat[4]*256 + dat[5]) * 0.1f;    //.1 mA to mA

        /* sum of motor currents */
        float Imot = motorcord[0].get_data().current
                   + motorcord[1].get_data().current
                   + motorcord[2].get_data().current;

        /* low-pass filters */
        status.Umot = 0.99f * status.Umot + 0.01f * Umot;
        status.Ubus = 0.99f * status.Ubus + 0.01f * Ubus;
        status.Ubat = 0.99f * status.Ubat + 0.01f * Ubat;
        status.Imot = 0.99f * status.Imot + 0.01f * Imot;
        status.Ilim = 0.99f * status.Ilim + 0.01f * Ilim;

        status.ttlive   = dat[6];
        status.state    = dat[7];
        status.flags    = dat[8]*256 + dat[9];
        status.flag_str = std::bitset<sizeof(status.flags) * 8>(status.flags).to_string();

        /* get battery module status */
        std::string state_str = "";
        switch(status.state) {
            case 0: state_str = "initializing     "; break;
	        case 1: state_str = "powered off      "; break;
	        case 2: state_str = "shutting down    "; break;
	        case 3: state_str = "notify subsystems"; break;
	        case 4: state_str = "standby vbus low "; break;
	        case 5: state_str = "active vbus high "; break;
            default: break;
        }
    }

};

} /* namespace supreme */

#endif /* FLATCAT_ROBOT_HPP */

