#ifndef FLATCAT_ROBOT_HPP
#define FLATCAT_ROBOT_HPP

#include <cassert>
#include <array>

/* framework */
#include <common/log_messages.h>
#include <robots/robot.h>
#include <robots/joint.h>
#include <robots/accel.h>

/* libsensorimotor */
#include <communication_ctrl.hpp>
#include <motorcord.hpp>
#include <sensorimotor.hpp>

/* flatcat_ux0 */
#include <system/settings.hpp>

namespace supreme {

struct SoC_t {
	float vol, soc;
};

namespace constants {
	const unsigned num_joints = 3;

	const std::array<int16_t, num_joints> dir = { +1, +1, +1};
	const float position_scale = 270.0/360.0;

	const float max_bat_voltage = 4.2f;
	const float min_bat_voltage = 3.0f;

	/* simple SoC estimation from open battery voltage */
	const std::array<SoC_t,13> SoC_table = {{{ 4.2, 100.00 },
	                                         { 4.1,  96.48 },
	                                         { 4.0,  88.58 },
	                                         { 3.9,  79.53 },
	                                         { 3.8,  69.74 },
	                                         { 3.7,  57.03 },
	                                         { 3.6,  44.66 },
	                                         { 3.5,  29.63 },
	                                         { 3.4,  17.95 },
	                                         { 3.3,   9.45 },
	                                         { 3.2,   3.95 },
	                                         { 3.1,   1.29 },
	                                         { 3.0,   0.00 }}};

} /* namespace constants */

class FlatcatRobot : public robots::Robot_Interface
{
public: //TODO undo "all public"
	supreme::motorcord     motorcord;
	supreme::sensorimotor& battery;
	FlatcatSettings const& settings;

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
		float SoC  = 50.f;
		float Tmot = 20.f;
		uint8_t ttlive = 10;
		uint8_t state = 0;
		uint16_t flags = 0;
		std::string flag_str = "";

		struct MotorcordStatus_t {
			unsigned errors = 0;
			unsigned timeouts = 0;
		} motorcord;

		bool motor_temperature_critical = false;

	} status;

	enum StatusBits : uint8_t
	{
		button_pressed      =  0,
		charger_connected   =  1,
		battery_charging    =  2,
		limiter_fault       =  3,
		vbus_charging       =  4,
		no_battery_inserted =  5,
	// 	reserved_6          =  6,
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
		resting      = 1,
		active       = 2,
		halting      = 3,
	} state = FlatcatState_t::initializing;

public:


	FlatcatRobot(FlatcatSettings const& settings)
	: motorcord( constants::num_joints + 1/*battery*/
	           , settings.update_rate_Hz
	           , /*verbose=*/false
	           , settings.serial_devicename
	           , /*disable_at_exit=*/false
	           )
	, battery(motorcord[3])
	, settings(settings)
	, number_of_joints(constants::num_joints)
	, number_of_joints_sym(/* will be counted */)
	, number_of_accels(1)
	, joints()
	, accels()
	, status()
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

		state = FlatcatState_t::active;
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
		//TODO: write_motorcord();  /* write motors     */
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
			j.s_cur = m.get_data().current/2.f; //TODO current max
			j.s_tmp = m.get_data().temperature/100.0; //TODO temp max value
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

	void inhibit(void) {
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
	bool is_button_pressed   (void) const { return status.flags & (1 << StatusBits::button_pressed); }
	bool is_limiter_faulted  (void) const { return status.flags & (1 << StatusBits::limiter_fault); }

	bool is_resting(void) const { return state == FlatcatState_t::resting; }

	void set_bus_voltage(float voltage) {
		battery.set_controller_type(supreme::sensorimotor::Controller_t::voltage);
		battery.set_target_voltage(voltage);
	}

	bool is_shutdown_signaled(void) {
		return battery.is_active() and status.ttlive < 10;
	}

	bool is_powerfail(void) {
		return battery.is_active() and status.Ubat < settings.voltage_emergency_shutdown;
	}

private:

	void update_status_data(void) {
		/* motors' voltages */
		float Umot = std::min( motorcord[0].get_data().voltage_supply,
		             std::min( motorcord[1].get_data().voltage_supply
		                     , motorcord[2].get_data().voltage_supply ));

		float Tmot = std::max( motorcord[0].get_data().temperature,
		             std::max( motorcord[1].get_data().temperature
		                     , motorcord[2].get_data().temperature ));

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
		status.Tmot = 0.99f * status.Tmot + 0.01f * Tmot;

		status.SoC  = get_state_of_charge_from_voltage(status.Ubat);

		status.ttlive   = dat[6];
		status.state    = dat[7];
		status.flags    = dat[8]*256 + dat[9];
		status.flag_str = std::bitset<sizeof(status.flags) * 8>(status.flags).to_string();

		/* motor temperature critical flag with hysteresis */
		if (status.Tmot > settings.motor_temperature_off) status.motor_temperature_critical = true;
		if (status.Tmot < settings.motor_temperature_on ) status.motor_temperature_critical = false;
	}

	float get_state_of_charge_from_voltage(float bat_voltage) {
		const float ub = clip(bat_voltage, constants::min_bat_voltage
		                                 , constants::max_bat_voltage ); // 3.0 .. 4.2V
		const float ubd = (constants::max_bat_voltage - ub) * 10; // 0.0 .. 1.2V -> 0.0 .. 12.0

		const uint8_t i0 = clip(floor(ubd), 0, constants::SoC_table.size()-1);
		const uint8_t i1 = clip(ceil(ubd) , 0, constants::SoC_table.size()-1);

		const float top = constants::SoC_table[i1].soc;
		const float bot = constants::SoC_table[i0].soc;

		float rem = ub - constants::SoC_table[i0].vol;
		float tot = constants::SoC_table[i1].vol
		          - constants::SoC_table[i0].vol; // eg = 0.1

		float p = (tot != 0.f) ? clip(rem/tot, 0.f, 1.f) : 0.f;
		return p * (top - bot) + bot;
	}

};

} /* namespace supreme */

#endif /* FLATCAT_ROBOT_HPP */
