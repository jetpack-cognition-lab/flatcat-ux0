#ifndef FLATCAT_SETTINGS_HPP
#define FLATCAT_SETTINGS_HPP

#include <string.h>

#include <common/settings.h>
#include <common/vector_n.h>


namespace supreme {

namespace defaults {
    const std::string settings_filename = "flatcat.dat";

    const VectorN joint_offsets = { .0, /* HEAD 0 */
                                    .0, /* BODY 1 */
                                    .0  /* TAIL 2 */
                                  };

    const float voltage_limit = 0.25;

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
    unsigned tcp_port = 7332;
    unsigned udp_port = 1900;
    std::string group = "224.0.0.1";//"239.255.255.252";


    const float update_rate_Hz = 100.0f;
    const float voltage_disable_motors = 4.7; // h-bridges disable themselves at approx. 4V5
    const float voltage_regenerate_lo  = 4.8;
    const float voltage_regenerate_hi  = 5.5; // TODO lower this value
    const float normed_active_bus_voltage = 1.0;

    /* CSL motor control */
    float motor_voltage_limit = 0.5; // in %
    float motor_csl_param_gf = 1.00;
    float motor_csl_param_gi = 1.5;
    unsigned motor_pwm_frequency = 22500;


    VectorN joint_offsets;

    VectorN sarsa_learning_rates = {0.05, 0.05, 0.005, 0.005};
    uint64_t trial_time_s = 60;
    uint64_t eigenzeit_steps = 1000; // 10 seconds max.

    float epsilon_exploration = 0.1;

    unsigned learning_save_cycles = 12000;
    std::string save_state_name;
    std::string save_folder = "./data/";
    bool clear_state;



    /* gmes */
    struct GMES_Settings_t {
        unsigned number_of_experts = 64;
        unsigned experience_size = 1;
        float joint_gmes_learning_rate = 100.0;
        float local_learning_rate = 0.001;
    } gmes;

    FlatcatSettings(int argc, char **argv)
    : Settings_Base         (argc, argv                          , defaults::settings_filename.c_str())
    , voltage_disable_motors(read_float("voltage_disable_motors" , voltage_disable_motors            ))
    , voltage_regenerate_lo (read_float("voltage_regenerate_lo"  , voltage_regenerate_lo             ))
    , voltage_regenerate_hi (read_float("voltage_regenerate_hi"  , voltage_regenerate_hi             ))
    , motor_voltage_limit   (read_float("motor_voltage_limit"    , motor_voltage_limit               ))
    , motor_csl_param_gf    (read_float("motor_csl_param_gf"     , motor_csl_param_gf                ))
    , motor_csl_param_gi    (read_float("motor_csl_param_gi"     , motor_csl_param_gi                ))
    , motor_pwm_frequency   (read_uint ("motor_pwm_frequency"    , motor_pwm_frequency               ))
    , joint_offsets         (read_vec  ("joint_offsets"          , defaults::joint_offsets           ))
    , save_state_name       (read_string_option(argc, argv, "-n", "--name", "default"                ))
    , clear_state           (read_option_flag  (argc, argv, "-c", "--clear"                          ))
    {
        save_folder += save_state_name + "/";
    }
};

} /* namespace supreme */

#endif /* FLATCAT_SETTINGS_HPP */

