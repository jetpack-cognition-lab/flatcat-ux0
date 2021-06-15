#include "./ux0_serial.h"
#include <unistd.h>
//#include <sys/reboot.h>

/* framework */
#include <common/modules.h>
#include "tones.h"

GlobalFlag do_quit;

void
signal_terminate_handler(int signum)
{
    sts_msg("Got a SIGINT(%d) from user\n", signum);
    do_quit.enable();
}

bool
MainApplication::execute_cycle(void)
{
    motors.execute_cycle();
    /* data logging */
    //if (logger.is_enabled())
    //    logger.log("%llu%s", cycles, motors_log.log());

    /* get energymodule's data */
    auto const& data = battery.get_data().raw_recv;

    float Ubat = (data[0]*256 + data[1]) * 0.004262673; // TODO consider to report voltage normed over bus
    float Ubus = (data[2]*256 + data[3]) * 0.006451613;

    uint8_t ttlive = data[4];
    uint8_t status = data[5];
    uint16_t flags = data[6]*256 + data[7];

    /* get motors' data */
    float Umot = std::min( motors[0].get_data().voltage_supply,
                 std::min( motors[1].get_data().voltage_supply
                         , motors[2].get_data().voltage_supply ));

    /* low-pass filterss */
    avg_Umot = 0.99f * avg_Umot + 0.01f * Umot;
    avg_Ubus = 0.99f * avg_Ubus + 0.01f * Ubus;


    /*
    float Im = .0f;
    for (std::size_t i = 0; i < motors.size()-1; ++i)
            Im += motors[i].get_data().current;
    */
    //TODO solve current scale issue, different motor versions have different sensors.
    //     firmware should report "normed" current in .1 mA or so. 60000 = 6.000mA


    /* get battery module status */
    std::string status_str = "";
    switch(status) {
        case 0: status_str = "initializing     "; break;
	    case 1: status_str = "powered off      "; break;
	    case 2: status_str = "shutting down    "; break;
	    case 3: status_str = "notify subsystems"; break;
	    case 4: status_str = "standby vbus low "; break;
	    case 5: status_str = "active vbus high "; break;
        default: break;
    }

    /* print on terminal each second */
    if (cycles % 100 == 0)
        sts_msg("Ubat=%4.2f Ubus=%4.2f Umot=%4.2f T=%02u S=%s F=%x U=%s", Ubat, avg_Ubus, avg_Umot, ttlive, status_str.c_str(), flags, user_enable? "ENA" : "INH");

    /* set tones for all motors */
    for (std::size_t i = 0; i < motors.size()-1; ++i) if (motors[i].is_active())
    {
        auto& m = motors[i];
        auto const& data = m.get_data();

        if (fabs(data.output_voltage) > 0.01 && fabs(data.output_voltage) < 0.02)
            m.set_pwm_frequency(tonetable[3*i+2]);
        else
            m.set_pwm_frequency(20000-i*1000);
    }

    /* check charger */
    if (flags & (1 << StatusBits::charger_connected)) { // charger connected = standby bus power
        motors.disable_all();
        user_enable = false;
    }
    else {
        battery.set_controller_type(supreme::sensorimotor::Controller_t::voltage);
        battery.set_target_voltage(1.0);
        user_enable = true;
    }

    /* check motors's voltage */
    if (avg_Umot < 4.75) motors.disable_all();
    else if (avg_Umot > 5.8) {
        for (std::size_t i = 0; i < motors.size()-1; ++i)
            motors[i].set_controller_type(supreme::sensorimotor::Controller_t::csl);
    }

    /* emergency shutdown */
    if (ttlive < 5) {
        sts_msg("Energymodule is notifying us to shutdown power soon.");
        motors.disable_all();
        motors.execute_cycle(); // update motors before quitting
        return false;
    }

    ++cycles;
    return true;
}



int main(int argc, char* argv[])
{
    sts_msg("Initializing Flatcat <3");
    srand((unsigned) time(NULL));
    signal(SIGINT, signal_terminate_handler);

    MainApplication app(argc, argv);

    //std::thread tcp_thread(&MainApplication::tcp_serv_loop, &app);
    //std::thread udp_thread(&MainApplication::udp_send_loop, &app);

    Stopwatch watch;

    const bool verbose = (argc == 2 && strcmp (argv[1],"-v") == 0) ? true : false;
    sts_msg("Verbose mode: %s", (verbose)? "on" : "off");

    sts_msg("Starting main loop.");
    watch.reset();
    while(!do_quit.status())
    {
        if (!app.execute_cycle()) {
            app.finish();
            sts_add("synchronizing filesystem...");
            sync();
            sts_msg("DONE.");
            sts_msg("________\nSHUTDOWN");
            //reboot(RB_POWER_OFF);
            system("sudo shutdown -h now");
            return EXIT_FAILURE;
        }

        if (verbose)
            sts_msg("%05.2f ms", watch.get_time_passed_us()/1000.0);
    }
    //sts_msg("Waiting for UDP communication thread to join.");
    //udp_thread.join();
    //sts_msg("Waiting for TCP communication thread to join.");
    //tcp_thread.join();
    app.finish();
    sts_msg("____\nDONE.");
    return EXIT_SUCCESS;
}

