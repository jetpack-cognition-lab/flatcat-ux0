#ifndef FLATCAT_COMMUNICATION_HPP
#define FLATCAT_COMMUNICATION_HPP

#include <thread>

#include <common/udp.hpp>
#include <common/socket_server.h>
#include <common/log_messages.h>

#include "../system/parse_commands.hpp"
#include "../system/handle_commands.hpp"
#include "../learning/learning.hpp"

namespace supreme {

class FlatcatCommunication {

    FlatcatRobot&              robot;
    FlatcatControl&            control;
    GlobalFlag const&          exitflag;

    network::Socket_Server     tcp_server;

    network::UDPSender <256>   udp_sender;
    network::Sendbuffer<256>   sendbuffer;

    FlatcatLearning const& learning;

    bool tcp_connected = false;

    uint8_t reserved_8 = 0;
    float   reserved32 = 0;

    std::thread                 tcp_thread,
                                udp_thread;

public:

    FlatcatCommunication( FlatcatRobot& robot
                        , FlatcatControl& control
                        , FlatcatSettings const& settings
                        , GlobalFlag const& exitflag
                        , FlatcatLearning const& learning
                        )
    : robot(robot)
    , control(control)
    , exitflag(exitflag)
    , tcp_server(settings.tcp_port)
    , udp_sender(settings.group, settings.udp_port)
    , sendbuffer()
    , learning(learning)
    , tcp_thread(&FlatcatCommunication::tcp_serv_loop, this)
    , udp_thread(&FlatcatCommunication::udp_send_loop, this)
    {
        sts_msg("starting robot network communication module");
        static_assert(sizeof(float) == 4, "Unexpected size of Float Type");
        static_assert(sizeof(unsigned) == 4, "Unexpected size of Unsigned Type");
    }

    bool is_connected(void) const { return tcp_connected; }

    void execute_cycle(uint64_t cycle, uint32_t remaining_time_us)
    {
        if (tcp_connected) {
            fill_sendbuffer(cycle, remaining_time_us);
            udp_sender.set_buffer(sendbuffer.get(), sendbuffer.size());
        }
    }

    ~FlatcatCommunication() {
        assert(exitflag.status()); // make sure it is set, otherwise threads will not join
        sts_add("Waiting for UDP communication thread to join.");
        udp_thread.join(); sts_msg("DONE.");

        /**TODO send byebye to clients */
        sts_add("Waiting for TCP communication thread to join.");
        tcp_thread.join(); sts_msg("DONE.");
    }


    void udp_send_loop(void)
    {
        while(!exitflag.status())
        {
            if (tcp_connected and udp_sender.data_ready()) {
                udp_sender.transmit();
            }
            else usleep(1000); // reduce polling
        }
    }


    void fill_sendbuffer(uint64_t cycle, uint32_t remaining_time_us) {
        sendbuffer.reset(); // sync bytes      2
        sendbuffer.add(cycle)               // 8
                  .add(robot.status.state ) // 1
                  .add(robot.status.ttlive) // 1
                  .add(robot.status.motor_temperature_critical) // 1
                  .add(reserved_8         ) // 1
                  .add(robot.status.flags ) // 2
                  .add(robot.status.Umot  ) // 4
                  .add(robot.status.Ubus  ) // 4
                  .add(robot.status.Ubat  ) // 4
                  .add(robot.status.Ilim  ) // 4
                  .add(robot.status.Imot  ) // 4
                  .add(robot.status.SoC   ) // 4
                  .add(robot.status.Tmot  ) // 4
                  .add(reserved32         ) // 4
                  .add(reserved32         ) // 4
                  .add(reserved32         ) // 4
                  .add(reserved32         ) // 4
                  .add(remaining_time_us  ) // 4
                  ; // 56 bytes
        /* N motors */
        for (unsigned i = 0; i < robot.get_motors().size()-1; ++i)
        {
            auto const& m = robot.get_motors()[i];
            auto const& d = m.get_data();
            sendbuffer
            .add(m.get_id()                                  ) // 1
            .add(d.statistics.faulted                        ) // 1
            .add(reserved_8                                  ) // 1
            .add(reserved_8                                  ) // 1
            .add(static_cast<uint16_t>(d.statistics.timeouts)) // 2
            .add(static_cast<uint16_t>(d.statistics.errors  )) // 2
            .add(reserved32                                  ) // 4
            .add(reserved32                                  ) // 4
            .add(d.position                                  ) // 4
            .add(d.velocity                                  ) // 4
            .add(d.current                                   ) // 4
            .add(d.voltage_supply                            ) // 4
            .add(d.output_voltage                            ) // 4
            .add(d.temperature                               ) // 4
            .add(d.capsense_touch                            ) // 4
            .add(reserved32                                  ) // 4
            ; // 3 x 48 bytes
        }

        /* mirror controls */
        auto const& c = control;
        sendbuffer
        .add(c.paused_by_user)                                                         // 1
        .add(static_cast<uint8_t>(learning.actions.get_number_of_actions()))           // 1
        .add(static_cast<uint8_t>(learning.agent.get_current_action()))                // 1
        .add(reserved32)                                                               // 4
        .add(static_cast<float  >(control.csl_settings.gf))                            // 4
        .add(static_cast<float  >(control.csl_settings.gi))                            // 4
        .add(static_cast<float  >(learning.learning_progress_avg))                     // 4
        .add(static_cast<float  >(learning.learning_progress_var))                     // 4
        .add(static_cast<float  >(learning.super_layer.gmes.get_learning_progress()))  // 4
        .add(static_cast<uint8_t>(learning.super_layer.gmes.get_number_of_experts()))  // 1
        .add(static_cast<uint8_t>(learning.super_layer.gmes.get_winner()))             // 1
        .add(learning.timer_surprise.get_elapsed_percent())                            // 1
        .add(learning.timer_boredom .get_elapsed_percent())                            // 1
        .add(reserved32)                                                               // 4
        .add(reserved32)                                                               // 4
        .add(reserved32)                                                               // 4
        .add(reserved32)                                                               // 4
        .add_checksum();                                                               // 1

        assertion( sendbuffer.size() == sendbuffer.get_capacity()
                 ,"sendbuffer not filled entirely. Remaining bytes: %d"
                 , sendbuffer.get_capacity() - sendbuffer.size());
    }


    void handle_tcp_commands(std::string const& msg)
    {
        //dbg_msg("handle command %s", msg.c_str());

        if (starts_with(msg, "PAUSE")) { recv_variable(tcp_server, control.paused_by_user, msg, "PAUSE=%u" ); return; }
        if (starts_with(msg, "MODE" )) { recv_variable(tcp_server, control.tar_mode      , msg, "MODE=%u"  ); return; }
        if (starts_with(msg, "USER" )) { recv_vector  (tcp_server, control.usr_params    , msg, "USER%u=%f"); return; }

        if (msg == "SLEEP") {
            control.deep_sleep_user_req = true;
            sts_msg("client requests for deep sleep");
            return;
        }

        if (starts_with(msg, "CSLGF")) { recv_variable(tcp_server, control.csl_settings.gf, msg, "CSLGF=%f"); return; }
        if (starts_with(msg, "CSLGI")) { recv_variable(tcp_server, control.csl_settings.gi, msg, "CSLGI=%f"); return; }

        if (msg == "paused") { send_variable(tcp_server, control.paused_by_user, "paused"); return; }
        if (msg == "SoC"   ) { send_variable(tcp_server, robot.status.SoC      , "SoC");    return; }
        if (msg == "flags" ) { send_variable(tcp_server, robot.status.flags    , "flags");  return; }

        if (msg == "HELLO" ) { sts_msg("client says hello"); acknowledge(tcp_server); return; }

        dbg_msg("unknown msg: '%s'", msg.c_str());
    }


    void tcp_serv_loop(void)
    {
        std::string msg = "";

        sts_msg("Starting TCP command server, waiting for incoming connections.");
        while(!exitflag.status())
        {
            while(!tcp_server.open_connection()) {
                usleep(1000);
                if (exitflag.status())
                    break;
            }

            if (!exitflag.status()) {
                tcp_connected = true;
                sts_msg("Setting new destination for UDP packets.");
                udp_sender.change_destination(tcp_server.get_current_client_address(), 7331);
            }

            while(!exitflag.status()) {
                msg = tcp_server.get_next_line();
                if (msg == "EXIT") {
                    sts_msg("Client requested to close connection.");
                    break;
                } else if (msg.size() != 0)
                    handle_tcp_commands(msg);
                else
                    usleep(1000);
            }
            if (tcp_connected) tcp_server.send_message("GOODBYE\n");
            tcp_server.close_connection();
            tcp_connected = false;
        }
    }

};

} /* namespace supreme */

#endif /* FLATCAT_COMMUNICATION_HPP */
