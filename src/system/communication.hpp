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
    double  reserved64 = 0;


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
    }

	bool is_connected(void) const { return tcp_connected; }

    void execute_cycle(uint64_t cycle)
    {
        if (tcp_connected) {
            fill_sendbuffer(cycle);
            udp_sender.set_buffer(sendbuffer.get(), sendbuffer.size());
        }
    }

    ~FlatcatCommunication() {
        assert(exitflag.status()); // make sure it is set, otherwise threads will not join
        sts_add("Waiting for UDP communication thread to join.");
        udp_thread.join(); sts_msg("DONE.");

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


    void fill_sendbuffer(uint64_t cycle) {
        sendbuffer.reset();
        sendbuffer.add(cycle)
                  .add(robot.status.state )
                  .add(robot.status.ttlive)
                  .add(reserved_8         )
                  .add(reserved_8         )
                  .add(robot.status.flags )
                  .add(robot.status.Umot  )
                  .add(robot.status.Ubus  )
                  .add(robot.status.Ubat  )
                  .add(robot.status.Ilim  )
                  .add(robot.status.Imot  )
                  .add(robot.status.SoC   )
                  .add(reserved32         )
                  .add(reserved32         )
                  .add(reserved32         )
                  .add(reserved32         )
                  ;
        /* N motors */
        for (unsigned i = 0; i < robot.get_motors().size()-1; ++i)
        {
            auto const& m = robot.get_motors()[i];
            auto const& d = m.get_data();
            sendbuffer
            .add(m.get_id()      )
            .add(reserved_8      )
            .add(reserved_8      )
            .add(reserved_8      )
            .add(reserved32      )
            .add(reserved32      )
            .add(reserved32      )
            .add(d.position      )
            .add(d.velocity      )
            .add(d.current       )
            .add(d.voltage_supply)
            .add(d.output_voltage)
            .add(d.temperature   )
            .add(reserved32      )
            .add(reserved32      )
            .add(reserved32      )
            .add(reserved32      )
            ;
        }

        /* TODO: energy module data */

        /* mirror controls */
        auto const& c = control;
        sendbuffer
        .add(c.paused_by_user)
        .add(reserved_8)
        .add(reserved_8)
        .add(reserved_8)
        .add(reserved32)
        .add(float(control.csl_settings.gf))
        .add(float(control.csl_settings.gi))
        .add(reserved64)
        .add(static_cast<float  >(learning.super_layer.gmes.get_learning_progress()))
        .add(static_cast<uint8_t>(learning.super_layer.gmes.get_number_of_experts()))
        .add(reserved_8)
        .add(reserved_8);

        sendbuffer.add_checksum();
    }


	void handle_tcp_commands(std::string const& msg)
	{
		//dbg_msg("handle command %s", msg.c_str());

		if (starts_with(msg, "PAUSE")) { recv_variable(tcp_server, control.paused_by_user, msg, "PAUSE=%u" ); return; }
		if (starts_with(msg, "MODE" )) { recv_variable(tcp_server, control.tar_mode      , msg, "MODE=%u"  ); return; }
		if (starts_with(msg, "USER" )) { recv_vector  (tcp_server, control.usr_params    , msg, "USER%u=%f"); return; }

		if (starts_with(msg, "CSLGF")) { recv_variable(tcp_server, control.csl_settings.gf, msg, "CSLGF=%f"); return; }
		if (starts_with(msg, "CSLGI")) { recv_variable(tcp_server, control.csl_settings.gi, msg, "CSLGI=%f"); return; }

		if (msg == "paused") { send_variable(tcp_server, control.paused_by_user, "paused");    return; }
		if (msg == "SoC"   ) { send_variable(tcp_server, robot.status.SoC      , "SoC");       return; }
		if (msg == "flags" ) { send_variable(tcp_server, robot.status.flags    , "flags");     return; }

		if (msg == "HELLO" ) { sts_msg("client says hello"); acknowledge(tcp_server); return; }

		dbg_msg("unknown msg: %s", msg.c_str());
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

            tcp_server.close_connection();
            tcp_connected = false;
        }
    }

};


} /* namespace supreme */


#endif /* FLATCAT_COMMUNICATION_HPP */
