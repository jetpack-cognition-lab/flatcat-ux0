#ifndef FLATCAT_COMMUNICATION_HPP
#define FLATCAT_COMMUNICATION_HPP

#include <thread>

#include <common/udp.hpp>
#include <common/socket_server.h>
#include <common/log_messages.h>

#include "../learning/learning.hpp"

namespace supreme {

inline bool starts_with(std::string msg, const char c_str[]) { return (msg.compare(0, strlen(c_str), c_str) == 0); }

template <typename T>
void parse_command(T& result, std::string const& msg, const char* keystr) {
    T value;
    if (1 == sscanf(msg.c_str(), keystr, &value)) {
        result = static_cast<T>(value);
        //dbg_msg("'%s' command received.", keystr);
    } else wrn_msg("'%s' command broken.", keystr);
}


template <typename Vector_t>
void parse_midi_channel(Vector_t& vec, std::string const& msg) {
    unsigned idx;
    float value;
    if (2 == sscanf(msg.c_str(), "MDI%u=%f", &idx, &value) and idx < vec.size()) {
        vec.at(idx) = value;
        //dbg_msg("MIDI %02u = %+5.2f", idx, value);
    } else wrn_msg("Midi command broken.", msg);
}


class FlatcatCommunication {

    FlatcatRobot& robot;
    GlobalFlag const& exitflag;

    network::Socket_Server     tcp_server;

    network::UDPSender <256>   udp_sender;
    network::Sendbuffer<256>   sendbuffer;

    FlatcatLearning const& learning;

    bool tcp_connected = false;

    struct DummyControl_t {
        bool enabled = true;
        unsigned tar_mode = 0;
        std::array<float, 256> usr_params;
    } control;


    uint8_t reserved_8 = 0;
    float   reserved32 = 0;
    double  reserved64 = 0;


    std::thread                 tcp_thread,
                                udp_thread;

public:


    FlatcatCommunication( FlatcatRobot& robot
                        , FlatcatSettings const& settings
                        , GlobalFlag const& exitflag
                        , FlatcatLearning const& learning
                        )
    : robot(robot)
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

    void execute_cycle(uint64_t cycle)
    {
        robot.set_enable(control.enabled);

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
        sendbuffer.add(cycle);
        sendbuffer.add(reserved_8);
        sendbuffer.add(reserved_8);
        sendbuffer.add(reserved32);

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
            .add(reserved32      )
            .add(reserved32      )
            ;
        }

        /* TODO: energy module data */

        /* mirror controls */
        auto const& c = control;
        sendbuffer
        .add(c.enabled )
        .add(reserved_8)
        .add(reserved_8)
        .add(reserved_8)
        .add(reserved32)
        .add(reserved32)
        .add(reserved32)
        .add(reserved64)
        .add(reserved64)
        .add(reserved64)
        .add(static_cast<float  >(learning.super_layer.gmes.get_learning_progress()))
        .add(static_cast<uint8_t>(learning.super_layer.gmes.get_number_of_experts()))
        .add(reserved_8)
        .add(reserved_8);

        sendbuffer.add_checksum();
    }

    void handle_tcp_commands(std::string const& msg)
    {
        if (starts_with(msg, "ENA")) { parse_command(control.enabled        , msg, "ENA=%u"); return; }
        if (starts_with(msg, "CTL")) { parse_command(control.tar_mode       , msg, "CTL=%u"); return; }
        if (starts_with(msg, "MDI")) { parse_midi_channel(control.usr_params, msg          ); return; }

        if (msg == "HELLO") { sts_msg("client says hello"   ); return; }

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
