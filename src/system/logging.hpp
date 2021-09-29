#ifndef FLATCAT_LOGGING_HPP
#define FLATCAT_LOGGING_HPP

namespace supreme {

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

} /* namespace supreme */

#endif /* FLATCAT_LOGGING_HPP */
