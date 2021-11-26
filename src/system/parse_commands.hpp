#ifndef FLATCAT_PARSE_CMD_HPP
#define FLATCAT_PARSE_CMD_HPP

namespace supreme {

	inline bool starts_with(std::string msg, const char c_str[]) {
		return (msg.compare(0, strlen(c_str), c_str) == 0);
	}

	inline void acknowledge(network::Socket_Server& socket) {
		//dbg_msg("sending ACK");
		socket.send_message("ACK\n"); };

	template <typename T>
	bool recv_variable(network::Socket_Server& socket, T& result, std::string const& msg, const char* keystr) {
		T value;
		if (1 == sscanf(msg.c_str(), keystr, &value)) {
			result = static_cast<T>(value);
			//dbg_msg("received variable command %s", msg.c_str());
			acknowledge(socket);
			return true;
		} else wrn_msg("'%s' command broken.", keystr);
		return false;
	}


	template <typename Vector_t>
	bool recv_vector(network::Socket_Server& socket, Vector_t& vec, std::string const& msg, const char* keystr) {
		unsigned idx;
		float value;
		if (2 == sscanf(msg.c_str(), keystr, &idx, &value) and idx < vec.size()) {
			vec.at(idx) = value;
			//dbg_msg("received vector command %s", msg.c_str());
			acknowledge(socket);
			return true;
		} else wrn_msg("control channel command broken.", msg);
		return false;
	}

	template <typename T>
	void send_variable(network::Socket_Server& socket, T const& var, std::string keystr) {
		std::ostringstream msg;
		msg << keystr << "=" << var << "\n";
		//dbg_msg("sending response %s", msg.str().c_str());
		socket.send_message(msg.str());
	}

} /* namespace supreme */

#endif /* FLATCAT_PARSE_CMD_HPP */
