#include <iostream>
#include <cstdio>
#include <sys/time.h>

#include <boost/exception/get_error_info.hpp>
#include <boost/array.hpp>
#include <boost/lexical_cast.hpp>

#include "robot/canopen/gateway_socketcan.h"
#include "epos.h"

using namespace mrrocpp::edp::canopen;
using namespace mrrocpp::edp::maxon;

int main(int argc, char *argv[])
{
	if(argc < 2) {
		std::cerr << "Usage: " << argv[0] << " can_id ..." << std::endl;
	}

	gateway_socketcan gateway("can0");

	try {
		gateway.open();

		for(int i = 1; i < argc; ++i) {

			int nodeId = boost::lexical_cast<int>(argv[i]);

			if(nodeId < 1 || nodeId > 127) {
				std::cerr << "can node ID " << nodeId << " out of range" << std::endl;
			}

			std::cout << "Resetting node " << nodeId << std::endl;

			gateway.SendNMTService(nodeId, gateway::Start_Remote_Node);
			gateway.SendNMTService(nodeId, gateway::Reset_Node);

		}

		gateway.close();
	} catch (fe_canopen_error & error) {
		std::cerr << "EPOS Error." << std::endl;

		if ( std::string const * r = boost::get_error_info<reason>(error) )
			std::cerr << " Reason: " << *r << std::endl;

		if ( std::string const * call = boost::get_error_info<errno_call>(error) )
			std::cerr << " Errno call: " << *call << std::endl;

		if ( int const * errno_value = boost::get_error_info<errno_code>(error) )
			std::cerr << "Errno value: " << *errno_value << std::endl;
	} catch (...) {
		std::cerr << "Unhandled exception" << std::endl;
	}

	return 0;
}
