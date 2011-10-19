#include <iostream>
#include <boost/exception/get_error_info.hpp>
#include <boost/array.hpp>
#include <boost/foreach.hpp>
#include <sys/time.h>

#include "robot/canopen/gateway_epos_usb.h"
#include "epos.h"

using namespace mrrocpp::edp::canopen;
using namespace mrrocpp::edp::maxon;

int main(int argc, char *argv[])
{
	gateway_epos_usb gateway;

	try {
		// timestamps variables
		struct timespec t1, t2;

		gateway.open();

		epos node(gateway, 1);

		node.printState();
//			node.SendNMTService(epos::Reset_Node);
//			usleep(500000);
//			node.SendNMTService(epos::Start_Remote_Node);
//			usleep(500000);

		// Check if in a FAULT state
		if(node.getState() == 11) {
			UNSIGNED8 errNum = node.getNumberOfErrors();
			std::cout << "readNumberOfErrors() = " << (int) errNum << std::endl;
			for(UNSIGNED8 i = 1; i <= errNum; ++i) {

				UNSIGNED32 errCode = node.getErrorHistory(i);

				std::cout << node.ErrorCodeMessage(errCode) << std::endl;
			}
			if (errNum > 0) {
				node.clearNumberOfErrors();
			}
			node.setState(epos::FAULT_RESET);
		}

		// Change to the operational mode
		node.reset();

		node.clearPvtBuffer();

		uint16_t status;

		status = node.getStatusWord();
		std::cout << "node.remote = " << (int) (epos::isRemoteOperationEnabled(status)) << std::endl;

		gateway.SendNMTService(1, gateway::Start_Remote_Node);

		status = node.getStatusWord();
		std::cout << "node.remote = " << (int) (epos::isRemoteOperationEnabled(status)) << std::endl;

		std::cout << "node.readActualBufferSize() = " << (int) node.getActualBufferSize() << std::endl;

		gateway.setDebugLevel(0);
		clock_gettime(CLOCK_MONOTONIC, &t1);

		node.setInterpolationDataRecord(0, 0, 0);
		clock_gettime(CLOCK_MONOTONIC, &t2);
		double t = (t2.tv_sec + t2.tv_nsec/1e9) - (t1.tv_sec + t1.tv_nsec/1e9);
		printf("%.9f\n", t);
//		node.writeInterpolationDataRecord(1, 2, 3);
//		node.writeInterpolationDataRecord(1, 2, 3);
		gateway.setDebugLevel(0);

		std::cout << "node.readActualBufferSize() = " << (int) node.getActualBufferSize() << std::endl;

		gateway.close();
	} catch (canopen_error & error) {
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
