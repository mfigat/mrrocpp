/*!
 * @file ecp_g_sbench.cc
 * @brief Power supply and pressure control transparent generators methods definitions.
 *
 * @date Jan 19, 2012
 * @author tkornuta
 */

#include "base/lib/sr/sr_ecp.h"
#include "base/ecp/ecp_task.h"
#include "base/ecp/ecp_robot.h"
#include "ecp_g_sbench.h"

#include <boost/archive/text_oarchive.hpp>

namespace mrrocpp {
namespace ecp {
namespace sbench {
namespace generator {

power_supply::power_supply(task_t & _ecp_task) :
		generator_t(_ecp_task)
{

}

bool power_supply::first_step()
{
	sr_ecp_msg.message("power_supply: first_step");

	// Receive command.
	ecp_t.mp_command.ecp_next_state.sg_buf.get(mp_ecp_power_supply_command);

	// Copy received parameters to output data port.
	the_robot->power_supply_data_port.data = mp_ecp_power_supply_command;

	// Prepare adequate commands.
	the_robot->power_supply_data_port.set();
	the_robot->data_request_port.set_request();

	return true;
}

bool power_supply::next_step()
{
	sr_ecp_msg.message("power_supply: next_step");
	return false;
}

power_status::power_status(task_t & _ecp_task) :
		generator_t(_ecp_task)
{

}

bool power_status::first_step()
{
	sr_ecp_msg.message("power_status: first_step");

	// Prepare adequate requests.
	the_robot->data_request_port.set_request();

	return true;
}

bool power_status::next_step()
{
	the_robot->data_request_port.data.voltage_buf;

	// Create the text representation.
	std::ostringstream ostr;
	{
		boost::archive::text_oarchive oa(ostr);

		// serialize data
		oa << the_robot->data_request_port.data.voltage_buf;
	}

	// Check for overwrite.
	assert(ostr.str().length() < lib::ECP_2_MP_STRING_SIZE);

	std::cerr << "power_status: data buffer size = " << ostr.str().length() << std::endl;

	strncpy(ecp_t.ecp_reply.recognized_command, ostr.str().c_str(), lib::ECP_2_MP_STRING_SIZE);

	return false;
}

cleaning::cleaning(task_t & _ecp_task) :
		generator_t(_ecp_task)
{

}

bool cleaning::first_step()
{
	sr_ecp_msg.message("cleaning: first_step");

	// Receive command.
	ecp_t.mp_command.ecp_next_state.sg_buf.get(mp_ecp_cleaning_command);

	// Copy received parameters to output data port.
	the_robot->cleaning_state_data_port.data = mp_ecp_cleaning_command;

	// Prepare adequate commands.
	the_robot->cleaning_state_data_port.set();
	the_robot->data_request_port.set_request();

	return true;
}

bool cleaning::next_step()
{
	sr_ecp_msg.message("cleaning: next_step");
	return false;
}


} // namespace generator
} // namespace sbench
} // namespace ecp
} // namespace mrrocpp

