/*!
 * @file ecp_t_sbench.cc
 * @brief Transparent task methods definition.
 *
 * @date Jan 19, 2012
 * @author tkornuta
 */

#include "base/lib/sr/srlib.h"

#include "robot/shead/ecp_r_shead1.h"
#include "robot/shead/ecp_r_shead2.h"

#include "ecp_t_sbench.h"
#include "ecp_mp_g_sbench.h"

namespace mrrocpp {
namespace ecp {
namespace sbench {
namespace task {

transparent::transparent(lib::configurator &_config) :
		common::task::_task <ecp::sbench::robot>(_config)
{
	// Create the bench robot.
	ecp_m_robot = (boost::shared_ptr <robot_t>) new sbench::robot(*this);

	sr_ecp_msg->message("Transparent task loaded");
}

void transparent::mp_2_ecp_next_state_string_handler(void)
{
	// Choose generator basing on the received command.
	if (mp_2_ecp_next_state_string == ecp_mp::sbench::generator::CLEANING_COMMAND) {

		generator::cleaning g_cleaning(*this);

		g_cleaning.Move();

	} else if (mp_2_ecp_next_state_string == ecp_mp::sbench::generator::POWER_SUPPLY_COMMAND) {

		generator::power_supply g_power_supply(*this);

		g_power_supply.Move();

	} else if (mp_2_ecp_next_state_string == ecp_mp::sbench::generator::POWER_SUPPLY_STATUS) {

		generator::power_status g_status(*this);

		g_status.Move();

	} else {
		assert(0);
	}
}

} // namespace task
} // namespace sbench

namespace common {
namespace task {

task_base* return_created_ecp_task(lib::configurator &_config)
{
	return new sbench::task::transparent(_config);
}

} // namespace task
} // namespace common
} // namespace ecp
} // namespace mrrocpp
