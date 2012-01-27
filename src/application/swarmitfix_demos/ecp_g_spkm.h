/*
 * generator/ecp_g_epos.h
 *
 *Author: yoyek
 */

#ifndef ECP_G_SPKM_SWARM_DEMO_SINGLE_AGENT_H_
#define ECP_G_SPKM_SWARM_DEMO_SINGLE_AGENT_H_

#include "robot/spkm/ecp_r_spkm.h"
#include "robot/spkm/dp_spkm.h"

#include "base/ecp/ecp_generator.h"

namespace mrrocpp {
namespace ecp {
namespace spkm {
namespace generator {

/*!
 * @brief generator to send the command prepared by MP to EDP spkm motors
 * it waits for the command execution finish
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup generators
 */
class joint_epos_command : public common::generator::_generator <ecp::spkm::robot>
{
private:
	lib::epos::epos_simple_command mp_ecp_epos_simple_command;

public:

	/**
	 * @brief Constructor
	 * @param _ecp_task ecp task object reference.
	 */
	joint_epos_command(task_t & _ecp_task);

	bool first_step();
	bool next_step();

	void create_ecp_mp_reply();
	void get_mp_ecp_command();
};

/*!
 * @brief generator to send the command prepared by MP to EDP spkm motors
 * it waits for the command execution finish
 * @author twiniars <twiniars@ia.pw.edu.pl>, Warsaw University of Technology
 * @ingroup generators
 */
class external_epos_command : public common::generator::_generator <ecp::spkm::robot>
{
private:
	lib::spkm::spkm_epos_simple_command mp_ecp_epos_simple_command;

	boost::system_time wakeup;

	//! Effector query interval
	const boost::posix_time::time_duration query_interval;

public:

	/**
	 * @brief Constructor
	 * @param _ecp_task ecp task object reference.
	 */
	external_epos_command(task_t & _ecp_task);

	bool first_step();
	bool next_step();

	void create_ecp_mp_reply();
	void get_mp_ecp_command();
};

class brake_command : public common::generator::_generator <ecp::spkm::robot>
{
public:
	/**
	 * @brief Constructor
	 * @param _ecp_task ecp task object reference.
	 */
	brake_command(task_t & _ecp_task);

	bool first_step();
	bool next_step();
};

} // namespace generator
} // namespace spkm
} // namespace ecp
} // namespace mrrocpp

#endif
