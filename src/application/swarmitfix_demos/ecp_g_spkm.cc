/*
 * Author: yoyek
 */

#include "base/lib/sr/sr_ecp.h"
#include "base/ecp/ecp_task.h"
#include "base/ecp/ecp_robot.h"
#include "ecp_g_spkm.h"

namespace mrrocpp {
namespace ecp {
namespace spkm {
namespace generator {

////////////////////////////////////////////////////////
//
//                  joint_epos_command
//
////////////////////////////////////////////////////////

joint_epos_command::joint_epos_command(task_t & _ecp_task) :
		generator_t(_ecp_task)
{
}

bool joint_epos_command::first_step()
{
	sr_ecp_msg.message("joint_command: first_step");

	// parameters copying
	get_mp_ecp_command();

	the_robot->epos_joint_command_data_port.data = mp_ecp_epos_simple_command;
	the_robot->epos_joint_command_data_port.set();

	the_robot->epos_joint_reply_data_request_port.set_request();

	return true;
}

bool joint_epos_command::next_step()
{
	the_robot->epos_joint_reply_data_request_port.get();

	bool motion_in_progress = false;

	for (int i = 0; i < lib::spkm::NUM_OF_SERVOS; i++) {
		if (the_robot->epos_joint_reply_data_request_port.data.epos_controller[i].motion_in_progress == true) {
			motion_in_progress = true;
			break;
		}
	}

	if (motion_in_progress) {
		// waits 20ms to check epos state
		delay(20);

		the_robot->epos_joint_reply_data_request_port.set_request();
		return true;
	}

	return false;

}

void joint_epos_command::create_ecp_mp_reply()
{

}

void joint_epos_command::get_mp_ecp_command()
{
	ecp_t.mp_command.ecp_next_state.sg_buf.get(mp_ecp_epos_simple_command);
}

////////////////////////////////////////////////////////
//
//                  external_epos_command
//
////////////////////////////////////////////////////////

external_epos_command::external_epos_command(task_t & _ecp_task) :
		generator_t(_ecp_task)
{
}

bool external_epos_command::first_step()
{
	sr_ecp_msg.message("external_command: first_step");

	// parameters copying
	get_mp_ecp_command();

	the_robot->epos_external_command_data_port.data = mp_ecp_epos_simple_command;
	the_robot->epos_external_command_data_port.set();

//	the_robot->epos_external_reply_data_request_port.set_data = lib::spkm::WRIST_XYZ_EULER_ZYZ;
//	the_robot->epos_external_reply_data_request_port.set_request();

	the_robot->epos_joint_reply_data_request_port.set_request();

	return true;
}

bool external_epos_command::next_step()
{
//	the_robot->epos_external_reply_data_request_port.get();
	the_robot->epos_joint_reply_data_request_port.get();

	bool motion_in_progress = false;

	for (int i = 0; i < lib::spkm::NUM_OF_SERVOS; i++) {
		if (the_robot->epos_joint_reply_data_request_port.data.epos_controller[i].motion_in_progress == true) {
			motion_in_progress = true;
			break;
		}
	}

	if (motion_in_progress) {
//		the_robot->epos_external_reply_data_request_port.set_data = lib::spkm::WRIST_XYZ_EULER_ZYZ;
//		the_robot->epos_external_reply_data_request_port.set_request();

		the_robot->epos_joint_reply_data_request_port.set_request();

		// Wait 20ms to check EPOS state.
		delay(20);

		return true;
	}

	// Motion is finished.
	return false;

}

void external_epos_command::create_ecp_mp_reply()
{

}

void external_epos_command::get_mp_ecp_command()
{
	ecp_t.mp_command.ecp_next_state.sg_buf.get(mp_ecp_epos_simple_command);
}

////////////////////////////////////////////////////////
//
//                  brake_command
//
////////////////////////////////////////////////////////

brake_command::brake_command(task_t & _ecp_task) :
		generator_t(_ecp_task)
{
}

bool brake_command::first_step()
{
	sr_ecp_msg.message("brake_command: first_step");

	the_robot->epos_brake_command_data_port.set();

	return true;
}

bool brake_command::next_step()
{
	return false;
}

} // namespace generator
} // namespace spkm
} // namespace ecp
} // namespace mrrocpp
