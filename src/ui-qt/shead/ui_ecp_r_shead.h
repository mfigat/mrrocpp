#ifndef _UI_ECP_R_SHEAD_H
#define _UI_ECP_R_SHEAD_H

#include "../base/ui.h"

#include "base/ecp/ecp_robot.h"
#include "robot/shead/ecp_r_shead.h"
#include "../base/ui_ecp_robot/ui_ecp_r_data_port.h"

namespace mrrocpp {
namespace ui {
namespace shead {

// ---------------------------------------------------------------
class EcpRobot : public common::_EcpRobotDataPort <ecp::shead::robot>
{
public:

	EcpRobot(common::UiRobot& _ui_robot); // Konstruktor

	void move_motors(const double final_position[lib::shead::NUM_OF_SERVOS]);

	void move_joints(const double final_position[lib::shead::NUM_OF_SERVOS]);

	void clear_fault();

	void stop_motors();
};

}
} //namespace ui
} //namespace mrrocpp

#endif
