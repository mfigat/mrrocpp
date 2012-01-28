#include <QHideEvent>
#include <QPlainTextEdit>

#include "wgt_plan.h"

#include "../interface.h"
#include "../ui_ecp.h"
#include "../ui_robot.h"

// HEAD.
#include "../../shead/ui_ecp_r_shead.h"
#include "../../shead/ui_r_shead.h"
#include "robot/shead/const_shead1.h"
#include "robot/shead/const_shead2.h"
#include "robot/shead/kinematic_model_shead.h"

// PKM.
#include "../../spkm/ui_ecp_r_spkm.h"
#include "../../spkm/ui_r_spkm.h"
#include "robot/spkm/const_spkm1.h"
#include "robot/spkm/const_spkm2.h"
#include "robot/spkm/dp_spkm.h"
#include "robot/maxon/epos.h"

// MBASE.
#include "../../smb/ui_r_smb.h"
#include "../../smb/ui_ecp_r_smb.h"
#include "robot/smb/const_smb1.h"
#include "robot/smb/const_smb2.h"
#include "robot/smb/dp_smb.h"

#include "application/swarmitfix_plan/plan.hxx"

wgt_plan::wgt_plan(mrrocpp::ui::common::Interface& _interface, QWidget *parent) :
		wgt_base("Plan control", _interface, parent), ui(new Ui::wgt_planClass)
{
	ui->setupUi(this);

	// Setup button icons.
	ui->pushButton_prev->setIcon(QPixmap(":/trolltech/styles/commonstyle/images/media-seek-backward-32.png"));
	ui->pushButton_next->setIcon(QPixmap(":/trolltech/styles/commonstyle/images/media-seek-forward-32.png"));
	ui->pushButton_reload->setIcon(QPixmap(":/trolltech/styles/commonstyle/images/refresh-32.png"));
	ui->pushButton_save->setIcon(QPixmap(":/trolltech/styles/commonstyle/images/standardbutton-save-32.png"));
	ui->pushButton_exec->setIcon(QPixmap(":/trolltech/styles/commonstyle/images/standardbutton-apply-32.png"));

	ui->pushbutton_copy->setIcon(QPixmap(":/trolltech/styles/commonstyle/images/standardbutton-copy-32.png"));
	ui->pushbutton_paste->setIcon(QPixmap(":/trolltech/styles/commonstyle/images/standardbutton-paste-32.png"));

	// Initially finetuning is disabled.
	finetuning_active = false;
	last_executed_ind = -1;

	// Initialize status of the clipboard.
	clipboard.mbase_item.present = false;
	clipboard.pkm_item.present = false;
}

wgt_plan::~wgt_plan()
{
	delete ui;
}

void wgt_plan::hideEvent(QHideEvent *event)
{
	//interface.ui_msg->message("wgt_plan::hideEvent");
	event->accept();
}

void wgt_plan::reload()
{
	// Use local reference with short name.
	const lib::ECP_message & request = interface.ui_ecp_obj->ecp_to_ui_msg;

	std::cout << "reload()" << std::endl
			<< request.plan_item_string << std::endl;

	// Extract data from string.
	std::istringstream istr(request.plan_item_string);
	boost::archive::text_iarchive ia(istr);
	xml_schema::istream<boost::archive::text_iarchive> is (ia);

	// Handle current plan item
	switch(request.plan_item_type) {
		case lib::PKM_AND_HEAD:
			{
				// Deserialize data.
				Pkm::ItemType item(is);

				// Check widget constraints.
				checkInputWidgetLimits(*ui->agent_input, item.agent());
				checkInputWidgetLimits(*ui->TBeg_input, item.TBeg());
				checkInputWidgetLimits(*ui->TEnd_input, item.TEnd());
				checkInputWidgetLimits(*ui->ind_input, item.ind());

				checkInputWidgetLimits(*ui->x_input, item.Xyz_Euler_Zyz()->x());
				checkInputWidgetLimits(*ui->y_input, item.Xyz_Euler_Zyz()->y());
				checkInputWidgetLimits(*ui->z_input, item.Xyz_Euler_Zyz()->z());
				checkInputWidgetLimits(*ui->alpha_input, item.Xyz_Euler_Zyz()->alpha());
				checkInputWidgetLimits(*ui->beta_input, item.Xyz_Euler_Zyz()->beta());
				checkInputWidgetLimits(*ui->gamma_input, item.Xyz_Euler_Zyz()->gamma());
				checkInputWidgetLimits(*ui->head_input, item.beta7());

				// Setup common frame.
				ui->agent_input->setValue(item.agent());
				ui->TBeg_input->setValue(item.TBeg());
				ui->TEnd_input->setValue(item.TEnd());
				ui->ind_input->setValue(item.ind());

				// Setup input widgets.
				ui->x_input->setValue(item.Xyz_Euler_Zyz()->x());
				ui->y_input->setValue(item.Xyz_Euler_Zyz()->y());
				ui->z_input->setValue(item.Xyz_Euler_Zyz()->z());
				ui->alpha_input->setValue(item.Xyz_Euler_Zyz()->alpha());
				ui->beta_input->setValue(item.Xyz_Euler_Zyz()->beta());
				ui->gamma_input->setValue(item.Xyz_Euler_Zyz()->gamma());
				ui->head_input->setValue(item.beta7());

				// Setup commentary input.
				if(item.comment().present()) {
					ui->comment_input->setPlainText(item.comment().get().c_str());
				} else {
					ui->comment_input->clear();
				}

				// Allow fine-tuning and saving.
				if(last_executed_ind == item.ind()) {
					finetuning_active = true;
					ui->pushButton_save->setEnabled(true);
				} else {
					finetuning_active = false;
					ui->pushButton_save->setEnabled(false);
				}
			}

			// Disable/enable input containers
			this->setEnabled(true);
			ui->pkm_frame->setEnabled(true);
			ui->mbase_frame->setEnabled(false);

			break;
		case lib::MBASE_AND_BENCH:
			{
				// Deserialize data.
				Mbase::ItemType item(is);

				// Check widget constraints.
				checkInputWidgetLimits(*ui->agent_input, item.agent());
				checkInputWidgetLimits(*ui->TBeg_input, item.TBeg());
				checkInputWidgetLimits(*ui->TEnd_input, item.TEnd());
				checkInputWidgetLimits(*ui->ind_input, item.ind());

				checkInputWidgetLimits(*ui->row1_input, item.pinIndices().item().at(0).row());
				checkInputWidgetLimits(*ui->column1_input, item.pinIndices().item().at(0).column());
				checkInputWidgetLimits(*ui->row2_input, item.pinIndices().item().at(1).row());
				checkInputWidgetLimits(*ui->column2_input, item.pinIndices().item().at(1).column());
				checkInputWidgetLimits(*ui->row3_input, item.pinIndices().item().at(2).row());
				checkInputWidgetLimits(*ui->column3_input, item.pinIndices().item().at(2).column());

				checkInputWidgetLimits(*ui->pin_input, item.actions().item().front().pin());
				checkInputWidgetLimits(*ui->dThetaInd_input, item.actions().item().front().dThetaInd());
				checkInputWidgetLimits(*ui->pkmTheta_input, item.pkmTheta());

				// Setup common frame
				ui->agent_input->setValue(item.agent());
				ui->TBeg_input->setValue(item.TBeg());
				ui->TEnd_input->setValue(item.TEnd());
				ui->ind_input->setValue(item.ind());

				// Setup input widgets
				ui->row1_input->setValue(item.pinIndices().item().at(0).row());
				ui->column1_input->setValue(item.pinIndices().item().at(0).column());
				ui->row2_input->setValue(item.pinIndices().item().at(1).row());
				ui->column2_input->setValue(item.pinIndices().item().at(1).column());
				ui->row3_input->setValue(item.pinIndices().item().at(2).row());
				ui->column3_input->setValue(item.pinIndices().item().at(2).column());

				// FIXME: we do not support dynamic action lists.
				assert(item.actions().item().size() == 1);

				ui->pin_input->setValue(item.actions().item().front().pin());
				ui->dThetaInd_input->setValue(item.actions().item().front().dThetaInd());

				// Disable rotation input widget if rotation pin is set to zero.
				ui->dThetaInd_input->setEnabled(item.actions().item().front().pin() ? true : false);

				ui->pkmTheta_input->setValue(item.pkmTheta());

				// Setup commentary input.
				if(item.comment().present()) {
					ui->comment_input->setPlainText(item.comment().get().c_str());
				} else {
					ui->comment_input->clear();
				}

				// Allow fine-tuning and saving.
				if(last_executed_ind == item.ind()) {
					finetuning_active = true;
					ui->pushButton_save->setEnabled(true);
				} else {
					finetuning_active = false;
					ui->pushButton_save->setEnabled(false);
				}
			}

			// Disable/enable input containers
			this->setEnabled(true);
			ui->pkm_frame->setEnabled(false);
			ui->mbase_frame->setEnabled(true);

			break;
		default:
			assert(0);
			break;
	}

	// Enable navigation.
	enableNavigation(true);
}

void wgt_plan::my_open(bool set_on_top)
{
	// Handle current plan item
	reload();

	wgt_base::my_open(set_on_top);
}

void wgt_plan::on_pushButton_prev_clicked()
{
	interface.ui_ecp_obj->ui_rep.reply = lib::PLAN_PREV;

	reply();
}

void wgt_plan::on_pushButton_next_clicked()
{
	interface.ui_ecp_obj->ui_rep.reply = lib::PLAN_NEXT;

	reply();
}

void wgt_plan::on_pushButton_exec_clicked()
{
	interface.ui_ecp_obj->ui_rep.reply = lib::PLAN_EXEC;

	// Use local references with short names.
	const lib::ECP_message & request = interface.ui_ecp_obj->ecp_to_ui_msg;
	lib::UI_reply & reply = interface.ui_ecp_obj->ui_rep;

	// Extract data from string.
	std::istringstream istr(request.plan_item_string);
	boost::archive::text_iarchive ia(istr);
	xml_schema::istream<boost::archive::text_iarchive> is (ia);

	// String for serialized data.
	std::ostringstream ostr;
	boost::archive::text_oarchive oa(ostr);
	xml_schema::ostream<boost::archive::text_oarchive> os(oa);

	// Copy the read-only part.
	reply.plan_item_type = request.plan_item_type;

	// Sent back the current item.
	switch(reply.plan_item_type) {
		case lib::PKM_AND_HEAD:
			{
				// Deserialize data.
				Pkm::ItemType item(is);

				// Setup input widgets
				item.Xyz_Euler_Zyz()->x() = ui->x_input->value();
				item.Xyz_Euler_Zyz()->y() = ui->y_input->value();
				item.Xyz_Euler_Zyz()->z() = ui->z_input->value();
				item.Xyz_Euler_Zyz()->alpha() = ui->alpha_input->value();
				item.Xyz_Euler_Zyz()->beta() = ui->beta_input->value();
				item.Xyz_Euler_Zyz()->gamma() = ui->gamma_input->value();
				item.beta7() = ui->head_input->value();

				// Handle comment widget.
				if(ui->comment_input->toPlainText().length() > 0 ) {
					item.comment().set(ui->comment_input->toPlainText().toStdString());
				} else {
					item.comment().reset();
				}

				// serialize data
				os << item;
			}

			break;
		case lib::MBASE_AND_BENCH:
			{
				// Deserialize data.
				Mbase::ItemType item(is);

				// Setup input widgets data.
				item.pinIndices().item().at(0).row() = ui->row1_input->value();
				item.pinIndices().item().at(0).column() = ui->column1_input->value();
				item.pinIndices().item().at(1).row() = ui->row2_input->value();
				item.pinIndices().item().at(1).column() = ui->column2_input->value();
				item.pinIndices().item().at(2).row() = ui->row3_input->value();
				item.pinIndices().item().at(2).column() = ui->column3_input->value();

				item.actions().item().front().pin() = ui->pin_input->value();
				item.actions().item().front().dThetaInd() = ui->dThetaInd_input->value();
				item.pkmTheta() = ui->pkmTheta_input->value();

				// Handle comment widget.
				if(ui->comment_input->toPlainText().length() > 0 ) {
					item.comment().set(ui->comment_input->toPlainText().toStdString());
				} else {
					item.comment().reset();
				}

				// serialize data
				os << item;
			}

			break;
		default:
			assert(0);
			break;
	}

	// Copy string to the reply buffer.
	reply.plan_item_string = ostr.str();

	this->reply();

	// Record the last executed item.
	last_executed_ind = ui->ind_input->value();
}

void wgt_plan::on_pushButton_save_clicked()
{
	interface.ui_ecp_obj->ui_rep.reply = lib::PLAN_SAVE;

	reply();
}

void wgt_plan::on_pushButton_reload_clicked()
{
	reload();
}

void wgt_plan::on_pushbutton_clearComment_clicked()
{
	ui->comment_input->clear();
}

void wgt_plan::on_pushbutton_copy_clicked()
{
	if(ui->mbase_frame->isEnabled()) {
		clipboard.mbase_item.pkmTheta = ui->pkmTheta_input->value();

		clipboard.mbase_item.rows[0] = ui->row1_input->value();
		clipboard.mbase_item.rows[1] = ui->row2_input->value();
		clipboard.mbase_item.rows[2] = ui->row3_input->value();

		clipboard.mbase_item.columns[0] = ui->column1_input->value();
		clipboard.mbase_item.columns[1] = ui->column2_input->value();
		clipboard.mbase_item.columns[2] = ui->column3_input->value();

		// Common part.
		clipboard.mbase_item.comment = ui->comment_input->toPlainText().toStdString();
		clipboard.mbase_item.agent = ui->agent_input->value();
		clipboard.mbase_item.present = true;
	}

	if(ui->pkm_frame->isEnabled()) {
		clipboard.pkm_item.x = ui->x_input->value();
		clipboard.pkm_item.y = ui->y_input->value();
		clipboard.pkm_item.z = ui->z_input->value();
		clipboard.pkm_item.alpha = ui->alpha_input->value();
		clipboard.pkm_item.beta = ui->beta_input->value();
		clipboard.pkm_item.gamma = ui->gamma_input->value();
		clipboard.pkm_item.head = ui->head_input->value();
		clipboard.pkm_item.ind = (ui->ind_input->value() + 100) % 100;

		// Common part.
		clipboard.pkm_item.comment = ui->comment_input->toPlainText().toStdString();
		clipboard.pkm_item.agent = ui->agent_input->value();
		clipboard.pkm_item.present = true;
	}
}

void wgt_plan::on_pushbutton_paste_clicked()
{
	finetuning_active = false;

	if(ui->mbase_frame->isEnabled() && clipboard.mbase_item.present) {
		if(clipboard.mbase_item.agent == ui->agent_input->value()) {
			ui->pkmTheta_input->setValue(clipboard.mbase_item.pkmTheta);

			ui->row1_input->setValue(clipboard.mbase_item.rows[0]);
			ui->row2_input->setValue(clipboard.mbase_item.rows[1]);
			ui->row3_input->setValue(clipboard.mbase_item.rows[2]);

			ui->column1_input->setValue(clipboard.mbase_item.columns[0]);
			ui->column2_input->setValue(clipboard.mbase_item.columns[1]);
			ui->column3_input->setValue(clipboard.mbase_item.columns[2]);

			ui->comment_input->setPlainText(clipboard.mbase_item.comment.c_str());
		} else {
			interface.ui_msg->message(lib::NON_FATAL_ERROR, "Pasting clipboard from different agent not allowed");
		}
	}

	if(ui->pkm_frame->isEnabled() && clipboard.pkm_item.present) {
		if(clipboard.pkm_item.agent != ui->agent_input->value()) {
			interface.ui_msg->message(lib::NON_FATAL_ERROR, "Pasting command from different agent not allowed");
		} else {
			bool allowed = false;

			if(clipboard.pkm_item.ind == (ui->ind_input->value() + 100) % 100) {
				allowed = true;
			}

			switch(clipboard.pkm_item.ind) {
				// PRE/SUPPORT/POST->PRE/SUPPORT/POST
				case 0:
				case 20:
				case 80:
					switch ((ui->ind_input->value() + 100) % 100) {
						case 0:
						case 20:
						case 80:
							allowed = true;
							break;
						default:
							break;
					}
					break;
				// Copy between NEUTRALs.
				case 40:
				case 60:
					switch ((ui->ind_input->value() + 100) % 100) {
						case 40:
						case 60:
						case 80:
							allowed = true;
							break;
						default:
							break;
					}
					break;
				default:
					break;
			}

			if (allowed) {
				ui->x_input->setValue(clipboard.pkm_item.x);
				ui->y_input->setValue(clipboard.pkm_item.y);
				ui->z_input->setValue(clipboard.pkm_item.z);
				ui->alpha_input->setValue(clipboard.pkm_item.alpha);
				ui->beta_input->setValue(clipboard.pkm_item.beta);
				ui->gamma_input->setValue(clipboard.pkm_item.gamma);
				ui->head_input->setValue(clipboard.pkm_item.head);

				ui->comment_input->setPlainText(clipboard.mbase_item.comment.c_str());
			} else {
				interface.ui_msg->message(lib::NON_FATAL_ERROR, "Pasting not allowed");
			}
		}
	}

	finetuning_active = true;
}

void wgt_plan::on_pin_input_valueChanged(int i)
{
	// Disable rotation input widget when the pin is set to zero.
	ui->dThetaInd_input->setEnabled((i != 0) ? true : false);
}

void wgt_plan::on_x_input_valueChanged(double value)
{
	finetune_pkm();
}

void wgt_plan::on_y_input_valueChanged(double value)
{
	finetune_pkm();
}

void wgt_plan::on_z_input_valueChanged(double value)
{
	finetune_pkm();
}

void wgt_plan::on_alpha_input_valueChanged(double value)
{
	finetune_pkm();
}

void wgt_plan::on_beta_input_valueChanged(double value)
{
	finetune_pkm();
}

void wgt_plan::on_gamma_input_valueChanged(double value)
{
	finetune_pkm();
}

void wgt_plan::on_head_input_valueChanged(double value)
{
	finetune_head();
}

void wgt_plan::on_pkmTheta_input_valueChanged(double value)
{
	finetune_mbase();
}

void wgt_plan::finetune_head()
{
	if(!finetuning_active)
		return;

	// Initialize robot iterator.
	ui::common::robots_t::const_iterator it = interface.getRobots().end();

	// Use local reference with short name.
	const lib::ECP_message & request = interface.ui_ecp_obj->ecp_to_ui_msg;

	// Check if we are at PKM/HEAD plan item.
	if(request.plan_item_type == lib::PKM_AND_HEAD) {

		// Select agent.
		switch(ui->agent_input->value()) {
			case 1:
				it = interface.getRobots().find(lib::shead1::ROBOT_NAME);
				break;
			case 2:
				it = interface.getRobots().find(lib::shead2::ROBOT_NAME);
				break;
			default:
				assert(0);
				break;
		}

		// Check if robot is active.
		if (it != interface.getRobots().end()) {
			// Get the UI robot container. NOTE: the "robot" name is required for CATCH macro.
			ui::shead::UiRobot * robot = dynamic_cast<ui::shead::UiRobot *>(it->second);

			assert(robot);

			// Get the Ecp robot object.
			if(robot->ui_ecp_robot) {
				// Setup command.
				double final_position[lib::shead::NUM_OF_SERVOS];

				final_position[0] = ui->head_input->value();

				// Adjust head offset.
				if(final_position[0] < kinematics::shead::model::getLowerJointLimit()) {
					final_position[0] += M_PI/3;
				}
				if(final_position[0] > kinematics::shead::model::getUpperJointLimit()) {
					final_position[0] -= M_PI/3;
				}

				// Execute.
				try {
					robot->ui_ecp_robot->move_joints(final_position);
				}
				CATCH_SECTION_UI_PTR
			}
		}
	}

	// Disable navigation until fine-tuned pose is executed or discarded.
	enableNavigation(false);

	// Disable saving until fine-tuned pose is executed.
	ui->pushButton_save->setEnabled(false);
}

void wgt_plan::finetune_pkm()
{
	if(!finetuning_active)
		return;

	// Initialize robot iterator.
	ui::common::robots_t::const_iterator it = interface.getRobots().end();

	// Use local reference with short name.
	const lib::ECP_message & request = interface.ui_ecp_obj->ecp_to_ui_msg;

	// Check if we are at PKM/HEAD plan item.
	if(request.plan_item_type == lib::PKM_AND_HEAD) {

		// Select agent.
		switch(ui->agent_input->value()) {
			case 1:
				it = interface.getRobots().find(lib::spkm1::ROBOT_NAME);
				break;
			case 2:
				it = interface.getRobots().find(lib::spkm2::ROBOT_NAME);
				break;
			default:
				assert(0);
				break;
		}

		// Check if robot is active.
		if (it != interface.getRobots().end()) {
			// Get the UI robot container. NOTE: the "robot" name is required for CATCH macro.
			ui::spkm::UiRobot * robot = dynamic_cast<ui::spkm::UiRobot *>(it->second);

			assert(robot);

			// Get the Ecp robot object.
			if(robot->ui_ecp_robot) {
				// Setup command.
				double final_position[6];

				final_position[0] = ui->x_input->value();
				final_position[1] = ui->y_input->value();
				final_position[2] = ui->z_input->value();
				final_position[3] = ui->alpha_input->value();
				final_position[4] = ui->beta_input->value();
				final_position[5] = ui->gamma_input->value();

				// Execute.
				try {
					robot->ui_ecp_robot->move_external(final_position,
							lib::epos::SYNC_TRAPEZOIDAL,
							lib::spkm::WRIST_XYZ_EULER_ZYZ,
							0.0);
				}
				CATCH_SECTION_UI_PTR
			}
		}
	}

	// Disable navigation until fine-tuned pose is executed or discarded.
	enableNavigation(false);

	// Disable saving until fine-tuned pose is executed.
	ui->pushButton_save->setEnabled(false);
}

void wgt_plan::finetune_mbase()
{
	if(!finetuning_active)
		return;

	// Initialize robot iterator.
	ui::common::robots_t::const_iterator it = interface.getRobots().end();

	// Use local reference with short name.
	const lib::ECP_message & request = interface.ui_ecp_obj->ecp_to_ui_msg;

	// Check if we are at PKM/HEAD plan item.
	if(request.plan_item_type == lib::MBASE_AND_BENCH) {

		// Select agent.
		switch(ui->agent_input->value()) {
			case 1:
				it = interface.getRobots().find(lib::smb1::ROBOT_NAME);
				break;
			case 2:
				it = interface.getRobots().find(lib::smb2::ROBOT_NAME);
				break;
			default:
				assert(0);
				break;
		}

		// Check if robot is active.
		if (it != interface.getRobots().end()) {
			// Get the UI robot container. NOTE: the "robot" name is required for CATCH macro.
			ui::smb::UiRobot * robot = dynamic_cast<ui::smb::UiRobot *>(it->second);

			assert(robot);

			// Get the Ecp robot object.
			if(robot->ui_ecp_robot) {
				// Setup command.
				double final_position[lib::smb::NUM_OF_SERVOS];

				final_position[0] = 0;
				final_position[1] = ui->pkmTheta_input->value();

				// Execute.
				try {
					robot->ui_ecp_robot->move_external(final_position, 0.0);
				}
				CATCH_SECTION_UI_PTR
			}
		}
	}

	// Disable navigation until fine-tuned pose is executed or discarded.
	enableNavigation(false);

	// Disable saving until fine-tuned pose is executed.
	ui->pushButton_save->setEnabled(false);
}

void wgt_plan::enableNavigation(bool enabled)
{
	// Change state of the navigation buttons.
	ui->pushButton_prev->setEnabled(enabled);
	ui->pushButton_next->setEnabled(enabled);
}

void wgt_plan::reply()
{
	interface.ui_ecp_obj->communication_state = ui::common::UI_ECP_REPLY_READY;

	if (interface.ui_ecp_obj->communication_state != ui::common::UI_ECP_REPLY_READY) {
		interface.ui_ecp_obj->ui_rep.reply = lib::QUIT;
	}
	interface.ui_ecp_obj->synchroniser.command();

	this->setEnabled(false);

	// Disable fine-tuning until new plan item will be loaded.
	finetuning_active = false;

	// Enable navigation.
	enableNavigation(true);
}
