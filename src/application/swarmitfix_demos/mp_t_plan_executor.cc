/*!
 * @file mp_t_plan_executor.cpp
 *
 * @date Jan 20, 2012
 * @author tkornuta
 */

#include <fstream>

#include <boost/foreach.hpp>
#include <boost/thread/thread.hpp>

#include "mp_t_plan_executor.h"

#include "base/lib/configurator.h"
#include "base/ecp_mp/ecp_ui_msg.h"

#include "../swarmitfix_plan/plan_iface.h"
#include "plan.hxx"

#include "robot/shead/kinematic_model_shead.h"

#include "ecp_mp_g_sbench.h"

namespace mrrocpp {
namespace mp {
namespace task {

task* return_created_mp_task(lib::configurator &_config)
{
	return new swarmitfix::plan_executor(_config);
}

namespace swarmitfix {

void plan_executor::executeCommandItem(const Plan::PkmType::ItemType & pkmCmd)
{
	// Make sure that there are only Xyz-Euler-Zyz coordinates.
	assert(pkmCmd.pkmToWrist().present() == false);

	// Check if command is allowed in a given state.
	const Plan::PkmType::ItemType::TypeType last_state = pkm_last_state.at(pkmCmd.agent());
	const Plan::PkmType::ItemType::TypeType new_state = pkmCmd.type();

	switch(last_state) {
		case Type::SUPPORT:
			switch(new_state) {
				case Type::POST:
				case Type::PRE:
				case Type::SUPPORT:
					// SUPPORT->PRE/POST pose.
					break;
				default:
					sr_ecp_msg->message(lib::NON_FATAL_ERROR, "Only PRE/POST/SUPPORT pose allowed from SUPPORT");
					return;
			}
			break;
		case Type::POST:
		case Type::PRE:
			// Everything is allowed from PRE/POST pose.
			break;
		case Type::NEUTRAL:
			switch(new_state) {
				case Type::POST:
				case Type::NEUTRAL:
				case Type::PRE:
					// NEUTRAL->PRE/POST/NEUTRAL pose.
					break;
				default:
					sr_ecp_msg->message(lib::NON_FATAL_ERROR, "Only PRE/POST/NEUTRAL pose allowed from NEUTRAL");
					return;
			}
			break;
		default:
			// This should not happend.
			switch(new_state) {
				case Type::PRE:
				case Type::POST:
					// UNKNOWN->PRE/POST pose.
					break;
				default:
					sr_ecp_msg->message(lib::NON_FATAL_ERROR, "Only PRE/POST pose allowed from UNKNOWN");
					return;
			}
			break;
	}

	// PKM pose
	lib::Xyz_Euler_Zyz_vector goal(
			pkmCmd.Xyz_Euler_Zyz()->x(),
			pkmCmd.Xyz_Euler_Zyz()->y(),
			pkmCmd.Xyz_Euler_Zyz()->z(),
			pkmCmd.Xyz_Euler_Zyz()->alpha(),
			pkmCmd.Xyz_Euler_Zyz()->beta(),
			pkmCmd.Xyz_Euler_Zyz()->gamma()
			);

	std::cout << "Xyz_Euler_Zyz: " <<
			pkmCmd.Xyz_Euler_Zyz()->x() << " " <<
			pkmCmd.Xyz_Euler_Zyz()->y() << " " <<
			pkmCmd.Xyz_Euler_Zyz()->z() << " " <<
			pkmCmd.Xyz_Euler_Zyz()->alpha() << " " <<
			pkmCmd.Xyz_Euler_Zyz()->beta() << " " <<
			pkmCmd.Xyz_Euler_Zyz()->gamma() << std::endl;

	// Head pose.
	double head_pose = pkmCmd.beta7();

	// Adjust head offset.
	if(pkmCmd.beta7() < kinematics::shead::model::getLowerJointLimit()) {
		head_pose += M_PI/3;
	}
	if(pkmCmd.beta7() > kinematics::shead::model::getUpperJointLimit()) {
		head_pose -= M_PI/3;
	}

	// Check if above is enough.
	assert(head_pose >= kinematics::shead::model::getLowerJointLimit());
	assert(head_pose <= kinematics::shead::model::getUpperJointLimit());

	// Check if robot name match with item.
	lib::robot_name_t spkm_robot_name;
	lib::robot_name_t shead_robot_name;

	if (pkmCmd.agent() == 1) {
		spkm_robot_name = lib::spkm1::ROBOT_NAME;
		shead_robot_name = lib::shead1::ROBOT_NAME;
	} else if (pkmCmd.agent() == 2) {
		spkm_robot_name = lib::spkm2::ROBOT_NAME;
		shead_robot_name = lib::shead2::ROBOT_NAME;
	} else {
		assert(0);
	}

	// PRE-head command;
	if(is_robot_activated(shead_robot_name)) {

		// Disable solidification and vacuum.
		shead_solidify(shead_robot_name, false);
		shead_vacuum(shead_robot_name, false);

		switch(pkmCmd.type()) {
			case Type::SUPPORT:
			case Type::NEUTRAL:
			case Type::PRE:
				move_shead_joints(shead_robot_name, head_pose);
				break;
			default:
				break;
		}
	}

	// Execute PKM command.
	if(is_robot_activated(spkm_robot_name)) move_spkm_external(spkm_robot_name, lib::epos::SYNC_TRAPEZOIDAL, goal);

	// POST-head command;
	if(is_robot_activated(shead_robot_name)) {
		switch(pkmCmd.type()) {
			case Type::SUPPORT:
				// Apply vacuum...
				shead_vacuum(shead_robot_name, vacuum_active);
				// ...wait a while...
				boost::this_thread::sleep(boost::posix_time::milliseconds(100));
				// ...apply solidification...
				shead_solidify(shead_robot_name, solidify_active);
				// ...and brake.
				spkm_brake(spkm_robot_name);
				break;
			default:
				break;
		}
	}

	// Update state of the agent.
	pkm_last_state.at(pkmCmd.agent()) = pkmCmd.type();
}

void plan_executor::executeCommandItem(const Plan::MbaseType::ItemType & smbCmd, int dir)
{
	// TODO: Only single-item actions are supported at this time.
	assert(smbCmd.actions().item().size() == 1);

	// Check if rotation is allowed in the current state.
	switch(pkm_last_state.at(smbCmd.agent())) {
		case Type::NEUTRAL:
			// PKM is in neutral pose.
			break;
		default:
			sr_ecp_msg->message(lib::NON_FATAL_ERROR, "Rotation prohibited in non-neutral pose");
			return;
	}

	// Check if robot name match with item.
	lib::robot_name_t smb_robot_name;

	if (smbCmd.agent() == 1) {
		smb_robot_name = lib::smb1::ROBOT_NAME;
	} else if (smbCmd.agent() == 2) {
		smb_robot_name = lib::smb2::ROBOT_NAME;
	} else {
		assert(0);
	}

	// Setup final pose.
	lib::sbench::bench_pose final_pose;

	for(int i = 0; i < 3; ++i) {
		final_pose.pins[i] = lib::sbench::pin(smbCmd.pinIndices().item().at(i).row(), smbCmd.pinIndices().item().at(i).column());
	}

	// Check if robot name match with item.
	if(is_robot_activated(smb_robot_name)) {
		// Do we want to translocate?
		if(smbCmd.actions().item().front().pin() && smbCmd.actions().item().front().dThetaInd() && walking_enabled > 0) {

			// Extract data about rotation pin.
			const lib::sbench::pin rotation_pin(
					smbCmd.pinIndices().item().at(smbCmd.actions().item().front().pin()-1).row(),
					smbCmd.pinIndices().item().at(smbCmd.actions().item().front().pin()-1).column()
					);

			// Shutdown all the pins but the one, which we rotate around.
			if(voltage_from_bench) {
				// Proceed only if we know where we are.
				if(bench_locations.count(smbCmd.agent()) > 0) {
					// Setup start pose.
					lib::sbench::bench_pose start_pose = bench_locations.at(smbCmd.agent());

					if(start_pose.pins[smbCmd.actions().item().front().pin()-1].column != final_pose.pins[smbCmd.actions().item().front().pin()-1].column
							|| start_pose.pins[smbCmd.actions().item().front().pin()-1].row != final_pose.pins[smbCmd.actions().item().front().pin()-1].row)
					{
						throw std::runtime_error("start_pose@rotation pin is different than final_pose@rotation_pin");
					}

					supply_state.set_off(start_pose);
					supply_state.set_on(rotation_pin);

					// Execute command.
					control_bench_power_supply(supply_state, 0);
				}
			}

			// Jump.
			smb_stan_on_one_leg(smb_robot_name, smbCmd.actions().item().front().pin());

			// Rotate.
			if(walking_enabled > 1) {
				smb_rotate_external(smb_robot_name, smbCmd.actions().item().front().dThetaInd(), smbCmd.pkmTheta());
			} else {
				smb_rotate_external(smb_robot_name, 0, smbCmd.pkmTheta());
			}

			// Clean.
			if(cleaning_active && cleaning_time > 0) {
				lib::sbench::cleaning_state cleaning;

				cleaning.set_on(final_pose);
				cleaning.set_off(rotation_pin);

				control_bench_cleaning(cleaning, cleaning_time);

				cleaning.set_all_off();

				control_bench_cleaning(cleaning, 0);
			}

			// Land.
			smb_pull_legs(smb_robot_name, lib::smb::OUT, lib::smb::OUT, lib::smb::OUT);

		} else {
			smb_rotate_external(smb_robot_name, 0, smbCmd.pkmTheta());
		}

		// Record new position.
		bench_locations[smbCmd.agent()] = final_pose;

		if(voltage_from_bench) {
			// Enable power for all the pins.
			supply_state.set_on(final_pose);

			control_bench_power_supply(supply_state, 0);

			bench_locations[smbCmd.agent()] = final_pose;
		}
	}
}

plan_executor::plan_executor(lib::configurator &config_) :
		demo_base(config_),
		walking_enabled(0)
{
	// Read plan from file.
	try {
		p = plan_iface::readPlanFromFile(config.value<std::string>(plan_iface::planpath));
	} catch (const xml_schema::Exception & e) {
		// The string operator gives more details than what().
		std::ostringstream osr;
		osr << e;

		// Print error to the system console.
		sr_ecp_msg->message(lib::FATAL_ERROR, osr.str());

		// And fail.
		throw;
	}

	// Initialize state of the agents.
	pkm_last_state.insert(pkm_state_t::value_type(Plan::PkmType::ItemType::AgentType(1), Type(Type::UNKNOWN)));
	pkm_last_state.insert(pkm_state_t::value_type(Plan::PkmType::ItemType::AgentType(2), Type(Type::UNKNOWN)));

	// Get the configuration.
	walking_enabled = config.value<int>("walking_enabled");
	cleaning_active = config.value<int>("cleaning_active");
	cleaning_time = config.value <int>("cleaning_time");
	voltage_from_bench = config.value <int>("voltage_from_bench");
	vacuum_active = config.value <int>("vacuum_active");
	solidify_active = config.value <int>("solidify_active");
}

void plan_executor::create_robots()
{
	// Activate robots (depending on the configuration settings).
	// SMB.

	// SPKM.
	ACTIVATE_MP_ROBOT(spkm1)
	ACTIVATE_MP_ROBOT(spkm2)

	// SHEAD.
	ACTIVATE_MP_ROBOT(shead1)
	ACTIVATE_MP_ROBOT(shead2)

	// SMB.
	ACTIVATE_MP_ROBOT(smb1)
	ACTIVATE_MP_ROBOT(smb2)

	// Activate the SBENCH robot.
	ACTIVATE_MP_ROBOT(sbench)
}

void plan_executor::save_plan(const Plan & p)
{
	if(config.exists(plan_iface::savepath)) {
		std::cout << "Save to file." << std::endl;
		std::string savepath = config.value<std::string>(plan_iface::savepath);
		std::ofstream ofs (savepath.c_str());
		plan(ofs, p);
	} else {
		std::stringstream ss;

		ss << "Path for saving plan file is missing, add '"
				<< plan_iface::savepath
				<< "' key to configuration file";

		sr_ecp_msg->message(lib::NON_FATAL_ERROR, ss.str());
	}
}

lib::UI_TO_ECP_REPLY plan_executor::step_mode(Mbase::ItemType & item)
{
	// Create the text representation.
	std::ostringstream ostr;
	{
		boost::archive::text_oarchive oa(ostr);
		xml_schema::ostream<boost::archive::text_oarchive> os(oa);

		// serialize data
		os << item;
	}

	// Request
	lib::ECP_message ecp_to_ui_msg;

	// Setup plan item
	ecp_to_ui_msg.ecp_message = lib::PLAN_STEP_MODE;
	ecp_to_ui_msg.plan_item_type = lib::MBASE_AND_BENCH;
	ecp_to_ui_msg.plan_item_string = ostr.str();

	// Reply
	lib::UI_reply ui_to_ecp_rep;

	if (messip::port_send(UI_fd, 0, 0, ecp_to_ui_msg, ui_to_ecp_rep) < 0) {

		uint64_t e = errno;
		perror("ecp operator_reaction(): Send() to UI failed");
		sr_ecp_msg->message(lib::SYSTEM_ERROR, e, "ecp: Send() to UI failed");
		BOOST_THROW_EXCEPTION(lib::exception::system_error());
	}

	if(ui_to_ecp_rep.reply == lib::PLAN_EXEC) {
		std::istringstream istr(ui_to_ecp_rep.plan_item_string);
		boost::archive::text_iarchive ia(istr);
		xml_schema::istream<boost::archive::text_iarchive> is (ia);

		// deserialize data
		item = Mbase::ItemType(is);
	}

	return ui_to_ecp_rep.reply;
}

lib::UI_TO_ECP_REPLY plan_executor::step_mode(Pkm::ItemType & item)
{
	// create archive
	std::ostringstream ostr;
	{
		boost::archive::text_oarchive oa(ostr);
		xml_schema::ostream<boost::archive::text_oarchive> os(oa);

		// serialize data
		os << item;
	}

	// Request
	lib::ECP_message ecp_to_ui_msg;

	// Setup plan item
	ecp_to_ui_msg.ecp_message = lib::PLAN_STEP_MODE;
	ecp_to_ui_msg.plan_item_type = lib::PKM_AND_HEAD;
	ecp_to_ui_msg.plan_item_string = ostr.str();

	// Reply
	lib::UI_reply ui_to_ecp_rep;

	if (messip::port_send(UI_fd, 0, 0, ecp_to_ui_msg, ui_to_ecp_rep) < 0) {

		uint64_t e = errno;
		perror("ecp operator_reaction(): Send() to UI failed");
		sr_ecp_msg->message(lib::SYSTEM_ERROR, e, "ecp: Send() to UI failed");
		BOOST_THROW_EXCEPTION(lib::exception::system_error());
	}

	if(ui_to_ecp_rep.reply == lib::PLAN_EXEC) {
		std::istringstream istr(ui_to_ecp_rep.plan_item_string);
		boost::archive::text_iarchive ia(istr);
		xml_schema::istream<boost::archive::text_iarchive> is (ia);

		// deserialize data
		item = Pkm::ItemType(is);
	}

	return ui_to_ecp_rep.reply;
}

#define EXECUTE_PLAN_IN_STEP_MODE	1

void plan_executor::main_task_algorithm(void)
{
	if(voltage_from_bench) {
		//! Get state of the bench.
		if(robot_m.count(lib::sbench::ROBOT_NAME)) {
			int foo;

			set_next_ecp_state(ecp_mp::sbench::generator::POWER_SUPPLY_STATUS, 0, foo, lib::sbench::ROBOT_NAME);
			wait_for_task_termination(false, 1, lib::sbench::ROBOT_NAME.c_str());

			// Create the text representation.
			std::istringstream istr(robot_m[lib::sbench::ROBOT_NAME]->ecp_reply_package.recognized_command);
			{
				boost::archive::text_iarchive ia(istr);

				// serialize data
				ia >> supply_state;
			}

			sr_ecp_msg->message(supply_state.display());
		}
	}

	/////////////////////////////////////////////////////////////////////////////////
	using namespace mrrocpp::lib::sbench;
	/////////////////////////////////////////////////////////////////////////////////

	bool execute_plan_in_step_mode = true;

	// Time index counter
	int indMin = 99999999, indMax = 0;

	// Setup index counter at the beginning of the plan
	BOOST_FOREACH(const Plan::PkmType::ItemType & it, p->pkm().item()) {
		if(indMin > it.ind()) indMin = it.ind();
		if(indMax < it.ind()) indMax = it.ind();
	}
	BOOST_FOREACH(const Plan::MbaseType::ItemType & it, p->mbase().item()) {
		if(indMin > it.ind()) indMin = it.ind();
		if(indMax < it.ind()) indMax = it.ind();
	}

	// Make sure that modulo 100 arithmetics operate on positives.
	assert(indMin > -99);

	for (int ind = indMin, dir = 0; true; ind += dir) {

		if(ind < indMin) ind = indMin;
		if(ind > indMax) ind = indMax;

		std::cout << "plan index = " << ind << std::endl;

		// Diagnostic timestamp
		boost::system_time start_timestamp;

		// Plan iterators
		const Plan::PkmType::ItemSequence::iterator pkm_it = StateAtInd(ind, p->pkm().item());
		const Plan::MbaseType::ItemSequence::iterator smb_it = StateAtInd(ind, p->mbase().item());

		// Current state
		State * currentActionState;

		bool record_timestamp = false;

		// Execute matching command item
		if(pkm_it != p->pkm().item().end()) {

			if(execute_plan_in_step_mode) {
				switch(step_mode(*pkm_it)) {
					case lib::PLAN_PREV:
						dir = -1;
						break;
					case lib::PLAN_NEXT:
						dir = +1;
						break;
					case lib::PLAN_EXEC:
						currentActionState = (State *) &(*pkm_it);
						start_timestamp = boost::get_system_time();
						executeCommandItem(*pkm_it);
						record_timestamp = true;
						dir = 0;
						break;
					case lib::PLAN_SAVE:
						save_plan(*p);
						dir = 0;
						break;
					case lib::PLAN_PLAY:
//						currentActionState = (State *) &(*pkm_it);
//						start_timestamp = boost::get_system_time();
//						executeCommandItem(*pkm_it);
//						record_timestamp = true;
						dir = 1;
						execute_plan_in_step_mode = false;
						break;
					default:
						break;
				}
			} else {
				currentActionState = (State *) &(*pkm_it);
				start_timestamp = boost::get_system_time();
				executeCommandItem(*pkm_it);
				record_timestamp = true;
				dir = 1;
			}

		} else if(smb_it != p->mbase().item().end()) {

			if(execute_plan_in_step_mode) {
				switch(step_mode(*smb_it)) {
					case lib::PLAN_PREV:
						dir = -1;
						break;
					case lib::PLAN_NEXT:
						dir = +1;
						break;
					case lib::PLAN_EXEC:
						currentActionState = (State *) &(*smb_it);
						start_timestamp = boost::get_system_time();
						executeCommandItem(*smb_it, dir);
						record_timestamp = true;
						dir = 0;
						break;
					case lib::PLAN_SAVE:
						save_plan(*p);
						dir = 0;
						break;
					case lib::PLAN_PLAY:
//						currentActionState = (State *) &(*smb_it);
//						start_timestamp = boost::get_system_time();
//						executeCommandItem(*smb_it, dir);
//						record_timestamp = true;
						dir = 1;
						execute_plan_in_step_mode = false;
						break;
					default:
						break;
				}
			} else {
				currentActionState = (State *) &(*smb_it);
				start_timestamp = boost::get_system_time();
				executeCommandItem(*smb_it, dir);
				record_timestamp = true;
				dir = 1;
			}
		} else {
			continue;
		}


		// Diagnostic timestamp
		if (record_timestamp) {
			// Get the stop timestamp
			boost::system_time stop_timestamp = boost::get_system_time();

			// Calculate command duration
			boost::posix_time::time_duration td = stop_timestamp - start_timestamp;

			std::cout << "Command duration in [ms] is " << td.total_milliseconds() << std::endl;
			currentActionState->state_reached_in_time().set(td.total_milliseconds()/1000.0);

//			// Wait for trigger
//			while(!(ui_pulse.isFresh() && ui_pulse.Get() == MP_TRIGGER)) {
//				// TODO: handle PAUSE/RESUME/STOP commands as well
//				if(ui_pulse.isFresh()) ui_pulse.markAsUsed();
//				ReceiveSingleMessage(true);
//			}
//
//			ui_pulse.markAsUsed();

		}

		// If all iterators are at the end...
		if(ind == indMax && dir > 0)
		{
			// ...then finish
			break;
		}
	}

	// Serialize to a file.
	//
	{
		if(config.exists(plan_iface::savepath)) {
			std::cout << "Serializing timing results to a file." << std::endl;
			std::ofstream ofs (config.value<std::string>(plan_iface::savepath).c_str());
			plan(ofs, *p);
		}
	}

	sr_ecp_msg->message("END");


	/////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////

	return;

}

} /* namespace swarmitfix */
} /* namespace task */
} /* namespace mp */
} /* namespace mrrocpp */
