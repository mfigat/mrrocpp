/*!
 * @file mp_t_plan.h
 *
 * @date Jan 20, 2012
 * @author ptroja
 */

#ifndef SWARMITFIX_PLAN_EXECUTOR_H_
#define SWARMITFIX_PLAN_EXECUTOR_H_

#include <boost/shared_ptr.hpp>
#include <boost/unordered_map.hpp>

#include "base/ecp_mp/ecp_ui_msg.h"
#include "mp_t_demo_base.h"

#include "plan.hxx"

namespace mrrocpp {
namespace mp {
namespace task {
namespace swarmitfix {

/** @defgroup swarmitfix swarmitfix
 *  @ingroup application
 *  @{
 */

/*!
 * @brief Agent1 SwarmItFIX demo executed in Piaggio.
 *
 * @author tkornuta
 * @date Jan 20, 2012
 */
class plan_executor : public mrrocpp::mp::task::swarmitfix::demo_base
{
public:
	//! Calls the base class constructor.
	plan_executor(lib::configurator &config_);

	//! Creates robots on the base of configuration.
	void create_robots(void);

	//! Executes the plan for supporting of a wooden plate in few, learned positions.
	void main_task_algorithm(void);

protected:
	//! Plan.
	boost::shared_ptr<Plan> p;

	//! Handle PKM+HEAD plan item.
	void executeCommandItem(const Plan::PkmType::ItemType & pkmCmd);

	//! Handle BENCH+MBASE plan item.
	void executeCommandItem(const Plan::MbaseType::ItemType & smbCmd, int dir);

	//! Save modified plan to file
	void save_plan(const Plan & p);

	//! Step-mode execution of pkm item
	lib::UI_TO_ECP_REPLY step_mode(Pkm::ItemType & item);

	//! Step-mode execution of mbase item
	lib::UI_TO_ECP_REPLY step_mode(Mbase::ItemType & item);

	//! Access to plan items at given index
	template <typename T>
	typename T::iterator StateAtInd(int ind, T & items)
	{
		typename T::iterator it = items.begin();

		while ((it != items.end()) && it->ind() != ind)
			++it;

		return it;
	}

	//! Indices of the last command executed by the PKMs.
	typedef boost::unordered_map<Plan::PkmType::ItemType::AgentType, Plan::PkmType::ItemType::TypeType> pkm_state_t;
	pkm_state_t pkm_last_state;

	//! Locations on the bench.
	typedef boost::unordered_map<Plan::PkmType::ItemType::AgentType, lib::sbench::bench_pose> bench_locations_t;
	bench_locations_t bench_locations;

	//! State of the bench supply.
	lib::sbench::power_supply_state supply_state;

	//! Flag for enabling walking.
	int walking_enabled;

	//! Enable cleaning.
	int cleaning_active;

	//! Cleaning time.
	int cleaning_time;

	//! Control power supply.
	int voltage_from_bench;

	//! Head control.
	int vacuum_active, solidify_active;
};

/** @} */// end of swarmitfix

} /* namespace swarmitfix */
} /* namespace task */
} /* namespace mp */
} /* namespace mrrocpp */

#endif /* SWARMITFIX_PLAN_EXECUTOR_H_ */
