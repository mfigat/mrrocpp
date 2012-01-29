/*
 * plan_iface.cc
 *
 *  Created on: Jan 20, 2012
 *      Author: ptroja
 */

#ifndef PLAN_IFACE_H_
#define PLAN_IFACE_H_

#include <memory>
#include <string>

// Forward declaration.
class Plan;

class plan_iface {
public:
	//! Read and validate plan from file.
	static ::std::auto_ptr< ::Plan > readPlanFromFile(const std::string & path);

	//! Key with path load to plan file from.
	static std::string planpath;

	//! Key with path to save plan file to.
	static std::string savepath;
};

#endif /* PLAN_IFACE_H_ */
