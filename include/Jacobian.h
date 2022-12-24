/*
 * Jacobian.h
 *
 *  Created on: 24-Dec-2022
 *      Author: abhishek
 */

#ifndef INCLUDE_JACOBIAN_H_
#define INCLUDE_JACOBIAN_H_

#include "ForwardKinematics.h"

namespace motion_planning {

class Jacobian: public ForwardKinematics {
public:
	Jacobian();
	virtual ~Jacobian();
};

} /* namespace motion_planning */

#endif /* INCLUDE_JACOBIAN_H_ */
