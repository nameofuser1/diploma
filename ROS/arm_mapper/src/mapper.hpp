/*
 * mapper.hpp
 *
 *  Created on: May 12, 2018
 *      Author: kript0n
 */

#ifndef ARM_CONTROL_SRC_MAPPER_HPP_
#define ARM_CONTROL_SRC_MAPPER_HPP_





class Map {

public:
	Map(void);
	virtual ~Map(void);
};


class GazeboMap : Map {

public:
	GazeboMap(void);
};




#endif /* ARM_CONTROL_SRC_MAPPER_HPP_ */
