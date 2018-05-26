/*
 * environement.h
 *
 *  Created on: May 12, 2018
 *      Author: kript0n
 */

#ifndef ARM_CONTROL_SRC_ENVIRONMENT_HPP_
#define ARM_CONTROL_SRC_ENVIRONMENT_HPP_


#include <stdint.h>
#include <iostream>
#include <vector>


namespace environment {


class object_position_t {
public:
	float x;
	float y;
	float z;

	float normal[3];

private:
    //friend std::ostream& operator<<(std::ostream&, const object_position_t&);
} ;




class EnvironmentObject {

    public:

        object_position_t& getPosition(void)
        {
            return (this->position);
        }

    protected:
        object_position_t	position;
};


class Cell : public EnvironmentObject{

    public:
        Cell(float x_size, float z_size, object_position_t pos);
        bool isFree(void);
        void setFree(bool free);

        float getSizeX(void)
        {
            return this->x_size;
        }

        float setSizeX(float x)
        {
            this->x_size = x;
        }

        float getSizeZ(void)
        {
            return this->z_size;
        }

        float setSizeZ(float z)
        {
            this->z_size = z;
        }

    private:
        bool	 	free;

        float 		x_size;
        float 		z_size;
};


class CellBase : public EnvironmentObject{

    public:
        CellBase(float x_size, float z_size, object_position_t pos);
        float x_size;
        float z_size;
};


class Table : public EnvironmentObject {

    public:
        Table(float x_size, float z_size, object_position_t pos);

    private:
        float x_size;
        float z_size;
};


class CellGroup : public EnvironmentObject {

    public:
        CellGroup(uint32_t xn, uint32_t zn,
                float cell_x_size, float cell_z_size,
                float base_x_size, float base_z_size,
                object_position_t pos);

        Cell* getCellByXZ(uint32_t x, uint32_t z);
        Cell* getNextFullCell(void);
        const object_position_t getCellPickPosition(Cell* cell);

    private:
        uint32_t xn;
        uint32_t zn;

        std::vector<CellBase> bases;
        std::vector<std::vector<Cell>> cells;
};

}

/*
std::ostream& operator<<(std::ostream& strm, const object_position_t& pos)
{
    return (strm << "Position:\n" << "X: " << pos.x << "; Y: " << pos.y <<
            "; Z: " << pos.z << "\nNormal vector: " << pos.normal[0] << " " << pos.normal[1] << " " << pos.normal[2]);
}*/

#endif /* ARM_CONTROL_SRC_ENVIRONMENT_HPP_ */
