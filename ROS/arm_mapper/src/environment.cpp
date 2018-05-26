/*
 * environment.cpp
 *
 *  Created on: May 12, 2018
 *      Author: kript0n
 */

#include "environment.hpp"

#include <iostream>
#include <string.h>


namespace environment
{


/************************** CELL GROUP ***************************/
CellGroup::CellGroup(uint32_t xn, uint32_t zn,
				float cell_x_size, float cell_z_size,
				float base_x_size, float base_z_size,
				object_position_t pos)
{
	this->position = pos;
	this->zn = zn;
	this->xn = xn;

	object_position_t new_obj_pos;
	bzero(&new_obj_pos, sizeof new_obj_pos);
	memcpy(new_obj_pos.normal, pos.normal, sizeof(new_obj_pos.normal));

	for(uint32_t i=0; i<xn; i++)
	{
		new_obj_pos.x = i*base_x_size;
		this->bases.push_back(CellBase(base_x_size, base_z_size, new_obj_pos));
	}

	bzero(&new_obj_pos, sizeof new_obj_pos);
	memcpy(new_obj_pos.normal, pos.normal, sizeof(new_obj_pos.normal));
	for(uint32_t i=0; i<zn; i++)
	{
		std::vector<Cell> x_cells;
		this->cells.push_back(x_cells);

		new_obj_pos.z = i*cell_z_size + base_z_size;
		new_obj_pos.y = 0;

		for(uint32_t j=0; j<xn; j++)
		{
			new_obj_pos.x = j*cell_x_size;
			this->cells[i].push_back(Cell(cell_x_size, cell_z_size,
					new_obj_pos));
		}
	}
}


Cell* CellGroup::getCellByXZ(uint32_t x, uint32_t z)
{
    if(x > this->xn || z > this->zn)
    {
        return NULL;
    }

    auto &row_vec = this->cells.at(z);
    return &(row_vec.at(x));
}


Cell* CellGroup::getNextFullCell(void)
{
	for(auto &row_vec : this->cells)
	{
		for(auto &cell : row_vec)
		{
			if(!cell.isFree())
			{
                return &cell;
			}
		}
	}

	return NULL;
}


const object_position_t CellGroup::getCellPickPosition(Cell* cell)
{
    object_position_t position = this->getPosition();
    object_position_t &cell_position = cell->getPosition();

    position.x += cell_position.x;
    position.y += cell_position.y;
    position.z += cell_position.z;

    position.x += cell->getSizeX() / 2.0f;
    position.z += 0.005;

	return position;
}


/*********************** CELL BASE ***************************/
CellBase::CellBase(float x_size, float z_size, object_position_t pos)
{
	this->x_size = x_size;
	this->z_size = z_size;
	this->position = pos;
}


/*********************** CELL ******************************/
Cell::Cell(float x_size, float z_size, object_position_t pos)
{
	this->x_size = x_size;
	this->z_size = z_size;
	this->position = pos;
	this->free = false;
}


bool Cell::isFree(void)
{
    return this->free;
}


void Cell::setFree(bool free)
{
    this->free = free;
}

/*********************** TABLE ******************************/
Table::Table(float x_size, float z_size, object_position_t pos)
{
	this->x_size = x_size;
	this->z_size = z_size;
	this->position = pos;
}

}
