/*
 /*!
 *****************************************************************
 * \file
 *
 * \note
 * Copyright (c) 2017 \n
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 * Project name: acado_learing_tutorial
 * \note
 * ROS stack name: rocket_flight
 * \note
 * ROS package name: rocket_flight
 *
 * \author
 * Author: Patel, Mayank email: mayank.jitendrakumar.patel@ipa.fraunhofer.de
 *
 * \date Date of creation: June, 2017
 *
 * \brief
 *
 * *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer. \n
 * - Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution. \n
 * - Neither the name of the Fraunhofer Institute for Manufacturing
 * Engineering and Automation (IPA) nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 *
 ******************************************************************/

#ifndef ACADO_LEARING_TUTORIAL_ROCKET_FILGHT_H
#define ACADO_LEARING_TUTORIAL_ROCKET_FILGHT_H

#include <ros/ros.h>

#include <iostream>
#include <fstream>
#include <string>

class RocketFight {
public:
	RocketFight();
	~RocketFight();
	void initializeClassVariable(void);

private:
};



#endif //ACADO_LEARING_TUTORIAL_ROCKET_FILGHT_H
