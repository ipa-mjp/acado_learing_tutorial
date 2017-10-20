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

//#include <rocket_flight/rocket_flight.h>
#include <ros/ros.h>
#include <ros/package.h>

#include <acado_toolkit.hpp>
#include <acado/acado_optimal_control.hpp>
#include <acado/utils/acado_io_utils.hpp>
#include <acado/utils/acado_utils.hpp>
#include <acado/acado_gnuplot.hpp>

#include <iostream>
#include <stdio.h>
#include <acado/utils/acado_message_handling.hpp>

int main(int argc, char **argv)
{
	ros::init(argc, argv,"acado_tut_semi_implicit_dae");
	ros::NodeHandle nh;

	using namespace ACADO;

    // INTRODUCE THE VARIABLES:
    // -------------------------
    DifferentialState          phi, omega;    // the states of the pendulum
    Parameter                  l, alpha  ;    // its length and the friction
    const double               g = 9.81  ;    // the gravitational constant
    DifferentialEquation       f         ;    // the model equations
    Function                   h         ;    // the measurement function


    Grid timeGrid( 0.0, 2.0, 10	)		;
    //VariablesGrid measurements;       // read the measurements
    //measurements = readFromFile( "data.txt" );  // from a file.
    VariablesGrid measurements( 2, timeGrid )	;

    measurements(0,0) = 0.0000e01;	    measurements(0,0) = 1.0000000;
    measurements(0,1) = 0.2723210;		measurements(0,1) = 0.000;
    measurements(0,2) = 0.3723210;		measurements(0,2) = 0.5751460;
    measurements(0,3) = 0.7257520;		measurements(0,3) = -0.0591794;
    measurements(0,4) = 0.9061070;		measurements(0,4) = -0.3543470;
    measurements(0,5) = 1.2365100;		measurements(0,5) = -0.3030560;
    measurements(0,6) = 1.4261900;		measurements(0,6) = 0.000;
    measurements(0,7) = 1.5946900;		measurements(0,7) = -0.09642080;
    measurements(0,8) = 1.7202900;		measurements(0,8) = -0.01976710;
    measurements(0,9) = 2.0000000;		measurements(0,9) = 0.09351380;

    //  --------------------------------------
    OCP ocp(measurements.getTimePoints());    // construct an OCP
    h << phi                             ;    // the state phi is measured
    ocp.minimizeLSQ( h, measurements )   ;    // fit h to the data

    f << dot(phi  ) == omega             ;    // a symbolic implementation
    f << dot(omega) == -(g/l) *sin(phi )      // of the model
                     - alpha*omega     ;    // equations

    ocp.subjectTo( f                    );    // solve OCP s.t. the model,
    ocp.subjectTo( 0.0 <= alpha <= 4.0  );    // the bounds on alpha
    ocp.subjectTo( 0.0 <=   l   <= 2.0  );    // and the bounds on l.
    //  --------------------------------------

    GnuplotWindow window;
    window.addSubplot( phi  , "The angle phi", "time [s]", "angle [rad]" );
    window.addSubplot( omega, "The angular velocity dphi"                );
    window.addData( 0, measurements(0) );

    //  --------------------------------------
    ParameterEstimationAlgorithm algorithm(ocp); // the parameter estimation
    algorithm << window;
    algorithm.solve();                           // solves the problem

    return 0;
}



