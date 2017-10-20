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

#include <acado/acado_toolkit.hpp>
#include <acado/acado_optimal_control.hpp>
#include <acado/utils/acado_io_utils.hpp>
#include <acado/utils/acado_utils.hpp>
#include <acado/acado_gnuplot.hpp>
#include <acado/bindings/acado_gnuplot/gnuplot_window.hpp>

#include <acado/process/process.hpp>
#include <acado/matrix_vector/vector.hpp>
#include <acado/matrix_vector/matrix_vector_tools.hpp>
#include <acado/utils/acado_types.hpp>

#include <iostream>
#include <vector>
#include <stdio.h>
#include <acado/utils/acado_message_handling.hpp>

int main(int argc, char **argv)
{
	ros::init(argc, argv,"acado_tut_semi_implicit_dae");
	ros::NodeHandle nh;

	using namespace ACADO;

    // INTRODUCE THE VARIABLES:
    // -------------------------
    DifferentialState xB;
    DifferentialState xW;
    DifferentialState vB;
    DifferentialState vW;

    Disturbance R;
    Control F;

    Parameter mB;
    double mW = 50.0;
    double kS = 20000.0;
    double kT = 200000.0;

    // DEFINE THE DYNAMIC SYSTEM:
    // --------------------------
    DifferentialEquation f;

    f << dot(xB) == vB;
    f << dot(xW) == vW;
    f << dot(vB) == ( -kS*xB + kS*xW + F ) / mB;
    f << dot(vW) == (  kS*xB - (kT+kS)*xW + kT*R - F ) / mW;

    OutputFcn g;
    g << xB;
    g << 500.0*vB + F;

    DynamicSystem dynSys( f,g );

    // SETUP THE PROCESS:
    // ------------------
    Process myProcess;

    myProcess.setDynamicSystem( dynSys,INT_RK45 );
    myProcess.set( ABSOLUTE_TOLERANCE,1.0e-8 );

	//DVector x0( 4 );
    std::vector<double> x0( 4 , 0.0 );
    //x0.setZero( );
    //x0( 0 ) = 0.01;
    x0[0] = 0.01;

    myProcess.initializeStartValues( x0 );
    myProcess.setProcessDisturbance( "/home/bfb-ws/mpc_ws/src/acado_learing_tutorial/basic_mpc_implementation/road.txt" );

    myProcess.set( PLOT_RESOLUTION,HIGH );

    GnuplotWindow window;
        window.addSubplot( xB, "Body Position [m]" );
        window.addSubplot( xW, "Wheel Position [m]" );
        window.addSubplot( vB, "Body Velocity [m/s]" );
        window.addSubplot( vW, "Wheel Velocity [m/s]" );

        window.addSubplot( F,"Damping Force [N]" );
        window.addSubplot( mB,"Body Mass [kg]" );
        window.addSubplot( R, "Road Disturbance" );
        window.addSubplot( g(0),"Output 1" );
        window.addSubplot( g(1),"Output 2" );

    myProcess << window;


    // SIMULATE AND GET THE RESULTS:
    // -----------------------------
    VariablesGrid u( 1,0.0,1.0,6 );

    u( 0,0 ) = 10.0;
    u( 1,0 ) = -200.0;
    u( 2,0 ) = 200.0;
    u( 3,0 ) = 0.0;
    u( 4,0 ) = 0.0;
    u( 5,0 ) = 0.0;

    //Vector p( 1 );
    std::vector<double> p( 1 , 0.0 );
    //p(0) = 350.0;
    p[0] = 350.0;

    myProcess.init( 0.0 );
    myProcess.run( u,p );

    VariablesGrid xSim, ySim;

    myProcess.getLast( LOG_SIMULATED_DIFFERENTIAL_STATES,xSim );
    xSim.print( "Simulated Differential States" );
    //xSim.print();

    myProcess.getLast( LOG_PROCESS_OUTPUT,ySim );
    ySim.print( "Process Output" );

    return 0;
}



