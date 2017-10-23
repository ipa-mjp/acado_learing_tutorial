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
	ros::init(argc, argv,"acado_tut_close_loop_mpc");
	ros::NodeHandle nh;

	using namespace ACADO;

    // INTRODUCE THE VARIABLES:
    // -------------------------
    DifferentialState xB;
    DifferentialState xW;
    DifferentialState vB;
    DifferentialState vW;

    Control F;
    Disturbance R;

    double mB = 350.0;
    double mW = 50.0;
    double kS = 20000.0;
    double kT = 200000.0;


    // DEFINE A DIFFERENTIAL EQUATION:
    // -------------------------------
    DifferentialEquation f;

    f << dot(xB) == vB;
    f << dot(xW) == vW;
    f << dot(vB) == ( -kS*xB + kS*xW + F ) / mB;
    f << dot(vW) == ( -kT*xB - (kT+kS)*xW + kT*R - F ) / mW;


    // DEFINE LEAST SQUARE FUNCTION:
    // -----------------------------
    Function h;

    h << xB;
    h << xW;
    h << vB;
    h << vW;
    h << F;

    // LSQ coefficient matrix
    DMatrix Q(5,5);
    Q(0,0) = 10.0;
    Q(1,1) = 10.0;
    Q(2,2) = 1.0;
    Q(3,3) = 1.0;
    Q(4,4) = 1.0e-8;

    // Reference
    DVector r(5);
    r.setAll( 0.0 );


    // DEFINE AN OPTIMAL CONTROL PROBLEM:
    // ----------------------------------
    const double tStart = 0.0;
    const double tEnd   = 1.0;

    OCP ocp( tStart, tEnd, 20 );

    ocp.minimizeLSQ( Q, h, r );

    ocp.subjectTo( f );

    ocp.subjectTo( -200.0 <= F <= 200.0 );
    ocp.subjectTo( R == 0.0 );


    // SETTING UP THE REAL-TIME ALGORITHM:
    // -----------------------------------
    RealTimeAlgorithm alg( ocp,0.025 );
    alg.set( MAX_NUM_ITERATIONS, 1 );
    alg.set( PLOT_RESOLUTION, MEDIUM );

    GnuplotWindow window;
        window.addSubplot( xB, "Body Position [m]" );
        window.addSubplot( xW, "Wheel Position [m]" );
        window.addSubplot( vB, "Body Velocity [m/s]" );
        window.addSubplot( vW, "Wheel Velocity [m/s]" );
        window.addSubplot( F,  "Damping Force [N]" );
        window.addSubplot( R,  "Road Excitation [m]" );

    alg << window;

    // SETUP CONTROLLER AND PERFORM A STEP:
    // ------------------------------------
    StaticReferenceTrajectory zeroReference( "/home/bfb-ws/mpc_ws/src/acado_learing_tutorial/closed_loop_mpc/ref.txt" );

    Controller controller( alg,zeroReference );

    DVector y( 4 );
    y.setAll(0.0);
    y(0) = 0.01;

    controller.init( 0.0,y );
    controller.step( 0.0,y );

    DVector u;
    controller.getU(u);
    u.print( "Feedback control" );

    std::cout<<"\033[36;1m"<<"Hello"<<"\033[36;0m"<<std::endl;

    return 0;
}


