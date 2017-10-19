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
#include <acado/acado_gnuplot.hpp>

int main(int argc, char **argv)
{
	ros::init(argc, argv,"acado_tut_multi_stage_ocp");
	ros::NodeHandle nh;

	using namespace ACADO;

	DifferentialState		x			;	// the position of the "hand"
	DifferentialState		v			;	// the velocity of the "hand"
	DifferentialState		y			;	// the position of the "jojo"
	DifferentialState		w			;	// the velocity of the "jojo"

	Control					u			;	//	the control action of the "hand"

	Parameter				T1			;	//	the duration of phase I
	Parameter				T2			;	//	the duration of phase II


	//THE GIVEN JOJO PARAMETERS:
	const double 			m = 0.200	;	//	the mass of the jojo (200g)
	const double 			J = 1e-4	;	//	the inertia of the jojo (1e-4 kg*m^2)
	const double 			r = 0.010	;	//	the coiling radius of the jojo (1cm)
	const double 			g = 9.81	;	//	the gravitational constant (9.81m/s^2)
    const double       		a = 1e-2 	;   // the coiling friction (1e-2/s)
    const double       		L = 1.00 	;   // the length of the rope (1m)

    //OTHER USEFUL CONSTANTS:
    const double 	k  = J/(m*r*r+J)	;   // the jojo's damping ratio
    const double 	mu = 1.0 - k    	;   // the effective mass ratio


    //THE MODEL EQUATION FOR PHASE I:
    //----------------------------------
    DifferentialEquation  f1( 0.0, T1 )	;
    f1	<<	dot(x) == v					;
    f1	<<	dot(v) == u					;
    f1	<<	dot(y) == w					;
    f1	<<	dot(w) == -mu*g + k*u + a*(v-w);


    //THE EQUATIONS FOR THE JUMP:
    //---------------------------------
    Transition		j		;
    IntermediateState	z	;

    z	= k*v;

    j	<<	x == x	;
    j	<<	v == v	;
    j	<<	y == y	;
    j	<<	w == z	+ sqrt( z*z + k*( w*w - 2.0*w*v ) );

    //THE MODEL EQUATIONS FOR PHASE II:
    //----------------------------------
    DifferentialEquation  f2( T1, T2 )		;
    f2	<<	dot(x) == v						;
    f2	<<	dot(x) == u						;
    f2	<<	dot(x) == w						;
    f2	<<	dot(x) == -mu*g + k*u + a*(v-w)	;


    //	DEFINE AN OPTIMAL CONTROL PROBLEM:
    //-------------------------------------
    OCP ocp;
    ocp.minimizeLagrangeTerm( u*u )	;
/*
    ocp.subjectTo( f1, 20 );
    ocp.subjectTo( j      );
    ocp.subjectTo( f2, 20 );
*/

    ocp.subjectTo( f1 )	;
    ocp.subjectTo( f2 )	;

    ocp.subjectTo( AT_START     , x   ==  0.00 );
    ocp.subjectTo( AT_START     , v   ==  0.00 );
    ocp.subjectTo( AT_START     , y   ==  0.00 );
    ocp.subjectTo( AT_START     , w   ==  0.00 );

    ocp.subjectTo( AT_TRANSITION, x-y ==  L    );

    ocp.subjectTo( AT_END       , x   == -0.10 );
    ocp.subjectTo( AT_END       , v   ==  0.00 );
    ocp.subjectTo( AT_END       , x-y ==  0.00 );
    ocp.subjectTo( AT_END       , w   ==  0.00 );

    ocp.subjectTo( 0.0 <= T1 <= 2.0 );
    ocp.subjectTo( 0.0 <= T2 <= 4.0 );


    // SETUP A GNUPLOT WINDOW TO DISPLAY THE RESULTS:
    // ---------------------------------------------------
    GnuplotWindow window;
    window.addSubplot( x, "POSITION OF THE HAND: x " );
    window.addSubplot( v, "VELOCITY OF THE HAND: v " );
    window.addSubplot( y, "POSITION OF THE JOJO: y " );
    window.addSubplot( w, "VELOCITY OF THE JOJO: w " );
    window.addSubplot( u, "THE CONTROL INPUT   : u " );

    // DEFINE AN OPTIMIZATION ALGORITHM AND SOLVE THE OCP:
    // ---------------------------------------------------
    OptimizationAlgorithm algorithm(ocp);

    algorithm << window;
    algorithm.solve();

return 0;
}



