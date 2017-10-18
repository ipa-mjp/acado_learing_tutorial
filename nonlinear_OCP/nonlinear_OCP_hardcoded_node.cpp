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
	ros::init(argc, argv,"acado_tut_nonlinear_OPC");
	ros::NodeHandle nh;

	using namespace ACADO;

	DifferentialState		s,v,m		;	//the differential states
	Control 		  		u			;	//the control input u
	Parameter				T			;	//the time horizon T
	DifferentialEquation	f( 0.0, T)	;	//the differential equation

	//----------------------------------------------------------------------
	OCP ocp( 0.0, T)					;	//time horizon of the OCP: [0,T]
	ocp.minimizeMayerTerm( T )			;	//the time T should be optimized

	f	<<	dot(s) == v					;	//an implementation
	f	<<	dot(v) == (u-0.2*v*v)/m		;	//of the model equations
	f	<<	dot(m) == -0.01*u*u			;	//for the rocket.

	//set initial values, end values or set equalities constraints
	ocp.subjectTo( f )					;	//minimize T s.t the model,
	ocp.subjectTo( AT_START, s == 0.0 )	;	//the initial values for s,
	ocp.subjectTo( AT_START, v == 0.0 )	;	//the initial values for v,
	ocp.subjectTo( AT_START, m == 1.0 )	;	//the initial values for m,

	ocp.subjectTo( AT_END, s == 10.0 )	;	//the end values for s,
	ocp.subjectTo( AT_END, v == 0.0 )	;	//the end values for vs,

	//set inequalities constraints
	ocp.subjectTo( -0.1 <= v <= 1.7 )	;	//as well as the bounds on v
	ocp.subjectTo( -1.1 <= u <= 1.1 )	;	//as well as the bounds on control input u
	ocp.subjectTo(  5.0 <= T <= 15.0 )	;	//as well as the bounds on the time horizon T

	//--------------------------------------------------------------------------
	GnuplotWindow window				;	//visualize the results in a
	window.addSubplot( s, "DISTANCE s")	;	// Gnuplot window.
	window.addSubplot( v, "VELOCITY v")	;	// Gnuplot window.
	window.addSubplot( m, "MASS m")		;	// Gnuplot window.
	window.addSubplot( u, "CONTROL u")	;	// Gnuplot window.

	//--------------------------------------------------------------------------

	OptimizationAlgorithm algorithm(ocp);	//construct optimization algorithm,

	Grid timeGrid( 0.0, 1.0, 11	)		;

	VariablesGrid x_init( 3, timeGrid )	;
	VariablesGrid u_init( 1, timeGrid )	;
	VariablesGrid p_init( 1, timeGrid )	;

    x_init(0,0 ) = 0.00e+00; x_init(1,0 ) = 0.00e+00; x_init(2,0 ) = 1.00e+00;
    x_init(0,1 ) = 2.99e-01; x_init(1,1 ) = 7.90e-01; x_init(2,1 ) = 9.90e-01;
    x_init(0,2 ) = 1.13e+00; x_init(1,2 ) = 1.42e+00; x_init(2,2 ) = 9.81e-01;
    x_init(0,3 ) = 2.33e+00; x_init(1,3 ) = 1.69e+00; x_init(2,3 ) = 9.75e-01;
    x_init(0,4 ) = 3.60e+00; x_init(1,4 ) = 1.70e+00; x_init(2,4 ) = 9.73e-01;
    x_init(0,5 ) = 4.86e+00; x_init(1,5 ) = 1.70e+00; x_init(2,5 ) = 9.70e-01;
    x_init(0,6 ) = 6.13e+00; x_init(1,6 ) = 1.70e+00; x_init(2,6 ) = 9.68e-01;
    x_init(0,7 ) = 7.39e+00; x_init(1,7 ) = 1.70e+00; x_init(2,7 ) = 9.65e-01;
    x_init(0,8 ) = 8.66e+00; x_init(1,8 ) = 1.70e+00; x_init(2,8 ) = 9.63e-01;
    x_init(0,9 ) = 9.67e+00; x_init(1,9 ) = 8.98e-01; x_init(2,9 ) = 9.58e-01;
    x_init(0,10) = 1.00e+01; x_init(1,10) = 0.00e+00; x_init(2,10) = 9.49e-01;

    u_init(0,0 ) =  1.10e+00;
    u_init(0,1 ) =  1.10e+00;
    u_init(0,2 ) =  1.10e+00;
    u_init(0,3 ) =  5.78e-01;
    u_init(0,4 ) =  5.78e-01;
    u_init(0,5 ) =  5.78e-01;
    u_init(0,6 ) =  5.78e-01;
    u_init(0,7 ) =  5.78e-01;
    u_init(0,8 ) = -2.12e-01;
    u_init(0,9 ) = -1.10e+00;
    u_init(0,10) = -1.10e+00;

    p_init(0,0 ) =  7.44e+00;

	algorithm.initializeDifferentialStates( x_init )	;	//initialize x from text file
	algorithm.initializeControls( u_init )				;	//initialize u from text file
	algorithm.initializeParameters( p_init )			;	//initialize p from text file

	//algorithm.set( MAX_NUM_ITERATIONS, 20)					;	//set number of iteration as 20.0
	//algorithm.set( HESSIAN_APPROXIMATION, EXACT_HESSIAN )	;	//Approximation of hessian is nothing but exact
	//algorithm.set( HESSIAN_PROJECTION_FACTOR, 2.0)			;	//hessian factor 1.0

	algorithm	<<	window				;	//flush the plot window,
	algorithm.solve()					;	//and solve the problem.

return 0;
}



