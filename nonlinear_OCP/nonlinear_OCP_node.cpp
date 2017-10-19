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
	ros::init(argc, argv,"acado_tut_fight_rocket");
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

	//read data from filess
	algorithm.initializeDifferentialStates( "/home/bfb-ws/mpc_ws/src/acado_learing_tutorial/nonlinear_OCP/x.txt");	//initialize x from text file
	algorithm.initializeControls( "/home/bfb-ws/mpc_ws/src/acado_learing_tutorial/nonlinear_OCP/u.txt");	//initialize u from text file
	algorithm.initializeParameters( "/home/bfb-ws/mpc_ws/src/acado_learing_tutorial/nonlinear_OCP/p.txt");	//initialize p from text file

	//algorithm.set( MAX_NUM_ITERATIONS, 20)					;	//set number of iteration as 20.0
	//algorithm.set( HESSIAN_APPROXIMATION, EXACT_HESSIAN )	;	//Approximation of hessian is nothing but exact
	//algorithm.set( HESSIAN_PROJECTION_FACTOR, 2.0)			;	//hessian factor 1.0

	algorithm	<<	window				;	//flush the plot window,
	algorithm.solve()					;	//and solve the problem.

	//store output to files
	//algorithm.getDifferentialStates( "/home/bfb-ws/mpc_ws/src/acado_learing_tutorial/nonlinear_OCP/states.txt" )	;
	//algorithm.getParameters( "/home/bfb-ws/mpc_ws/src/acado_learing_tutorial/nonlinear_OCP/parameters.txt" )		;
	//algorithm.getControls( "/home/bfb-ws/mpc_ws/src/acado_learing_tutorial/nonlinear_OCP/controls.txt" )			;

	//print out-put on terminal
	VariablesGrid states, paramters, controls	;
	algorithm.getDifferentialStates( states )	;
	algorithm.getParameters( paramters )		;
	algorithm.getControls( controls )			;

	states.print()		;
	paramters.print()	;
	controls.print()	;

return 0;
}



