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
	ros::init(argc, argv,"acado_tut_semi_implicit_dae");
	ros::NodeHandle nh;

	using namespace ACADO;

	DifferentialState		v, s, m		;
	Control					u			;	//	the control action

    //OTHER USEFUL CONSTANTS:
    const double 	t_start  = 	0.0		;
    const double 	t_end	 =	10.0    ;
    const double 	h		 =	0.01    ;

    DiscretizedDifferentialEquation f(h);

    //DEFINE A DIFFERENTIAL EQUATION:
    //----------------------------------
    f << next(s) == s + h*v				;
    f << next(v) == v + h*(u-0.02*v*v)/m;
    f << next(m) == m - h*0.01*u*u		;

    // DEFINE AN OPTIMAL CONTROL PROBLEM:
    // ----------------------------------
    OCP ocp( t_start, t_end, 50 )		;
    ocp.minimizeLagrangeTerm( u*u )		;

    ocp.subjectTo( f );

    ocp.subjectTo( AT_START, s == 0.0 )	;
    ocp.subjectTo( AT_START, v == 0.0 )	;
    ocp.subjectTo( AT_START, m == 1.0 )	;

    ocp.subjectTo( AT_END, s == 10.0 )	;
    ocp.subjectTo( AT_END, v == 0.0 )	;

    ocp.subjectTo( -0.01 <= v <= 1.3)	;

    //DEFINE A PLOT WINDOW:
    //------------------------------------
    GnuplotWindow window;
    window.addSubplot( s,"DifferentialState s" );
    window.addSubplot( v,"DifferentialState v" );
    window.addSubplot( m,"DifferentialState m" );
    window.addSubplot( u,"Control u" );
    window.addSubplot( PLOT_KKT_TOLERANCE,"KKT Tolerance" );
    window.addSubplot( 0.5 * m * v*v,"Kinetic Energy" );

    // DEFINE AN OPTIMIZATION ALGORITHM AND SOLVE THE OCP:
    // ----------------------------------------------------
    OptimizationAlgorithm algorithm(ocp);

    /*
    algorithm.set( ABSOLUTE_TOLERANCE    , 1e-7          );
    algorithm.set( INTEGRATOR_TOLERANCE  , 1e-7          );
    algorithm.set( HESSIAN_APPROXIMATION , EXACT_HESSIAN );
    */

    algorithm.set( HESSIAN_APPROXIMATION, EXACT_HESSIAN );
    algorithm.set( KKT_TOLERANCE        , 1e-10         );

    algorithm << window;
    algorithm.solve();

return 0;
}



