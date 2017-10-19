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

	DifferentialState		x			;	// the position of the "hand"
	DifferentialState		l			;
	AlgebraicState			z			;
	Control					u			;	//	the control action
	DifferentialEquation	f			;

    //OTHER USEFUL CONSTANTS:
    const double 	t_start  = 	0.0		;
    const double 	t_end	 =	10.0    ;


    //DEFINE A DIFFERENTIAL EQUATION:
    //----------------------------------
    f << dot(x) == -x + 0.5*x*x + u + 0.5*z	;
    f << dot(l) == x*x + 3.0*u*u			;
    f << 	 0  == z + exp(z) - 1.0 + x		;

    // DEFINE AN OPTIMAL CONTROL PROBLEM:
    // ----------------------------------
    OCP ocp( t_start, t_end, 10 );
    ocp.minimizeMayerTerm( l );

    ocp.subjectTo( f );
    ocp.subjectTo( AT_START, x == 1.0 );
    ocp.subjectTo( AT_START, l == 0.0 );

    GnuplotWindow window;
    window.addSubplot(x,"DIFFERENTIAL STATE  x");
    window.addSubplot(z,"ALGEBRAIC STATE  z"   );
    window.addSubplot(u,"CONTROL  u"   );

    // DEFINE AN OPTIMIZATION ALGORITHM AND SOLVE THE OCP:
    // ----------------------------------------------------
    OptimizationAlgorithm algorithm(ocp);

    algorithm.set( ABSOLUTE_TOLERANCE    , 1e-7          );
    algorithm.set( INTEGRATOR_TOLERANCE  , 1e-7          );
    algorithm.set( HESSIAN_APPROXIMATION , EXACT_HESSIAN );

    algorithm << window;
    algorithm.solve();

return 0;
}



