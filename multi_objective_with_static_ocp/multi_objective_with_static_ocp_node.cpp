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

    // INTRODUCE THE VARIABLES:
    // -------------------------
    Parameter y1,y2;


    // DEFINE AN OPTIMIZATION PROBLEM:
    // -------------------------------
    NLP nlp;
    nlp.minimize( 0, y1 );
    nlp.minimize( 1, y2 );

    nlp.subjectTo( 0.0 <= y1 <= 5.0 );
    nlp.subjectTo( 0.0 <= y2 <= 5.2 );
    nlp.subjectTo( 0.0 <= y2 - 5.0*exp(-y1) - 2.0*exp(-0.5*(y1-3.0)*(y1-3.0)) );
    //nlp.subjectTo( y1*y1 + y2*y2 <= 2.0 );

    // DEFINE A MULTI-OBJECTIVE ALGORITHM AND SOLVE THE NLP:
    // -----------------------------------------------------
    MultiObjectiveAlgorithm algorithm(nlp);

    algorithm.set( PARETO_FRONT_GENERATION, PFG_NORMAL_BOUNDARY_INTERSECTION );
    // algorithm.set(PARETO_FRONT_GENERATION,PFG_WEIGHTED_SUM);
    // algorithm.set(PARETO_FRONT_GENERATION,PFG_NORMALIZED_NORMAL_CONSTRAINT);
    algorithm.set( PARETO_FRONT_DISCRETIZATION, 41 );
    algorithm.set( KKT_TOLERANCE, 1e-12 );

    // Minimize individual objective function
    algorithm.initializeParameters("/home/bfb-ws/mpc_ws/src/acado_learing_tutorial/multi_objective_with_static_ocp/initial_scalar2_2.txt");
    algorithm.solveSingleObjective(1);

    // Minimize individual objective function
    algorithm.solveSingleObjective(0);

    // Generate Pareto set
    algorithm.solve();


    // GET THE RESULT FOR THE PARETO FRONT AND PLOT IT:
    // ------------------------------------------------
    VariablesGrid paretoFront;
    algorithm.getParetoFront( paretoFront );
    algorithm.getWeights("/home/bfb-ws/mpc_ws/src/acado_learing_tutorial/multi_objective_with_static_ocp/scalar2_nbi_weights.txt");

    GnuplotWindow window1;
    window1.addSubplot( paretoFront, "Pareto Front y1 vs y2", "y1","y2", PM_POINTS );
    window1.plot( );

    paretoFront.print();


    // FILTER THE PARETO FRONT AND PLOT IT:
    // ------------------------------------
    algorithm.getParetoFrontWithFilter( paretoFront );
    algorithm.getWeightsWithFilter("/home/bfb-ws/mpc_ws/src/acado_learing_tutorial/multi_objective_with_static_ocp/scalar2_nbi_weights_filtered.txt");

    GnuplotWindow window2;
    window2.addSubplot( paretoFront, "Pareto Front (with filter) y1 vs y2", "y1","y2", PM_POINTS );
    window2.plot( );

    paretoFront.print();


    // PRINT INFORMATION ABOUT THE ALGORITHM:
    // --------------------------------------
    algorithm.printInfo();

    return 0;
}



