/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2021 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See https://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 *****************************************************************************/

//! \example tutorial-franka-coppeliasim-cartesian-impedance-control.cpp

#include <iostream>
#include <string>
#include <visp3/gui/vpPlot.h>
#include <visp_ros/vpROSRobotFrankaCoppeliasim.h>
#include <visp_ros/vpROSRobot.h>

//#include <visp3/core/vpColVector.h>
//#include <cmath>

vpMatrix
Ta( const vpHomogeneousMatrix &edMe )
{
    vpMatrix Lx( 6, 6 ), Lw( 3, 3 ), skew_u( 3, 3 );

    Lx.eye();
    vpThetaUVector tu( edMe );

    vpColVector u;
    double theta;

    tu.extract( theta, u );
    skew_u = vpColVector::skew( u );
    Lw.eye();
    if ( theta != 0.0 )
    {
        Lw -= 0.5 * theta * skew_u;
        Lw += ( 1 - ( ( vpMath::sinc( theta ) ) / ( vpMath::sqr( vpMath::sinc( theta * 0.5 ) ) ) ) ) * skew_u * skew_u;
    }

    Lx.insert( Lw, 3, 3 );

    return Lx;
}

void MoveToInitial(vpROSRobotFrankaCoppeliasim& robot)
{
    // Move to a secure initial position
    vpColVector q_init( { 0, vpMath::rad( -45 ), 0, vpMath::rad( -135 ), 0, vpMath::rad( 90 ), vpMath::rad( 45 ) } );
    robot.setRobotState( vpRobot::STATE_POSITION_CONTROL );
    robot.setPosition( vpRobot::JOINT_STATE, q_init );
    vpTime::wait( 500 );
}

void LMoveToPoint(vpROSRobotFrankaCoppeliasim& robot, bool opt_verbose, bool opt_coppeliasim_sync_mode, vpPlot* plotter, vpColVector desired_pos)
{
    plotter = new vpPlot( 4, 800, 800, 10, 10, "Real time curves plotter" );
    plotter->setTitle( 0, "Joint positions [rad]" );
    plotter->initGraph( 0, 7 );
    plotter->setLegend( 0, 0, "q1" );
    plotter->setLegend( 0, 1, "q2" );
    plotter->setLegend( 0, 2, "q3" );
    plotter->setLegend( 0, 3, "q4" );
    plotter->setLegend( 0, 4, "q5" );
    plotter->setLegend( 0, 5, "q6" );
    plotter->setLegend( 0, 6, "q7" );

    plotter->setTitle( 1, "Joint torques measure [Nm]" );
    plotter->initGraph( 1, 7 );
    plotter->setLegend( 1, 0, "Tau1" );
    plotter->setLegend( 1, 1, "Tau2" );
    plotter->setLegend( 1, 2, "Tau3" );
    plotter->setLegend( 1, 3, "Tau4" );
    plotter->setLegend( 1, 4, "Tau5" );
    plotter->setLegend( 1, 5, "Tau6" );
    plotter->setLegend( 1, 6, "Tau7" );

    plotter->setTitle( 2, "Cartesian EE pose error [m] - [rad]" );
    plotter->initGraph( 2, 6 );
    plotter->setLegend( 2, 0, "e_x" );
    plotter->setLegend( 2, 1, "e_y" );
    plotter->setLegend( 2, 2, "e_z" );
    plotter->setLegend( 2, 3, "e_tu_x" );
    plotter->setLegend( 2, 4, "e_tu_y" );
    plotter->setLegend( 2, 5, "e_tu_z" );

    plotter->setTitle( 3, "Pose error norm [m] - [rad]" );
    plotter->initGraph( 3, 2 );
    plotter->setLegend( 3, 0, "||e_p||" );
    plotter->setLegend( 3, 1, "||e_o||" );

    vpColVector q( 7, 0 ), dq( 7, 0 ), tau_d( 7, 0 ), C( 7, 0 ), F( 7, 0 ), tau_d0( 7, 0 ), tau_cmd( 7, 0 ),
            x_e( 6, 0 ), dx_e( 6, 0 ), dx_ed( 6, 0 ), ddx_ed( 6, 0 );
    vpMatrix fJe( 6, 7 ), Ja( 6, 7 ), dJa( 6, 7 ), Ja_old( 6, 7 ), B( 7, 7 ), I7( 7, 7 ), Ja_pinv_B_t( 6, 7 );
    vpColVector pose_err_norm( 2, 0 ), tau( 7, 0 );

    robot.getPosition( vpRobot::JOINT_STATE, q );
    robot.setRobotState( vpRobot::STATE_FORCE_TORQUE_CONTROL );
    robot.setCoppeliasimSyncMode( opt_coppeliasim_sync_mode );

    vpHomogeneousMatrix fMed, fMed0, current_fMe;
    fMed0 = robot.get_fMe();
    fMed  = fMed0;
    std::cout << "Begin position: " << fMed0[0][3] << "," << fMed0[1][3] << "," << fMed0[2][3] << std::endl;

    // Error: distance between initial and desired position
    vpColVector error({desired_pos[0] - fMed[0][3], desired_pos[1] - fMed[1][3], desired_pos[2] - fMed[2][3]});

    bool final_quit       = false;
    bool first_time       = false;
    bool start_trajectory = false;

    vpMatrix K( 6, 6 ), D( 6, 6 ), edVf( 6, 6 );

    double wp = 50;
    double wo = 20;
    K.diag( { wp * wp, wp * wp, wp * wp, wo * wo, wo * wo, wo * wo } );
    D.diag( { 2 * wp, 2 * wp, 2 * wp, 2 * wo, 2 * wo, 2 * wo } );
    I7.eye();

    double mu = 4;
    double dt = 0;

    double time_start_trajectory, time_prev, time_cur, time_final;
    double delay_before_trajectory = 0.2;

    // Vmax has influence on the accuracy of the robotarm, value between 0.1 and 0.5.
    double vmax = 0.1;

    time_final = (time_cur - time_start_trajectory) + sqrt(pow((desired_pos[0] - error[0]), 2) + pow((desired_pos[1] - error[1]), 2) + pow((desired_pos[2] - error[2]), 2)) / vmax;

    // Control loop
    while ( !final_quit ) {
        time_cur = robot.getCoppeliasimSimulationTime();

        robot.getPosition(vpRobot::JOINT_STATE, q);
        robot.getVelocity(vpRobot::JOINT_STATE, dq);
        robot.getMass(B);
        robot.getCoriolis(C);
        robot.getFriction(F);
        robot.get_fJe(fJe);
        robot.getForceTorque(vpRobot::JOINT_STATE, tau);

        if (time_cur < delay_before_trajectory) {
            time_start_trajectory = time_cur;
            first_time = true;
        } else if (!start_trajectory) {
            time_start_trajectory = time_cur;
            start_trajectory = true;
        }

        // Compute Cartesian trajectories
        current_fMe = robot.get_fMe();
        vpColVector current_error({desired_pos[0] - current_fMe[0][3], desired_pos[1] - current_fMe[1][3],
                                   desired_pos[2] - current_fMe[2][3]});

        double error_norm = sqrt(pow(current_error[0], 2) + pow(current_error[1], 2) + pow(current_error[2], 2));

        // If error norm > sqrt( 0.001^2 + 0.001^2 + 0.001^2) 1 mm
        if (error_norm > 1.7e-3)
        {
            for (int i = 0; i <= 2; i++)
            {
                fMed[i][3] = fMed0[i][3] + (start_trajectory ? (desired_pos[i] - fMed0[i][3]) * (time_cur - time_start_trajectory): 0); // position
                dx_ed[i] = (start_trajectory ? (desired_pos[i] - fMed0[i][3]) / (time_final - (time_cur - time_start_trajectory)) : 0); // velocity
                ddx_ed[i] = (start_trajectory ? 0 : 0); // acceleration
            }
        }
        else
        {
            final_quit = true;
            std::cout << "Final position: " << fMed[0][3] << ";" << fMed[1][3] << ";" << fMed[2][3] << std::endl;
            std::cout << "Desired position: " << desired_pos[0] << ";" << desired_pos[1] << ";" << desired_pos[2] << std::endl;
            std::cout << "Reached point!" << std::endl;
        }

        edVf.insert(fMed.getRotationMatrix().t(), 0, 0);
        edVf.insert(fMed.getRotationMatrix().t(), 3, 3);

        x_e = (vpColVector) vpPoseVector(fMed.inverse() * robot.get_fMe());

        Ja = Ta(fMed.inverse() * robot.get_fMe()) * edVf * fJe;

        dx_e = Ta(fMed.inverse() * robot.get_fMe()) * edVf * (dx_ed - fJe * dq);

        dt = time_cur - time_prev;

        if (dt != 0) {
            dJa = (Ja - Ja_old) / dt;
        } else {
            dJa = 0;
        }
        Ja_old = Ja;

        Ja_pinv_B_t = (Ja * B.inverseByCholesky() * Ja.t()).inverseByCholesky() * Ja * B.inverseByCholesky();

        // Compute the control law
        tau_d = B * Ja.pseudoInverse() * (-K * (x_e) + D * (dx_e) - dJa * dq + ddx_ed) + C + F -
                (I7 - Ja.t() * Ja_pinv_B_t) * B * dq * 100;

        if (first_time) {
            tau_d0 = tau_d;
        }

        tau_cmd = tau_d - tau_d0 * std::exp(-mu * (time_cur - time_start_trajectory));

        robot.setForceTorque(vpRobot::JOINT_STATE, tau_cmd);

        plotter->plot(0, time_cur, q);
        plotter->plot(1, time_cur, tau);
        plotter->plot(2, time_cur, x_e);
        pose_err_norm[0] = sqrt(x_e.extract(0, 3).sumSquare());
        pose_err_norm[1] = sqrt(x_e.extract(3, 3).sumSquare());
        plotter->plot(3, time_cur, pose_err_norm);

        vpMouseButton::vpMouseButtonType button;
        if (vpDisplay::getClick(plotter->I, button, false)) {
            if (button == vpMouseButton::button3) {
                final_quit = true;
                tau_cmd = 0;
                std::cout << "Stop the robot " << std::endl;
                robot.setRobotState(vpRobot::STATE_STOP);
            }
        }

        if (opt_verbose) {
            std::cout << "dt: " << dt << std::endl;
        }

        time_prev = time_cur;
        robot.wait(time_cur, 0.001);
    }
}

void ArcMoveToPoint(vpROSRobotFrankaCoppeliasim& robot, bool opt_verbose, bool opt_coppeliasim_sync_mode, vpColVector desired_pos)
{
    std::cout << "Move in an arch" << std::endl;

    vpColVector q( 7, 0 ), dq( 7, 0 ), tau_d( 7, 0 ), C( 7, 0 ), F( 7, 0 ), tau_d0( 7, 0 ), tau_cmd( 7, 0 ),
            x_e( 6, 0 ), dx_e( 6, 0 ), dx_ed( 6, 0 ), ddx_ed( 6, 0 );
    vpMatrix fJe( 6, 7 ), Ja( 6, 7 ), dJa( 6, 7 ), Ja_old( 6, 7 ), B( 7, 7 ), I7( 7, 7 ), Ja_pinv_B_t( 6, 7 );
    vpColVector pose_err_norm( 2, 0 ), tau( 7, 0 );

    robot.getPosition( vpRobot::JOINT_STATE, q );
    robot.setRobotState( vpRobot::STATE_FORCE_TORQUE_CONTROL );
    robot.setCoppeliasimSyncMode( opt_coppeliasim_sync_mode );

    vpHomogeneousMatrix fMed, fMed0, current_fMe;
    fMed0 = robot.get_fMe();
    fMed  = fMed0;
    std::cout << "Begin position: " << fMed0[0][3] << "," << fMed0[1][3] << "," << fMed0[2][3] << std::endl;

    // Error: distance between initial and desired position
    vpColVector error({desired_pos[0] - fMed[0][3], desired_pos[1] - fMed[1][3], desired_pos[2] - fMed[2][3]});

    bool final_quit       = false;
    bool first_time       = false;
    bool start_trajectory = false;

    vpMatrix K( 6, 6 ), D( 6, 6 ), edVf( 6, 6 );

    double wp = 50;
    double wo = 20;
    K.diag( { wp * wp, wp * wp, wp * wp, wo * wo, wo * wo, wo * wo } );
    D.diag( { 2 * wp, 2 * wp, 2 * wp, 2 * wo, 2 * wo, 2 * wo } );
    I7.eye();

    double mu = 4;
    double dt = 0;

    double time_start_trajectory, time_prev, time_cur, time_final;
    double delay_before_trajectory = 0.2;

    // Diameter of y,z circle and radius of x circle
    double d = sqrt(pow(desired_pos[1]-fMed[1][3],2) + pow(desired_pos[2]-fMed[2][3],2)); // pow(desired_pos[0]-fMed[0][3],2) +
    std::cout << "The diameter: " << d << std::endl;
    double r = sqrt(pow(desired_pos[0]-fMed[0][3],2))/2;
    std::cout << "The radius of x: " << r << std::endl;

    double center_x = (fMed[0][3] + desired_pos[0])/2;
    double center_y = (fMed[1][3] + desired_pos[1])/2;
    double center_z = (fMed[2][3] + desired_pos[2])/2;

    std::cout << "Center x: " << center_x << std::endl;
    std::cout << "Center y: " << center_y << std::endl;
    std::cout << "Center z: " << center_z << std::endl;

    while ( !final_quit ) {
        time_cur = robot.getCoppeliasimSimulationTime();

        robot.getPosition(vpRobot::JOINT_STATE, q);
        robot.getVelocity(vpRobot::JOINT_STATE, dq);
        robot.getMass(B);
        robot.getCoriolis(C);
        robot.getFriction(F);
        robot.get_fJe(fJe);
        robot.getForceTorque(vpRobot::JOINT_STATE, tau);

        if (time_cur < delay_before_trajectory) {
            time_start_trajectory = time_cur;
            first_time = true;
        } else if (!start_trajectory) {
            time_start_trajectory = time_cur;
            start_trajectory = true;
        }

        // Compute Cartesian trajectories
        current_fMe = robot.get_fMe();
        vpColVector current_error({desired_pos[0] - current_fMe[0][3], desired_pos[1] - current_fMe[1][3],
                                   desired_pos[2] - current_fMe[2][3]});


        double error_norm = sqrt(pow(current_error[0], 2) + pow(current_error[1], 2) + pow(current_error[2], 2));

        if (error_norm > 2e-2)
        {
            std::cout << "Error norm: " << error_norm << std::endl;

        // ------------------------------------------------------------------------------------------------------------
//         2D / 3D
            fMed[0][3] = center_x + (start_trajectory ? (r * cos(M_PI * (time_cur - time_start_trajectory))) : 0); // position y
            fMed[1][3] = center_y + (start_trajectory ? (d/2 * cos(M_PI * (time_cur - time_start_trajectory))) : 0); // position y
            fMed[2][3] = center_z + (start_trajectory ? (d/2 * sin(M_PI * (time_cur - time_start_trajectory))) : 0);  // position z

            dx_ed[0] = (start_trajectory ? (r * cos(M_PI * (time_cur - time_start_trajectory)) * M_PI) : 0); // velocity z
            dx_ed[1] = (start_trajectory ? (-d/2 * sin(M_PI * (time_cur - time_start_trajectory)) * M_PI) : 0); // velocity y
            dx_ed[2] = (start_trajectory ? (d/2 * cos(M_PI * (time_cur - time_start_trajectory)) * M_PI) : 0); // velocity z

            ddx_ed[0] = (start_trajectory ? (r * sin(M_PI * (time_cur - time_start_trajectory)) * M_PI * M_PI) : 0); // acceleration y
            ddx_ed[1] = (start_trajectory ? (-d/2 * cos(M_PI * (time_cur - time_start_trajectory)) * M_PI * M_PI) : 0); // acceleration y
            ddx_ed[2] = (start_trajectory ? (-d/2 * sin(M_PI * (time_cur - time_start_trajectory)) * M_PI * M_PI) : 0); // acceleration z

        // 3D
//            fMed[0][3] = center_x + (start_trajectory ? (d/2 * sin(M_PI * (time_cur - time_start_trajectory))) : 0); // position x
//            fMed[1][3] = center_y + (start_trajectory ? (d/2 * cos(M_PI * (time_cur - time_start_trajectory)) * cos(M_PI * (time_cur - time_start_trajectory))) : 0); // position y
//            fMed[2][3] = center_z + (start_trajectory ? (d/2 * sin(M_PI * (time_cur - time_start_trajectory)) * cos(M_PI * (time_cur - time_start_trajectory))) : 0);  // position z
//
//            dx_ed[0] = (start_trajectory ? (d/2 * M_PI * cos(M_PI * (time_cur - time_start_trajectory))) : 0); // velocity x
//            dx_ed[1] = (start_trajectory ? (-d/2 * M_PI * sin(M_PI * (time_cur - time_start_trajectory)) * cos(M_PI * (time_cur - time_start_trajectory)) + d/2 * M_PI * cos(M_PI * (time_cur - time_start_trajectory)) * sin(M_PI * (time_cur -time_start_trajectory))) : 0); // velocity y
//            dx_ed[2] = (start_trajectory ? (-d/2 * M_PI * sin(M_PI * (time_cur - time_start_trajectory)) * sin(M_PI * (time_cur - time_start_trajectory)) - d/2 * M_PI * cos(M_PI * (time_cur - time_start_trajectory)) * cos(M_PI * (time_cur - time_start_trajectory)))  : 0); // velocity z
//
//            ddx_ed[0] = (start_trajectory ? (-d/2 * M_PI * sin(M_PI * (time_cur - time_start_trajectory))) : 0); // acceleration x
//            ddx_ed[1] = (start_trajectory ? (-d/2 * M_PI * M_PI * cos(M_PI * (time_cur - time_start_trajectory)) * cos(M_PI * (time_cur - time_start_trajectory)) - d/2 * M_PI * M_PI * sin(M_PI * (time_cur - time_start_trajectory)) * cos(M_PI * (time_cur - time_start_trajectory)) - 2 * d/2 * M_PI * M_PI * sin(M_PI * (time_cur-time_start_trajectory)) * sin(M_PI * (time_cur-time_start_trajectory))) : 0); // acceleration y
//            ddx_ed[2] = (start_trajectory ? (-d/2 * M_PI * M_PI * cos(M_PI * (time_cur - time_start_trajectory)) * sin(M_PI * (time_cur - time_start_trajectory)) + d/2 * M_PI * M_PI * sin(M_PI * (time_cur - time_start_trajectory)) * sin(M_PI * (time_cur - time_start_trajectory)) + 2 * d/2 * M_PI * M_PI * sin(M_PI * (time_cur-time_start_trajectory)) * cos(M_PI * (time_cur-time_start_trajectory))) : 0); // acceleration z
        }
        else
        {
            final_quit = true;
            std::cout << "Final position: " << fMed[0][3] << ";" << fMed[1][3] << ";" << fMed[2][3] << std::endl;
            std::cout << "Desired position: " << desired_pos[0] << ";" << desired_pos[1] << ";" << desired_pos[2] << std::endl;
            std::cout << "Reached point!" << std::endl;
        }
        edVf.insert(fMed.getRotationMatrix().t(), 0, 0);
        edVf.insert(fMed.getRotationMatrix().t(), 3, 3);

        x_e = (vpColVector) vpPoseVector(fMed.inverse() * robot.get_fMe()); // edMe
//            std::cout << "X_e: " << x_e << std::endl;

        Ja = Ta(fMed.inverse() * robot.get_fMe()) * edVf * fJe;

        dx_e = Ta(fMed.inverse() * robot.get_fMe()) * edVf * (dx_ed - fJe * dq);

        dt = time_cur - time_prev;

        if (dt != 0) {
            dJa = (Ja - Ja_old) / dt;
        } else {
            dJa = 0;
        }
        Ja_old = Ja;

        Ja_pinv_B_t = (Ja * B.inverseByCholesky() * Ja.t()).inverseByCholesky() * Ja * B.inverseByCholesky();

        // Compute the control law
        tau_d = B * Ja.pseudoInverse() * (-K * (x_e) + D * (dx_e) - dJa * dq + ddx_ed) + C + F -
                (I7 - Ja.t() * Ja_pinv_B_t) * B * dq * 100;

        if (first_time) {
            tau_d0 = tau_d;
        }

        tau_cmd = tau_d - tau_d0 * std::exp(-mu * (time_cur - time_start_trajectory));

        robot.setForceTorque(vpRobot::JOINT_STATE, tau_cmd);

        if (opt_verbose) {
            std::cout << "dt: " << dt << std::endl;
        }

        time_prev = time_cur;
        robot.wait(time_cur, 0.001);
    }
}




// MAIN ---------------------------------------------------------------------------------------------------------------
int
main( int argc, char **argv )
{
    bool opt_coppeliasim_sync_mode = false;
    bool opt_verbose               = false;
    bool opt_save_data             = false;

    for ( int i = 1; i < argc; i++ )
    {
        if ( std::string( argv[i] ) == "--enable-coppeliasim-sync-mode" )
        {
            opt_coppeliasim_sync_mode = true;
        }
        else if ( std::string( argv[i] ) == "--verbose" || std::string( argv[i] ) == "-v" )
        {
            opt_verbose = true;
        }
        else if ( std::string( argv[i] ) == "--save" )
        {
            opt_save_data = true;
        }
        else if ( std::string( argv[i] ) == "--help" || std::string( argv[i] ) == "-h" )
        {
            std::cout << argv[0] << "[--enable-coppeliasim-sync-mode]"
                      << " [--save]"
                      << " [--verbose] [-v] "
                      << " [--help] [-h]" << std::endl;
            return EXIT_SUCCESS;
        }
    }

    vpROSRobotFrankaCoppeliasim robot;

    try
    {
        //------------------------------------------------------------------------//
        //------------------------------------------------------------------------//
        // ROS node
        ros::init( argc, argv, "visp_ros" );
        ros::NodeHandlePtr n = boost::make_shared< ros::NodeHandle >();
        ros::Rate loop_rate( 1000 );
        ros::spinOnce();

        robot.setVerbose( opt_verbose );
        robot.connect();

        std::cout << "Coppeliasim sync mode enabled: " << ( opt_coppeliasim_sync_mode ? "yes" : "no" ) << std::endl;
        robot.coppeliasimStopSimulation(); // Allows to reset simulation, moving the robot to initial position
        robot.setCoppeliasimSyncMode( false );
        robot.coppeliasimStartSimulation();

        // Define plot
        vpPlot *plotter = nullptr;

        // Desired location end-effector
        vpColVector desired_pos; // x, y, z of end-effector

        // Switch case ------------------------------------------------------------------------------------------------
        int movement = 0;
        switch (movement)
        {
            case 0:
                std::cout << " 1) Move to initial position" << std::endl;
                MoveToInitial(robot);
                movement = 1;
            case 1:
                std::cout << " 2) Move to point 1" << std::endl;
                desired_pos = {0.4, 0.2, 0.3};
                LMoveToPoint(robot, opt_verbose, opt_coppeliasim_sync_mode, nullptr, desired_pos);
                movement = 2;
            case 2:
                std::cout << " 3) Grab package..." << std::endl;
                movement = 3;
            case 3:
                std::cout << " 4) Move to point 2" << std::endl;
                desired_pos = {0.2, -0.2, 0.3};
                ArcMoveToPoint(robot, opt_verbose, opt_coppeliasim_sync_mode, desired_pos);
                movement = 4;
            case 4:
                std::cout << " 2) Move to point 1" << std::endl;
                desired_pos = {0.4, 0.0, 0.5};
                LMoveToPoint(robot, opt_verbose, opt_coppeliasim_sync_mode, nullptr, desired_pos);
                break;
            default:
                std::cout << "Invalid option" << std::endl;
        }

        if ( opt_save_data )
        {
            plotter->saveData( 0, "sim-cart-joint-position.txt", "# " );
            plotter->saveData( 1, "sim-cart-joint-torques.txt", "# " );
            plotter->saveData( 2, "sim-cart-pose-error.txt", "# " );
            plotter->saveData( 3, "sim-cart-pose-error-norm.txt", "# " );
        }

        if ( plotter != nullptr )
        {
            delete plotter;
            plotter = nullptr;
        }
        vpTime::wait(1000);
        robot.coppeliasimStopSimulation();
    }
    catch ( const vpException &e )
    {
        std::cout << "ViSP exception: " << e.what() << std::endl;
        std::cout << "Stop the robot " << std::endl;
        robot.setRobotState( vpRobot::STATE_STOP );
        return EXIT_FAILURE;
    }

    return 0;
}


