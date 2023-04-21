#include <iostream>
#include <visp3/core/vpColVector.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp_ros/vpROSRobotFrankaCoppeliasim.h>
#include <cmath>

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
#include <visp3/gui/vpPlot.h>
#include <visp_ros/vpROSRobotFrankaCoppeliasim.h>

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

        // Move to a secure initial position
        vpColVector q_init( { 0, vpMath::rad( -45 ), 0, vpMath::rad( -135 ), 0, vpMath::rad( 90 ), vpMath::rad( 45 ) } );

        robot.setRobotState( vpRobot::STATE_POSITION_CONTROL );
        robot.setPosition( vpRobot::JOINT_STATE, q_init );
        vpTime::wait( 500 );

        vpPlot *plotter = nullptr;

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

        std::cout << "Reading current joint position" << std::endl;
        robot.getPosition( vpRobot::JOINT_STATE, q );
        std::cout << "Initial joint position: " << q.t() << std::endl;

        robot.setRobotState( vpRobot::STATE_FORCE_TORQUE_CONTROL );
        robot.setCoppeliasimSyncMode( opt_coppeliasim_sync_mode );

        vpHomogeneousMatrix fMed, fMed0, current_fMe;
        fMed0 = robot.get_fMe();
        fMed  = fMed0;
        std::cout << "Begin position!" << fMed0[0][3] << fMed0[1][3] << fMed0[2][3] << std::endl;

        // Desired location end-effector
        vpColVector desired_pos({fMed[0][3], fMed[1][3], fMed[2][3] + 0.05}); // x, y, z of end-effector

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
        double delay_before_trajectory = 0.5; // Start sinusoidal joint trajectory after this delay in [s]

        current_fMe = robot.get_fMe();
        vpColVector error({desired_pos[0] - current_fMe[0][3], desired_pos[1] - current_fMe[1][3], desired_pos[2] - current_fMe[2][3]});
        std::cout << "Error x,y,z : " << error << std::endl;

        time_final = (time_cur - time_start_trajectory) + sqrt(pow((desired_pos[0] - error[0]), 2) + pow((desired_pos[1] - error[1]), 2) + pow((desired_pos[2] - error[2]), 2)) / 1;
        std::cout << "Final time: " << time_final << std::endl;

        // Control loop
        while ( !final_quit )
        {
            time_cur = robot.getCoppeliasimSimulationTime();

            robot.getPosition( vpRobot::JOINT_STATE, q );
            robot.getVelocity( vpRobot::JOINT_STATE, dq );
            robot.getMass( B );
            robot.getCoriolis( C );
            robot.getFriction( F );
            robot.get_fJe( fJe );
            robot.getForceTorque( vpRobot::JOINT_STATE, tau );

            if ( time_cur < delay_before_trajectory )
            {
                time_start_trajectory = time_cur; // To ensure exp() = 1
                first_time            = true;
            }
            else if ( !start_trajectory ) // After the delay we start joint trajectory
            {
                time_start_trajectory = time_cur;
                start_trajectory      = true;
            }

            // Compute Cartesian trajectories
            current_fMe = robot.get_fMe();
            vpColVector error({desired_pos[0] - current_fMe[0][3], desired_pos[1] - current_fMe[1][3], desired_pos[2] - current_fMe[2][3]});

            // Use norm() to simplify if statements and adjust the trajectory
            if (sqrt(pow(error[0],2) + pow(error[1],2) + pow(error[2],2)) > 1.7e-3)
            {
                std::cout << "Error: " << sqrt(pow(error[0],2) + pow(error[1],2) + pow(error[2],2)) << std::endl;
                for (int i = 0; i <= 2; i++)
                {
                    fMed[i][3] = fMed0[i][3] + (start_trajectory ? (desired_pos[i] - fMed0[i][3]) * (time_cur - time_start_trajectory)  : 0); // position
//                    std::cout << "Position " << i << " : " << fMed[i][3] << std::endl;
                    dx_ed[i] = ( start_trajectory ? (desired_pos[i] - fMed0[i][3]) / (time_final - (time_cur - time_start_trajectory)) : 0) ; // velocity
//                    std::cout << "Velocity " << i << " : " << dx_ed[i] << std::endl;
                    ddx_ed[i] = ( start_trajectory ? 0 : 0) ; // acceleration
                }
            }
            else
            {
                final_quit = true;
                std::cout << "Final position: " << fMed[0][3] << ";" << fMed[1][3] << ";" << fMed[2][3] << std::endl;
                std::cout << "Difference in position: " << fMed0[0][3] - fMed[0][3] << ";" << fMed0[1][3] - fMed[1][3] << ";" << fMed0[2][3] - fMed[2][3] << std::endl;

                std::cout << "Reached point!" << std::endl;
            }

            edVf.insert( fMed.getRotationMatrix().t(), 0, 0 );
            edVf.insert( fMed.getRotationMatrix().t(), 3, 3 );

            x_e = (vpColVector)vpPoseVector( fMed.inverse() * robot.get_fMe() ); // edMe

            Ja  = Ta( fMed.inverse() * robot.get_fMe() ) * edVf * fJe;

            dx_e = Ta( fMed.inverse() * robot.get_fMe() ) * edVf * ( dx_ed - fJe * dq );

            dt = time_cur - time_prev;

            if ( dt != 0 )
            {
                dJa = ( Ja - Ja_old ) / dt;
            }
            else
            {
                dJa = 0;
            }
            Ja_old = Ja;

            Ja_pinv_B_t = ( Ja * B.inverseByCholesky() * Ja.t() ).inverseByCholesky() * Ja * B.inverseByCholesky();

            // Compute the control law
            tau_d = B * Ja.pseudoInverse() * ( -K * ( x_e ) + D * (dx_e)-dJa * dq + ddx_ed ) + C + F -
                    ( I7 - Ja.t() * Ja_pinv_B_t ) * B * dq * 100;

            if ( first_time )
            {
                tau_d0 = tau_d;
            }

            tau_cmd = tau_d - tau_d0 * std::exp( -mu * ( time_cur - time_start_trajectory ) );

            robot.setForceTorque( vpRobot::JOINT_STATE, tau_cmd );

            plotter->plot( 0, time_cur, q );
            plotter->plot( 1, time_cur, tau );
            plotter->plot( 2, time_cur, x_e );
            pose_err_norm[0] = sqrt( x_e.extract( 0, 3 ).sumSquare() );
            pose_err_norm[1] = sqrt( x_e.extract( 3, 3 ).sumSquare() );
            plotter->plot( 3, time_cur, pose_err_norm );

//            if (std::abs(error[0]) <= 0.001 && std::abs(error[1]) <= 0.001 && std::abs(error[2]) <= 0.001)
//            {
//                std::cout << "Reached point!" << std::endl;
//                final_quit = true;
//            }

            vpMouseButton::vpMouseButtonType button;
            if ( vpDisplay::getClick( plotter->I, button, false ) )
            {
                if ( button == vpMouseButton::button3 )
                {
                    final_quit = true;
                    tau_cmd    = 0;
                    std::cout << "Stop the robot " << std::endl;
                    robot.setRobotState( vpRobot::STATE_STOP );
                }
            }

            if ( opt_verbose )
            {
                std::cout << "dt: " << dt << std::endl;
            }

            time_prev = time_cur;
            robot.wait( time_cur, 0.001 ); // Sync loop at 1000 Hz (1 ms)
        }                                // end while

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




//vpMatrix
//Ta( const vpHomogeneousMatrix &edMe )
//{
//    vpMatrix Lx( 6, 6 ), Lw( 3, 3 ), skew_u( 3, 3 );
//
//    Lx.eye();
//    vpThetaUVector tu( edMe );
//
//    vpColVector u;
//    double theta;
//
//    tu.extract( theta, u );
//    skew_u = vpColVector::skew( u );
//    Lw.eye();
//    if ( theta != 0.0 )
//    {
//        Lw -= 0.5 * theta * skew_u;
//        Lw += ( 1 - ( ( vpMath::sinc( theta ) ) / ( vpMath::sqr( vpMath::sinc( theta * 0.5 ) ) ) ) ) * skew_u * skew_u;
//    }
//
//    Lx.insert( Lw, 3, 3 );
//
//    return Lx;
//}
//
//
//int main(int argc, char **argv ) {
//    // Initialize the robot arm
//    vpROSRobotFrankaCoppeliasim robot;
//    try
//    {
//        // ROS node
//        ros::init(argc, argv, "visp_ros");
//        ros::NodeHandlePtr n = boost::make_shared<ros::NodeHandle>();
//        ros::Rate loop_rate(1000);
//        ros::spinOnce();
//
//        robot.connect();
//
//        robot.coppeliasimStopSimulation(); // Allows to reset simulation, moving the robot to initial position
//        robot.setCoppeliasimSyncMode(false);
//        robot.coppeliasimStartSimulation();
//
//        // Set the initial position of the robot arm
//        std::cout << "Go to initial position " << std::endl;
//        vpColVector pos_init({0, vpMath::rad(-45), 0, vpMath::rad(-135), 0, vpMath::rad(90), vpMath::rad(45)});
//        robot.setRobotState(vpRobot::STATE_POSITION_CONTROL);
//        robot.setPosition(vpRobot::JOINT_STATE, pos_init);
//        vpTime::wait(500);
//
//        // Get homogeneous matrix of frame to end-effector
//        vpHomogeneousMatrix fme, fme0;
//        fme = robot.get_fMe();
//        fme0 = fme;
//        std::cout << "Frame to End-effector: " << fme0 << std::endl;
//
//        // Position and error vectors
//        vpColVector pos_xyz({fme[0][3], fme[1][3], fme[2][3]});
//        vpColVector desired_pos_xyz({fme[0][3], 0.05, fme[2][3]});
//        vpColVector error_xyz({desired_pos_xyz[0] - pos_xyz[0], desired_pos_xyz[1] - pos_xyz[1], desired_pos_xyz[2] - pos_xyz[2]});
//        std::cout << "Current pos end-effector: " << pos_xyz << std::endl;
//        std::cout << "Desired pos end-effector: " << desired_pos_xyz << std::endl;
//        std::cout << "Error x,y,z: " << error_xyz << std::endl;
//
//        // Joint pos, velocity, acceleration and more vectors
//        vpColVector q( 7, 0 ), dq( 7, 0 ), tau_d( 7, 0 ), tau_d0( 7, 0 ), tau_cmd( 7, 0 ),
//                x_e( 6, 0 ), dx_e( 6, 0 ), dx_ed( 6, 0 ), ddx_ed( 6, 0 );
//        vpMatrix fJe( 6, 7 ), Ja( 6, 7 ), dJa( 6, 7 ), Ja_old( 6, 7 );
//        vpColVector pose_err_norm( 2, 0 ), tau( 7, 0 );
//
//        std::cout << "Reading current joint position" << std::endl;
//        robot.getPosition( vpRobot::JOINT_STATE, q );
//        std::cout << "Initial joint position: " << q.t() << std::endl;
//
//        robot.setRobotState(vpRobot::STATE_FORCE_TORQUE_CONTROL);
//
//        // K, D
//        vpMatrix K( 6, 6 ), D( 6, 6 ), edVf( 6, 6 );
//
//        double wp = 50;
//        double wo = 20;
//        K.diag( { wp * wp, wp * wp, wp * wp, wo * wo, wo * wo, wo * wo } );
//        D.diag( { 2 * wp, 2 * wp, 2 * wp, 2 * wo, 2 * wo, 2 * wo } );
//
//        vpColVector integral( 7, 0 );
//
//        // Time
//        double time_start_trajectory, time_prev, time_cur;
//        double delay_before_trajectory = 0.5;
//
//        double mu = 4;
//        double dt = 0;
//
//        bool quit = false;
//        bool first_time       = false;
//        bool start_trajectory = false;
//
//        int loops = 0;
//
//        while(!quit)
//        {
//            std::cout<< "Loop: " << loops << std::endl;
//            time_cur = robot.getCoppeliasimSimulationTime();
//
//            robot.getPosition( vpRobot::JOINT_STATE, q );
//            robot.getVelocity( vpRobot::JOINT_STATE, dq );
//            robot.get_fJe(fJe); //Jacobian matrix, relates the velocity of the joints to the velocity of the end-effector
//            robot.getForceTorque( vpRobot::JOINT_STATE, tau );
//
//            if ( time_cur < delay_before_trajectory )
//            {
//                time_start_trajectory = time_cur;
//                first_time            = true;
//            }
//            else if ( !start_trajectory )
//            {
//                time_start_trajectory = time_cur;
//                start_trajectory      = true;
//            }
//
//            // Compute Cartesian trajectories
//            // X, Y, Z position
//            fme[0][3] = fme0[0][3] + (start_trajectory ? (error_xyz[0] * (time_cur - time_start_trajectory)) : 0);
//            fme[1][3] = fme0[1][3] + (start_trajectory ? (error_xyz[1] * (time_cur - time_start_trajectory)) : 0);
//            fme[2][3] = fme0[2][3] + (start_trajectory ? (error_xyz[2] * (time_cur - time_start_trajectory)) : 0);
//            // X, Y, Z velocity (derivative of position)
//            dx_ed[0] = (start_trajectory ? (fme[0][3] * (time_cur - time_start_trajectory)) : 0);
//            dx_ed[1] = (start_trajectory ? (fme[1][3] * (time_cur - time_start_trajectory)) : 0);
//            dx_ed[2] = (start_trajectory ? (fme[2][3] * (time_cur - time_start_trajectory)) : 0);
//            // X, Y, Z, acceleration
//            ddx_ed[0] = (start_trajectory ? (dx_ed[0] * (time_cur - time_start_trajectory)) : 0);
//            ddx_ed[1] = (start_trajectory ? (dx_ed[1] * (time_cur - time_start_trajectory)) : 0);
//            ddx_ed[2] = (start_trajectory ? (dx_ed[2] * (time_cur - time_start_trajectory)) : 0);
//
//            // Determine ...
//            std::cout << "fme rotation: " << fme.getRotationMatrix().t() << std::endl;
//            edVf.insert(fme.getRotationMatrix().t(), 0, 0 );
//            edVf.insert(fme.getRotationMatrix().t(), 3, 3 );
//
//            x_e = (vpColVector)vpPoseVector( fme.inverse() * robot.get_fMe() );
//            std::cout << "x_e: " << x_e << std::endl;
//            Ja = Ta( fme.inverse() * robot.get_fMe() ) * edVf * fJe;
//            std::cout << "Ja: " << Ja << std::endl;
//            dx_e = Ta( fme.inverse() * robot.get_fMe() ) * edVf * ( dx_ed - fJe * dq );
//            std::cout << "dx_e: " << dx_e << std::endl;
//
//            dt = time_cur - time_prev;
//
//            if ( dt != 0 )
//            {
//                dJa = ( Ja - Ja_old ) / dt;
//            }
//            else
//            {
//                dJa = 0;
//            }
//            Ja_old = Ja;
//
//            // Compute the control law
////            tau_d = Ja.pseudoInverse() * ( -K * ( x_e ) + D * (dx_e)-dJa * dq + ddx_ed);
//            tau_d = -K * ( x_e ) + D * (dx_e) ;
//            std::cout << "Tau: " << tau_d << std::endl;
//
//            if ( first_time )
//            {
//                tau_d0 = tau_d;
//            }
//
////            tau_cmd = tau_d - tau_d0 * std::exp( -mu * ( time_cur - time_start_trajectory ) );
//
//            robot.setForceTorque( vpRobot::JOINT_STATE, tau_d );
//
//            // Check if position is reached
//            float absErrorX = std::abs(error_xyz[0]);
//            float absErrorY = std::abs(error_xyz[1]);
//            float absErrorZ = std::abs(error_xyz[2]);
//            if (absErrorX - fme[0][3] > 0.001)
//            {
//                if (absErrorY - fme[1][3] > 0.001)
//                {
//                    if (absErrorZ - fme[2][3] > 0.001)
//                    {
//                        quit = true;
//                        std::cout << "Reached position!" << std::endl;
//                    }
//                }
//            }
////            else
////            {
////                std::cout << "Error X = " << absErrorX - fme[0][3] << std::endl;
////                std::cout << "Error Y = " << absErrorY - fme[1][3] << std::endl;
////                std::cout << "Error Z = " << absErrorZ - fme[2][3] << std::endl;
////            }
//            if (loops == 20){
//                quit = true;
//            }
//            else{
//                loops += 1;
//            }
//        }
//
//        vpTime::wait(1000);
//        robot.coppeliasimStopSimulation();
//
//    }
//    catch ( const vpException &e )
//    {
//        std::cout << "ViSP exception: " << e.what() << std::endl;
//        std::cout << "Stop the robot " << std::endl;
//        robot.setRobotState( vpRobot::STATE_STOP );
//        robot.coppeliasimStopSimulation();
//        return EXIT_FAILURE;
//    }
//
//    return 0;
//}






//int main(int argc, char **argv )
//{
//    // Initialize the robot arm
//    vpROSRobotFrankaCoppeliasim robot;
//    try
//    {
//        // ROS node
//        ros::init( argc, argv, "visp_ros" );
//        ros::NodeHandlePtr n = boost::make_shared< ros::NodeHandle >();
//        ros::Rate loop_rate( 1000 );
//        ros::spinOnce();
//
//        robot.connect();
//
//        robot.coppeliasimStopSimulation(); // Allows to reset simulation, moving the robot to initial position
//        robot.setCoppeliasimSyncMode( false );
//        robot.coppeliasimStartSimulation();
//
//        // Set the initial position of the robot arm
//        std::cout << "Go to initial position " << std::endl;
//        vpColVector pos_init( { 0, vpMath::rad( -45 ), 0, vpMath::rad( -135 ), 0, vpMath::rad( 90 ), vpMath::rad( 45 ) } );
//        robot.setRobotState( vpRobot::STATE_POSITION_CONTROL );
//        robot.setPosition( vpRobot::JOINT_STATE, pos_init );
//        vpTime::wait( 500 );
//
//        // Define position
//        vpColVector pos;
//        pos = pos_init;
//
//        // Define the desired position of point B
//        vpColVector pos_d( { 0, vpMath::rad( -45 ), 0, vpMath::rad( -135 ), 0, vpMath::rad( 90 ), vpMath::rad( -45 ) } );
//
//        // Set the desired velocity of the robot arm
//        vpColVector v(7, 0.1);
//        // Set the gains for the velocity controller
//        vpColVector Kp(7, 10.0);
//        vpColVector Ki(7, 0.0);
//        vpColVector Kd(7, 1.0);
//        // Initialize variables for the velocity controller
//        vpColVector error(7, 0.0);
//        vpColVector integral(7, 0.0);
//        vpColVector derivative(7, 0.0);
//        vpColVector prev_error(7, 0.0);
//        vpColVector torque(7, 0.0);
//
//        robot.getPosition( vpRobot::JOINT_STATE, pos);
//        std::cout << "Current position of robot: " << pos << std::endl;
//        std::cout << "Desired position of robot: " << pos_d << std::endl;
//
//        vpHomogeneousMatrix fmed, fmed0, end_effector;
//        end_effector = robot.get_eMh();
//        fmed = robot.get_fMe();
//        fmed0 = fmed;
//        std::cout << "Homogeneous matrix: " << fmed << std::endl;
//
//        robot.setRobotState( vpRobot::STATE_FORCE_TORQUE_CONTROL);
//
//        bool quit = false;
//
//        // Loop until the robot arm reaches the desired position
//        while (!quit) {
//            // Try out ----------------------------------------------------------------------------------------------
//            // Move in the x-direction only
//            fmed[0][3] = fmed[0][3] + 0.01;
//            std::cout << "Test: " << fmed.getRotationMatrix().t() << std::endl;
//
//
//            // Try out ----------------------------------------------------------------------------------------------
//
//            // Calculate the error
//            robot.getPosition(vpRobot::JOINT_STATE, pos);
//            error = pos_d - pos;
//            std::cout << "Error: " << error << std::endl;
//
//            // Calculate the integral and derivative of the error
//            integral += error;
//            derivative = error - prev_error;
//            prev_error = error;
//
//            // Calculate the torque commands using the velocity controller
//            torque = Kp*error + Ki*integral + Kd*derivative;
//
//            // Send the torque commands to the robot arm
//            robot.setForceTorque( vpRobot::JOINT_STATE, torque );
//
//            // Wait for a short time before checking the position again
//            vpTime::wait(10);
//            robot.getPosition(vpRobot::JOINT_STATE, pos);
//
//            bool done = true;
//            std::cout << "error 6:" << error[6] << std::endl;
//
//            float absError = std::abs(error[6]);
//            if (absError > 0.001)
//            {
//                done = false;
//            }
//
//            if (done)
//            {
//                break;
//            }
//        }
//
//        // Stop the robot arm
//        robot.coppeliasimStopSimulation();
//    }
//    catch ( const vpException &e )
//    {
//        std::cout << "ViSP exception: " << e.what() << std::endl;
//        std::cout << "Stop the robot " << std::endl;
//        robot.setRobotState( vpRobot::STATE_STOP );
//        return EXIT_FAILURE;
//    }
//
//    return 0;
//}

