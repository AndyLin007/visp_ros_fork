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

//! \example tutorial-franka-coppeliasim-pbvs-apriltag.cpp

#include <iostream>

#include <visp3/core/vpCameraParameters.h>
#include <visp3/detection/vpDetectorAprilTag.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpPlot.h>
#include <visp3/io/vpImageIo.h>
#include <visp3/visual_features/vpFeatureThetaU.h>
#include <visp3/visual_features/vpFeatureTranslation.h>
#include <visp3/vs/vpServo.h>
#include <visp3/vs/vpServoDisplay.h>

#include <visp_ros/vpROSGrabber.h>
#include <visp_ros/vpROSRobotFrankaCoppeliasim.h>

struct Box
  {
    double length;
    double width;
    double height;
  };

void
display_point_trajectory( const vpImage< unsigned char > &I, const std::vector< vpImagePoint > &vip,
                          std::vector< vpImagePoint > *traj_vip )
{
    for ( size_t i = 0; i < vip.size(); i++ )
    {
        if ( traj_vip[i].size() )
        {
            // Add the point only if distance with the previous > 1 pixel
            if ( vpImagePoint::distance( vip[i], traj_vip[i].back() ) > 1. )
            {
                traj_vip[i].push_back( vip[i] );
            }
        }
        else
        {
            traj_vip[i].push_back( vip[i] );
        }
    }
    for ( size_t i = 0; i < vip.size(); i++ )
    {
        for ( size_t j = 1; j < traj_vip[i].size(); j++ )
        {
            vpDisplay::displayLine( I, traj_vip[i][j - 1], traj_vip[i][j], vpColor::green, 2 );
        }
    }
}

int
main( int argc, char **argv )
{
    double opt_tagSize             = 0.08;
    bool display_tag               = true;
    int opt_quad_decimate          = 2;
    bool opt_verbose               = false;
    bool opt_plot                  = false;
    bool opt_adaptive_gain         = false;
    bool opt_task_sequencing       = true;
    double convergence_threshold_t = 0.0005, convergence_threshold_tu = vpMath::rad( 0.5 );
    bool opt_coppeliasim_sync_mode = true;

    for ( int i = 1; i < argc; i++ )
    {
        if ( std::string( argv[i] ) == "--tag_size" && i + 1 < argc )
        {
            opt_tagSize = std::stod( argv[i + 1] );
        }
        else if ( std::string( argv[i] ) == "--verbose" || std::string( argv[i] ) == "-v" )
        {
            opt_verbose = true;
        }
        else if ( std::string( argv[i] ) == "--plot" )
        {
            opt_plot = true;
        }
        else if ( std::string( argv[i] ) == "--adaptive_gain" )
        {
            opt_adaptive_gain = true;
        }
        else if ( std::string( argv[i] ) == "--task_sequencing" )
        {
            opt_task_sequencing = true;
        }
        else if ( std::string( argv[i] ) == "--quad_decimate" && i + 1 < argc )
        {
            opt_quad_decimate = std::stoi( argv[i + 1] );
        }
        else if ( std::string( argv[i] ) == "--no-convergence-threshold" )
        {
            convergence_threshold_t  = 0.;
            convergence_threshold_tu = 0.;
        }
        else if ( std::string( argv[i] ) == "--enable-coppeliasim-sync-mode" )
        {
            opt_coppeliasim_sync_mode = true;
        }
        else if ( std::string( argv[i] ) == "--help" || std::string( argv[i] ) == "-h" )
        {
            std::cout << argv[0] << "[--tag_size <marker size in meter; default " << opt_tagSize << ">] "
                      << "[--quad_decimate <decimation; default " << opt_quad_decimate << ">] "
                      << "[--adaptive_gain] "
                      << "[--plot] "
                      << "[--task_sequencing] "
                      << "[--no-convergence-threshold] "
                      << "[--enable-coppeliasim-sync-mode] "
                      << "[--verbose] [-v] "
                      << "[--help] [-h]" << std::endl;
            return EXIT_SUCCESS;
        }
    }

    try
    {
        //------------------------------------------------------------------------//
        //------------------------------------------------------------------------//
        // ROS node
        ros::init( argc, argv, "visp_ros" );
        ros::NodeHandlePtr n = boost::make_shared< ros::NodeHandle >();
        ros::Rate loop_rate( 1000 );
        ros::spinOnce();

        ros::Publisher pub_suctionpad = n->advertise<std_msgs::Int32>("/suctionpad_activate", 1);
        std_msgs::Int32 activate;
        activate.data = 0;

        vpROSRobotFrankaCoppeliasim robot;
        robot.setVerbose( opt_verbose );
        robot.connect();

        std::cout << "Coppeliasim sync mode enabled: " << ( opt_coppeliasim_sync_mode ? "yes" : "no" ) << std::endl;
        robot.coppeliasimStopSimulation(); // Allows to reset simulation, moving the robot to initial position
        robot.setCoppeliasimSyncMode( false );
        robot.coppeliasimStartSimulation();

        if ( 0 )
        {
            robot.setRobotState( vpRobot::STATE_POSITION_CONTROL );
            vpColVector q;
            robot.getPosition( vpRobot::JOINT_STATE, q );
            std::cout << "Initial joint position: " << q.t() << std::endl;

            q[0] += vpMath::rad( 10 ); // Add 10 deg axis 1
            std::cout << "Move to joint position: " << q.t() << std::endl;
            robot.setPosition( vpRobot::JOINT_STATE, q );
        }

        vpImage< unsigned char > I;
        vpROSGrabber g;
        g.setImageTopic( "/coppeliasim/franka/camera/image" );
        g.setCameraInfoTopic( "/coppeliasim/franka/camera/camera_info" );
        g.open( argc, argv );

        g.acquire( I );

        std::cout << "Image size: " << I.getWidth() << " x " << I.getHeight() << std::endl;
        vpCameraParameters cam;

        g.getCameraInfo( cam );
        std::cout << cam << std::endl;
        vpDisplayOpenCV dc( I, 10, 10, "Color image" );

        vpDetectorAprilTag::vpAprilTagFamily tagFamily                  = vpDetectorAprilTag::TAG_36h11;
        vpDetectorAprilTag::vpPoseEstimationMethod poseEstimationMethod = vpDetectorAprilTag::HOMOGRAPHY_VIRTUAL_VS;
        vpDetectorAprilTag detector( tagFamily );
        detector.setAprilTagPoseEstimationMethod( poseEstimationMethod );
        detector.setDisplayTag( display_tag );
        detector.setAprilTagQuadDecimate( opt_quad_decimate );

        //Box information
        // Define the sizes of parcels
        vpMatrix SizeParcel(3,10);
        SizeParcel[0][0] = 0.02;
        SizeParcel[1][0] = 0.02;
        SizeParcel[2][0] = 0.02;
        SizeParcel[0][1] = 0.015;
        SizeParcel[1][1] = 0.015;
        SizeParcel[2][1] = 0.01;
        SizeParcel[0][2] = 0.02;
        SizeParcel[1][2] = 0.02;
        SizeParcel[2][2] = 0.02;
        SizeParcel[0][3] = 0.02;
        SizeParcel[1][3] = 0.02;
        SizeParcel[2][3] = 0.02;
        SizeParcel[0][4] = 0.02;
        SizeParcel[1][4] = 0.02;
        SizeParcel[2][4] = 0.02;
        SizeParcel[0][5] = 0.02;
        SizeParcel[1][5] = 0.02;
        SizeParcel[2][5] = 0.02;
        SizeParcel[0][6] = 0.02;
        SizeParcel[1][6] = 0.02;
        SizeParcel[2][6] = 0.02;
        SizeParcel[0][7] = 0.02;
        SizeParcel[1][7] = 0.02;
        SizeParcel[2][7] = 0.02;
        SizeParcel[0][8] = 0.02;
        SizeParcel[1][8] = 0.02;
        SizeParcel[2][8] = 0.02;
        SizeParcel[0][9] = 0.02;
        SizeParcel[1][9] = 0.02;
        SizeParcel[2][9] = 0.02;
        SizeParcel[0][10] = 0.02;
        SizeParcel[1][10] = 0.02;
        SizeParcel[2][10] = 0.02;
        // Size of the crate in x- and y-direction
        double Ycrate = 0.575;
        double Xcrate = 0.365;
        double Xmargin, Ymargin, Zmargin;
        Xmargin = 0.02;
        Ymargin = 0.02;
        Zmargin = 0.01;

        // Servo
        vpHomogeneousMatrix cMo, oMo, active_cdMc;

        // Desired pose used to compute the desired features
        vpHomogeneousMatrix cdMo( vpTranslationVector( 0, 0.0, opt_tagSize * 3 ),
                                  vpRotationMatrix( { 1, 0, 0, 0, -1, 0, 0, 0, -1 } ) );
        vpHomogeneousMatrix eedMo( vpTranslationVector( 0.1, 0.0, 0.01 ),
                                   vpRotationMatrix( { 1, 0, 0, 0, -1, 0, 0, 0, -1 } ) );
        // pick the bin
        vpHomogeneousMatrix edMo(vpTranslationVector(0.0, 0.0, 0.002),
                                 vpRotationMatrix( {0, 1, 0, 1, 0, 0, 0, 0, -1} ) );

        // pick the bin on conveyor
        vpHomogeneousMatrix fM_eed_up_conveyor(vpTranslationVector(-0.43, 0.32, 0.15),
                                               vpRotationMatrix( {0, 1, 0, 1, 0, 0, 0, 0, -1} ) );

        double x = 0.785;
        double c = cos(x);
        double s = sin(x);
        // home picking position
        vpHomogeneousMatrix fM_eed_home(vpTranslationVector(0.2, 0, 0.2), // vpTranslationVector(0.0, 0.3, 0.2),
                                        vpRotationMatrix( {c, 0, -s, 0, -1, 0, -s, 0, -c} ) ); //vpRotationMatrix( {0, 1, 0, 1, 0, 0, 0, 0, -1} ) );
        // top of left tote
        vpHomogeneousMatrix fM_eed_l_tote(vpTranslationVector(0.45, -0.4, 0.15),
                                          vpRotationMatrix( {1, 0, 0, 0, -1, 0, 0, 0, -1} ) );
        // top of conveyor
        vpHomogeneousMatrix fM_eed_r_tote(vpTranslationVector(0.2, 0.3, 0.1),
                                          vpRotationMatrix( {0, 1, 0, 1, 0, 0, 0, 0, -1} ) );

        // origin of left tote in robot base frame
        vpHomogeneousMatrix fM_r_tote(vpTranslationVector(-0.000, -0.2922, -0.3025),
                                      vpRotationMatrix( {1, 0, 0, 0, 1, 0, 0, 0, 1} ) );

        // rotating motion Y1
        vpHomogeneousMatrix fM_Y1_rotate(vpTranslationVector(-0.000, -0.2922, -0.3425),
                                      vpRotationMatrix( {c, 0, s, 0, 1, 0, -s, 0, c} ) );

        // rotating motion Y2
        vpHomogeneousMatrix fM_Y2_rotate(vpTranslationVector(-0.050, -0.2922, -0.4225),
                                      vpRotationMatrix( {1, 0, 0, 0, 1, 0, 0, 0, 1} ) );

        // Box placement in tote origin coordinates (remember that the box's frame is on its top at the center)
        double X, Y, Z,
            X0, X1, X2, X3, X4, X5, X6, X7, X8, X9, X10,
            Y0, Y1, Y2, Y3, Y4, Y5, Y6, Y7, Y8, Y9, Y10,
            Z0, Z1, Z2, Z3, Z4, Z5, Z6, Z7, Z8, Z9, Z10;
        X = 0; X0 = 0; X1 = 0; X2 = 0; X3 = 0; X4 = 0; X5 = 0; X6 = 0; X7 = 0; X8 = 0; X9 = 0; X10 = 0;
        Y = 0; Y0 = 0; Y1 = 0; Y2 = 0; Y3 = 0; Y4 = 0; Y5 = 0; Y6 = 0; Y7 = 0; Y8 = 0; Y9 = 0; Y10 = 0;
        Z = 0; Z0 = 0; Z1 = 0; Z2 = 0; Z3 = 0; Z4 = 0; Z5 = 0; Z6 = 0; Z7 = 0; Z8 = 0; Z9 = 0; Z10 = 0;


        vpHomogeneousMatrix r_toteM_tag(vpTranslationVector(X, -Y, Z),
                                         vpRotationMatrix( {0, 1, 0, -1, 0, 0, 0, 0, 1} ) );


        X0 += SizeParcel[0][0];
        Y0 += SizeParcel[1][0];
        Z0 += SizeParcel[2][0];

        vpHomogeneousMatrix r_toteM_tag0(vpTranslationVector(X0, -Y0, Z0),
                                         vpRotationMatrix( {0, 1, 0, -1, 0, 0, 0, 0, 1} ) );

        if((Y0 + Ymargin + SizeParcel[1][1]) < 0.365 && (X0 + Xmargin + SizeParcel[0][1]) < 0.575)
        {
            X1 = SizeParcel[0][1];
            Y1 += SizeParcel[1][1];
            Z1 == Z0;
        }
        else if((Y0 + Ymargin + SizeParcel[1][1]) > 0.365 && (X0 + Xmargin + SizeParcel[0][1]) < 0.575){ //0.365 is on Y-axis and 0.575 is on X-axis
            X1 += SizeParcel[0][1];
            Y1 = SizeParcel[1][1];
            Z1 == Z0;
        }

        vpHomogeneousMatrix r_toteM_tag1(vpTranslationVector(X1, -Y1, Z1),
                                          vpRotationMatrix( {0, 1, 0, -1, 0, 0, 0, 0, 1} ) );





        cdMo = robot.get_eMc().inverse()*eedMo;
        active_cdMc = cdMo * cMo.inverse();

        // Create visual features
        vpFeatureTranslation t( vpFeatureTranslation::cdMc );
        vpFeatureThetaU tu( vpFeatureThetaU::cdRc );
        t.buildFrom( active_cdMc );
        tu.buildFrom( active_cdMc );

        vpFeatureTranslation td( vpFeatureTranslation::cdMc );
        vpFeatureThetaU tud( vpFeatureThetaU::cdRc );

        vpServo task;
        // Add the visual features
        task.addFeature( t, td );
        task.addFeature( tu, tud );

        task.setServo( vpServo::EYEINHAND_CAMERA );
        task.setInteractionMatrixType( vpServo::CURRENT );

        if ( opt_adaptive_gain )
        {
            std::cout << "Enable adaptive gain" << std::endl;
            vpAdaptiveGain lambda( 4, 1.2, 25 ); // lambda(0)=4, lambda(oo)=1.2 and lambda'(0)=25
            task.setLambda( lambda );
        }
        else
        {
            task.setLambda( 1.2 );
        }

        vpPlot *plotter = nullptr;

        if ( opt_plot )
        {
            plotter = new vpPlot( 2, static_cast< int >( 250 * 2 ), 500, static_cast< int >( I.getWidth() ) + 80, 10,
                                  "Real time curves plotter" );
            plotter->setTitle( 0, "Visual features error" );
            plotter->setTitle( 1, "Camera velocities" );
            plotter->initGraph( 0, 6 );
            plotter->initGraph( 1, 6 );
            plotter->setLegend( 0, 0, "error_feat_tx" );
            plotter->setLegend( 0, 1, "error_feat_ty" );
            plotter->setLegend( 0, 2, "error_feat_tz" );
            plotter->setLegend( 0, 3, "error_feat_theta_ux" );
            plotter->setLegend( 0, 4, "error_feat_theta_uy" );
            plotter->setLegend( 0, 5, "error_feat_theta_uz" );
            plotter->setLegend( 1, 0, "vc_x" );
            plotter->setLegend( 1, 1, "vc_y" );
            plotter->setLegend( 1, 2, "vc_z" );
            plotter->setLegend( 1, 3, "wc_x" );
            plotter->setLegend( 1, 4, "wc_y" );
            plotter->setLegend( 1, 5, "wc_z" );
        }

        bool final_quit                           = false;
        bool has_converged                        = false;
        bool send_velocities                      = false;
        bool servo_started                        = false;
        std::vector< vpImagePoint > *traj_corners = nullptr; // To memorize point trajectory

        double sim_time            = robot.getCoppeliasimSimulationTime();
        double sim_time_prev       = sim_time;
        double sim_time_init_servo = sim_time;
        double sim_time_img        = sim_time;

        if ( 0 )
        {
            // Instead of setting eMc from /coppeliasim/franka/eMc topic, we can set its value to introduce noise for example
            vpHomogeneousMatrix eMc;
            eMc.buildFrom( 0.05, -0.05, 0, 0, 0, M_PI_4 );
            robot.set_eMc( eMc );
        }
        vpHomogeneousMatrix eMc;
        eMc = robot.get_eMc() ;
        std::cout << "eMc:\n" << eMc << std::endl;

        robot.setRobotState( vpRobot::STATE_VELOCITY_CONTROL );
        robot.setCoppeliasimSyncMode( opt_coppeliasim_sync_mode );

        int TagID = 1;
        int State = 0;
        int c_idx = 0;
        typedef struct{
            int ID;
            vpHomogeneousMatrix wMo;
            vpHomogeneousMatrix oMo;
        } scene_obj;

        std::vector<scene_obj> obj_vec;  // vector of all objects in the scene w.r.t. the world frame

        vpColVector  v_0( 6 ), v_c( 6 );

        while ( !final_quit )
        {
            sim_time = robot.getCoppeliasimSimulationTime();

            g.acquire( I, sim_time_img );
            vpDisplay::display( I );

            std::vector< vpHomogeneousMatrix > cMo_vec;
            std::vector<int> TagsID;
            detector.detect( I, opt_tagSize, cam, cMo_vec );
            TagsID = detector.getTagsId();

            {
                std::stringstream ss;
                ss << "Left click to " << ( send_velocities ? "stop the robot" : "servo the robot" )
                   << ", right click to quit.";
                vpDisplay::displayText( I, 20, 20, ss.str(), vpColor::red );
            }


            // One or more targets are detected
            if ( cMo_vec.size() >= 1 )
            {
                static bool first_time = true;
                if ( first_time ) // the first time
                {
                    for(int idx=0;idx<cMo_vec.size();idx++)
                    { // scan all the objects in the FoV and add them to the object vector
                        scene_obj ob;
                        // Introduce security wrt tag positionning in order to avoid PI rotation
                        std::vector<vpHomogeneousMatrix> v_oMo(2), v_ccMc(2);
                        v_oMo[1].buildFrom(0, 0, 0, 0, 0, M_PI);
                        for (size_t i = 0; i < 2; i++) {
                            v_ccMc[i] = active_cdMc * v_oMo[i] * cMo.inverse();
                        }
                        if (std::fabs(v_ccMc[0].getThetaUVector().getTheta()) < std::fabs(v_ccMc[1].getThetaUVector().getTheta())) {
                            ob.oMo = v_oMo[0];
                        }
                        else {
                            std::cout << "Desired frame modified to avoid PI rotation of the camera" << std::endl;
                            ob.oMo = v_oMo[1];   // Introduce PI rotation
                        }

                        ob.ID = TagsID[idx];
                        ob.wMo = robot.get_fMe()*eMc*cMo_vec[idx];
                        obj_vec.push_back(ob);

                    }
                    // Initialize first pose to go to (the home position)
                    std::cout << "Initializing the pose to track to home \n";
                    State = 0;
                    active_cdMc = (fM_eed_home*eMc).inverse()*robot.get_fMe()*eMc ;
                    t.buildFrom(active_cdMc);
                    tu.buildFrom(active_cdMc);
                    v_0 = task.computeControlLaw();

                }else{ // scan all the visible objects in the FoV and update those already in the vector and add the new objects
                    for(int idx=0;idx<cMo_vec.size();idx++){ // scan all the objects in the FoV and ...
                        bool found = false;
                        for(int j=0;j<obj_vec.size();j++){             //if one of them is already in the object vector, update it, or
                            if(obj_vec[j].ID == TagsID[idx]){obj_vec[j].wMo = robot.get_fMe()*eMc*cMo_vec[idx]; // if you assume the objects are static, you can comment this line
                                found = true;
                                break;
                            }
                        }
                        if(!found){ // if is a new element in the scene... add it in the vector
                            scene_obj ob;
                            ob.ID = TagsID[idx];
                            ob.wMo = robot.get_fMe()*eMc*cMo_vec[idx];
                            obj_vec.push_back(ob);
                        }
                    }
                }


                if ( first_time )
                {
                    first_time = false;
                }
            } // end if (cMo_vec.size() >= 1)
            else
            {
                // if no object is detected, do something.

            }
            // -------------------------------------------------------------------------------------------------------------
            // -------------------------------------------------------------------------------------------------------------
            // Update active pose to track <- this must be done by the FSM

            // FSM States
            if( State == 0){
                active_cdMc = (fM_eed_r_tote*eMc).inverse()*robot.get_fMe()*eMc ;
                t.buildFrom( active_cdMc );
                tu.buildFrom( active_cdMc );

                if ( !servo_started )
                {
                    if ( send_velocities )
                    {
                        servo_started = true;
                    }
                    v_0 = task.computeControlLaw();
                    sim_time_init_servo = robot.getCoppeliasimSimulationTime();
                }

            }else if( State == 1 ){
                cdMo = eMc.inverse()*edMo;
                cMo = (robot.get_fMe()*eMc).inverse() * obj_vec[c_idx].wMo;
                active_cdMc = cdMo * obj_vec[c_idx].oMo * cMo.inverse();
                t.buildFrom(active_cdMc);
                tu.buildFrom(active_cdMc);
                if ( !servo_started )
                {
                    if ( send_velocities )
                    {
                        servo_started = true;
                    }
                    v_0 = task.computeControlLaw();
                    sim_time_init_servo = robot.getCoppeliasimSimulationTime();
                }
            }else if( State == 2 ){
                active_cdMc = (fM_eed_r_tote*eMc).inverse()*robot.get_fMe()*eMc ;
                t.buildFrom(active_cdMc);
                tu.buildFrom(active_cdMc);
                if ( !servo_started )
                {
                    if ( send_velocities )
                    {
                        servo_started = true;
                    }
                    v_0 = task.computeControlLaw();
                    sim_time_init_servo = robot.getCoppeliasimSimulationTime();
                }
            }else if( State == 3 ){
                active_cdMc = (fM_eed_l_tote*eMc).inverse()*robot.get_fMe()*eMc ;
                t.buildFrom(active_cdMc);
                tu.buildFrom(active_cdMc);
                if ( !servo_started )
                {
                    if ( send_velocities )
                    {
                        servo_started = true;
                    }
                    v_0 = task.computeControlLaw();
                    sim_time_init_servo = robot.getCoppeliasimSimulationTime();
                }
            }
            else if( State == 4 )
            {
                active_cdMc = (fM_r_tote* r_toteM_tag*edMo.inverse()*eMc).inverse()*robot.get_fMe()*eMc ; //Place it
                t.buildFrom(active_cdMc);
                tu.buildFrom(active_cdMc);
                if ( !servo_started )
                {
                    if ( send_velocities )
                    {
                        servo_started = true;
                    }
                    v_0 = task.computeControlLaw();
                    sim_time_init_servo = robot.getCoppeliasimSimulationTime();
                }
            }
//            else if( State == 5 )
//            {
//                active_cdMc = (fM_Y1_rotate* r_toteM_tag*edMo.inverse()*eMc).inverse()*robot.get_fMe()*eMc ; //rotate and Place it
//                t.buildFrom(active_cdMc);
//                tu.buildFrom(active_cdMc);
//                if ( !servo_started )
//                {
//                    if ( send_velocities )
//                    {
//                        servo_started = true;
//                    }
//                    v_0 = task.computeControlLaw();
//                    sim_time_init_servo = robot.getCoppeliasimSimulationTime();
//                }
//            }
//            else if( State == 6 ){
//
//                active_cdMc = (fM_Y2_rotate* r_toteM_tag*edMo.inverse()*eMc).inverse()*robot.get_fMe()*eMc ; //rotate and Place it
//                t.buildFrom(active_cdMc);
//                tu.buildFrom(active_cdMc);
//                if ( !servo_started )
//                {
//                    if ( send_velocities )
//                    {
//                        servo_started = true;
//                    }
//                    v_0 = task.computeControlLaw();
//                    sim_time_init_servo = robot.getCoppeliasimSimulationTime();
//                }
//            }
            else if( State == 7){
                active_cdMc = (fM_eed_home*eMc).inverse()*robot.get_fMe()*eMc ;
                t.buildFrom(active_cdMc);
                tu.buildFrom(active_cdMc);
                if ( !servo_started )
                {
                    if ( send_velocities )
                    {
                        servo_started = true;
                    }
                    v_0 = task.computeControlLaw();
                    sim_time_init_servo = robot.getCoppeliasimSimulationTime();
                }
            }else if( State == 100 ){ // idle state
                if( !has_converged ){
                    std::cout << "State: Idle \n";
                    has_converged = true;
                }
                active_cdMc.eye();
                t.buildFrom(active_cdMc);
                tu.buildFrom(active_cdMc);
                v_c = 0;
            }
            double error_tr  = sqrt( active_cdMc.getTranslationVector().sumSquare() );           // translation error
            double error_tu  = vpMath::deg( sqrt( active_cdMc.getThetaUVector().sumSquare() ) ); // orientation error

            // FSM transition conditions
            if(error_tr <= 0.01 && error_tu <= 5 && State == 0)// once reached the top of the right tote, go to pick the parcel on the conveyor
            {
                std::cout << "Left tote reached... moving to the next state \n";

                bool found = false;
                for(int idx=0;idx<obj_vec.size();idx++){  // search in the object vector if the next object already exist
                    if(obj_vec[idx].ID == TagID)
                    {
                        for(int m=0;m<2;m++)
                      {
                        std::cout << "The parcel is visible and I know where to go to pick it! \n";
                        State = 1; // New state
                        TagID = m; // box with tag ID have to be picked in order
                        c_idx = idx;
                        found = true;
                        servo_started = false;
                        break;
                      }
                    }
                }
                if ( !found ){
                    std::cout << "No box was found to pick, look somewhere else... \n";
                    // here you can implement an alternative motion to look for the packages around.
                    // for the moment, we suppose the box is always there
                    State = 100;
                    v_c = 0;
                }


            }else if(error_tr <= 0.002 && error_tu <= 1 && State == 1){ // once you reach the box to pick, activate vacuum and move up

                std::cout << "On top of the parcel to pick \n";
                std::cout << "Activating suction pad \n";
                std::cout << "Parcel was picked, going to lift it \n";
                activate.data = 1;
                pub_suctionpad.publish(activate);
                State = 2;
                servo_started = false;

            }else if(error_tr <= 0.01 && error_tu <= 5 && State == 2){ // once reached the position above the right tote, move to the left tote

                std::cout << "Moving the parcel to the left tote \n";
                State = 3;
                servo_started = false;

            }else if(error_tr <= 0.01 && error_tu <= 5 && State == 3){ // once reached the position above the left tote, palce the box

                std::cout << "Left tote reached, going to place the parcel \n";
                State = 4;
                servo_started = false;

            }
            else if(error_tr <= 0.01 && error_tu <= 5 && State == 4){ // once you have placed the box, got o home position

                std::cout << "Parcel placed... Deactivating vacuum\n";
                activate.data = 0;
                State = 6;
                servo_started = false;

            }
//            else if(error_tr <= 0.01 && error_tu <= 5 && State == 5){ // once you have a box, rotate it.
//
//                std::cout << "Parcel placed... Deactivating vacuum\n";
//                activate.data = 0;
//                State = 6;
//                servo_started = false;
//
//            }
            else if(error_tr <= 0.01 && error_tu <= 1 && State == 6){ // once you have placed the box, got o home position

                std::cout << "Parcel placed... Deactivating vacuum\n";
                activate.data = 0;
                pub_suctionpad.publish(activate);
                State = 7;
                servo_started = false;

            }
            else if(error_tr <= 0.01 && error_tu <= 5 && State == 7){ // once you have placed the box, got o home position

                std::cout << "Home position reached, going Idle \n";
                State = 0; //

            }



//      if ( opt_task_sequencing )
//      {
//        if ( !servo_started )
//        {
//          if ( send_velocities )
//          {
//            servo_started = true;
//          }
//          sim_time_init_servo = robot.getCoppeliasimSimulationTime();
//        }
//        v_c = task.computeControlLaw( robot.getCoppeliasimSimulationTime() - sim_time_init_servo );
//      }
//      else
//      {
            v_c = task.computeControlLaw() - v_0*exp(-10.0*(robot.getCoppeliasimSimulationTime() - sim_time_init_servo));
//      }

            // Display the current and desired feature points in the image display
            // Display desired and current pose features
            vpDisplay::displayFrame( I, cdMo * oMo, cam, opt_tagSize / 1.5, vpColor::yellow, 2 );
            vpDisplay::displayFrame( I, cMo, cam, opt_tagSize / 2, vpColor::none, 3 );

            if ( opt_plot )
            {
                plotter->plot( 0, static_cast< double >( sim_time ), task.getError() );
                plotter->plot( 1, static_cast< double >( sim_time ), v_c );
            }




            std::stringstream ss;
            ss << "error_t: " << error_tr;
            vpDisplay::displayText( I, 20, static_cast< int >( I.getWidth() ) - 150, ss.str(), vpColor::red );
            ss.str( "" );
            ss << "error_tu: " << error_tu;
            vpDisplay::displayText( I, 40, static_cast< int >( I.getWidth() ) - 150, ss.str(), vpColor::red );
            ss.str( "" );
            ss << "current state: " << State;
            vpDisplay::displayText( I, 60, static_cast< int >( I.getWidth() ) - 150, ss.str(), vpColor::red );


            // -------------------------------------------------------------------------------------------------------------
            // -------------------------------------------------------------------------------------------------------------

            if ( !send_velocities )
            {
                v_c = 0; // Stop the robot
                servo_started = false;
            }

            robot.setVelocity( vpRobot::CAMERA_FRAME, v_c );

//      std::stringstream ss;
            ss << "Loop time [s]: " << std::round( ( sim_time - sim_time_prev ) * 1000. ) / 1000.;
            ss << " Simulation time [s]: " << sim_time;
            sim_time_prev = sim_time;
            vpDisplay::displayText( I, 40, 20, ss.str(), vpColor::red );

            vpMouseButton::vpMouseButtonType button;
            if ( vpDisplay::getClick( I, button, false ) )
            {
                switch ( button )
                {
                    case vpMouseButton::button1:
                        send_velocities = !send_velocities;
                        break;

                    case vpMouseButton::button3:
                        final_quit = true;
                        v_c        = 0;
                        break;

                    default:
                        break;
                }
            }

            vpDisplay::flush( I );
            robot.wait( sim_time, 0.020 ); // Slow down the loop to simulate a camera at 50 Hz
        }                                // end while

        if ( opt_plot && plotter != nullptr )
        {
            delete plotter;
            plotter = nullptr;
        }
        robot.coppeliasimStopSimulation();

        if ( !final_quit )
        {
            while ( !final_quit )
            {
                g.acquire( I );
                vpDisplay::display( I );

                vpDisplay::displayText( I, 20, 20, "Click to quit the program.", vpColor::red );
                vpDisplay::displayText( I, 40, 20, "Visual servo converged.", vpColor::red );

                if ( vpDisplay::getClick( I, false ) )
                {
                    final_quit = true;
                }

                vpDisplay::flush( I );
            }
        }
        if ( traj_corners )
        {
            delete[] traj_corners;
        }
    }
    catch ( const vpException &e )
    {
        std::cout << "ViSP exception: " << e.what() << std::endl;
        std::cout << "Stop the robot " << std::endl;
        return EXIT_FAILURE;
    }

    return 0;
}
