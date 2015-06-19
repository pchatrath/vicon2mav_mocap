///////////////////////////////////////////////////////////////////////////////
//
// Copyright (C) OMG Plc 2009.
// All rights reserved.  This software is protected by copyright
// law and international treaties.  No part of this software / document
// may be reproduced or distributed in any form or by any means,
// whether transiently or incidentally to some other use of this software,
// without the written permission of the copyright owner.
//
///////////////////////////////////////////////////////////////////////////////

// only 1 (!) Debug Mode should be enabled, otherwise vicon2mav_mocap will not do anything
#define VICON_STREAM_DEBUG 0
        // everything concerning MavLink and the TCP/IP connection will be ignored
        // enables comprehensive information in console
#define MAVLINK_DEBUG 0
        // everything concerning ViconDataStream will be ignored
#define TCP_CONNECT_DEBUG 0
        // everything concerning MavLink and the ViconDataStream will be ignored





#include <QCoreApplication>
#include <iostream>
//#include <time.h>
//#include <sys/time.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <cstdlib>
#include <unistd.h>

#include "functions.h"  // constants are declared here

#include "vicon/client.h"

#if (!VICON_STREAM_DEBUG && !TCP_CONNECT_DEBUG)
//    #include "mavlink/mavlink.h" // for #define several variables
//    #include "mavlink/mavlink_msg_att_pos_mocap.h"
//    #include "mavlink/mavlink_types.h"
    #include "/home/gcs/programming/mavlink_lib/c_library-master/common/mavlink.h"
#endif

using namespace ViconDataStreamSDK::CPP;


namespace
{
  std::string Adapt( const bool i_Value )
  {
    return i_Value ? "True" : "False";
  }

  std::string Adapt( const Direction::Enum i_Direction )
  {
    switch( i_Direction )
    {
      case Direction::Forward:
        return "Forward";
      case Direction::Backward:
        return "Backward";
      case Direction::Left:
        return "Left";
      case Direction::Right:
        return "Right";
      case Direction::Up:
        return "Up";
      case Direction::Down:
        return "Down";
      default:
        return "Unknown";
    }
  }

  std::string Adapt( const DeviceType::Enum i_DeviceType )
  {
    switch( i_DeviceType )
    {
      case DeviceType::ForcePlate:
        return "ForcePlate";
      case DeviceType::Unknown:
      default:
        return "Unknown";
    }
  }

  std::string Adapt( const Unit::Enum i_Unit )
  {
    switch( i_Unit )
    {
      case Unit::Meter:
        return "Meter";
      case Unit::Volt:
        return "Volt";
      case Unit::NewtonMeter:
        return "NewtonMeter";
      case Unit::Newton:
        return "Newton";
      case Unit::Unknown:
      default:
        return "Unknown";
    }
  }
}

int main( )
{
#if VICON_STREAM_DEBUG
    std::cout << "vicon2mav_mocap in VICON_STREAM_DEBUG mode, everything concerning MavLink and the TCP/IP connection will be ignored!" << std::endl;
#endif
#if MAVLINK_DEBUG
    std::cout << "vicon2mav_mocap in MAVLINK_DEBUG mode, everything concerning the ViconDataStream will be ignored!" << std::endl;
#endif
#if TCP_CONNECT_DEBUG
    std::cout << "vicon2mav_mocap in TCP_CONNECT_DEBUG mode, everything concerning MavLink and the ViconDataStream will be ignored!" << std::endl;
#endif

  /********************************************************/
  /*  configure and enable Connection to ViconDataStream  */
  /********************************************************/
#if (!TCP_CONNECT_DEBUG && !MAVLINK_DEBUG)
  // Program options
  bool TransmitMulticast = false;

  // Make a new Vicon-Client
  Client ViconClient;

  // Connect to a server
  int n = 1;
  std::cout << "Connecting to " << VICON_SERVER_ADDRESS << " ..." << std::flush;
#if VICON_STREAM_DEBUG
  unsigned long long int stime = getMSecondoftheday();
#endif // VICON_STREAM_DEBUG
  while( !ViconClient.IsConnected().Connected )
  {
      // Direct connection
      ViconClient.Connect( VICON_SERVER_ADDRESS );

      if (n >= 2)
      {
          n = 0;
      #if VICON_STREAM_DEBUG
          std::cout << getMSecondoftheday() - stime << "." << std::flush;
      #else // VICON_STREAM_DEBUG
          std::cout << "." << std::flush;
      #endif // else VICON_STREAM_DEBUG
      n++;
      }
      millisleep(500);
  }
  std::cout << std::endl;
  std::cout << "ViconCLient is connected: success!" << std::endl;

  // Enable some different data types
  ViconClient.EnableSegmentData();
  ViconClient.EnableMarkerData();
  ViconClient.EnableUnlabeledMarkerData();

  // Set the streaming mode
  ViconClient.SetStreamMode( ViconDataStreamSDK::CPP::StreamMode::ClientPull );

  // Set the global up axis
  ViconClient.SetAxisMapping( Direction::Forward,
                           Direction::Left,
                           Direction::Up ); // Z-up
  // Discover the version number
//  Output_GetVersion _Output_GetVersion = ViconClient.GetVersion();
//  std::cout << "Version: " << _Output_GetVersion.Major << "."
//                           << _Output_GetVersion.Minor << "."
//                           << _Output_GetVersion.Point << std::endl;

#endif // (!TCP_CONNECT_DEBUG && !MAVLINK_DEBUG)

  /********************************************/
  /*  configure and enable TCP/IP connection  */
  /********************************************/
#if !VICON_STREAM_DEBUG
  int mySocket;
  // create socket
  if ((mySocket = socket(AF_INET, SOCK_STREAM, 0)) < 0) {
      std::cout << "cannot create socket!" << std::endl;
      return 0;
  }

  // adress of my socket
  sockaddr_in myaddr;
  myaddr.sin_family = AF_INET;
  myaddr.sin_port = htons(0);
  myaddr.sin_addr.s_addr = htonl(MY_NET_ADDRESS);

  // bind socket
  if (bind(mySocket, (struct sockaddr *)&myaddr, sizeof(myaddr)) < 0) {
      std::cout << "bind socket failed" << std::endl;
      return 0;
  }
  else std::cout << "bind socket successful" << std::endl;

  // adress of TCP2Serial socket
  sockaddr_in servaddr;
  servaddr.sin_family = AF_INET;
  servaddr.sin_port = htons(TCP2SERIAL_PORT);
  servaddr.sin_addr.s_addr = inet_addr(TCP2SERIAL_ADRESS);

  // connect to TCP2Serial Server
  if (connect(mySocket, (struct sockaddr *)&servaddr, sizeof(servaddr)) < 0) {
      std::cout << "failed to connect to TCP2Serial server" << std::endl;
      return 0;
  }
  else std::cout << "connection to TCP2Serial server successful" << std::endl;




  /***************************************/
  /* calculate Transformation Quaternion */
  /***************************************/
  float rotation_x_axis_rad[3] = {PI, 0, 0};
  float rotation_z_axis_rad[3] = {0, 0, ROOM_ALIGNMENT};

  float rotation_x_axis_quat[4];
  float rotation_z_axis_quat[4];

  quat_from_axisrad(rotation_x_axis_quat, rotation_x_axis_rad);
  quat_from_axisrad(rotation_z_axis_quat, rotation_z_axis_rad);

#endif // !VICON_STREAM_DEBUG



  bool ViconClientListens = true;
  unsigned char mavlink_seq_no = 0x00;

  int SubjectIndex = -1;
  std::string SubjectName;

  unsigned long long int time_last_loop = 0;
  unsigned int loop_time_desired = 1000 / FREQUENCY; // desired looptime in miliseconds

  std::cout << "Main loop starts now!" << std::endl;

  while( ViconClientListens )
  {
          // wait for some time, so that the actual frequency isn't faster than FREQUENCY
          while (time_last_loop > getMSecondoftheday() - loop_time_desired) {
                millisleep(loop_time_desired/20);
          }
          time_last_loop = getMSecondoftheday();

  #if (!TCP_CONNECT_DEBUG && !MAVLINK_DEBUG)
    #if VICON_STREAM_DEBUG
          std::cout << "ViconClient tries to get a frame ..." << std::flush;
    #endif // VICON_STREAM_DEBUG

         // Get a frame
         int m = 0;
         while( ViconClient.GetFrame().Result != Result::Success )
         {
                 // Sleep a little so that we don't lumber the CPU with a busy poll
                 millisleep( 100 ); // sleep for 100 ms
                 m++;
                 if (m >= 10)
                 {
                         std::cout << "." << std::flush;
                         m = 0;
                 }
         }
         std::cout << std::endl;
    #if VICON_STREAM_DEBUG
         std::cout << "ViconClient got a new frame: success!" << std::endl;
    #endif // VICON_STREAM_DEBUG
         // create array for relevant Data
         float data_mavlinkMsgNo138[7] = { };

         if (SubjectIndex == -1)
             std::cout << "Searching for SubjectIndex of: " << UAV_NAME << std::endl;

         m = 0;
         while (SubjectIndex == -1) {
                SubjectName = ViconClient.GetSubjectName(m).SubjectName;
                if (SubjectName == UAV_NAME) {
                        SubjectIndex = m;
                        std::cout << "SubjectIndex found: " << UAV_NAME << " has #" << SubjectIndex << std::endl;
                }
                ++m;
         }

        // std::cout << "  Subject #" << SubjectIndex << std::endl;
        // std::string SubjectName = ViconClient.GetSubjectName( SubjectIndex ).SubjectName; // Get the subject name

         // Count the number of segments
         unsigned int SegmentCount = ViconClient.GetSegmentCount( SubjectName ).SegmentCount;
         std::cout << "    Segments (" << SegmentCount << "):" << std::endl;
         for( unsigned int SegmentIndex = 0 ; SegmentIndex < SegmentCount ; ++SegmentIndex )
         {
                std::cout << "      Segment #" << SegmentIndex << std::endl;
                // Get the segment name
                std::string SegmentName = ViconClient.GetSegmentName( SubjectName, SegmentIndex ).SegmentName;
//                  std::cout << "          Name: " << SegmentName << std::endl;

                // Get the global segment translation
                Output_GetSegmentGlobalTranslation _Output_GetSegmentGlobalTranslation =
                        ViconClient.GetSegmentGlobalTranslation( SubjectName, SegmentName );

                data_mavlinkMsgNo138[0] = _Output_GetSegmentGlobalTranslation.Translation[ 0 ];
                data_mavlinkMsgNo138[1] = _Output_GetSegmentGlobalTranslation.Translation[ 1 ];
                data_mavlinkMsgNo138[2] = _Output_GetSegmentGlobalTranslation.Translation[ 2 ];

                // Get the global segment rotation in quaternion co-ordinates
                Output_GetSegmentGlobalRotationQuaternion _Output_GetSegmentGlobalRotationQuaternion =
                       ViconClient.GetSegmentGlobalRotationQuaternion( SubjectName, SegmentName );

                data_mavlinkMsgNo138[3] = _Output_GetSegmentGlobalRotationQuaternion.Rotation[ 0 ];
                data_mavlinkMsgNo138[4] = _Output_GetSegmentGlobalRotationQuaternion.Rotation[ 1 ];
                data_mavlinkMsgNo138[5] = _Output_GetSegmentGlobalRotationQuaternion.Rotation[ 2 ];
                data_mavlinkMsgNo138[6] = _Output_GetSegmentGlobalRotationQuaternion.Rotation[ 3 ];
                // Attention: order of variables is different then in mavlink! In Vicon it is: (x_quat,y_quat,z_quat,w_quat)

                // if segment was not present at this frame then Occluded = false
                // w = 1 then, so the quaternion doesn't represent a rotation
                if (!_Output_GetSegmentGlobalRotationQuaternion.Occluded)
                    data_mavlinkMsgNo138[6] = 1;

                #if VICON_STREAM_DEBUG
                    std::cout << "received Vicon positions:" << std::endl;
                    std::cout << "     pos: x: " << data_mavlinkMsgNo138[0] << std::endl;
                    std::cout << "          y: " << data_mavlinkMsgNo138[1] << std::endl;
                    std::cout << "          z: " << data_mavlinkMsgNo138[2] << std::endl;
                    std::cout << "     quat: w: " << data_mavlinkMsgNo138[6] << std::endl;
                    std::cout << "           x: " << data_mavlinkMsgNo138[3] << std::endl;
                    std::cout << "           y: " << data_mavlinkMsgNo138[4] << std::endl;
                    std::cout << "           z: " << data_mavlinkMsgNo138[5] << std::endl;
                #endif // VICON_DEBUG

                // Get the unlabeled markers
                unsigned int UnlabeledMarkerCount = ViconClient.GetUnlabeledMarkerCount().MarkerCount;
                std::cout << "  Unlabeled Markers (" << UnlabeledMarkerCount << "):" << std::endl;

    #endif // (!TCP_CONNECT_DEBUG && !MAVLINK_DEBUG)
    #if MAVLINK_DEBUG
                float data_mavlinkMsgNo138[7] = {500, 500, 500, 0, 0, 0, 1}; // some test data for MavLink, still in Vicon-coordinate order
                // (x, y, z, x_quat, y_quat, z_quat, w_quat)
    #endif // MAVLINK_DEBUG

                /* create MavLink Data */
    #if (!VICON_STREAM_DEBUG && !TCP_CONNECT_DEBUG)
                // Vicon Data is in milli meters, PX4 calculates in NED and meters
                // and change the order of the variables from Vicon -> att_pos_mocap
                // (x, y, z, x_quat, y_quat, z_quat, w_quat) -> (w_quat, x_quat, y_quat, z_quat, x, y, z)
                coordinateTrans_ViconToNED(data_mavlinkMsgNo138, rotation_x_axis_quat, rotation_z_axis_quat);

                std::cout << "calculated positions, after coordinate Transformation:" << std::endl;
                std::cout << "  pos (NED): x: " << data_mavlinkMsgNo138[4] << std::endl;
                std::cout << "             y: " << data_mavlinkMsgNo138[5] << std::endl;
                std::cout << "             z: " << data_mavlinkMsgNo138[6] << std::endl;
                std::cout << "     quat: w: " << data_mavlinkMsgNo138[0] << std::endl;
                std::cout << "           x: " << data_mavlinkMsgNo138[1] << std::endl;
                std::cout << "           y: " << data_mavlinkMsgNo138[2] << std::endl;
                std::cout << "           z: " << data_mavlinkMsgNo138[3] << std::endl;

                mavlink_message_t msg; // defined in mavlink_types.h

                float attitude_quat[4] = {data_mavlinkMsgNo138[0], data_mavlinkMsgNo138[1],
                                        data_mavlinkMsgNo138[2], data_mavlinkMsgNo138[3]};

                mavlink_msg_att_pos_mocap_pack(1, 200, &msg, getMicroSecondoftheday(), attitude_quat,
                             data_mavlinkMsgNo138[4], data_mavlinkMsgNo138[5], data_mavlinkMsgNo138[6]);

//                mavlink_msg_att_pos_mocap_pack(1, 200, &msg, 0, attitude_quat,
//                             data_mavlinkMsgNo138[4], data_mavlinkMsgNo138[5], data_mavlinkMsgNo138[6]);
              // defined in mavlink_msg_att_pos_mocap.h

               // unsigned char Buffer[BUFFER_LENGTH];
                unsigned char Buffer[MAVLINK_MAX_PACKET_LEN];
               // pack_vicon2mav138Buffer(Buffer, msg, 187);
                uint16_t len = mavlink_msg_to_send_buffer(Buffer, &msg);

                std::cout << "@MAV: ViconClientListens: " << ViconClientListens << std::endl;

      #if MAVLINK_DEBUG
//                if (mavlink_seq_no > 1) {
//                       ViconClientListens = 0;        // stop after 3 messages
//                }
      #endif // MAVLINK_DEBUG

    #endif // (!VICON_STREAM_DEBUG && !TCP_CONNECT_DEBUG)

     /*################### send MavLink Data wit TCP ####################*/

    #if !VICON_STREAM_DEBUG
      #if TCP_CONNECT_DEBUG
                unsigned char Buffer[3]; //some data for TCP_CONNECT_DEBUG
                Buffer[0] = 0x41;
                Buffer[1] = 0x42;
                Buffer[2] = 0x43;

                if(write(mySocket, &Buffer, 3) < 0)
                        std::cout << "write failed! Sequence No: " << (int)mavlink_seq_no << std::endl;
                else
                        std::cout << "write Socket successful, Sequence No: " << (int)mavlink_seq_no << std::endl;

                if (mavlink_seq_no > 1)
                        ViconClientListens = 0;        // stop after 3 messages
      #else // TCP_CONNECT_DEBUG

                // send the data
                std::cout << "@TCP: ViconClientListens: " << ViconClientListens << std::endl;

                if(write(mySocket, &Buffer, len) < 0)
                        std::cout << "write failed! Sequence No: " << (int)mavlink_seq_no << std::endl;
                else
                        std::cout << "write Socket successful, Sequence No: " << (int)mavlink_seq_no << std::endl;

                std::cout << "sent message: message content of SeqNo: " << (int)mavlink_seq_no << std::endl;
                for(int i = 0; i < len; i++) {
                      if (10 > (unsigned int)Buffer[i])
                                std::cout << " |   " << (unsigned int)Buffer[i];
                      else if (100 > (unsigned int)Buffer[i])
                                std::cout << " |  " << (unsigned int)Buffer[i];
                      else
                                std::cout << " | " << (unsigned int)Buffer[i];
                }
                std::cout << " | " << std::endl;

      #endif // else TCP_CONNECT_DEBUG
    #endif // !VICON_STREAM_DEBUG

                ++mavlink_seq_no; //messagecounter +1
    #if (!TCP_CONNECT_DEBUG && !MAVLINK_DEBUG)
         } // end of for( unsigned int SegmentIndex = 0 ; SegmentIndex < SegmentCount ; ++SegmentIndex ) - loop
    #endif // (!TCP_CONNECT_DEBUG && !MAVLINK_DEBUG)
  } //end of while( ViconClientListens ) - Loop

#if (!TCP_CONNECT_DEBUG && !MAVLINK_DEBUG)

  if( TransmitMulticast )
  {
      ViconClient.StopTransmittingMulticast();
  }

  ViconClient.DisableSegmentData();
  ViconClient.DisableMarkerData();
  ViconClient.DisableUnlabeledMarkerData();
  ViconClient.DisableDeviceData();

  // Disconnect and dispose
  unsigned long long int t = getMSecondoftheday();
  std::cout << " Disconnecting..." << std::endl;
  ViconClient.Disconnect();
  int dt = getMSecondoftheday() - t;
  double secs = (double) (dt)/(double)CLOCKS_PER_SEC;
  std::cout << " Disconnect time = " << secs << " secs" << std::endl;
#endif // (!TCP_CONNECT_DEBUG && !MAVLINK_DEBUG)

}


















