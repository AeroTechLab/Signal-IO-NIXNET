//////////////////////////////////////////////////////////////////////////////////////////
//                                                                                      //
//  Original work Copyright (c) 2012 Guilherme Fernandes                                //
//  Modified work Copyright (c) 2016-2017 Leonardo Consoni <consoni_2519@hotmail.com>   //
//                                                                                      //
//  This file is part of RobRehabSystem.                                                //
//                                                                                      //
//  RobRehabSystem is free software: you can redistribute it and/or modify              //
//  it under the terms of the GNU Lesser General Public License as published            //
//  by the Free Software Foundation, either version 3 of the License, or                //
//  (at your option) any later version.                                                 //
//                                                                                      //
//  RobRehabSystem is distributed in the hope that it will be useful,                   //
//  but WITHOUT ANY WARRANTY; without even the implied warranty of                      //
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the                        //
//  GNU Lesser General Public License for more details.                                 //
//                                                                                      //
//  You should have received a copy of the GNU Lesser General Public License            //
//  along with RobRehabSystem. If not, see <http://www.gnu.org/licenses/>.              //
//                                                                                      //
//////////////////////////////////////////////////////////////////////////////////////////


#ifndef CAN_NETWORK_H
#define CAN_NETWORK_H

#include "can_frame.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "klib/khash.h"

enum CANFrameTypes { SDO, PDO01, PDO02, CAN_FRAME_TYPES_NUMBER };

//CAN database addressing
const char* CAN_DATABASE_NAME = "database";
const char* CAN_CLUSTER_NAME = "NETCAN";

// Network control frames
static CANFrame NMT = NULL;
static CANFrame SYNC = NULL;

KHASH_MAP_INIT_INT( FrameInt, CANFrame )
static khash_t( FrameInt )* framesList = NULL;

void CANNetwork_Reset();

void CANNetwork_Start()
{
  // Address and initialize NMT (Network Master) frame
  NMT = CANFrame_Init( FRAME_OUT, "CAN2", CAN_DATABASE_NAME, CAN_CLUSTER_NAME, "NMT" );
  // Address and initialize SYNC (Syncronization) frame
  SYNC = CANFrame_Init( FRAME_OUT, "CAN2", CAN_DATABASE_NAME, CAN_CLUSTER_NAME, "SYNC" );
  
  framesList = kh_init( FrameInt );

  CANNetwork_Reset();
}

// Stop CAN network communications
void CANNetwork_Stop()
{
  // Stop PDOs sending Stop payload to the network
  u8 payload[8] = { 0x80 }; // Rest of the array as 0x0
  CANFrame_Write( NMT, payload );

  kh_destroy( FrameInt, framesList );
  framesList = NULL;
  
  CANFrame_End( NMT );
  CANFrame_End( SYNC );
}

void CANNetwork_Reset()
{
  u8 payload[8] = { 0x82 }; // Rest of the array as 0x0
  CANFrame_Write( NMT, payload );
  
  Timing.Delay( 200 );
  
  payload[0] = 0x01; // Rest of the array as 0x0
  CANFrame_Write( NMT, payload );
}

void CANNetwork_InitNode( uint8_t nodeID )
{
  // Start PDOs sending Start payload to the network
  u8 payload[8] = { 0x01, nodeID }; // Rest of the array as 0x0
  CANFrame_Write( NMT, payload );
}

void CANNetwork_EndNode( uint8_t nodeID )
{
  // Stop PDOs sending Stop payload to the network
  u8 payload[8] = { 0x80, nodeID }; // Rest of the array as 0x0
  CANFrame_Write( NMT, payload );
}

void CANNetwork_ResetNodes()
{
  u8 payload[8] = { 0x81 };//{ 0x821 }; // Rest of the array as 0x0
  CANFrame_Write( NMT, payload );
}

const size_t ADDRESS_MAX_LENGTH = 16;
const char* CAN_FRAME_NAMES[ CAN_FRAME_TYPES_NUMBER ] = { "SDO", "PDO01", "PDO02" };
CANFrame CANNetwork_InitFrame( enum CANFrameTypes type, enum CANFrameMode mode, unsigned int nodeID )
{
  char frameAddress[ ADDRESS_MAX_LENGTH ];
  
  DEBUG_PRINT( "Trying to create %s frame (type %d) on node %u", CAN_FRAME_NAMES[ type ], type, nodeID );
  
  if( type >= CAN_FRAME_TYPES_NUMBER ) return NULL;
  
  const char* interfaceName = ( mode == FRAME_IN ) ? "CAN1" : "CAN2";
  const char* modeName = ( mode == FRAME_IN ) ? "RX" : "TX";
  
  int frameKey = ( type << 16 ) + ( mode << 8 ) + nodeID;
  
  snprintf( frameAddress, ADDRESS_MAX_LENGTH, "%s_%s_%02u", CAN_FRAME_NAMES[ type ], modeName, nodeID );
  
  DEBUG_PRINT( "creating frame %s on mode %d (key: %d)", frameAddress, mode, frameKey );
  
  if( framesList == NULL ) CANNetwork_Start();
  
  int insertionStatus;
  khint_t newFrameID = kh_put( FrameInt, framesList, frameKey, &insertionStatus );
  if( insertionStatus > 0 )
  {
    kh_value( framesList, newFrameID ) = CANFrame_Init( mode, interfaceName, CAN_DATABASE_NAME, CAN_CLUSTER_NAME, frameAddress );
    if( kh_value( framesList, newFrameID ) == NULL )
    {
      DEBUG_PRINT( "error creating frame %s for CAN interface %s", frameAddress, interfaceName );
      kh_del( FrameInt, framesList, newFrameID );
      return NULL;
    }
  }

  return kh_value( framesList, newFrameID );
}

void CANNetwork_EndFrame( CANFrame frame )
{
  for( khint_t frameID = 0; frameID != kh_end( framesList ); frameID++ )
  {
    if( !kh_exist( framesList, frameID ) ) continue;
    
    if( kh_value( framesList, frameID ) == frame )
    {
      CANFrame_End( frame );
      kh_del( FrameInt, framesList, frameID );
      
      if( kh_size( framesList ) == 0 ) CANNetwork_Stop();
      
      break;
    }
  }
}

void CANNetwork_Sync()
{
  // Build Sync payload (all 0x0) 
  static u8 payload[ 8 ];
  CANFrame_Write( SYNC, payload );
}

int CANNetwork_ReadSingleValue( CANFrame requestFrame, CANFrame readFrame, uint16_t index, uint8_t subIndex )
{
  // Build read requisition buffer for defined value
  static u8 payload[ 8 ];
  
  payload[ 0 ] = 0x40; 
  payload[ 1 ] = (uint8_t) ( index & 0x000000FF );
  payload[ 2 ] = (uint8_t) ( ( index & 0x0000FF00 ) / 0x100 );
  payload[ 3 ] = subIndex;
  payload[ 4 ] = 0x00;
  payload[ 5 ] = 0x00;
  payload[ 6 ] = 0x00;
  payload[ 7 ] = 0x00;

  // Write value requisition to SDO frame 
  CANFrame_Write( requestFrame, payload );

  //Timing.Delay( 100 );

  // Read requested value from SDO frame
  CANFrame_Read( readFrame, payload );

  int value = payload[ 7 ] * 0x1000000 + payload[ 6 ] * 0x10000 + payload[ 5 ] * 0x100 + payload[4];

  return value;
}

void CANNetwork_WriteSingleValue( CANFrame writeFrame, uint16_t index, uint8_t subIndex, int value )
{
  // Build read requisition buffer for defined value
  static u8 payload[ 8 ];
  
  payload[ 0 ] = 0x22; 
  payload[ 1 ] = (uint8_t) ( index & 0x000000FF );
  payload[ 2 ] = (uint8_t) ( ( index & 0x0000FF00 ) / 0x100 );
  payload[ 3 ] = subIndex;
  payload[ 4 ] = (uint8_t) ( value & 0x000000FF );
  payload[ 5 ] = (uint8_t) ( ( value & 0x0000FF00 ) / 0x100 );
  payload[ 6 ] = (uint8_t) ( ( value & 0x00FF0000 ) / 0x10000 );
  payload[ 7 ] = (uint8_t) ( ( value & 0xFF000000 ) / 0x1000000 );

  // Write value requisition to SDO frame 
  CANFrame_Write( writeFrame, payload );
}

#endif	/* CAN_NETWORK_H */

