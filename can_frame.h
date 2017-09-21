//////////////////////////////////////////////////////////////////////////////////////////
//                                                                                      //
//  Original work Copyright (c) 2012 Guilherme Fernandes                                //
//  Modified work Copyright (c) 2016-2017 Leonardo Consoni <consoni_2519@hotmail.com>   //
//                                                                                      //
//  This file is part of Signal-IO-NIXNET.                                              //
//                                                                                      //
//  Signal-IO-NIXNET is free software: you can redistribute it and/or modify            //
//  it under the terms of the GNU Lesser General Public License as published            //
//  by the Free Software Foundation, either version 3 of the License, or                //
//  (at your option) any later version.                                                 //
//                                                                                      //
//  Signal-IO-NIXNET is distributed in the hope that it will be useful,                 //
//  but WITHOUT ANY WARRANTY; without even the implied warranty of                      //
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the                        //
//  GNU Lesser General Public License for more details.                                 //
//                                                                                      //
//  You should have received a copy of the GNU Lesser General Public License            //
//  along with Signal-IO-NIXNET. If not, see <http://www.gnu.org/licenses/>.            //
//                                                                                      //
//////////////////////////////////////////////////////////////////////////////////////////


#ifndef CAN_FRAME_H
#define	CAN_FRAME_H

#ifdef _CVI_
  #include <nixnet.h>
#elif NIXNET
  #include "nixnet.h"
#else
  #include "nixnet_stub.h"
#endif

#include "debug/data_logging.h"

#define CAN_FRAME_ID_MAX_SIZE 16

enum CANFrameMode { FRAME_IN = nxMode_FrameInSinglePoint, FRAME_OUT = nxMode_FrameOutSinglePoint };

// CAN Frame data structure
typedef struct _CANFrameData
{
  nxSessionRef_t ref_session;
  char id[ CAN_FRAME_ID_MAX_SIZE ];
  u8 flags;
  u8 type;
  u8 buffer[ sizeof(nxFrameVar_t) ];
} 
CANFrameData;

typedef CANFrameData* CANFrame;

// Display CAN error string based on status code
static void PrintFrameStatus( nxStatus_t statusCode, const char* frameID, const char* source )
{
  static char statusString[ 1024 ];
    
  nxStatusToString( statusCode, sizeof(statusString), statusString );
  /*ERROR_EVENT*/DEBUG_PRINT( "%s - NI-XNET Status: %s", source, statusString );
}

// CAN Frame initializer
CANFrame CANFrame_Init( enum CANFrameMode mode, const char* interfaceName, const char* databaseName, const char* clusterName, const char* frameID )
{
  CANFrame frame = (CANFrame) malloc( sizeof(CANFrameData) );

  frame->flags = 0;
  frame->type = nxFrameType_CAN_Data;	//MACRO

  strcpy( frame->id, frameID );
  
  //DEBUG_PRINT( "creating frame %s of type %d and mode %d", frame->id, frame->type, mode );
  
  //Create an xnet session
  nxStatus_t statusCode = nxCreateSession( databaseName, clusterName, frameID, interfaceName, (u32) mode, &(frame->ref_session) );
  if( statusCode != nxSuccess )
  {
    DEBUG_PRINT( "error: %x", statusCode );
    PrintFrameStatus( statusCode, frameID, "(nxCreateSession)" );
    nxClear( frame->ref_session );
    return NULL;
  }
  
  //DEBUG_PRINT( "created frame %s session %u", frameID, frame->ref_session );

  return frame;
}

void CANFrame_End( CANFrame frame )
{
  if( frame != NULL )
  {
    nxClear( frame->ref_session );
    free( frame );
    frame = NULL;
  }
}

// Read data from CAN frame to array
void CANFrame_Read( CANFrame frame, u8 payload[8] )
{
  nxFrameVar_t* ptr_frame = (nxFrameVar_t*) frame->buffer;

  u32 temp;
    
  nxStatus_t statusCode = nxReadFrame( frame->ref_session, frame->buffer, sizeof(frame->buffer), 0, &temp );   
  if( statusCode != nxSuccess )
    PrintFrameStatus( statusCode, frame->id, "(nxReadFrame)" );
  else
    memcpy( payload, ptr_frame->Payload, sizeof(u8) * ptr_frame->PayloadLength );
}

// Write data from payload to CAN frame
void CANFrame_Write( CANFrame frame, u8 payload[8] )
{
  nxFrameVar_t* ptr_frame = (nxFrameVar_t*) frame->buffer;
  
  ptr_frame->Timestamp = 0;
  ptr_frame->Flags = frame->flags;
  ptr_frame->Identifier = 66; 
  ptr_frame->Type = frame->type;
  ptr_frame->PayloadLength= 8;

  memcpy( ptr_frame->Payload, payload, sizeof(u8) * ptr_frame->PayloadLength );

  //DEBUG_EVENT( 1,  "trying to write with session %u", frame->ref_session );
  
  nxStatus_t statusCode = nxWriteFrame( frame->ref_session, &(frame->buffer), sizeof(nxFrameVar_t), 0.0 );
  if( statusCode != nxSuccess )
    PrintFrameStatus( statusCode, frame->id, "(nxWriteFrame)" );
}

#endif	/* CAN_FRAME_H */

