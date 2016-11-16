/*
		Copyright 2015 Tyler Sorey, ARL, University of Nevada, Reno, USA

    This file is part of cortex_bridge.

    cortex_bridge is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    cortex_bridge is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with Foobar.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "cortex.h"
#include "ros/ros.h"
#include <stdio.h>

int FindBodyFrameIndex ( sFrameOfData* FrameOfData, const char* body )
{
	// initialize variables
	int numBodies = FrameOfData->nBodies;
	for ( int i = 0; i < numBodies; ++i )
  {
		if ( strcmp(body, FrameOfData->BodyData[i].szName) == 0 )
			return i;
	}
	return -1;
}

int FindBodyIndex ( const char* body, int num_bodies, char** bodies )
{
	for ( int i = 0; i < num_bodies; i++ )
	{
		// make sure we have the correct body
		if ( strcmp(body, bodies[i]) == 0 )
			return i;
	}
	return -1;
}

int GetKnownBodies ( char** &bodies )
{
  sBodyDefs* pBodyDefs = NULL;
  pBodyDefs = Cortex_GetBodyDefs();

  // No body definitions
  if (pBodyDefs == NULL) 
  {
    ROS_ERROR ("Failed to get body defs.");
		return -1;
  }
  // Body Def info received from Cortex
  else 
  {
    int numBodies = pBodyDefs -> nBodyDefs;
    if ( numBodies > 0 )
    {
			bodies = new char*[numBodies];
      for ( int i = 0; i < numBodies; i++ )
      {
				// store the body name (assume no bodies are longer than 50 chars)
				bodies[i] = new char[50];
				strcpy(bodies[i], pBodyDefs -> BodyDefs[i].szName);
      }
			// return the num bodies
			return numBodies;
    }
    // Free Body Def data
    Cortex_FreeBodyDefs(pBodyDefs);
    pBodyDefs = NULL;
  }
	return -1;
}

void MyErrorMsgHandler(int iLevel, const char *szMsg)
{
  const char *szLevel = NULL;

  if (iLevel == VL_Debug) {
    szLevel = "Debug";
  } else if (iLevel == VL_Info) {
    szLevel = "Info";
  } else if (iLevel == VL_Warning) {
    szLevel = "Warning";
  } else if (iLevel == VL_Error) {
    szLevel = "Error";
  }

  ROS_INFO ("    %s: %s\n", szLevel, szMsg);
}

void InitializeCortexHandlers()
{
  unsigned char SDK_Version[4];

  Cortex_SetVerbosityLevel(VL_Error);
  Cortex_GetSdkVersion(SDK_Version);
  ROS_INFO ("Using SDK Version %d.%d.%d.%d", SDK_Version[0], SDK_Version[1], SDK_Version[2], SDK_Version[3]);

  Cortex_SetErrorMsgHandlerFunc(MyErrorMsgHandler);
}

int InitializeCortexConnection( char local[], char cortex[] )
{
	if( Cortex_Initialize(local, cortex) != RC_Okay )
	{
		Cortex_Exit();
		return -1;
	}
	ROS_INFO ( "Cortex Connection Initialized" );
	return 0;
}

void GetCortexFrameRate()
{
  void *pResponse;
  int nBytes;

  if ( Cortex_Request("GetContextFrameRate", &pResponse, &nBytes) != RC_Okay )
    ROS_WARN ("ERROR, GetContextFrameRate");

  float *contextFrameRate = (float*) pResponse;
  ROS_INFO ("ContextFrameRate = %3.1f Hz", *contextFrameRate);
}
