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

void MyErrorMsgHandler ( int iLevel, const char *szMsg );
void InitializeCortexHandlers();
int InitializeCortexConnection( char local[], char cortex[] );
void GetCortexFrameRate();
int GetKnownBodies ( char** &bodies );
int FindBodyFrameIndex ( sFrameOfData* FrameOfData, const char* body );
int FindBodyIndex ( const char* body, int num_bodies, char** bodies );
