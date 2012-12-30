##
##  dh8 
##
##  Control software for the Diavite DH-8 surface roughness measurement 
##  device.
##
##
##  Copyright (C) 2012 Erwin Nindl <nine-dh8@wirdorange.org>
##
##  This file is part of DH8.
##
##  DH8 is free software: you can redistribute it and/or modify
##  it under the terms of the GNU General Public License as published by
##  the Free Software Foundation, either version 3 of the License, or
##  any later version.
##
##  DH8 is distributed in the hope that it will be useful,
##  but WITHOUT ANY WARRANTY; without even the implied warranty of
##  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
##  GNU General Public License for more details.
##
##  You should have received a copy of the GNU General Public License
##  along with DH8.  If not, see <http://www.gnu.org/licenses/>.
##
##--------------------------------------------------------

CC=gcc
RM=rm -f
CFLAGS=-Wall -I$(CLSOCKET_HEADER) 
LDFLAGS=
LIBS=-L$(CLSOCKET_LIB) -lclsocket -lboost_program_options

#--------------------------------------------------------

OBJECTS    = main.o Serial.o SerialCmd.o
HEADER     = Serial.h SerialCmd.h 
EXECUTABLE = dh8

#--------------------------------------------------------

all::   $(EXECUTABLE)

clean:  
	$(RM) $(OBJECTS)
	$(RM) $(EXECUTABLE)
	$(RM) *~

$(OBJECTS): $(HEADER) $(OBJECTS:.o=.cpp)
	-$(CC) -c $(CFLAGS) $(OBJECTS:.o=.cpp)

$(EXECUTABLE): $(OBJECTS)
	-$(CC) $(LDFLAGS) $(OBJECTS) -o $(EXECUTABLE) $(LIBS)
