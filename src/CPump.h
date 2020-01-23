#ifndef __CROBOT_H__
#define __CROBOT_H__

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>

class CPump
{
	public:
		CPump(const char* portName,float maxForward = 0.5,float maxBackward = 0.5, float maxTurn = 0.5, float fwGain = 10,float turnGain = 10);
		~CPump();

		//set robot speeds
		int on();
		int off();

	protected:
		//comm port settings etc
		unsigned char buffer[100];
		int port;
		bool initialized;
		int commDelay;
};

#endif
