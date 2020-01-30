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
		CPump(const char* portName);
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
