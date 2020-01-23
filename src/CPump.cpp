#include "CPump.h"

CPump::CPump(const char* portName,float maxForward,float maxBackward, float maxTurn, float fwGain,float turnGain)
{
	initialized = false;

	//initialize and setup serial connection
	commDelay = 20000;
	char cfgStr[1000];
	sprintf(cfgStr,"stty -F %s 57600 -parenb -parodd cs8 -hupcl -cstopb cread clocal -crtscts -ignbrk -brkint -ignpar -parmrk inpck -istrip -inlcr -igncr -icrnl -ixon -ixoff  -iuclc -ixany -imaxbel -opost -olcuc -ocrnl -onlcr -onocr -onlret -ofill -ofdel nl0 cr0 tab0 bs0 vt0   ff0 -isig -icanon -iexten -echo -echoe -echok -echonl -noflsh -xcase -tostop  -echoprt -echoctl -echoke",portName);
	system(cfgStr);			
	port = open(portName, O_RDWR|O_NOCTTY);
	if (port == -1){
		fprintf(stderr,"Cannot open port %s\n,",portName);
		fprintf(stderr,"Error initializing the robot\n");
	}
	initialized = (port != -1);
}


CPump::~CPump()
{
}

int CPump::on()
{
	unsigned char buffer = 31;
	write(port, &buffer, 1);
	return 0;
}

int CPump::off()
{
	unsigned char buffer = 30;
	write(port, &buffer, 1);
	return 0;
}
