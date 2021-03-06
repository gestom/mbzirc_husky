#include "CPump.h"

CPump::CPump(const char* portName)
{
	initialized = false;

	//initialize and setup serial connection
	commDelay = 20000;
	char cfgStr[1000];
	//sprintf(cfgStr,"stty -F %s",portName);;// 115200 -parenb -parodd cs8 -hupcl -cstopb cread clocal -crtscts -ignbrk -brkint -ignpar -parmrk inpck -istrip -inlcr -igncr -icrnl -ixon -ixoff  -iuclc -ixany -imaxbel -opost -olcuc -ocrnl -onlcr -onocr -onlret -ofill -ofdel nl0 cr0 tab0 bs0 vt0   ff0 -isig -icanon -iexten -echo -echoe -echok -echonl -noflsh -xcase -tostop  -echoprt -echoctl -echoke",portName);
	//sprintf(cfgStr,"stty -F %s 115200 -parenb -parodd cs8 -hupcl -cstopb cread clocal -crtscts -ignbrk -brkint -ignpar -parmrk inpck -istrip -inlcr -igncr -icrnl -ixon -ixoff  -iuclc -ixany -imaxbel -opost -olcuc -ocrnl -onlcr -onocr -onlret -ofill -ofdel nl0 cr0 tab0 bs0 vt0   ff0 -isig -icanon -iexten -echo -echoe -echok -echonl -noflsh -xcase -tostop  -echoprt -echoctl -echoke",portName);
	//system(cfgStr);			
	port = open(portName, O_RDWR|O_NOCTTY);
	if (port == -1){
		fprintf(stderr,"Cannot open port %s\n,",portName);
		fprintf(stderr,"Error initializing the robot\n");
	}
	initialized = (port != -1);
}


CPump::~CPump()
{
	off();
}

int CPump::on()
{
	unsigned char buffer[4] = {0x62,0x01,0x91,0xf4};
	write(port, buffer, 4);
	return 0;
}

int CPump::off()
{
	unsigned char buffer[4] = {0x62,0x01,0x90,0xf3};
	write(port, buffer, 4);
	return 0;
}
