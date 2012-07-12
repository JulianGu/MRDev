#include "mrcore/mrcore.h"

int main()
{
	VoiceTTSClient client;
	//client.connect("127.0.0.1",12000);
	client.connect("192.168.0.150",20000);
	while(1)
	{
		char buffer[512];
		cin.getline(buffer,512);
		string text(buffer);
		if(!client.say(text))
			break;
	}
	client.close();
}
