#include <stdio.h>
#include <iostream>

#include "serialize.h"

using namespace std;

u_char buf[100]; // already a pointer


unsigned int PACKET_OK = 0, PACKET_BAD_CHECKSUM = 1;



unsigned int serialize(void *p, size_t size)
{
    char checksum = 0;
    buf[0]=size;
    memcpy(buf+1, p, size);
    for(int i=1; i<=size; i++)
    {
        checksum ^= buf[i];
    }
    cout << "Checksum: " << (int)checksum << endl;
    buf[size+1]=checksum; // Last packet will be the checksum
    return size+2;
}

void sendSerialData(u_char *buffer, int len)
{
    for(int i=0; i<len; i++)
        cout << (int)buffer[i] << " ";
    cout << endl;
}


unsigned int deserialize(void *p, u_char *buf)
{
    size_t size = buf[0];
    char checksum = 0;
    
    for(int i=1; i<=size; i++)
        checksum ^= buf[i];
    
    cout << "Checksum: " << (int)checksum << endl;
    
    if(checksum == buf[size+1])
    {
        memcpy(p, buf+1, size);
        return PACKET_OK;
    }
    else
    {
        printf("CHECKSUM ERROR\n");
        return PACKET_BAD_CHECKSUM;
    }
}

/*
int main () {
    
    // I can send any struct I want
    // TODO: Put any struct I want
    struct test {
        int x;
        int y;
        string s;
    };

    test testPacket, receivePacket;
    testPacket.y = 1000;
    testPacket.x = 2000;
    testPacket.s = "Hello world";
    
    
    int len = serialize(&testPacket, sizeof(test));
    cout << "Packet length: " << len << endl;
    sendSerialData(buf, len); // In this case sending just means printing
    
    
    unsigned int ack = deserialize(&receivePacket, buf);

    
    
    if (!ack) cout << "Packet OK!" << endl;
    else cout << "Packet NOT OK!" << endl;
    
    cout << "Received string: " << receivePacket.s << endl;

}
*/
