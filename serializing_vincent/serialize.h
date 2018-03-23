#include <stdlib.h>

unsigned int serialize(void *p, size_t size);
void sendSerialData(u_char *buffer, int len);
unsigned int deserialize(void *p, u_char *buf);
