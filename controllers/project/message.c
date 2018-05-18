#include "../util/com.h"

#include "message.h"

void message_send(message_t msg)
{
    packet_t message;
    message.data = (message_t[]) {msg};
    message.size = sizeof(msg);
    
    com_send(message);
}

message_t message_receive()
{
    packet_t message = com_receive();
    if(message.data == NULL)
        return MESSAGE_NONE;
    
    message_t value = *(message_t *) message.data;
    free(message.data);
    
    return value;
}
