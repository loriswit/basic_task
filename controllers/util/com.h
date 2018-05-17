#ifndef ROBOTICS_COM_H
#define ROBOTICS_COM_H

#include <stdlib.h>

#define COM_CHANNEL 1

typedef struct
{
    void * data;
    size_t size;
} packet_t;

/**
 * Initializes the emitter and the receiver.
 */
void com_init();

/**
 * Sends a packet to others robots.
 *
 * @param p A packet structure containing a pointer to the data and the size of the data
 * @return TRUE if packet has been sent, FALSE if not
 */
bool com_send(packet_t p);

/**
 * Receives a packet from others robots.
 *
 * @return A packet structure containing a pointer to the data and the size of the data
 *          (note: the packet data has to be freed manually)
 */
packet_t com_receive();

#endif //ROBOTICS_COM_H
