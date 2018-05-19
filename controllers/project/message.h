#ifndef BASIC_TASK_MESSAGE_H
#define BASIC_TASK_MESSAGE_H

/**
 * Enum constants representing messages to be sent to others robots.
 * <ul>
 *   <li><b>MESSAGE_FIND_RED</b>: indicates that the others robot must find a red spot.</li>
 *   <li><b>MESSAGE_FIND_BLUE</b>: indicates that the others robot must find a blue spot.</li>
 *   <li><b>MESSAGE_NONE</b>: indicates that no message has been received.</li>
 * </ul>
 */
typedef enum
{
    MESSAGE_FIND_RED, MESSAGE_FIND_BLUE, MESSAGE_STOP, MESSAGE_NONE
} message_t;

/**
 * Sends a message value to others robots.
 *
 * @param value The message to be sent
 */
void message_send(message_t msg);

/**
 * Receives a message sent by others robots.
 * This function only returns the first message of the queue.
 *
 * @return The received message. If there are no more messge in the queue, <b>MESSAGE_NONE</b> is returned.
 */
message_t message_receive();

#endif //BASIC_TASK_MESSAGE_H
