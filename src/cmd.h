#ifndef _cmd_h
#define _cmd_h

#include "usart.h"

#define CMD_COMMAND_ECHO 1
#define CMD_COMMAND_CALIBRATE 2
#define CMD_COMMAND_START_TIMEOUT 3

#define CMD_ERROR_OK 0
#define CMD_ERROR_UNKNOWN_COMMAND 1
#define CMD_ERROR_ARGUMENT 2
#define CMD_ERROR_FORMAT 3
#define CMD_ERROR_BUFFER_OVERFLOW 4
/**
   Decode and execute commands.

   cmd_buffer.data:

   ---> | ID | STATUS | ARG0 | ... | ARGN |

   rsp_buffer.data:

   <--- | ID | STATUS | ARG0 | ... | ARGN |


   STATUS of command  is used for future use.
**/
int cmd_dispatcher(const buffer_t *cmd_buffer);

/**
   Used for parsing timeout command.

   ARG0: LSB of timeout value.
   ARG1: MSB of timeout value.

   @param rsp_buffer Buffer for response.
   @param timeout Here timeout value is returned.
   @param cmd_buffer Buffer for command.
   @return Error code. 
 */
int cmd_command_start_timeout(buffer_t *rsp_buffer, uint16_t *timeout, const buffer_t *cmd_buffer);

int cmd_command_calibrate(buffer_t *rsp_buffer, uint16_t *temperature, const buffer_t *cmd_buffer);

int cmd_command_echo(buffer_t *rsp_buffer, const buffer_t *cmd_buffer);

int cmd_format_error_message(buffer_t *buffer, uint8_t error_message);

#endif //_cmd_h
