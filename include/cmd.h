#ifndef _cmd_h
#define _cmd_h

#include "usart.h"
#include "buffer.h"

#define CMD_ERROR_OK 0
#define CMD_ERROR_UNKNOWN_COMMAND 1
#define CMD_ERROR_ARGUMENT 2
#define CMD_ERROR_FORMAT 3
#define CMD_ERROR_BUFFER_OVERFLOW 4
#define CMD_ERROR_UKNOWN_COMMAND 5

#define CMD_H

typedef int (*cmd_handler_t)(buffer_t *rsp_buffer, const buffer_t *cmd_buffer);

int cmd_register(uint8_t id, cmd_handler_t handler);

/**
   Decode and execute commands.

   cmd_buffer.data:

   ---> | ID | CNTRL  | ARG0 | ... | ARGN |

   rsp_buffer.data:

   <--- | ID | STATUS | ARG0 | ... | ARGN |


   STATUS of command  is used for future use.
**/
int cmd_dispatcher(buffer_t *rsp_buffer, const buffer_t *cmd_buffer);

int cmd_command_echo(buffer_t *rsp_buffer, const buffer_t *cmd_buffer);

int cmd_format_error_message(buffer_t *buffer, uint8_t error_message);
void cmd_init();

#endif //_cmd_h
