#ifndef _cmd_h
#define _cmd_h

#include "usart.h"

#define CMD_COMMAND_ECHO 1

#define CMD_ERROR_OK 0
#define CMD_ERROR_UNKNOWN_COMMAND 1
#define CMD_ERROR_ARGUMENT 2
#define CMD_ERROR_FORMAT 3
#define CMD_ERROR_BUFFER_OVERFLOW 4
/**
   Decode and execute commands.

   cmd_buffer.data:

   ---> | ID | ARG0 | ... | ARGN |

   rsp_buffer.data:

   <--- | ID | STATUS | ARG0 | ... | ARGN |

**/
int cmd_dispatcher(usart_buffer_t *rsp_buffer, const usart_buffer_t *cmd_buffer);

#endif //_cmd_h
