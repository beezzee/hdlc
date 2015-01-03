#include "cmd.h"

int cmd_command_echo(usart_buffer_t *rsp_buffer, const usart_buffer_t *cmd_buffer) {
  int i=0;
  rsp_buffer->fill = 0;

  if(rsp_buffer->size < 1) {
    return CMD_ERROR_BUFFER_OVERFLOW;
  }

  rsp_buffer->data[0] = CMD_ERROR_OK;

  while((rsp_buffer->size > rsp_buffer->fill) && (i<cmd_buffer->fill)) {
    rsp_buffer->data[i+1] = cmd_buffer->data[i];
    rsp_buffer->fill++;
    i++;
  }

  if (i < cmd_buffer->fill) {
    rsp_buffer->data[0] = CMD_ERROR_BUFFER_OVERFLOW;

  } 
  return rsp_buffer->data[0];
}

int cmd_dispatcher(usart_buffer_t *rsp_buffer, const usart_buffer_t *cmd_buffer) {
  if (cmd_buffer->fill<1) {
    return cmd_format_error_message(rsp_buffer,CMD_ERROR_ARGUMENT);
  }

  switch (cmd_buffer->data[0]) {
  case CMD_COMMAND_ECHO:
    return cmd_command_echo(rsp_bufer,cmd_buffer);
    break;
  default:
    return cmd_format_error_message(rsp_buffer,CMD_ERROR_UNKNOWN_COMMAND);
    break;
  }

}


