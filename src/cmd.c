#include "cmd.h"

int cmd_command_echo(usart_buffer_t *rsp_buffer, const usart_buffer_t *cmd_buffer) {
  int i=0;
  rsp_buffer->payload->fill = 0;

  if(rsp_buffer->payload->size < 1) {
    return CMD_ERROR_BUFFER_OVERFLOW;
  }

  rsp_buffer->payload->data[0] = CMD_ERROR_OK;

  while((rsp_buffer->payload->size > rsp_buffer->payload->fill) && (i<cmd_buffer->payload->fill)) {
    rsp_buffer->payload->data[i+1] = cmd_buffer->payload->data[i];
    rsp_buffer->payload->fill++;
    i++;
  }

  if (i < cmd_buffer->payload->fill) {
    rsp_buffer->payload->data[0] = CMD_ERROR_BUFFER_OVERFLOW;
  } 
  return rsp_buffer->payload->data[0];
}

int cmd_dispatcher(usart_buffer_t *rsp_buffer, const usart_buffer_t *cmd_buffer) {
  if (cmd_buffer->payload->fill<1) {
    return cmd_format_error_message(rsp_buffer,CMD_ERROR_ARGUMENT);
  }

  switch (cmd_buffer->payload->data[0]) {
  case CMD_COMMAND_ECHO:
    return cmd_command_echo(rsp_buffer,cmd_buffer);
    break;
  default:
    return cmd_format_error_message(rsp_buffer,CMD_ERROR_UNKNOWN_COMMAND);
    break;
  }

}


