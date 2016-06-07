#include "cmd.h"
#include "hdlc.h"
#include "utils.h"





static cmd_handler_t registered_commands[256];

int cmd_register(uint8_t id, cmd_handler_t handler) {
    registered_commands[id] = handler;
    return 0;
}

int cmd_format_error_message(buffer_t *buffer, uint8_t error_message) {
  /*
    we dont't have enougth space to format error message
  */
  if(buffer->size < 2) {
    return CMD_ERROR_BUFFER_OVERFLOW;
  }

  /*
    Keep the ID of the command.
  */
  buffer->fill = 1;

  /*0 bytes appended */
  if (0==buffer_append_byte(buffer,error_message)) {
    return CMD_ERROR_BUFFER_OVERFLOW;
  }

  return CMD_ERROR_OK;

}

int cmd_command_echo(buffer_t *rsp_buffer, const buffer_t *cmd_buffer) {
  int i=0;

  if(rsp_buffer->size<cmd_buffer->fill) {
    cmd_format_error_message(rsp_buffer,CMD_ERROR_BUFFER_OVERFLOW);
    return CMD_ERROR_BUFFER_OVERFLOW;
  }



  /*
    if we use the same buffers, we don't need to copy.
  */
  if(rsp_buffer == cmd_buffer) {

  } else {

    /*
      copy buffer
    */
    for(i=0;i<cmd_buffer->fill;i++) {
      buffer_append_byte(
             rsp_buffer,
             buffer_at_index(cmd_buffer,i)
             );
    }
  }

  /*
    set status to ok
   */
  rsp_buffer->data[HDLC_CTRL_OFFSET] = CMD_ERROR_OK;

  return rsp_buffer->data[HDLC_CTRL_OFFSET];
}


int cmd_dispatcher(buffer_t *rsp_buffer, const buffer_t *cmd_buffer) {
  if (cmd_buffer->fill < 2) {
    return CMD_ERROR_ARGUMENT;
  }

  //cmd_command_echo(rsp_buffer,cmd_buffer);
  //return 0;
  return (registered_commands[cmd_buffer->data[HDLC_ADDR_OFFSET]])(rsp_buffer,cmd_buffer);
    //return (registered_commands[1])(rsp_buffer,cmd_buffer);
}


