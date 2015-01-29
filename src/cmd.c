#include "cmd.h"


int cmd_format_error_message(buffer_t *buffer, uint8_t error_message) {
  /*
    Keep the ID of the command.
  */
  buffer->fill = 1;

  buffer_append_byte(buffer,error_message);
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
  rsp_buffer->data[1] = CMD_ERROR_OK;
 
  return rsp_buffer->data[1];
}

int cmd_command_start_timer(buffer_t *rsp_buffer, uint16_t *timeout, const buffer_t *cmd_buffer) {
  /*
    2 header bytes, 2 timout bytes, little-endian
   */
  if(cmd_buffer->fill != 4) {
    return cmd_format_error_message(rsp_buffer,CMD_ERROR_ARGUMENT);
  }

  *timeout = uint16_from_little_endian(cmd_buffer->data);

  return cmd_format_error_message,rsp_buffer,CMD_ERROR_OK);
  
}



int cmd_dispatcher(const buffer_t *cmd_buffer) {
  if (cmd_buffer->fill < 2) {
    return CMD_ERROR_ARGUMENT;
  }

  return CMD_ERROR_OK;
}


