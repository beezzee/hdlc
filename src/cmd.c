#include "cmd.h"

#include "utils.h"

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
  rsp_buffer->data[1] = CMD_ERROR_OK;

  return rsp_buffer->data[1];
}

int cmd_command_start_timeout(buffer_t *rsp_buffer, const buffer_t *cmd_buffer, uint16_t *timeout, uint16_t *target_temperature){
  /*
    2 header bytes, 2 timout bytes, 2 temperature bytes little-endian
   */
  if(cmd_buffer->fill != 6) {
    cmd_format_error_message(rsp_buffer,CMD_ERROR_ARGUMENT);
    return CMD_ERROR_ARGUMENT;
  }

  *timeout = uint16_from_little_endian(cmd_buffer->data+2);
  *target_temperature = uint16_from_little_endian(cmd_buffer->data+4);

  cmd_format_error_message(rsp_buffer,CMD_ERROR_OK);
  return CMD_ERROR_OK;

}

int cmd_command_calibrate(buffer_t *rsp_buffer, uint16_t *temperature, const buffer_t *cmd_buffer) {
  /*
    2 header bytes, 2 temperature bytes, little-endian
   */
  if(cmd_buffer->fill != 4) {
    cmd_format_error_message(rsp_buffer,CMD_ERROR_ARGUMENT);
    return CMD_ERROR_ARGUMENT;
  }

  *temperature = uint16_from_little_endian(cmd_buffer->data+2);

  cmd_format_error_message(rsp_buffer,CMD_ERROR_OK);
  return CMD_ERROR_OK;

}


int cmd_command_get_status(buffer_t *rsp_buffer,  const buffer_t *cmd_buffer, uint16_t timeout, uint16_t timestamp, uint16_t target_temperature, uint16_t temperature, uint16_t position) {
  uint8_t *ptr =  rsp_buffer->data;

  if (rsp_buffer->size < 12) {
    cmd_format_error_message(rsp_buffer,CMD_ERROR_BUFFER_OVERFLOW);
    return CMD_ERROR_BUFFER_OVERFLOW;
  }
  rsp_buffer->data[1]=CMD_ERROR_OK;

  uint16_to_little_endian(ptr+ 2,timeout);
  uint16_to_little_endian(ptr+ 4,timestamp);
  uint16_to_little_endian(ptr+ 6,target_temperature);
  uint16_to_little_endian(ptr+ 8,temperature);
  uint16_to_little_endian(ptr+10,position);

  rsp_buffer->fill = 12;
  return CMD_ERROR_OK;
}

int cmd_dispatcher(const buffer_t *cmd_buffer) {
  if (cmd_buffer->fill < 2) {
    return CMD_ERROR_ARGUMENT;
  }

  return CMD_ERROR_OK;
}


