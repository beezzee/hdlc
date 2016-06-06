#ifndef _cmd_h
#define _cmd_h

#include "usart.h"
#include "buffer.h"

#define CMD_COMMAND_ECHO 1
#define CMD_COMMAND_CALIBRATE 2
#define CMD_COMMAND_START_TIMEOUT 3
#define CMD_COMMAND_GET_STATUS 4

#define CMD_ERROR_OK 0
#define CMD_ERROR_UNKNOWN_COMMAND 1
#define CMD_ERROR_ARGUMENT 2
#define CMD_ERROR_FORMAT 3
#define CMD_ERROR_BUFFER_OVERFLOW 4
/**
   Decode and execute commands.

   cmd_buffer.data:

   ---> | ID | CNTRL  | ARG0 | ... | ARGN |

   rsp_buffer.data:

   <--- | ID | STATUS | ARG0 | ... | ARGN |


   STATUS of command  is used for future use.
**/
int cmd_dispatcher(const buffer_t *cmd_buffer);

/**
   Used for parsing timeout command.

   expected structure of cmd_buffer->data[2:]

   ARG0: LSB of timeout value in seconds.
   ARG1: MSB of timeout value in seconds.
   ARG2: LSB of target brewing temperature in Milli Kelvin.
   ARG3: MSB of target brewing temperature in Milli Kelvin.

   @param rsp_buffer Buffer for response.
   @param timeout Will be set to brewing timeout.
   @param target_temperature Will be set to target brewing temperature.
   @param cmd_buffer Buffer for command.
   @return Error code.
 */
int cmd_command_start_timeout(buffer_t *rsp_buffer, const buffer_t *cmd_buffer, uint16_t *timeout, uint16_t *target_temperature);

/**
   Used for parsing temperature calibration command.

   ARG0: LSB of calibration temperature in Milli Kelvin.
   ARG1: MSB of calibration temperature in Milli Kelvin.

 */
int cmd_command_calibrate(buffer_t *rsp_buffer, uint16_t *temperature, const buffer_t *cmd_buffer);

int cmd_command_echo(buffer_t *rsp_buffer, const buffer_t *cmd_buffer);

int cmd_format_error_message(buffer_t *buffer, uint8_t error_message);

/**
   Responds with current status of device:
   | ID | STATUS | Timeout | Time | Brewing Temperature | Current Temperature | Position |

   The following list is a list of unsigned 16 bit integers.

   ID: The ID of the command
   STATUS: The status message of the command processing.
   Timeout: The configured timeout in seconds
   Time: The time that is left until Timeout is reached in seconds.
   Brewing Temperature: The target brewing temperature in Milli Kelvin
   Current Temperature: The current temperature in Milli Kelvin
   Position: The current position


 */
int cmd_command_get_status(buffer_t *rsp_buffer,  const buffer_t *cmd_buffer, uint16_t timeout, uint16_t timestamp, uint16_t target_temperature, uint16_t temperature, uint16_t position);

#endif //_cmd_h
