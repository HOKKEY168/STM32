/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2018 Anh Vo Tuan <votuananhs@gmail.com>
 *
 * Copyright (c) 2020 Sovichea Tep <sovichea.tep@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the "Software"), to deal in the Software without restriction,
 * including without limitation the rights to use, copy, modify, merge, publish, distribute,
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software
 * is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies
 * or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
 * BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE
 * AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
 * DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */
#include "cli.h"
#include "usart.h"
#include "stdarg.h"
#include "tim.h"


/* ============================== Global Variable ================== */
static cli_t *cmd_list = NULL;
static cli_t *cmd_start = NULL;

uint8_t cmd_buffer[LEN_INPUT_BUFFER];
uint8_t char_count;
char rx_data = '\n';
extern char uart_buffer[100];


UART_HandleTypeDef *p_huart;

/* =========================Static Functions======================== */
/*
 * Name function    : clear_screen
 * Brief            : Clear terminal screen
 * Return           : None
 */
static void clear_screen(uint8_t argc, uint8_t **argv)
{
  CLI_WriteString(CLEAR_SCREEN);
}

/*
 * Name function    : help
 * Brief            : Show all commands on the terminal
 * Return           : None
 */
static void help(uint8_t argc, uint8_t **argv)
{
  cli_t *temp_command = cmd_start;

  while (temp_command != NULL)
  {
    if (0 == argc)
    {
      CLI_WriteString(BY "\n\r");
      CLI_WriteString(temp_command->command);
      CLI_WriteString(BG "\n\r\t");
      CLI_WriteString(temp_command->description);
      CLI_WriteString(RESET"\n\r");
    }
    else
    {
      if (!(strcmp((const char*) argv, temp_command->command)))
      {
        CLI_WriteString(BY "\n\r");
        CLI_WriteString(temp_command->command);
        CLI_WriteString(BG "\n\r\t");
        CLI_WriteString(temp_command->description);
        CLI_WriteString(RESET "\n\r");
      }
    }
    temp_command = temp_command->next_command;
  }
}

/*
 * Name function    : reset
 * Brief            : Soft reset the board
 * Return           : None
 */
static void reset(uint8_t argc, uint8_t **argv)
{
  HAL_NVIC_SystemReset();
}
static void stop(uint8_t argc, uint8_t **argv)
{
	CLI_Printf("Stopping All Motor...");
	HAL_TIM_Base_Stop_IT(&htim1);
	HAL_TIM_Base_Stop_IT(&htim2);
	HAL_TIM_Base_Stop_IT(&htim3);
	HAL_TIM_Base_Stop_IT(&htim4);
	HAL_TIM_Base_Stop_IT(&htim5);
	HAL_TIM_Base_Stop_IT(&htim6);
	CLI_Printf("Done!");
}
static void home(uint8_t argc, uint8_t **argv)
{
	char msg[20] = "set_home";
	CLI_Printf("Setting home...");
	HAL_TIM_Base_Stop_IT(&htim1);
	HAL_TIM_Base_Stop_IT(&htim2);
	HAL_TIM_Base_Stop_IT(&htim3);
	HAL_TIM_Base_Stop_IT(&htim4);
	HAL_TIM_Base_Stop_IT(&htim5);
	HAL_TIM_Base_Stop_IT(&htim6);
//	osMessageQueuePut(UartCommandHandle, &msg, 0, 0);

}
static void get_pos(uint8_t argc, uint8_t **argv)
{
	char msg[20] = "send_pos";
//	osMessageQueuePut(UartCommandHandle, &msg, 0, 0);
}
static void set_amax(uint8_t argc, uint8_t **argv)
{
	char msg[20] = "set_amax";
//	osMessageQueuePut(UartCommandHandle, &msg, 0, 0);
}
static void set_vmax(uint8_t argc, uint8_t **argv)
{
	char msg[20] = "set_vmax";
//	osMessageQueuePut(UartCommandHandle, &msg, 0, 0);
}
static void set_jmax(uint8_t argc, uint8_t **argv)
{
	char msg[20] = "set_jmax";
//	osMessageQueuePut(UartCommandHandle, &msg, 0, 0);
}
static void set_pos(uint8_t argc, uint8_t **argv)
{
	char msg[20] = "set_pos";
//	osMessageQueuePut(UartCommandHandle, &msg, 0, 0);
}
static void set_theta(uint8_t argc, uint8_t **argv)
{
	char msg[20] = "set_theta";
//	osMessageQueuePut(UartCommandHandle, &msg, 0, 0);
}
static void save_config(uint8_t argc, uint8_t **argv)
{
	char msg[20] = "save";
//	osMessageQueuePut(UartCommandHandle, &msg, 0, 0);
}
static void arm_limit(uint8_t argc, uint8_t **argv)
{
	CLI_Printf(BY"Angle Limits");
	CLI_Printf(BG"\ttheta1[0,360]\n\r\ttheta2[0,180]\n\r\ttheta3[-180,0]");
	CLI_Printf(BG"\ttheta4[-180,180]\n\r\ttheta5[-90,90]\n\r\ttheta6[-180,180]");
}
static void FreeRun(uint8_t argc, uint8_t **argv)
{
	char msg[20] = "freerun";
//	osMessageQueuePut(UartCommandHandle, &msg, 0, 0);
}
static cli_t m_cmd[] =
{
  {
    .command = "help",
    .entry_function = help,
    .num_param = 0,
    .description = "Show all commands on the terminal",
    .next_command = NULL
  },
  {
    .command = "reset",
    .entry_function = reset,
    .num_param = 0,
    .description = "Soft reset the board",
    .next_command = NULL
  },
  {
    .command = "clear",
    .entry_function = clear_screen,
    .num_param = 0,
    .description = "Clear terminal screen",
    .next_command = NULL
  },
  {	.command = "stop",
	.entry_function = stop,
	.num_param = 0,
	.description = "Stop all motor",
	.next_command = NULL
  },
  {	.command = "home",
	.entry_function = home,
	.num_param = 0,
	.description = "Move to initial position",
	.next_command = NULL
  },
  {	.command = "get_pos",
	.entry_function = get_pos,
	.num_param = 0,
	.description = "Get current position",
	.next_command = NULL
  },
  {	.command = "set_amax",
	.entry_function = set_amax,
	.num_param = 1,
	.description = "Set maximum acceleration",
	.next_command = NULL
  },
  {	.command = "set_vmax",
	.entry_function = set_vmax,
	.num_param = 1,
	.description = "Set maximum speed",
	.next_command = NULL
  },
  {	.command = "set_jmax",
	.entry_function = set_jmax,
	.num_param = 1,
	.description = "Set maximum jerk",
	.next_command = NULL
  },
  {	.command = "xyz",
	.entry_function = set_pos,
	.num_param = 3,
	.description = "Set position xyz",
	.next_command = NULL
  },
  {	.command = "theta",
	.entry_function = set_theta,
	.num_param = 6,
	.description = "Set input angle",
	.next_command = NULL
  },
  {	.command = "save_config",
	.entry_function = save_config,
	.num_param = 0,
	.description = "Save configuration",
	.next_command = NULL
  },
  {	.command = "limit",
	.entry_function = arm_limit,
	.num_param = 0,
	.description = "Angle limitation",
	.next_command = NULL
  },
  {	.command = "FreeRun",
	.entry_function = FreeRun,
	.num_param = 1,
	.description = "Random Movement",
	.next_command = NULL
  },
};

/* ============================== API ============================== */

/*
 * Name function    : HAL_UART_RxCpltCallback
 * Brief            : Store the received characters and echo to terminal
 * Return           : None
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == p_huart)
  {
    if (rx_data == '\b' || rx_data == '\x7f')
    {
      /* only echo "Backspace" if num_char is larger than 0 */
      if (char_count > 0)
        HAL_UART_Transmit(huart, (uint8_t*) &rx_data, 1, 10);
    }
    else
    {
      HAL_UART_Transmit(huart, (uint8_t*) &rx_data, 1, 10);
    }
    HAL_UART_Receive_IT(huart, (uint8_t*) &rx_data, 1);
  }
}

/*
 * Name function    : CLI_WriteString
 * Brief            : Send null-terminated strings to terminal
 * Return           : None
 */
void CLI_WriteString(const char *s)
{
  while (*s)
  {
    HAL_UART_Transmit(p_huart, (uint8_t*) s, 1, 10);
    s++;
  }
}

/*
 * Name function    : CLI_Printf
 * Brief            : "printf" function for CLI
 * Return           : None
 */
void CLI_Printf(const char *format, ...)
{
  char buf[LEN_INPUT_BUFFER];
  va_list arg;
  va_start(arg, format);
  vsprintf(buf, format, arg);
  CLI_WriteString("\n\r");
  CLI_WriteString(buf);
  va_end(arg);
}

/*
 * Name function    : CLI_Process
 * Brief            : Process the typed commands when "Return Key" is pressed.
 * Return           : None
 */
void CLI_Process(void)
{
  if (rx_data != 0)
  {
    if (CLI_GetChar(rx_data))
    {
      /* print CLI prompt on console */
      CLI_Printf(BY PROMPT " " RESET);
    }
    rx_data = 0;
  }
}

/*
 * Name function    : CLI_Init
 * Brief            : Initialize Command Line Interface on UART
 * Return           : None
 */
void CLI_Init(UART_HandleTypeDef *huart)
{
  p_huart = huart;
  HAL_UART_Receive_IT(p_huart, (uint8_t*) &rx_data, 1);

  cli_t *temp_command = (cli_t*) malloc(sizeof(cli_t));

  if (NULL == temp_command)
  {
    /* lock program here, because it can't allocate new
     area memory for this variable */
    while (1);
  }
  /* add help command to the first of the list */
  cmd_list = m_cmd;
  cmd_start = m_cmd;

  /* add the following commands to list*/
  CLI_AddCommand(&m_cmd[1], sizeof(m_cmd)/sizeof(cli_t) - 1);
  CLI_ClearBuffer();
}

/*
 * Name function    : CLI_AddCommand
 * Brief            : Add new commands to the list
 * Return           : None
 */
int8_t CLI_AddCommand(cli_t *new_command, uint8_t num_command)
{
  uint8_t i;
  int8_t result = 0;

  if (num_command < 1)
  {
    result = -1;
  }
  else
  {
    for (i = 0; i < num_command; i++)
    {
      cmd_list->next_command = &new_command[i];
      cmd_list = cmd_list->next_command;
      cmd_list->next_command = NULL;
    }
  }

  return result;
}

/*
 * Name function    : CLI_ParseCommand
 * Brief            : Execute the command
 * Return           : None
 */
void CLI_ParseCommand(const char *str_command, const uint8_t len_command)
{
  sprintf(uart_buffer,"%s", str_command);
  char *temp_str = NULL;
  uint8_t num_of_input = 0;
  cli_t *temp_command = cmd_start;
  uint8_t **input_parameter;

  temp_str = strtok(str_command, DELIMITER_CHARACTERS);
  while (temp_command != NULL)
  {
    if (!(strcmp(temp_str, temp_command->command)))
    {
      input_parameter = malloc(sizeof(uint32_t) * temp_command->num_param);
      num_of_input = 0;
      temp_str = strtok(NULL, DELIMITER_CHARACTERS);
      while (NULL != temp_str)
      {
        if (num_of_input > temp_command->num_param)
        {
          break;
        }
        else
        {
          input_parameter[num_of_input] = (uint8_t*)temp_str;
          temp_str = strtok(NULL, DELIMITER_CHARACTERS);
          num_of_input++;
        }
      }
      break;
    }
    temp_command = temp_command->next_command;
  }

  if (NULL != temp_command)
  {
    if (num_of_input == temp_command->num_param)
    {
      temp_command->entry_function(num_of_input, input_parameter);
    }
    else
    {
      CLI_Printf(BR "%s: expected %d argument(s), but %d found" RESET,
                 temp_command->command, temp_command->num_param, num_of_input);
    }
    free(input_parameter);
  }
  else
  {
    CLI_Printf(BR "%s: command not found." RESET, temp_str);
  }

}

/*
 * Name function    : CLI_GetChar
 * Brief            : Store received character depend on key press action
 * Return           : return 0 - command didn't finish; 1 - command finished
 */
uint8_t CLI_GetChar(const uint8_t c)
{
  uint8_t result = 0;
  char ch = c;
  if (c == '\r') ch = '\n';

  switch (ch)
  {
    /* Return key pressed */
    case '\n':
      if (char_count > 0)
      {
        CLI_ParseCommand((const char*) cmd_buffer, char_count);
        memset(cmd_buffer, 0, LEN_INPUT_BUFFER);
      }
      char_count = 0;
      result = 1;
      break;

      /* Backspace detected */
    case '\b':
    case '\x7f':
      if (char_count > 0)
      {
        char_count--;
        cmd_buffer[char_count] = 0;
      }
      break;

    default:
      cmd_buffer[char_count] = ch;
      char_count++;
      break;
  }

  return result;
}

/*
 * Name function    : CLI_ClearBuffer
 * Brief            :
 * Return           : None
 */
void CLI_ClearBuffer(void)
{
  char_count = 0;
  memset(cmd_buffer, 0, LEN_INPUT_BUFFER);
}
