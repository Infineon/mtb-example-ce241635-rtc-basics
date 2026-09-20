/******************************************************************************
* File Name:   main.c
*
* Description: This is the source code for the Infineon MCU PDL Real-Time Clock
*              Basics example of ModusToolbox.
*
* Related Document: See README.md
*
********************************************************************************
* (c) 2026, Infineon Technologies AG, or an affiliate of Infineon
* Technologies AG. All rights reserved.
* This software, associated documentation and materials ("Software") is
* owned by Infineon Technologies AG or one of its affiliates ("Infineon")
* and is protected by and subject to worldwide patent protection, worldwide
* copyright laws, and international treaty provisions. Therefore, you may use
* this Software only as provided in the license agreement accompanying the
* software package from which you obtained this Software. If no license
* agreement applies, then any use, reproduction, modification, translation, or
* compilation of this Software is prohibited without the express written
* permission of Infineon.
*
* Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS SOFTWARE
* IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
* INCLUDING, BUT NOT LIMITED TO, ALL WARRANTIES OF NON-INFRINGEMENT OF
* THIRD-PARTY RIGHTS AND IMPLIED WARRANTIES SUCH AS WARRANTIES OF FITNESS FOR A
* SPECIFIC USE/PURPOSE OR MERCHANTABILITY.
* Infineon reserves the right to make changes to the Software without notice.
* You are responsible for properly designing, programming, and testing the
* functionality and safety of your intended application of the Software, as
* well as complying with any legal requirements related to its use. Infineon
* does not guarantee that the Software will be free from intrusion, data theft
* or loss, or other breaches ("Security Breaches"), and Infineon shall have
* no liability arising out of any Security Breaches. Unless otherwise
* explicitly approved by Infineon, the Software may not be used in any
* application where a failure of the Product or any consequences of the use
* thereof can reasonably be expected to result in personal injury.
*******************************************************************************/

/******************************************************************************
 * Include header files
 ******************************************************************************/
#include "cy_pdl.h"
#include "cybsp.h"
#include "string.h"
#include "stdio.h"

/*******************************************************************************
* Macros
*******************************************************************************/

#define UART_TIMEOUT_MS (10u)      /* in milliseconds */
#define INPUT_TIMEOUT_MS (120000u) /* in milliseconds */
#define UART_GET_CHAR_DELAY  (1u)   /* in milliseconds */

#define MAX_ATTEMPTS             (500u)  /* Maximum number of attempts for RTC operation */
#define INIT_DELAY_MS             (5u)    /* delay 5 milliseconds before trying again */

#define STRING_BUFFER_SIZE (80)

/* Available commands */
#define RTC_CMD_SET_DATE_TIME ('1')
#define RTC_CMD_CONFIG_DST ('2')

#define RTC_CMD_ENABLE_DST ('1')
#define RTC_CMD_DISABLE_DST ('2')
#define RTC_CMD_QUIT_CONFIG_DST ('3')

#define FIXED_DST_FORMAT ('1')
#define RELATIVE_DST_FORMAT ('2')

/* Macro used for checking validity of user input */
#define MIN_SPACE_KEY_COUNT (5)

/* Flags to indicate the if the entered time is valid */
#define DST_DISABLED_FLAG (0)
#define DST_VALID_START_TIME_FLAG (1)
#define DST_VALID_END_TIME_FLAG (2)
#define DST_ENABLED_FLAG (3)


/* Checks whether the year passed through the parameter is leap or not */
#define LEAP_YEAR_MONTH1     (1u)
#define LEAP_YEAR_MONTH2     (2u)

#define IS_LEAP_YEAR(year) \
(((0U == (year % 4UL)) && (0U != (year % 100UL))) || (0U == (year % 400UL)))

/***********************************
 * ********************************************
* Global Variables
*******************************************************************************/
uint32_t dst_data_flag = 0;
char buffer[STRING_BUFFER_SIZE];

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
static cy_en_rtc_status_t rtc_init(void);
static void set_new_time(uint32_t timeout_ms);
static void set_dst_feature(uint32_t timeout_ms);
static cy_rslt_t fetch_time_data(char *buffer,
                             uint32_t timeout_ms, uint32_t *space_count);

static void convert_date_to_string(cy_stc_rtc_config_t *dateTime);
static cy_rslt_t DEBUG_UART_getc(uint8_t *value, uint32_t timeout);

static bool validate_date_time(int sec, int min, int hour, int mday,
                                    int month, int year);


/*******************************************************************************
* Function Name: handle_error
********************************************************************************
* Summary:
* User defined error handling function
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void handle_error(void)
{
    /* Disable all interrupts. */
    __disable_irq();

    CY_ASSERT(0);
}

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
*   This function:
*  - Initializes the device and board peripherals
*  - Initializes RTC
*  - The loop checks for the user command and process the commands
*
* Parameters :
*  void
*
* Return:
*  int
*
*******************************************************************************/
int main(void)
{
    cy_rslt_t result;
    cy_en_rtc_status_t rtcSta;
    cy_stc_rtc_config_t dateTime;

    cy_en_scb_uart_status_t uartSta;
    cy_stc_scb_uart_context_t DEBUG_UART_context;

    uint8_t cmd;

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    if (result != CY_RSLT_SUCCESS)
        {
            handle_error();
        }

    /* Initialize the DEBUG_UART */
    uartSta = Cy_SCB_UART_Init(DEBUG_UART_HW, &DEBUG_UART_config, &DEBUG_UART_context);
    if (uartSta!=CY_SCB_UART_SUCCESS)
       {
            handle_error();
       }
    Cy_SCB_UART_Enable(DEBUG_UART_HW);

    /* Transmit header to the terminal */
    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    Cy_SCB_UART_PutString(DEBUG_UART_HW, "\x1b[2J\x1b[;H");

    Cy_SCB_UART_PutString(DEBUG_UART_HW, "************************************************************\r\n");
    Cy_SCB_UART_PutString(DEBUG_UART_HW, "PSOC Control C3M/P8: Real-time clock basics\r\n");
    Cy_SCB_UART_PutString(DEBUG_UART_HW, "************************************************************\r\n\n");

    /* Initialize the USER_RTC */
    rtcSta = rtc_init();
    if (rtcSta != CY_RTC_SUCCESS)
    {
        handle_error();
    }
    /* Enable global interrupts */
        __enable_irq();

    /*Show the RTC commands*/
    Cy_SCB_UART_PutString(DEBUG_UART_HW, "Available commands\r\n");
    Cy_SCB_UART_PutString(DEBUG_UART_HW, "1 : Set new time and date\r\n");
    Cy_SCB_UART_PutString(DEBUG_UART_HW, "2 : Configure DST feature\r\n\n");

    for(;;)
    {
        /*Read out RTC value and show on the terminal*/
        Cy_RTC_GetDateAndTime(&dateTime);
        convert_date_to_string(&dateTime);
        Cy_SCB_UART_PutString(DEBUG_UART_HW, buffer);
        memset(buffer, '\0', sizeof(buffer));

        /*Read out UART data  */
        DEBUG_UART_getc(&cmd, UART_TIMEOUT_MS);

       if(RTC_CMD_SET_DATE_TIME == cmd)
       {
          cmd = 0;
          Cy_SCB_UART_PutString(DEBUG_UART_HW, "\r[Command] : Set new time              \r\n");
          set_new_time(INPUT_TIMEOUT_MS);

       }
       else if (RTC_CMD_CONFIG_DST == cmd)
       {
          cmd = 0;
          Cy_SCB_UART_PutString(DEBUG_UART_HW, "\r[Command] : Configure DST feature              \r\n");
          set_dst_feature(INPUT_TIMEOUT_MS);

       }
    }
}

/*******************************************************************************
* Function Name: rtc_init
********************************************************************************
* Summary:
*  This functions implement the USER_RTC initialize
*
* Parameter:
*
*  function
*
* Return:
*  void
*******************************************************************************/
static cy_en_rtc_status_t rtc_init(void)
{
    uint32_t attempts = MAX_ATTEMPTS;
    cy_en_rtc_status_t rtc_result;

    /* Setting the time and date can fail. For example the RTC might be busy.
       Check the result and try again, if necessary.  */
    do
    {
        rtc_result = Cy_RTC_Init(&USER_RTC_config);
        attempts--;

        Cy_SysLib_Delay(INIT_DELAY_MS);
    } while(( rtc_result != CY_RTC_SUCCESS) && (attempts != 0u));

    return (rtc_result);

}

/*******************************************************************************
* Function Name: DEBUG_UART_getc
********************************************************************************
* Summary:
*  This functions get the DEBUG_UART input value from terminal and convert to
*  uint8_t type.
*
* Parameter:
*  uint8_t *value : the DEBUG_UART input value from terminal
*  uint32_t timeout: UART timeout
*  function
*
* Return:
*  void
*******************************************************************************/
cy_rslt_t DEBUG_UART_getc(uint8_t *value, uint32_t timeout)
{
    uint32_t read_value = Cy_SCB_UART_Get(DEBUG_UART_HW);
    uint32_t timeoutTicks = timeout;
        while (read_value == CY_SCB_UART_RX_NO_DATA)
        {
            if(timeout != 0UL)
            {
                if(timeoutTicks > 0UL)
                {
                    Cy_SysLib_Delay(UART_GET_CHAR_DELAY);       /*delay 1ms*/
                    timeoutTicks--;
                }
                else
                {
                    return CY_SCB_UART_RX_NO_DATA;
                }
            }
            read_value = Cy_SCB_UART_Get(DEBUG_UART_HW);
        }
        *value = (uint8_t)read_value;
        return CY_RSLT_SUCCESS;
}


/*******************************************************************************
* Function Name: convert_date_to_string
********************************************************************************
* Summary:
*  This functions get the RTC time values from 'dateTime', convert the uint32_t
*  values to chars, then combine all chars to one string and save in 'buffer'
*
* Parameter:
*  cy_stc_rtc_config_t *dateTime : the RTC configure struct pointer
*  function
*
* Return:
*  void
*******************************************************************************/
static void convert_date_to_string(cy_stc_rtc_config_t *dateTime)
{
    /* Read out RTC time values */
    uint32_t sec, min, hour, day, month, year;
    sec = dateTime->sec;      /*value range is 0-59*/
    min = dateTime->min;      /*value range is 0-59*/
    hour = dateTime->hour;     /*0-23 or 1-12*/
    day = dateTime->date;     /*date of month, 1-31*/
    month = dateTime->month;      /*1-12*/
    year = dateTime ->year;     /*base value is 2000. value range is 0-99*/

    /* Convert uint32_t values to chars */
    char secbuf[2], minbuf[2], hourbuf[2], daybuf[2], monthbuf[2], yearbuf[2];
    sprintf(secbuf, "%d", (int)sec);
    sprintf(minbuf, "%d", (int)min);
    sprintf(hourbuf, "%d", (int)hour);
    sprintf(daybuf, "%d", (int)day);
    sprintf(monthbuf, "%d", (int)month);
    sprintf(yearbuf, "%d", (int)year);

    /* Merge all chars to one string */
    snprintf(buffer, sizeof(buffer), "%s %s %s %s %s %s %s %s %s %s %s %s %s %s",
            "Mon", monthbuf, "Date", daybuf, "  ", hourbuf, ":", minbuf, ":", secbuf, "  ", yearbuf, "Year", "\r");

}

/*******************************************************************************
* Function Name: set_dst_feature
********************************************************************************
* Summary:
*  This functions takes the user input ,sets the dst start/end date and time,
*  and then enables the DST feature.
*
* Parameter:
*  uint32_t timeout_ms : Maximum allowed time (in milliseconds) for the
*  function
*
* Return:
*  void
*******************************************************************************/
static void set_dst_feature(uint32_t timeout_ms)
{
    cy_rslt_t rslt;
    uint8_t dst_cmd = 0;
    char dst_start_buffer[STRING_BUFFER_SIZE] = {0};
    char dst_end_buffer[STRING_BUFFER_SIZE] = {0};
    uint32_t space_count = 0;

    /* Variables used to store DST start and end time information */
    cy_stc_rtc_dst_t dst_time;
    cy_stc_rtc_config_t timeDate;

    /* Variables used to store date and time information */
    int mday = 0, month = 0, year = 0, sec = 0, min = 0, hour = 0;
    uint8_t fmt = 0;
    if (DST_ENABLED_FLAG == dst_data_flag)
    {
        if (Cy_RTC_GetDstStatus(&USER_RTC_configDst, &USER_RTC_config))
        {
            Cy_SCB_UART_PutString(DEBUG_UART_HW, "\rCurrent DST Status :: Active\r\n\n");
        }
        else
        {
            Cy_SCB_UART_PutString(DEBUG_UART_HW, "\rCurrent DST Status :: Inactive\r\n\n");
        }
    }
    else
    {
        Cy_SCB_UART_PutString(DEBUG_UART_HW, "\rCurrent DST Status :: Disabled\r\n\n");
    }

    /* Display available commands */
    Cy_SCB_UART_PutString(DEBUG_UART_HW, "Available DST commands \r\n");
    Cy_SCB_UART_PutString(DEBUG_UART_HW, "1 : Enable DST feature\r\n");
    Cy_SCB_UART_PutString(DEBUG_UART_HW, "2 : Disable DST feature\r\n");
    Cy_SCB_UART_PutString(DEBUG_UART_HW, "3 : Quit DST Configuration\r\n\n");

    rslt = DEBUG_UART_getc(&dst_cmd, timeout_ms);

    if (rslt != CY_SCB_UART_RX_NO_DATA)
    {
        if (RTC_CMD_ENABLE_DST == dst_cmd)
        {
            /* Get DST start time information */
            Cy_SCB_UART_PutString(DEBUG_UART_HW, "Enter DST format \r\n");
            Cy_SCB_UART_PutString(DEBUG_UART_HW, "1 : Fixed DST format\r\n");
            Cy_SCB_UART_PutString(DEBUG_UART_HW, "2 : Relative DST format\r\n\n");

            rslt = DEBUG_UART_getc(&fmt, timeout_ms);
            if (rslt != CY_SCB_UART_RX_NO_DATA)
            {
                Cy_SCB_UART_PutString(DEBUG_UART_HW,"Enter DST start time in \"mm dd HH MM SS yy\" format\r\n");
                rslt = fetch_time_data(dst_start_buffer, timeout_ms,
                                                        &space_count);
                if (rslt != CY_SCB_UART_RX_NO_DATA)
                {
                    if (space_count != MIN_SPACE_KEY_COUNT)
                    {
                        Cy_SCB_UART_PutString(DEBUG_UART_HW, "\rInvalid values! Please enter "
                        "the values in specified format\r\n");
                    }
                    else
                    {
                        /* Convert RTC value char to int */
                        sscanf(dst_start_buffer, "%d %d %d %d %d %d",
                               &month, &mday, &hour,
                               &min, &sec, &year);

                    if((validate_date_time(sec, min, hour, mday, month, year))&&
                    ((fmt == FIXED_DST_FORMAT) || (fmt == RELATIVE_DST_FORMAT)))
                    {
                        dst_time.startDst.format =
                        (fmt == FIXED_DST_FORMAT) ? CY_RTC_DST_FIXED :
                                                    CY_RTC_DST_RELATIVE;
                        dst_time.startDst.hour = hour;
                        dst_time.startDst.month = month;
                        dst_time.startDst.dayOfWeek =
                        (fmt == FIXED_DST_FORMAT) ? 1 :
                                    Cy_RTC_ConvertDayOfWeek(mday, month, year);
                        dst_time.startDst.dayOfMonth =
                        (fmt == FIXED_DST_FORMAT) ? mday : 1;
                        dst_time.startDst.weekOfMonth =
                        (fmt == FIXED_DST_FORMAT) ? 1 :
                                     Cy_RTC_ConvertDayOfWeek(mday, month, year);
                        /* Update flag value to indicate that a
                            valid DST start time information has been received*/
                        dst_data_flag = DST_VALID_START_TIME_FLAG;
                    }
                    else
                    {
                        Cy_SCB_UART_PutString(DEBUG_UART_HW, "\rInvalid values! Please enter the values"
                                   " in specified format\r\n");
                    }
                    }
                }
                else
                {
                    Cy_SCB_UART_PutString(DEBUG_UART_HW, "\rTimeout \r\n");
                }

                if (DST_VALID_START_TIME_FLAG == dst_data_flag)
                {
                    /* Get DST end time information,
                    iff a valid DST start time information is received */
                    Cy_SCB_UART_PutString(DEBUG_UART_HW, "Enter DST end time "
                    " in \"mm dd HH MM SS yy\" format\r\n");
                    rslt = fetch_time_data(dst_end_buffer, timeout_ms,
                                            &space_count);
                    if (rslt != CY_SCB_UART_RX_NO_DATA)
                    {
                        if (space_count != MIN_SPACE_KEY_COUNT)
                        {
                            Cy_SCB_UART_PutString(DEBUG_UART_HW, "\rInvalid values! Please"
                            "enter the values in specified format\r\n");
                        }
                        else
                        {
                            sscanf(dst_end_buffer, "%d %d %d %d %d %d",
                                   &month, &mday, &hour,
                                   &min, &sec, &year);

                            if((validate_date_time(sec,min,hour,mday,month,year))&&
                                ((fmt == FIXED_DST_FORMAT) ||
                                (fmt == RELATIVE_DST_FORMAT)))
                            {
                                dst_time.stopDst.format = (fmt == FIXED_DST_FORMAT)?
                                              CY_RTC_DST_FIXED : CY_RTC_DST_RELATIVE;
                                dst_time.stopDst.hour = hour;
                                dst_time.stopDst.month = month;
                                dst_time.stopDst.dayOfWeek =
                                (fmt == FIXED_DST_FORMAT) ? 1 :
                                       Cy_RTC_ConvertDayOfWeek(mday, month, year);
                                dst_time.stopDst.dayOfMonth =
                                (fmt == FIXED_DST_FORMAT) ? mday : 1;
                                dst_time.stopDst.weekOfMonth =
                                (fmt == FIXED_DST_FORMAT) ? 1 :
                                        Cy_RTC_ConvertDayOfWeek(mday, month, year);

                                /* Update flag value to indicate that a valid
                                 DST end time information has been received*/
                                dst_data_flag = DST_VALID_END_TIME_FLAG;
                            }
                            else
                            {
                                Cy_SCB_UART_PutString(DEBUG_UART_HW, "\rInvalid values! Please enter the "
                                       " values in specified format\r\n");
                            }
                        }
                    }
                    else
                    {
                        Cy_SCB_UART_PutString(DEBUG_UART_HW, "\rTimeout \r\n");
                    }
                }

                if (DST_VALID_END_TIME_FLAG == dst_data_flag)
                {
                   /*set the DST start and end time*/
                rslt = Cy_RTC_EnableDstTime(&dst_time, &timeDate);
                    if (CY_RTC_SUCCESS == rslt)
                    {
                        dst_data_flag = DST_ENABLED_FLAG;
                        Cy_SCB_UART_PutString(DEBUG_UART_HW, "\rDST time updated\r\n\n");
                    }
                    else
                    {
                        handle_error();
                    }
                }
            }
            else
            {
                Cy_SCB_UART_PutString(DEBUG_UART_HW, "\rTimeout \r\n");
            }
        }
        else if (RTC_CMD_DISABLE_DST == dst_cmd)
        {
            dst_time.stopDst.format = CY_RTC_DST_FIXED;
            dst_time.stopDst.hour = 0;
            dst_time.stopDst.month = 1;
            dst_time.stopDst.dayOfWeek = 1;
            dst_time.stopDst.dayOfMonth = 1;
            dst_time.stopDst.weekOfMonth = 1;
            dst_time.startDst = dst_time.stopDst;

            rslt = Cy_RTC_EnableDstTime(&dst_time, &timeDate);
            if (CY_RTC_SUCCESS == rslt)
            {
                dst_data_flag = DST_DISABLED_FLAG;
                Cy_SCB_UART_PutString(DEBUG_UART_HW, "\rDST feature disabled\r\n\n");
            }
            else
            {
                handle_error();
            }
        }
        else if (RTC_CMD_QUIT_CONFIG_DST == dst_cmd)
        {
            Cy_SCB_UART_PutString(DEBUG_UART_HW, "\rExit from DST Configuration \r\n\n");
        }
    }
    else
    {
        Cy_SCB_UART_PutString(DEBUG_UART_HW, "\rTimeout \r\n");
    }
}

/*******************************************************************************
* Function Name: set_new_time
********************************************************************************
* Summary:
*  This functions takes the user input and sets the new date and time.
*
* Parameter:
*  uint32_t timeout_ms : Maximum allowed time (in milliseconds) for the
*  function
*
* Return :
*  void
*******************************************************************************/
static void set_new_time(uint32_t timeout_ms)
{
    cy_rslt_t rslt;
    char buffer[STRING_BUFFER_SIZE] = {0};
    uint32_t space_count;
    uint32_t attempts = MAX_ATTEMPTS;

    /* Variables used to store date and time information */
    int mday, month, year, sec, min, hour;

    Cy_SCB_UART_PutString(DEBUG_UART_HW,"\rEnter time in \"mm dd HH MM SS yy\" format \r\n");
    rslt = fetch_time_data(buffer, timeout_ms, &space_count);      /*Failed to read memory at 0x00000018*/
    if (rslt != CY_SCB_UART_RX_NO_DATA)
    {
        if (space_count != MIN_SPACE_KEY_COUNT)
        {
            Cy_SCB_UART_PutString(DEBUG_UART_HW, "\rInvalid values! Please enter the"
                    "values in specified format\r\n");
        }
        else
        {
           /* convert buffer element char to int */
            sscanf(buffer, "%d %d %d %d %d %d",
                   &month, &mday, &hour,
                   &min, &sec, &year);

         do{
            rslt = Cy_RTC_SetDateAndTimeDirect((uint32_t)sec, (uint32_t)min, (uint32_t)hour,
                     (uint32_t)mday, (uint32_t)month, (uint32_t)year);
            attempts--;

            Cy_SysLib_Delay(INIT_DELAY_MS);

           }while(( rslt != CY_RTC_SUCCESS) && (attempts != 0u));

          Cy_SCB_UART_PutString(DEBUG_UART_HW, "\rRTC time updated\r\n\n");

          if (CY_RTC_SUCCESS != rslt)
            {
                Cy_SCB_UART_PutString(DEBUG_UART_HW, "\rInvalid values! Please enter the values in specified"
                                   " format\r\n");
                handle_error();
            }
        }
    }
    else
    {
        Cy_SCB_UART_PutString(DEBUG_UART_HW, "\rTimeout \r\n");
    }
}

/*******************************************************************************
* Function Name: fetch_time_data
********************************************************************************
* Summary:
*  Function fetches data entered by the user through UART and stores it in the
*  buffer which is passed through parameters. The function also counts number of
*  spaces in the received data and stores in the variable, whose address are
*  passed as parameter.
*
* Parameter:
*  char* buffer        : Buffer to store the fetched data
*  uint32_t timeout_ms : Maximum allowed time (in milliseconds) for the function
*  uint32_t* space_count : The number of spaces present in the fetched data.
*
* Return:
*  Returns the status of the getc request
*
*******************************************************************************/
static cy_rslt_t fetch_time_data(char *buffer, uint32_t timeout_ms,
                                    uint32_t *space_count)
{
    cy_rslt_t rslt;
    uint32_t index = 0;
    uint8_t ch = 0;
    *space_count = 0;
    while (index < STRING_BUFFER_SIZE)
    {
        if (timeout_ms <= UART_TIMEOUT_MS)
        {
            rslt = CY_SCB_UART_RX_NO_DATA;
            break;
        }

        /* get char from DEBUG_UART terminal */
        rslt = DEBUG_UART_getc(&ch, UART_TIMEOUT_MS);

        if (rslt != CY_SCB_UART_RX_NO_DATA)
        {
            if (ch == '\n' || ch == '\r')
            {
                break;
            }
            else if (ch == ' ')
            {
                (*space_count)++;
            }

            buffer[index] = ch;
            rslt = Cy_SCB_UART_Put(DEBUG_UART_HW, ch);
            index++;
        }

        timeout_ms -= UART_TIMEOUT_MS;
    }

    Cy_SCB_UART_PutString(DEBUG_UART_HW, "\n\r");
    return rslt;
}

/*******************************************************************************
* Function Name: validate_date_time
********************************************************************************
* Summary:
*  This function validates date and time value.
*
* Parameters:
*  uint32_t sec     : The second valid range is [0-59].
*  uint32_t min     : The minute valid range is [0-59].
*  uint32_t hour    : The hour valid range is [0-23].
*  uint32_t date    : The date valid range is [1-31], if the month of February
*                     is selected as the Month parameter, then the valid range
*                     is [0-29].
*  uint32_t month   : The month valid range is [1-12].
*  uint32_t year    : The year valid range is [> 0].
*
* Return:
*  false - invalid ; true - valid
*
*******************************************************************************/
static bool validate_date_time(int sec, int min, int hour, int mday,
                                    int month, int year)
{
    bool rslt = false;
    uint8_t days_in_month;

    static const uint8_t days_in_month_table[CY_RTC_MONTHS_PER_YEAR] =
        {
            CY_RTC_DAYS_IN_JANUARY,
            CY_RTC_DAYS_IN_FEBRUARY,
            CY_RTC_DAYS_IN_MARCH,
            CY_RTC_DAYS_IN_APRIL,
            CY_RTC_DAYS_IN_MAY,
            CY_RTC_DAYS_IN_JUNE,
            CY_RTC_DAYS_IN_JULY,
            CY_RTC_DAYS_IN_AUGUST,
            CY_RTC_DAYS_IN_SEPTEMBER,
            CY_RTC_DAYS_IN_OCTOBER,
            CY_RTC_DAYS_IN_NOVEMBER,
            CY_RTC_DAYS_IN_DECEMBER,
        };

    rslt = CY_RTC_IS_SEC_VALID(sec) & CY_RTC_IS_MIN_VALID(min) & CY_RTC_IS_HOUR_VALID(hour) &
           CY_RTC_IS_MONTH_VALID(month) & CY_RTC_IS_YEAR_LONG_VALID(year);

    if(rslt)
    {
      days_in_month = days_in_month_table[month - LEAP_YEAR_MONTH1];

      if (IS_LEAP_YEAR(year) && (month == LEAP_YEAR_MONTH2))
       {
         days_in_month++;
       }

      rslt &= (mday > 0U) && (mday <= days_in_month);
    }
    return rslt;
}

/* [] END OF FILE */
