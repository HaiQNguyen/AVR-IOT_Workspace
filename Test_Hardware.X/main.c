/*
    (c) 2018 Microchip Technology Inc. and its subsidiaries. 
    
    Subject to your compliance with these terms, you may use Microchip software and any 
    derivatives exclusively with Microchip products. It is your responsibility to comply with third party 
    license terms applicable to your use of third party software (including open source software) that 
    may accompany Microchip software.
    
    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER 
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY 
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS 
    FOR A PARTICULAR PURPOSE.
    
    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE, 
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND 
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP 
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO 
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL 
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT 
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS 
    SOFTWARE.
*/

#include <stdio.h>

#include "mcc_generated_files/mcc.h"
#include <string.h>

#define MCP9809_ADDR 0x18
#define MCP9808_REG_TA 0x05
#define LIGHT_SENSOR_ADC_CHANNEL 5

#define CMD_TABLE_SIZE      3

typedef struct{
	char const *cmd_name;
	void (*test_function)(void);
}command_t;

static char cmd_buff[20];
static uint8_t  buff_count = 0;


void static LED_TEST(void);
void static I2C_TEST_TEMPERATURE(void);
void static ADC_TEST_LIGHT(void);
void static TEST_HARDWARE_CLI(char ch);
void static ProccessCommand(char * cmd);

void SW1_InterruptHandler(void);
void SW0_InterruptHandler(void);

command_t const cmd_table[CMD_TABLE_SIZE] = {
        {"test_temp", I2C_TEST_TEMPERATURE},
		{"test_led", LED_TEST},
        {"test_light", ADC_TEST_LIGHT},
};

/*
    Main application
*/
int main(void)
{
    uint8_t ch;
    /* Initializes MCU, drivers and middleware */
    SYSTEM_Initialize();
    
    PORTF_SW1_SetInterruptHandler(SW1_InterruptHandler) ;
    PORTF_SW0_SetInterruptHandler(SW0_InterruptHandler) ;
    
    printf("/*********************************\n");
    printf("/* Author Quang Hai Nguyen \n");
    printf("/* Version 1.0.0 \n");
    printf("/* Test hardware program\n");
    printf("/*********************************\n");
    /* Replace with your application code */
    while (1){
        ch = USART2_Read();
        TEST_HARDWARE_CLI((char)ch);
    }
}

void static LED_TEST(void)
{
    printf("\n\n");
    printf("LED TEST\n");
    LED_ERR_Toggle();
    DELAY_milliseconds(200);
    LED_DATA_Toggle();
    DELAY_milliseconds(200);
    LED_CONN_Toggle();
    DELAY_milliseconds(200);
    LED_WIFI_Toggle();
    DELAY_milliseconds(500);
}


void static I2C_TEST_TEMPERATURE(void)
{
    int32_t temperature;

	temperature = i2c_read2ByteRegister(MCP9809_ADDR, MCP9808_REG_TA);

	temperature = temperature << 19;
	temperature = temperature >> 19;

	temperature *= 100;
	temperature /= 16;
    
    printf("temperature: %d\n", (int)temperature);
}

void static ADC_TEST_LIGHT(void)
{
    adc_result_t res;
    
	ADC0_StartConversion(LIGHT_SENSOR_ADC_CHANNEL);
	while (!ADC0_IsConversionDone());
	res = ADC0_GetConversionResult();
	ADC0.INTFLAGS |= ADC_RESRDY_bm;
    
    printf("light %d\n", (int)res);
}

void static TEST_HARDWARE_CLI(char ch)
{
    USART2_Write(ch);
    if('\n' == ch || '\r' == ch){
        
        //detect single \n character, we skip it
        if(0 == buff_count)
            return;
        
        ProccessCommand(cmd_buff);
        buff_count = 0;
        memset(cmd_buff, 0, 20);
    }
    else{
        if(buff_count == 20){
            ProccessCommand(cmd_buff);
            buff_count = 0;
            memset(cmd_buff, 0, 20);
        }
        else{
            cmd_buff[buff_count] = ch;
            ++buff_count;
        }
    }
}


void SW1_InterruptHandler(void)
{
    LED_ERR_Toggle();
}

void SW0_InterruptHandler(void)
{
    LED_ERR_Toggle();
}


void static ProccessCommand(char * cmd)
{
    for(uint8_t i = 0; i < CMD_TABLE_SIZE; i++){
        if(strcmp((const char*)cmd_table[i].cmd_name, (const char*)cmd ) == 0){
            cmd_table[i].test_function();
            return;
        }
    }
    printf("no command found\n");
}
/**
    End of File
*/