
#include <stdio.h>
#include "mcc_generated_files/mcc.h"

#include "mcc_generated_files/mcc.h"
#include "m2m/m2m_types.h"
#include <string.h>
#include <stdbool.h>
#include "mcc_generated_files/winc/include/winc.h"
#include "mcc_generated_files/winc/include/winc_legacy.h"
#include "mcc_generated_files/examples/winc_example.h"

#define VERSION                     00
#define SUB_VERSION                 99
#define MCP9809_ADDR                0x18
#define MCP9808_REG_TA              0x05
#define LIGHT_SENSOR_ADC_CHANNEL    5

#define CMD_TABLE_SIZE              6
 

typedef struct{
	char const *cmd_name;
	void (*test_function)(void);
}command_t;

static uint8_t ch;
static char cmd_buff[20];
static uint8_t  buff_count = 0;
static bool cmd_found = false;

void static LED_TEST(void);
void static I2C_TEST_TEMPERATURE(void);
void static ADC_TEST_LIGHT(void);
void static WIFI_TEST_SCAN(void);
void static WIFI_TEST_CONN(void);
void static WIFI_TEST_DISC(void);
void static TEST_HARDWARE_CLI(char ch);
void static ProccessCommand(char * cmd);

void wifi_cb(uint8_t u8WiFiEvent, const void *const pvMsg);
void winc_register_init(void);

void SW1_InterruptHandler(void);
void SW0_InterruptHandler(void);
void UART2_RXCallback(void);


command_t const cmd_table[CMD_TABLE_SIZE] = {
        {"test_temp", I2C_TEST_TEMPERATURE},
		{"test_led", LED_TEST},
        {"test_light", ADC_TEST_LIGHT},
        {"test_wifi_scan", WIFI_TEST_SCAN},
        {"test_wifi_conn", WIFI_TEST_CONN},
        {"test_wifi_disc", WIFI_TEST_DISC},
};
/*
    Main application
*/
int main(void)
{
    
    /* Initializes MCU, drivers and middleware */
    SYSTEM_Initialize();
    
    PORTF_SW1_SetInterruptHandler(SW1_InterruptHandler) ;
    PORTF_SW0_SetInterruptHandler(SW0_InterruptHandler) ;
    USART2_SetISRCb(UART2_RXCallback, USART2_RX_CB);
    
    printf("/*********************************\n");
    printf("/* Author Quang Hai Nguyen \n");
    printf("/* Version %2.%2 \n", VERSION, SUB_VERSION);
    printf("/* Test hardware program\n");
    printf("/*********************************\n");
    /* Replace with your application code */

    winc_register_init();
    winc_adapter_init();
    tstrWifiInitParam   param;

    m2m_memset((uint8_t *)&param, 0, sizeof(param));
    param.pfAppWifiCb   = wifi_cb;

    int8_t ret = m2m_wifi_init(&param);
    if (M2M_SUCCESS != ret){
        printf("wifi init error\n");
        return 0;//quit application
    }
      
    while (1){
        //If we have any command to process
        if(cmd_found){
            //Process command and cleanup
            ProccessCommand(cmd_buff);
            buff_count = 0;
            memset(cmd_buff, 0, 20);
            cmd_found = false;
       }
        
       m2m_wifi_handle_events(NULL);
    }
}

void static LED_TEST(void)
{
    printf("\n\n");
    printf("LED TEST\n");
    LED_ERR_SetHigh();
    LED_CONN_SetHigh();
    LED_DATA_SetHigh();
    LED_WIFI_SetHigh();
    DELAY_milliseconds(500);
    LED_ERR_SetLow();
    LED_CONN_SetLow();
    LED_DATA_SetLow();
    LED_WIFI_SetLow();
    DELAY_milliseconds(500);
    LED_ERR_SetHigh();
    LED_CONN_SetHigh();
    LED_DATA_SetHigh();
    LED_WIFI_SetHigh();
    DELAY_milliseconds(500);
    LED_ERR_SetLow();
    LED_CONN_SetLow();
    LED_DATA_SetLow();
    LED_WIFI_SetLow();
    DELAY_milliseconds(500);
    LED_ERR_SetLow();
    LED_CONN_SetLow();
    LED_DATA_SetLow();
    LED_WIFI_SetLow();
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
    //waiting for cartridge return to process the command  
    if('\n' == ch || '\r' == ch){
        //detect single \n character, we skip it
        if(0 == buff_count)
            return;
        cmd_found = true;
    }
    
    else{
        //buffer full, dump it by invoking process command
        if(buff_count == 20){
            cmd_found = true;
        }
        else{
            //saved received character
            cmd_buff[buff_count] = ch;
            ++buff_count;
        }
    }
}
void static WIFI_TEST_CONN(void)
{
    printf("\n\n");
    printf("WIFI TEST\n");
    printf("Connecting network\n");
    m2m_wifi_connect((char *)CFG_MAIN_WLAN_SSID, sizeof(CFG_MAIN_WLAN_SSID), CFG_MAIN_WLAN_AUTH, (void *)CFG_MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);
}
void static WIFI_TEST_DISC(void)
{
    printf("\n\n");
    printf("WIFI TEST\n");
    printf("Disconnecting network\n");
    m2m_wifi_disconnect();
}
void static WIFI_TEST_SCAN(void)
{
    printf("\n\n");
    printf("WIFI TEST\n");
    printf("Scanning network\n");
    m2m_wifi_request_scan(M2M_WIFI_CH_ALL);
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

void wifi_cb(uint8_t u8WiFiEvent, const void *const pvMsg)
{
    static uint8    u8ScanResultIdx = 0;
    switch(u8WiFiEvent)
    {
        case M2M_WIFI_RESP_CON_STATE_CHANGED:
            {
                tstrM2mWifiStateChanged *pstrInfo = (tstrM2mWifiStateChanged*)pvMsg;
                switch(pstrInfo->u8CurrState){
                    case M2M_WIFI_CONNECTED:
                        printf("connected\n");
                        m2m_wifi_get_connection_info(); 
                    break;
                    case M2M_WIFI_DISCONNECTED:
                        printf("disconnected\n");
                        break;
                    case M2M_WIFI_ROAMED:
                        printf("roam to new AP\n");
                    break;
                    case M2M_WIFI_UNDEF:
                        printf("undefined state\n");
                    break;
                }
            }
            break;
        case M2M_WIFI_RESP_CONN_INFO:
            {
                tstrM2MConnInfo *pstrInfo = (tstrM2MConnInfo*)pvMsg;
                printf("Connected to: %s\n", pstrInfo->acSSID);
                printf("IP; %d:%d:%d:%d\n", pstrInfo->au8IPAddr[0], pstrInfo->au8IPAddr[1], pstrInfo->au8IPAddr[2], pstrInfo->au8IPAddr[3]);
            }
            break;
        case M2M_WIFI_RESP_SCAN_DONE:
            {
                tstrM2mScanDone *pstrInfo = (tstrM2mScanDone*)pvMsg;
                printf("Number of AP found %d\n", pstrInfo->u8NumofCh);
                if(pstrInfo->s8ScanState == M2M_SUCCESS){
                    u8ScanResultIdx = 0;
                    if(pstrInfo->u8NumofCh >= 1){
                        m2m_wifi_req_scan_result(u8ScanResultIdx);
                        u8ScanResultIdx ++;
                    }
                    else{
                        printf("No AP Found Rescan\n");
                        m2m_wifi_request_scan(M2M_WIFI_CH_ALL);
                    }
                }
                else{
                    printf("(ERR) Scan fail with error <%d>\n",pstrInfo->s8ScanState);
                }
            }        
            break;
        case M2M_WIFI_RESP_SCAN_RESULT:
            {
                tstrM2mWifiscanResult *pstrScanResult = (tstrM2mWifiscanResult*)pvMsg;
                uint8 u8NumFoundAPs = m2m_wifi_get_num_ap_found();
                printf(">>%02d RI %d SEC %s CH %02d BSSID %02X:%02X:%02X:%02X:%02X:%02X SSID %s\n",
                        pstrScanResult->u8index,pstrScanResult->s8rssi,
                        pstrScanResult->u8AuthType,
                        pstrScanResult->u8ch,
                        pstrScanResult->au8BSSID[0], 
                        pstrScanResult->au8BSSID[1], pstrScanResult->au8BSSID[2],
                        pstrScanResult->au8BSSID[3], pstrScanResult->au8BSSID[4],
                        pstrScanResult->au8BSSID[5], pstrScanResult->au8SSID);
                if(u8ScanResultIdx < u8NumFoundAPs)
                {
                    // Read the next scan result
                    m2m_wifi_req_scan_result(u8ScanResultIdx);
                    u8ScanResultIdx ++;
                }
            }
            break;
        default:
            break;
    }
}

void winc_register_init(void)
{
}

void UART2_RXCallback(void)
{
    ch = USART2.RXDATAL;
    USART2.TXDATAL = ch;
    TEST_HARDWARE_CLI((char)ch);
}
/**
    End of File
*/