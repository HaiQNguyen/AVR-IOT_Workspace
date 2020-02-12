
#include <stdio.h>
#include "clock_config.h"
#include <util/delay.h>
#include <string.h>
#include <stdbool.h>
#include "mcc_generated_files/mcc.h"
#include "m2m/m2m_types.h"
#include "mcc_generated_files/winc/include/winc.h"
#include "mcc_generated_files/winc/include/winc_legacy.h"
#include "mcc_generated_files/winc/m2m/m2m_wifi.h"
#include "mcc_generated_files/winc/socket/socket.h"
#include "mcc_generated_files/winc/common/winc_defines.h"
#include "mcc_generated_files/winc/driver/winc_adapter.h"
#include "mcc_generated_files/mqtt/mqtt_core/mqtt_core.h"
#include "mcc_generated_files/winc/m2m/m2m_types.h"
#include "mcc_generated_files/config/mqtt_config.h"


/* Define Section *************************************************************/
#define VERSION                     00
#define SUB_VERSION                 99
#define MCP9809_ADDR                0x18
#define MCP9808_REG_TA              0x05
#define LIGHT_SENSOR_ADC_CHANNEL    5
#define CMD_TABLE_SIZE              8
#define IBM_BLUEMIX                 1

#ifdef IBM_BLUEMIX
#define CLIENT_ID           "d:quickstart:ARROW_QUADRO:AABBCCDDEEFF"
#define MQTT_PUBTOPIC       "iot-2/evt/status/fmt/json"
#define MQTT_SUBTOPIC       "mchp/iot/config"
#define MQTT_HOSTURL        "quickstart.messaging.internetofthings.ibmcloud.com"
#define MQTT_TIMEOUT        120
#endif
/* Global variable*************************************************************/
typedef struct{
	char const *cmd_name;
	void (*test_function)(void);
}command_t;

typedef enum
{
    APP_STATE_INIT,
    APP_STATE_STA_CONNECTING,
    APP_STATE_STA_CONNECTED,
    APP_STATE_WAIT_FOR_DNS,
    APP_STATE_TLS_START,
    APP_STATE_TLS_CONNECTING,
    APP_STATE_TLS_CONNECTED,
    APP_STATE_ERROR,
    APP_STATE_STOPPED
} appStates_e;

/* Local variables*************************************************************/
static char mqttPublishMsg[100];
static char cmd_buff[20];
static const char mqttSubTopic[] = CFG_SUBTOPIC;
static const char mqttPublishTopic[] = CFG_PUBTOPIC;

static bool appMQTTPublishTimeoutOccured = false;
static bool mqtt_test_activated = false;
static bool cmd_found = false;

static uint8_t recvBuffer[256];
static uint8_t ch;
static uint8_t  buff_count = 0;

static uint32_t serverIPAddress = 0;

static appStates_e appState = APP_STATE_INIT;


/*Local function prototype*****************************************************/
static void app_buildMQTTConnectPacket(void);
static void changeState(appStates_e newState);
static void dnsHandler(uint8_t *pu8DomainName, uint32_t u32ServerIP);
static void app_buildPublishPacket(void);
static void socketHandler(SOCKET sock, uint8_t u8Msg, void *pvMsg);

static uint32_t appCheckMQTTPublishTimeout();
void static LED_TEST(void);
void static I2C_TEST_TEMPERATURE(void);
void static ADC_TEST_LIGHT(void);
void static WIFI_TEST_SCAN(void);
void static WIFI_TEST_CONN(void);
void static WIFI_TEST_DISC(void);
void static MQTT_TEST_START(void);
void static MQTT_TEST_STOP(void);
void static TEST_HARDWARE_CLI(char ch);
void static ProccessCommand(char * cmd);

static uint32_t GetTemperature(void);
static uint32_t GetLight(void);

/* Global function prototype***************************************************/
void wifi_cb(uint8_t u8WiFiEvent, const void *const pvMsg);
void winc_register_init(void);
void app_mqttScheduler(void);
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
        {"test_mqtt_start", MQTT_TEST_START},
        {"test_mqtt_stop", MQTT_TEST_STOP},
};

timerStruct_t appMQTTPublishTimer = {appCheckMQTTPublishTimeout, NULL};
/*
    Main application
*/
int main(void)
{
    /* Initializes MCU, drivers and middleware */
    SYSTEM_Initialize();

    /* Replace with your application code */
    printf("\x1b[2J");
    printf("/*********************************\n");
    printf("/* Author Quang Hai Nguyen \n");
    printf("/* Version %2d.%2d \n", VERSION, SUB_VERSION);
    printf("/* Test hardware program\n");
    printf("/*********************************\n");
    
    /*Init Wifi*/
    winc_register_init();
    winc_adapter_init();
    tstrWifiInitParam   param;
    
    m2m_memset((uint8_t *)&param, 0, sizeof(param));
    param.pfAppWifiCb   = wifi_cb;
    
    int8_t ret = m2m_wifi_init(&param);
    if (M2M_SUCCESS != ret){
        printf("[ERR] wifi init error\n");
        printf("[ERR] error code 0x%x. Quit application...\n", ret);
        return 0;//quit application  
    }
    
    PORTF_SW1_SetInterruptHandler(SW1_InterruptHandler) ;
    PORTF_SW0_SetInterruptHandler(SW0_InterruptHandler) ;
    USART2_SetISRCb(UART2_RXCallback, USART2_RX_CB);
      
    while (1){
        //If we have any command to process
        if(cmd_found){
            //Process command and cleanup
            ProccessCommand(cmd_buff);
            buff_count = 0;
            memset(cmd_buff, 0, 20);
            cmd_found = false;
       }
        // If mqtt test is activated, run the scheduler
       if(mqtt_test_activated){
           app_mqttScheduler();
       }       
       m2m_wifi_handle_events(NULL);
    }
}

/* Interrupt Handlers**********************************************************/
void SW1_InterruptHandler(void)
{
    LED_ERR_Toggle();
}

void SW0_InterruptHandler(void)
{
    LED_ERR_Toggle();
}

void UART2_RXCallback(void)
{
    ch = USART2.RXDATAL;
    USART2.TXDATAL = ch;
    TEST_HARDWARE_CLI((char)ch);
}

/* TEST CASE*******************************************************************/
void static LED_TEST(void)
{
    printf("\n\n");
    printf("[INFO] LED TEST\n");
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
    int temperature = GetTemperature();
    printf("[INFO] temperature: %d\n", temperature);
}

void static ADC_TEST_LIGHT(void)
{
    int res = GetLight();
    printf("[INFO] light %d\n", res);
}


void static WIFI_TEST_CONN(void)
{
    printf("\n\n");
    printf("[INFO] WIFI TEST\n");
    printf("[INFO] Connecting network\n");
    
    m2m_wifi_connect((char *)CFG_MAIN_WLAN_SSID, sizeof(CFG_MAIN_WLAN_SSID), 
            CFG_MAIN_WLAN_AUTH, (void *)CFG_MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);
}
void static WIFI_TEST_DISC(void)
{
    printf("\n\n");
    printf("[INFO] WIFI TEST\n");
    printf("[INFO] Disconnecting network\n");
    m2m_wifi_disconnect();
}
void static WIFI_TEST_SCAN(void)
{
    printf("\n\n");
    printf("[INFO] WIFI TEST\n");
    printf("[INFO] Scanning network\n");
    m2m_wifi_request_scan(M2M_WIFI_CH_ALL);
}

void static MQTT_TEST_START(void)
{
    printf("\n\n");
    printf("[INFO] MQTT Start\n");
    
    MQTT_ClientInitialise();
    app_buildMQTTConnectPacket();   
    timeout_create(&appMQTTPublishTimer, ((MQTT_TIMEOUT - 1) * SECONDS));
    
    m2m_wifi_set_sleep_mode(M2M_PS_DEEP_AUTOMATIC, 1);
    changeState(APP_STATE_INIT);
    mqtt_test_activated = true; 
}

void static MQTT_TEST_STOP(void)
{
    printf("\n\n");
    printf("[INFO] MQTT Stop\n");
    changeState(APP_STATE_ERROR);
}

/* Other Functions*************************************************************/

static uint32_t GetTemperature(void)
{
    int32_t temperature;

	temperature = i2c_read2ByteRegister(MCP9809_ADDR, MCP9808_REG_TA);

	temperature = temperature << 19;
	temperature = temperature >> 19;

	temperature *= 100;
	temperature /= 16;
    
    return temperature;
}

static uint32_t GetLight(void)
{
    adc_result_t res;
    
	ADC0_StartConversion(LIGHT_SENSOR_ADC_CHANNEL);
	while (!ADC0_IsConversionDone());
	res = ADC0_GetConversionResult();
	ADC0.INTFLAGS |= ADC_RESRDY_bm;
    
    return (uint32_t)res;
}

void static ProccessCommand(char * cmd)
{
    for(uint8_t i = 0; i < CMD_TABLE_SIZE; i++){
        if(strcmp((const char*)cmd_table[i].cmd_name, (const char*)cmd ) == 0){
            cmd_table[i].test_function();
            return;
        }
    }
    printf("[INFO] no command found\n");
}


void winc_register_init()
{
}


static void app_buildMQTTConnectPacket(void)
{
    mqttConnectPacket appConnectPacket;

    memset(&appConnectPacket, 0, sizeof(mqttConnectPacket));

    appConnectPacket.connectVariableHeader.connectFlagsByte.All = 0x02;
    
    //Packets need to be sent to the server every 10s.
    appConnectPacket.connectVariableHeader.keepAliveTimer = MQTT_TIMEOUT;
    appConnectPacket.clientID = (uint8_t *)CLIENT_ID;

    MQTT_CreateConnectPacket(&appConnectPacket);
}

static uint32_t appCheckMQTTPublishTimeout()
{
    appMQTTPublishTimeoutOccured = true; // Mark that timer has executed
    return ((CFG_MQTT_CONN_TIMEOUT - 1) * SECONDS);
}

static void changeState(appStates_e newState)
{
    if (newState != appState)
    {
        appState = newState;
    }
}

static void app_buildPublishPacket(void)
{
    mqttPublishPacket appPublishPacket;
    
    uint32_t light = GetLight();
    uint32_t temp = GetTemperature();
    memset(&appPublishPacket, 0, sizeof(mqttPublishPacket));
    sprintf(mqttPublishMsg, "{\"d\":{\"myName\":\"AVR-IoT\",\"atm\":%d,\"temp\":%d}}", (int)light, (int)temp);
    
    appPublishPacket.topic = (uint8_t *)mqttPublishTopic;
    appPublishPacket.payload = (uint8_t *)mqttPublishMsg;

    // Fixed header
    appPublishPacket.publishHeaderFlags.duplicate = 0;
    appPublishPacket.publishHeaderFlags.qos = CFG_QOS;
    appPublishPacket.publishHeaderFlags.retain = 0;
    
    // Variable header
    appPublishPacket.packetIdentifierLSB = 10;
    appPublishPacket.packetIdentifierMSB = 0;
    appPublishPacket.payloadLength = strlen((const char *)appPublishPacket.payload);

    MQTT_CreatePublishPacket(&appPublishPacket);
    
    printf("[INFO] sent message: %s\n", mqttPublishMsg);
}

/* Scheduler and Handler*******************************************************/
void static TEST_HARDWARE_CLI(char ch)
{
    //waiting for cartridge return to process the command  
    if('\n' == ch || '\r' == ch){
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

void wifi_cb(uint8_t u8WiFiEvent, const void *const pvMsg)
{
    static uint8    u8ScanResultIdx = 0;
    switch(u8WiFiEvent)
    {
        case M2M_WIFI_REQ_DHCP_CONF:
            {
                printf("[INFO] received an IP\n");
            }
        
            //If MQTT test is running, we tell the scheduler to move to the new state
            if(mqtt_test_activated){
                changeState(APP_STATE_STA_CONNECTED);
            }
        case M2M_WIFI_RESP_CON_STATE_CHANGED:
            {
                tstrM2mWifiStateChanged *pstrInfo = (tstrM2mWifiStateChanged*)pvMsg;
                switch(pstrInfo->u8CurrState){
                    case M2M_WIFI_CONNECTED:
                        printf("[INFO] connected to an AP\n");
                        m2m_wifi_get_connection_info(); 
                    break;
                    case M2M_WIFI_DISCONNECTED:
                        printf("[INFO] disconnected from an AP\n");
                        break;
                    case M2M_WIFI_ROAMED:
                        printf("[INFO] roam to new an AP\n");
                    break;
                    case M2M_WIFI_UNDEF:
                        printf("[INFO] undefined state\n");
                    break;
                }
            }
            break;
        case M2M_WIFI_RESP_CONN_INFO:
            {
                tstrM2MConnInfo *pstrInfo = (tstrM2MConnInfo*)pvMsg;
                printf("[INFO] Connected to: %s\n", pstrInfo->acSSID);
                
            }
            break;
        case M2M_WIFI_RESP_SCAN_DONE:
            {
                tstrM2mScanDone *pstrInfo = (tstrM2mScanDone*)pvMsg;
                printf("[INFO] Number of AP found %d\n", pstrInfo->u8NumofCh);
                if(pstrInfo->s8ScanState == M2M_SUCCESS){
                    u8ScanResultIdx = 0;
                    if(pstrInfo->u8NumofCh >= 1){
                        m2m_wifi_req_scan_result(u8ScanResultIdx);
                        u8ScanResultIdx ++;
                    }
                    else{
                        printf("[INFO] No AP Found Rescan\n");
                        m2m_wifi_request_scan(M2M_WIFI_CH_ALL);
                    }
                }
                else{
                    printf("[ERR] Scan fail with error <%d>\n",pstrInfo->s8ScanState);
                }
            }        
            break;
        case M2M_WIFI_RESP_SCAN_RESULT:
            {
                tstrM2mWifiscanResult *pstrScanResult = (tstrM2mWifiscanResult*)pvMsg;
                uint8 u8NumFoundAPs = m2m_wifi_get_num_ap_found();
                printf("[INFO] >>%02d RI %d SEC %s CH %02d BSSID %02X:%02X:%02X:%02X:%02X:%02X SSID %s\n",
                        pstrScanResult->u8index,
                        pstrScanResult->s8rssi,
                        pstrScanResult->u8AuthType,
                        pstrScanResult->u8ch,
                        pstrScanResult->au8BSSID[0], pstrScanResult->au8BSSID[1],
                        pstrScanResult->au8BSSID[2], pstrScanResult->au8BSSID[3], 
                        pstrScanResult->au8BSSID[4], pstrScanResult->au8BSSID[5], 
                        pstrScanResult->au8SSID);
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

static void dnsHandler(uint8_t *pu8DomainName, uint32_t u32ServerIP)
{
    if (u32ServerIP)
    {
        if (appState == APP_STATE_WAIT_FOR_DNS)
        {
            printf("[INFO] Getting IP of %s...\n", CLIENT_ID);
            printf("[INFO] IP = (%lu.%lu.%lu.%lu)\n",
                    (0x0FF & (u32ServerIP)),(0x0FF & (u32ServerIP>>8)),
                    (0x0FF & (u32ServerIP>>16)),(0x0FF & (u32ServerIP>>24)));
            changeState(APP_STATE_TLS_START);
        }
    }
    serverIPAddress = u32ServerIP;
}

void app_mqttScheduler(void)
{
    mqttContext* mqttConnnectionInfo = MQTT_GetClientConnectionInfo();
    
    timeout_callNextCallback();
    
    switch(appState)
    {
        case APP_STATE_INIT:
        {
            if(M2M_SUCCESS != m2m_wifi_connect((char *)CFG_MAIN_WLAN_SSID, 
                    sizeof(CFG_MAIN_WLAN_SSID), CFG_MAIN_WLAN_AUTH, 
                    (void *)CFG_MAIN_WLAN_PSK, M2M_WIFI_CH_ALL))
            {
                changeState(APP_STATE_ERROR);
                break;
            }

            changeState(APP_STATE_STA_CONNECTING);
            break;
        }

        case APP_STATE_STA_CONNECTING:
        {
            break;
        }

        case APP_STATE_STA_CONNECTED:
        {
            socketInit();
            registerSocketCallback(socketHandler, dnsHandler);
            if (gethostbyname((const char *)CFG_MQTT_HOSTURL) == SOCK_ERR_INVALID_ARG )
            {
				printf("[ERR] host look up error\n");
                printf("[ERR] MQTT test will be stopped\n");
                changeState(APP_STATE_ERROR);
            }
            changeState(APP_STATE_WAIT_FOR_DNS);
            break;
        }

        case APP_STATE_WAIT_FOR_DNS:
        {
            break;
        }
        
        case APP_STATE_TLS_START:
        {
            struct sockaddr_in addr;

            if (*mqttConnnectionInfo->tcpClientSocket != -1)
            {
                close(*mqttConnnectionInfo->tcpClientSocket);
            }

            if((*mqttConnnectionInfo->tcpClientSocket = socket(AF_INET, SOCK_STREAM, 0)))
            {
                changeState(APP_STATE_ERROR);
                break;
            }
            addr.sin_family      = AF_INET;
            addr.sin_port        = _htons(CFG_MQTT_PORT);
            addr.sin_addr.s_addr = serverIPAddress;
            
            if (connect(*mqttConnnectionInfo->tcpClientSocket, 
                    (struct sockaddr *)&addr, sizeof(struct sockaddr_in)) < 0)
            {
                
                close(*mqttConnnectionInfo->tcpClientSocket);
                *mqttConnnectionInfo->tcpClientSocket = -1;
                changeState(APP_STATE_ERROR);   
                break;
            }
            changeState(APP_STATE_TLS_CONNECTING);

            break;
        }

        case APP_STATE_TLS_CONNECTING:
        {
            break;
        }

        case APP_STATE_TLS_CONNECTED:
        {
            if(appMQTTPublishTimeoutOccured == true)
            {
                appMQTTPublishTimeoutOccured = false;
                app_buildPublishPacket();    
            }

            MQTT_ReceptionHandler(mqttConnnectionInfo);
            MQTT_TransmissionHandler(mqttConnnectionInfo);
        }
        break;

        case APP_STATE_ERROR:
        {
            m2m_wifi_deinit(NULL);
            timeout_delete(&appMQTTPublishTimer);
            changeState(APP_STATE_STOPPED);

            break;
        }

        case APP_STATE_STOPPED:
        {
            printf("[INFO] MQTT stopped");
            mqtt_test_activated = false;
            break;
        }
    }
}

static void socketHandler(SOCKET sock, uint8_t u8Msg, void *pvMsg)
{
    switch (u8Msg)
    {
        case SOCKET_MSG_CONNECT:
        {
            mqttContext* mqttConnnectionInfo = MQTT_GetClientConnectionInfo();
            tstrSocketConnectMsg *pstrConnect = (tstrSocketConnectMsg *)pvMsg;

            if (pstrConnect && pstrConnect->s8Error >= 0)
            {
                changeState(APP_STATE_TLS_CONNECTED);

                recv(sock, recvBuffer, sizeof(recvBuffer), 0);
                printf("[INFO] connected to socket\n");
            }
            else
            {
                printf("[ERR] socket error: %x\n", pstrConnect->s8Error);
                changeState(APP_STATE_ERROR);

                *mqttConnnectionInfo->tcpClientSocket = -1;
                close(sock);
            }

            break;
        }

        case SOCKET_MSG_RECV:
        {
            mqttContext* mqttConnnectionInfo = MQTT_GetClientConnectionInfo();
            tstrSocketRecvMsg *pstrRecv = (tstrSocketRecvMsg *)pvMsg;

            if (pstrRecv && pstrRecv->s16BufferSize > 0)
            {                
                MQTT_GetReceivedData(pstrRecv->pu8Buffer, 
                        pstrRecv->s16BufferSize);
            }
            else
            {
                printf("[ERR] socket receive error\n");
                changeState(APP_STATE_ERROR);
                *mqttConnnectionInfo->tcpClientSocket = -1;
                close(sock);
            }
            break;
        }

        case SOCKET_MSG_SEND:
        {
            printf("[INFO] socket send \n");
            break;
        }
    }
}
/**
    End of File
*/