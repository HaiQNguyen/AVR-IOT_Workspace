/*
    \file   mqtt_config.h

    \brief  MQTT Configuration File

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

#ifndef MQTT_CONFIG_H
#define	MQTT_CONFIG_H


#include <stdint.h>


/********************MQTT Client configurations***********************/
#define TOPIC_SIZE                  100     //Defines the topic length that is supported when we process a published packet
#define PAYLOAD_SIZE                200     //Defines the payload size that is supported when we process a published packet
#define NUM_TOPICS_SUBSCRIBE        1       //Defines number of topics which can be subscribed
#define NUM_TOPICS_UNSUBSCRIBE      NUM_TOPICS_SUBSCRIBE	// The MQTT client can unsubscribe only from those topics to which it has already subscribed

// MCC generated parameters
#define CFG_MQTT_PORT 1883
#define CFG_MQTT_CONN_TIMEOUT 120
#define CFG_MQTT_HOSTURL "quickstart.messaging.internetofthings.ibmcloud.com"
#define CFG_MQTT_BROKERIP 0x239ca017
#define CFG_MQTT_TXBUFFER_SIZE 400
#define CFG_MQTT_RXBUFFER_SIZE 200
#define CFG_MQTT_USERNAME NULL
#define CFG_MQTT_PASSWORD NULL
#define CFG_QOS 0
#define CFG_PUBTOPIC "iot-2/evt/status/fmt/json"
#define CFG_SUBTOPIC "mchp/iot/config"
#define TCPIP_BSD 1


/********************MQTT Client configurations*(END)***********************/


/********************Timeout Driver for MQTT definitions***********************/
#ifdef TCPIP_BSD
#include "../drivers/timeout.h"
#define timerstruct_t                   timerStruct_t
#define htons(a)                        (uint16_t)((((uint16_t) (a)) << 8) | (((uint16_t) (a)) >> 8))
#define ntohs(a)                        (uint16_t)((((uint16_t) (a)) << 8) | (((uint16_t) (a)) >> 8)) // Socket.h remapped to htons

// Timeout is calculated on the basis of clock frequency.
// This macro needs to be changed in accordance with the clock frequency.
#define SECONDS (uint32_t)1000

#endif /* TCPIP_BSD */


#ifdef TCPIP_LITE

#define absolutetime_t                  uint32_t
#define timerstruct_t                   timerStruct_t

// Timeout is calculated on the basis of clock frequency.
// This macro needs to be changed in accordance with the clock frequency.
#define SECONDS (uint32_t)100

#endif /* TCPIP_LITE */

/*******************Timeout Driver for MQTT definitions*(END)******************/



#endif	/* MQTT_CONFIG_H */
