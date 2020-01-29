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
#ifndef PINS_H_INCLUDED
#define PINS_H_INCLUDED

#include <avr/io.h>
#include "port.h"

//get/set PD5 aliases
#define PD5_SetHigh() do { PORTD_OUTSET = 0x20; } while(0)
#define PD5_SetLow() do { PORTD_OUTCLR = 0x20; } while(0)
#define PD5_Toggle() do { PORTD_OUTTGL = 0x20; } while(0)
#define PD5_GetValue() (VPORTD.IN & (0x1 << 5))
#define PD5_SetDigitalInput() do { PORTD_DIRCLR = 0x20; } while(0)
#define PD5_SetDigitalOutput() do { PORTD_DIRSET = 0x20; } while(0)
#define PD5_SetPullUp() do { PORTD_PIN5CTRL  |= PORT_PULLUPEN_bm; } while(0)
#define PD5_ResetPullUp() do { PORTD_PIN5CTRL  &= ~PORT_PULLUPEN_bm; } while(0)
#define PD5_SetInverted() do { PORTD_PIN5CTRL  |= PORT_INVEN_bm; } while(0)
#define PD5_ResetInverted() do { PORTD_PIN5CTRL  &= ~PORT_INVEN_bm; } while(0)
#define PD5_DisableInterruptOnChange() do { PORTD.PIN5CTRL = (PORTD.PIN5CTRL & ~PORT_ISC_gm) | 0x0 ; } while(0)
#define PD5_EnableInterruptForBothEdges() do { PORTD.PIN5CTRL = (PORTD.PIN5CTRL & ~PORT_ISC_gm) | 0x1 ; } while(0)
#define PD5_EnableInterruptForRisingEdge() do { PORTD.PIN5CTRL = (PORTD.PIN5CTRL & ~PORT_ISC_gm) | 0x2 ; } while(0)
#define PD5_EnableInterruptForFallingEdge() do { PORTD.PIN5CTRL = (PORTD.PIN5CTRL & ~PORT_ISC_gm) | 0x3 ; } while(0)
#define PD5_DisableDigitalInputBuffer() do { PORTD.PIN5CTRL = (PORTD.PIN5CTRL & ~PORT_ISC_gm) | 0x4 ; } while(0)
#define PD5_EnableInterruptForLowLevelSensing() do { PORTD.PIN5CTRL = (PORTD.PIN5CTRL & ~PORT_ISC_gm) | 0x5 ; } while(0)

//get/set PA2 aliases
#define PA2_SetHigh() do { PORTA_OUTSET = 0x4; } while(0)
#define PA2_SetLow() do { PORTA_OUTCLR = 0x4; } while(0)
#define PA2_Toggle() do { PORTA_OUTTGL = 0x4; } while(0)
#define PA2_GetValue() (VPORTA.IN & (0x1 << 2))
#define PA2_SetDigitalInput() do { PORTA_DIRCLR = 0x4; } while(0)
#define PA2_SetDigitalOutput() do { PORTA_DIRSET = 0x4; } while(0)
#define PA2_SetPullUp() do { PORTA_PIN2CTRL  |= PORT_PULLUPEN_bm; } while(0)
#define PA2_ResetPullUp() do { PORTA_PIN2CTRL  &= ~PORT_PULLUPEN_bm; } while(0)
#define PA2_SetInverted() do { PORTA_PIN2CTRL  |= PORT_INVEN_bm; } while(0)
#define PA2_ResetInverted() do { PORTA_PIN2CTRL  &= ~PORT_INVEN_bm; } while(0)
#define PA2_DisableInterruptOnChange() do { PORTA.PIN2CTRL = (PORTA.PIN2CTRL & ~PORT_ISC_gm) | 0x0 ; } while(0)
#define PA2_EnableInterruptForBothEdges() do { PORTA.PIN2CTRL = (PORTA.PIN2CTRL & ~PORT_ISC_gm) | 0x1 ; } while(0)
#define PA2_EnableInterruptForRisingEdge() do { PORTA.PIN2CTRL = (PORTA.PIN2CTRL & ~PORT_ISC_gm) | 0x2 ; } while(0)
#define PA2_EnableInterruptForFallingEdge() do { PORTA.PIN2CTRL = (PORTA.PIN2CTRL & ~PORT_ISC_gm) | 0x3 ; } while(0)
#define PA2_DisableDigitalInputBuffer() do { PORTA.PIN2CTRL = (PORTA.PIN2CTRL & ~PORT_ISC_gm) | 0x4 ; } while(0)
#define PA2_EnableInterruptForLowLevelSensing() do { PORTA.PIN2CTRL = (PORTA.PIN2CTRL & ~PORT_ISC_gm) | 0x5 ; } while(0)

//get/set SW0 aliases
#define SW0_SetHigh() do { PORTF_OUTSET = 0x40; } while(0)
#define SW0_SetLow() do { PORTF_OUTCLR = 0x40; } while(0)
#define SW0_Toggle() do { PORTF_OUTTGL = 0x40; } while(0)
#define SW0_GetValue() (VPORTF.IN & (0x1 << 6))
#define SW0_SetDigitalInput() do { PORTF_DIRCLR = 0x40; } while(0)
#define SW0_SetDigitalOutput() do { PORTF_DIRSET = 0x40; } while(0)
#define SW0_SetPullUp() do { PORTF_PIN6CTRL  |= PORT_PULLUPEN_bm; } while(0)
#define SW0_ResetPullUp() do { PORTF_PIN6CTRL  &= ~PORT_PULLUPEN_bm; } while(0)
#define SW0_SetInverted() do { PORTF_PIN6CTRL  |= PORT_INVEN_bm; } while(0)
#define SW0_ResetInverted() do { PORTF_PIN6CTRL  &= ~PORT_INVEN_bm; } while(0)
#define SW0_DisableInterruptOnChange() do { PORTF.PIN6CTRL = (PORTF.PIN6CTRL & ~PORT_ISC_gm) | 0x0 ; } while(0)
#define SW0_EnableInterruptForBothEdges() do { PORTF.PIN6CTRL = (PORTF.PIN6CTRL & ~PORT_ISC_gm) | 0x1 ; } while(0)
#define SW0_EnableInterruptForRisingEdge() do { PORTF.PIN6CTRL = (PORTF.PIN6CTRL & ~PORT_ISC_gm) | 0x2 ; } while(0)
#define SW0_EnableInterruptForFallingEdge() do { PORTF.PIN6CTRL = (PORTF.PIN6CTRL & ~PORT_ISC_gm) | 0x3 ; } while(0)
#define SW0_DisableDigitalInputBuffer() do { PORTF.PIN6CTRL = (PORTF.PIN6CTRL & ~PORT_ISC_gm) | 0x4 ; } while(0)
#define SW0_EnableInterruptForLowLevelSensing() do { PORTF.PIN6CTRL = (PORTF.PIN6CTRL & ~PORT_ISC_gm) | 0x5 ; } while(0)

//get/set CONF_WIFI_M2M_RESET_PIN aliases
#define CONF_WIFI_M2M_RESET_PIN_SetHigh() do { PORTA_OUTSET = 0x2; } while(0)
#define CONF_WIFI_M2M_RESET_PIN_SetLow() do { PORTA_OUTCLR = 0x2; } while(0)
#define CONF_WIFI_M2M_RESET_PIN_Toggle() do { PORTA_OUTTGL = 0x2; } while(0)
#define CONF_WIFI_M2M_RESET_PIN_GetValue() (VPORTA.IN & (0x1 << 1))
#define CONF_WIFI_M2M_RESET_PIN_SetDigitalInput() do { PORTA_DIRCLR = 0x2; } while(0)
#define CONF_WIFI_M2M_RESET_PIN_SetDigitalOutput() do { PORTA_DIRSET = 0x2; } while(0)
#define CONF_WIFI_M2M_RESET_PIN_SetPullUp() do { PORTA_PIN1CTRL  |= PORT_PULLUPEN_bm; } while(0)
#define CONF_WIFI_M2M_RESET_PIN_ResetPullUp() do { PORTA_PIN1CTRL  &= ~PORT_PULLUPEN_bm; } while(0)
#define CONF_WIFI_M2M_RESET_PIN_SetInverted() do { PORTA_PIN1CTRL  |= PORT_INVEN_bm; } while(0)
#define CONF_WIFI_M2M_RESET_PIN_ResetInverted() do { PORTA_PIN1CTRL  &= ~PORT_INVEN_bm; } while(0)
#define CONF_WIFI_M2M_RESET_PIN_DisableInterruptOnChange() do { PORTA.PIN1CTRL = (PORTA.PIN1CTRL & ~PORT_ISC_gm) | 0x0 ; } while(0)
#define CONF_WIFI_M2M_RESET_PIN_EnableInterruptForBothEdges() do { PORTA.PIN1CTRL = (PORTA.PIN1CTRL & ~PORT_ISC_gm) | 0x1 ; } while(0)
#define CONF_WIFI_M2M_RESET_PIN_EnableInterruptForRisingEdge() do { PORTA.PIN1CTRL = (PORTA.PIN1CTRL & ~PORT_ISC_gm) | 0x2 ; } while(0)
#define CONF_WIFI_M2M_RESET_PIN_EnableInterruptForFallingEdge() do { PORTA.PIN1CTRL = (PORTA.PIN1CTRL & ~PORT_ISC_gm) | 0x3 ; } while(0)
#define CONF_WIFI_M2M_RESET_PIN_DisableDigitalInputBuffer() do { PORTA.PIN1CTRL = (PORTA.PIN1CTRL & ~PORT_ISC_gm) | 0x4 ; } while(0)
#define CONF_WIFI_M2M_RESET_PIN_EnableInterruptForLowLevelSensing() do { PORTA.PIN1CTRL = (PORTA.PIN1CTRL & ~PORT_ISC_gm) | 0x5 ; } while(0)

//get/set PA3 aliases
#define PA3_SetHigh() do { PORTA_OUTSET = 0x8; } while(0)
#define PA3_SetLow() do { PORTA_OUTCLR = 0x8; } while(0)
#define PA3_Toggle() do { PORTA_OUTTGL = 0x8; } while(0)
#define PA3_GetValue() (VPORTA.IN & (0x1 << 3))
#define PA3_SetDigitalInput() do { PORTA_DIRCLR = 0x8; } while(0)
#define PA3_SetDigitalOutput() do { PORTA_DIRSET = 0x8; } while(0)
#define PA3_SetPullUp() do { PORTA_PIN3CTRL  |= PORT_PULLUPEN_bm; } while(0)
#define PA3_ResetPullUp() do { PORTA_PIN3CTRL  &= ~PORT_PULLUPEN_bm; } while(0)
#define PA3_SetInverted() do { PORTA_PIN3CTRL  |= PORT_INVEN_bm; } while(0)
#define PA3_ResetInverted() do { PORTA_PIN3CTRL  &= ~PORT_INVEN_bm; } while(0)
#define PA3_DisableInterruptOnChange() do { PORTA.PIN3CTRL = (PORTA.PIN3CTRL & ~PORT_ISC_gm) | 0x0 ; } while(0)
#define PA3_EnableInterruptForBothEdges() do { PORTA.PIN3CTRL = (PORTA.PIN3CTRL & ~PORT_ISC_gm) | 0x1 ; } while(0)
#define PA3_EnableInterruptForRisingEdge() do { PORTA.PIN3CTRL = (PORTA.PIN3CTRL & ~PORT_ISC_gm) | 0x2 ; } while(0)
#define PA3_EnableInterruptForFallingEdge() do { PORTA.PIN3CTRL = (PORTA.PIN3CTRL & ~PORT_ISC_gm) | 0x3 ; } while(0)
#define PA3_DisableDigitalInputBuffer() do { PORTA.PIN3CTRL = (PORTA.PIN3CTRL & ~PORT_ISC_gm) | 0x4 ; } while(0)
#define PA3_EnableInterruptForLowLevelSensing() do { PORTA.PIN3CTRL = (PORTA.PIN3CTRL & ~PORT_ISC_gm) | 0x5 ; } while(0)

//get/set PF1 aliases
#define PF1_SetHigh() do { PORTF_OUTSET = 0x2; } while(0)
#define PF1_SetLow() do { PORTF_OUTCLR = 0x2; } while(0)
#define PF1_Toggle() do { PORTF_OUTTGL = 0x2; } while(0)
#define PF1_GetValue() (VPORTF.IN & (0x1 << 1))
#define PF1_SetDigitalInput() do { PORTF_DIRCLR = 0x2; } while(0)
#define PF1_SetDigitalOutput() do { PORTF_DIRSET = 0x2; } while(0)
#define PF1_SetPullUp() do { PORTF_PIN1CTRL  |= PORT_PULLUPEN_bm; } while(0)
#define PF1_ResetPullUp() do { PORTF_PIN1CTRL  &= ~PORT_PULLUPEN_bm; } while(0)
#define PF1_SetInverted() do { PORTF_PIN1CTRL  |= PORT_INVEN_bm; } while(0)
#define PF1_ResetInverted() do { PORTF_PIN1CTRL  &= ~PORT_INVEN_bm; } while(0)
#define PF1_DisableInterruptOnChange() do { PORTF.PIN1CTRL = (PORTF.PIN1CTRL & ~PORT_ISC_gm) | 0x0 ; } while(0)
#define PF1_EnableInterruptForBothEdges() do { PORTF.PIN1CTRL = (PORTF.PIN1CTRL & ~PORT_ISC_gm) | 0x1 ; } while(0)
#define PF1_EnableInterruptForRisingEdge() do { PORTF.PIN1CTRL = (PORTF.PIN1CTRL & ~PORT_ISC_gm) | 0x2 ; } while(0)
#define PF1_EnableInterruptForFallingEdge() do { PORTF.PIN1CTRL = (PORTF.PIN1CTRL & ~PORT_ISC_gm) | 0x3 ; } while(0)
#define PF1_DisableDigitalInputBuffer() do { PORTF.PIN1CTRL = (PORTF.PIN1CTRL & ~PORT_ISC_gm) | 0x4 ; } while(0)
#define PF1_EnableInterruptForLowLevelSensing() do { PORTF.PIN1CTRL = (PORTF.PIN1CTRL & ~PORT_ISC_gm) | 0x5 ; } while(0)

//get/set PF0 aliases
#define PF0_SetHigh() do { PORTF_OUTSET = 0x1; } while(0)
#define PF0_SetLow() do { PORTF_OUTCLR = 0x1; } while(0)
#define PF0_Toggle() do { PORTF_OUTTGL = 0x1; } while(0)
#define PF0_GetValue() (VPORTF.IN & (0x1 << 0))
#define PF0_SetDigitalInput() do { PORTF_DIRCLR = 0x1; } while(0)
#define PF0_SetDigitalOutput() do { PORTF_DIRSET = 0x1; } while(0)
#define PF0_SetPullUp() do { PORTF_PIN0CTRL  |= PORT_PULLUPEN_bm; } while(0)
#define PF0_ResetPullUp() do { PORTF_PIN0CTRL  &= ~PORT_PULLUPEN_bm; } while(0)
#define PF0_SetInverted() do { PORTF_PIN0CTRL  |= PORT_INVEN_bm; } while(0)
#define PF0_ResetInverted() do { PORTF_PIN0CTRL  &= ~PORT_INVEN_bm; } while(0)
#define PF0_DisableInterruptOnChange() do { PORTF.PIN0CTRL = (PORTF.PIN0CTRL & ~PORT_ISC_gm) | 0x0 ; } while(0)
#define PF0_EnableInterruptForBothEdges() do { PORTF.PIN0CTRL = (PORTF.PIN0CTRL & ~PORT_ISC_gm) | 0x1 ; } while(0)
#define PF0_EnableInterruptForRisingEdge() do { PORTF.PIN0CTRL = (PORTF.PIN0CTRL & ~PORT_ISC_gm) | 0x2 ; } while(0)
#define PF0_EnableInterruptForFallingEdge() do { PORTF.PIN0CTRL = (PORTF.PIN0CTRL & ~PORT_ISC_gm) | 0x3 ; } while(0)
#define PF0_DisableDigitalInputBuffer() do { PORTF.PIN0CTRL = (PORTF.PIN0CTRL & ~PORT_ISC_gm) | 0x4 ; } while(0)
#define PF0_EnableInterruptForLowLevelSensing() do { PORTF.PIN0CTRL = (PORTF.PIN0CTRL & ~PORT_ISC_gm) | 0x5 ; } while(0)

//get/set CONF_WIFI_M2M_CHIP_ENABLE_PIN aliases
#define CONF_WIFI_M2M_CHIP_ENABLE_PIN_SetHigh() do { PORTF_OUTSET = 0x8; } while(0)
#define CONF_WIFI_M2M_CHIP_ENABLE_PIN_SetLow() do { PORTF_OUTCLR = 0x8; } while(0)
#define CONF_WIFI_M2M_CHIP_ENABLE_PIN_Toggle() do { PORTF_OUTTGL = 0x8; } while(0)
#define CONF_WIFI_M2M_CHIP_ENABLE_PIN_GetValue() (VPORTF.IN & (0x1 << 3))
#define CONF_WIFI_M2M_CHIP_ENABLE_PIN_SetDigitalInput() do { PORTF_DIRCLR = 0x8; } while(0)
#define CONF_WIFI_M2M_CHIP_ENABLE_PIN_SetDigitalOutput() do { PORTF_DIRSET = 0x8; } while(0)
#define CONF_WIFI_M2M_CHIP_ENABLE_PIN_SetPullUp() do { PORTF_PIN3CTRL  |= PORT_PULLUPEN_bm; } while(0)
#define CONF_WIFI_M2M_CHIP_ENABLE_PIN_ResetPullUp() do { PORTF_PIN3CTRL  &= ~PORT_PULLUPEN_bm; } while(0)
#define CONF_WIFI_M2M_CHIP_ENABLE_PIN_SetInverted() do { PORTF_PIN3CTRL  |= PORT_INVEN_bm; } while(0)
#define CONF_WIFI_M2M_CHIP_ENABLE_PIN_ResetInverted() do { PORTF_PIN3CTRL  &= ~PORT_INVEN_bm; } while(0)
#define CONF_WIFI_M2M_CHIP_ENABLE_PIN_DisableInterruptOnChange() do { PORTF.PIN3CTRL = (PORTF.PIN3CTRL & ~PORT_ISC_gm) | 0x0 ; } while(0)
#define CONF_WIFI_M2M_CHIP_ENABLE_PIN_EnableInterruptForBothEdges() do { PORTF.PIN3CTRL = (PORTF.PIN3CTRL & ~PORT_ISC_gm) | 0x1 ; } while(0)
#define CONF_WIFI_M2M_CHIP_ENABLE_PIN_EnableInterruptForRisingEdge() do { PORTF.PIN3CTRL = (PORTF.PIN3CTRL & ~PORT_ISC_gm) | 0x2 ; } while(0)
#define CONF_WIFI_M2M_CHIP_ENABLE_PIN_EnableInterruptForFallingEdge() do { PORTF.PIN3CTRL = (PORTF.PIN3CTRL & ~PORT_ISC_gm) | 0x3 ; } while(0)
#define CONF_WIFI_M2M_CHIP_ENABLE_PIN_DisableDigitalInputBuffer() do { PORTF.PIN3CTRL = (PORTF.PIN3CTRL & ~PORT_ISC_gm) | 0x4 ; } while(0)
#define CONF_WIFI_M2M_CHIP_ENABLE_PIN_EnableInterruptForLowLevelSensing() do { PORTF.PIN3CTRL = (PORTF.PIN3CTRL & ~PORT_ISC_gm) | 0x5 ; } while(0)

//get/set LED_DATA aliases
#define LED_DATA_SetHigh() do { PORTD_OUTSET = 0x2; } while(0)
#define LED_DATA_SetLow() do { PORTD_OUTCLR = 0x2; } while(0)
#define LED_DATA_Toggle() do { PORTD_OUTTGL = 0x2; } while(0)
#define LED_DATA_GetValue() (VPORTD.IN & (0x1 << 1))
#define LED_DATA_SetDigitalInput() do { PORTD_DIRCLR = 0x2; } while(0)
#define LED_DATA_SetDigitalOutput() do { PORTD_DIRSET = 0x2; } while(0)
#define LED_DATA_SetPullUp() do { PORTD_PIN1CTRL  |= PORT_PULLUPEN_bm; } while(0)
#define LED_DATA_ResetPullUp() do { PORTD_PIN1CTRL  &= ~PORT_PULLUPEN_bm; } while(0)
#define LED_DATA_SetInverted() do { PORTD_PIN1CTRL  |= PORT_INVEN_bm; } while(0)
#define LED_DATA_ResetInverted() do { PORTD_PIN1CTRL  &= ~PORT_INVEN_bm; } while(0)
#define LED_DATA_DisableInterruptOnChange() do { PORTD.PIN1CTRL = (PORTD.PIN1CTRL & ~PORT_ISC_gm) | 0x0 ; } while(0)
#define LED_DATA_EnableInterruptForBothEdges() do { PORTD.PIN1CTRL = (PORTD.PIN1CTRL & ~PORT_ISC_gm) | 0x1 ; } while(0)
#define LED_DATA_EnableInterruptForRisingEdge() do { PORTD.PIN1CTRL = (PORTD.PIN1CTRL & ~PORT_ISC_gm) | 0x2 ; } while(0)
#define LED_DATA_EnableInterruptForFallingEdge() do { PORTD.PIN1CTRL = (PORTD.PIN1CTRL & ~PORT_ISC_gm) | 0x3 ; } while(0)
#define LED_DATA_DisableDigitalInputBuffer() do { PORTD.PIN1CTRL = (PORTD.PIN1CTRL & ~PORT_ISC_gm) | 0x4 ; } while(0)
#define LED_DATA_EnableInterruptForLowLevelSensing() do { PORTD.PIN1CTRL = (PORTD.PIN1CTRL & ~PORT_ISC_gm) | 0x5 ; } while(0)

//get/set CONF_WIFI_M2M_INT_PIN aliases
#define CONF_WIFI_M2M_INT_PIN_SetHigh() do { PORTF_OUTSET = 0x4; } while(0)
#define CONF_WIFI_M2M_INT_PIN_SetLow() do { PORTF_OUTCLR = 0x4; } while(0)
#define CONF_WIFI_M2M_INT_PIN_Toggle() do { PORTF_OUTTGL = 0x4; } while(0)
#define CONF_WIFI_M2M_INT_PIN_GetValue() (VPORTF.IN & (0x1 << 2))
#define CONF_WIFI_M2M_INT_PIN_SetDigitalInput() do { PORTF_DIRCLR = 0x4; } while(0)
#define CONF_WIFI_M2M_INT_PIN_SetDigitalOutput() do { PORTF_DIRSET = 0x4; } while(0)
#define CONF_WIFI_M2M_INT_PIN_SetPullUp() do { PORTF_PIN2CTRL  |= PORT_PULLUPEN_bm; } while(0)
#define CONF_WIFI_M2M_INT_PIN_ResetPullUp() do { PORTF_PIN2CTRL  &= ~PORT_PULLUPEN_bm; } while(0)
#define CONF_WIFI_M2M_INT_PIN_SetInverted() do { PORTF_PIN2CTRL  |= PORT_INVEN_bm; } while(0)
#define CONF_WIFI_M2M_INT_PIN_ResetInverted() do { PORTF_PIN2CTRL  &= ~PORT_INVEN_bm; } while(0)
#define CONF_WIFI_M2M_INT_PIN_DisableInterruptOnChange() do { PORTF.PIN2CTRL = (PORTF.PIN2CTRL & ~PORT_ISC_gm) | 0x0 ; } while(0)
#define CONF_WIFI_M2M_INT_PIN_EnableInterruptForBothEdges() do { PORTF.PIN2CTRL = (PORTF.PIN2CTRL & ~PORT_ISC_gm) | 0x1 ; } while(0)
#define CONF_WIFI_M2M_INT_PIN_EnableInterruptForRisingEdge() do { PORTF.PIN2CTRL = (PORTF.PIN2CTRL & ~PORT_ISC_gm) | 0x2 ; } while(0)
#define CONF_WIFI_M2M_INT_PIN_EnableInterruptForFallingEdge() do { PORTF.PIN2CTRL = (PORTF.PIN2CTRL & ~PORT_ISC_gm) | 0x3 ; } while(0)
#define CONF_WIFI_M2M_INT_PIN_DisableDigitalInputBuffer() do { PORTF.PIN2CTRL = (PORTF.PIN2CTRL & ~PORT_ISC_gm) | 0x4 ; } while(0)
#define CONF_WIFI_M2M_INT_PIN_EnableInterruptForLowLevelSensing() do { PORTF.PIN2CTRL = (PORTF.PIN2CTRL & ~PORT_ISC_gm) | 0x5 ; } while(0)

//get/set LED_ERR aliases
#define LED_ERR_SetHigh() do { PORTD_OUTSET = 0x1; } while(0)
#define LED_ERR_SetLow() do { PORTD_OUTCLR = 0x1; } while(0)
#define LED_ERR_Toggle() do { PORTD_OUTTGL = 0x1; } while(0)
#define LED_ERR_GetValue() (VPORTD.IN & (0x1 << 0))
#define LED_ERR_SetDigitalInput() do { PORTD_DIRCLR = 0x1; } while(0)
#define LED_ERR_SetDigitalOutput() do { PORTD_DIRSET = 0x1; } while(0)
#define LED_ERR_SetPullUp() do { PORTD_PIN0CTRL  |= PORT_PULLUPEN_bm; } while(0)
#define LED_ERR_ResetPullUp() do { PORTD_PIN0CTRL  &= ~PORT_PULLUPEN_bm; } while(0)
#define LED_ERR_SetInverted() do { PORTD_PIN0CTRL  |= PORT_INVEN_bm; } while(0)
#define LED_ERR_ResetInverted() do { PORTD_PIN0CTRL  &= ~PORT_INVEN_bm; } while(0)
#define LED_ERR_DisableInterruptOnChange() do { PORTD.PIN0CTRL = (PORTD.PIN0CTRL & ~PORT_ISC_gm) | 0x0 ; } while(0)
#define LED_ERR_EnableInterruptForBothEdges() do { PORTD.PIN0CTRL = (PORTD.PIN0CTRL & ~PORT_ISC_gm) | 0x1 ; } while(0)
#define LED_ERR_EnableInterruptForRisingEdge() do { PORTD.PIN0CTRL = (PORTD.PIN0CTRL & ~PORT_ISC_gm) | 0x2 ; } while(0)
#define LED_ERR_EnableInterruptForFallingEdge() do { PORTD.PIN0CTRL = (PORTD.PIN0CTRL & ~PORT_ISC_gm) | 0x3 ; } while(0)
#define LED_ERR_DisableDigitalInputBuffer() do { PORTD.PIN0CTRL = (PORTD.PIN0CTRL & ~PORT_ISC_gm) | 0x4 ; } while(0)
#define LED_ERR_EnableInterruptForLowLevelSensing() do { PORTD.PIN0CTRL = (PORTD.PIN0CTRL & ~PORT_ISC_gm) | 0x5 ; } while(0)

//get/set SW1 aliases
#define SW1_SetHigh() do { PORTF_OUTSET = 0x20; } while(0)
#define SW1_SetLow() do { PORTF_OUTCLR = 0x20; } while(0)
#define SW1_Toggle() do { PORTF_OUTTGL = 0x20; } while(0)
#define SW1_GetValue() (VPORTF.IN & (0x1 << 5))
#define SW1_SetDigitalInput() do { PORTF_DIRCLR = 0x20; } while(0)
#define SW1_SetDigitalOutput() do { PORTF_DIRSET = 0x20; } while(0)
#define SW1_SetPullUp() do { PORTF_PIN5CTRL  |= PORT_PULLUPEN_bm; } while(0)
#define SW1_ResetPullUp() do { PORTF_PIN5CTRL  &= ~PORT_PULLUPEN_bm; } while(0)
#define SW1_SetInverted() do { PORTF_PIN5CTRL  |= PORT_INVEN_bm; } while(0)
#define SW1_ResetInverted() do { PORTF_PIN5CTRL  &= ~PORT_INVEN_bm; } while(0)
#define SW1_DisableInterruptOnChange() do { PORTF.PIN5CTRL = (PORTF.PIN5CTRL & ~PORT_ISC_gm) | 0x0 ; } while(0)
#define SW1_EnableInterruptForBothEdges() do { PORTF.PIN5CTRL = (PORTF.PIN5CTRL & ~PORT_ISC_gm) | 0x1 ; } while(0)
#define SW1_EnableInterruptForRisingEdge() do { PORTF.PIN5CTRL = (PORTF.PIN5CTRL & ~PORT_ISC_gm) | 0x2 ; } while(0)
#define SW1_EnableInterruptForFallingEdge() do { PORTF.PIN5CTRL = (PORTF.PIN5CTRL & ~PORT_ISC_gm) | 0x3 ; } while(0)
#define SW1_DisableDigitalInputBuffer() do { PORTF.PIN5CTRL = (PORTF.PIN5CTRL & ~PORT_ISC_gm) | 0x4 ; } while(0)
#define SW1_EnableInterruptForLowLevelSensing() do { PORTF.PIN5CTRL = (PORTF.PIN5CTRL & ~PORT_ISC_gm) | 0x5 ; } while(0)

//get/set LED_WIFI aliases
#define LED_WIFI_SetHigh() do { PORTD_OUTSET = 0x8; } while(0)
#define LED_WIFI_SetLow() do { PORTD_OUTCLR = 0x8; } while(0)
#define LED_WIFI_Toggle() do { PORTD_OUTTGL = 0x8; } while(0)
#define LED_WIFI_GetValue() (VPORTD.IN & (0x1 << 3))
#define LED_WIFI_SetDigitalInput() do { PORTD_DIRCLR = 0x8; } while(0)
#define LED_WIFI_SetDigitalOutput() do { PORTD_DIRSET = 0x8; } while(0)
#define LED_WIFI_SetPullUp() do { PORTD_PIN3CTRL  |= PORT_PULLUPEN_bm; } while(0)
#define LED_WIFI_ResetPullUp() do { PORTD_PIN3CTRL  &= ~PORT_PULLUPEN_bm; } while(0)
#define LED_WIFI_SetInverted() do { PORTD_PIN3CTRL  |= PORT_INVEN_bm; } while(0)
#define LED_WIFI_ResetInverted() do { PORTD_PIN3CTRL  &= ~PORT_INVEN_bm; } while(0)
#define LED_WIFI_DisableInterruptOnChange() do { PORTD.PIN3CTRL = (PORTD.PIN3CTRL & ~PORT_ISC_gm) | 0x0 ; } while(0)
#define LED_WIFI_EnableInterruptForBothEdges() do { PORTD.PIN3CTRL = (PORTD.PIN3CTRL & ~PORT_ISC_gm) | 0x1 ; } while(0)
#define LED_WIFI_EnableInterruptForRisingEdge() do { PORTD.PIN3CTRL = (PORTD.PIN3CTRL & ~PORT_ISC_gm) | 0x2 ; } while(0)
#define LED_WIFI_EnableInterruptForFallingEdge() do { PORTD.PIN3CTRL = (PORTD.PIN3CTRL & ~PORT_ISC_gm) | 0x3 ; } while(0)
#define LED_WIFI_DisableDigitalInputBuffer() do { PORTD.PIN3CTRL = (PORTD.PIN3CTRL & ~PORT_ISC_gm) | 0x4 ; } while(0)
#define LED_WIFI_EnableInterruptForLowLevelSensing() do { PORTD.PIN3CTRL = (PORTD.PIN3CTRL & ~PORT_ISC_gm) | 0x5 ; } while(0)

//get/set CONF_WIFI_M2M_WAKE_PIN aliases
#define CONF_WIFI_M2M_WAKE_PIN_SetHigh() do { PORTF_OUTSET = 0x10; } while(0)
#define CONF_WIFI_M2M_WAKE_PIN_SetLow() do { PORTF_OUTCLR = 0x10; } while(0)
#define CONF_WIFI_M2M_WAKE_PIN_Toggle() do { PORTF_OUTTGL = 0x10; } while(0)
#define CONF_WIFI_M2M_WAKE_PIN_GetValue() (VPORTF.IN & (0x1 << 4))
#define CONF_WIFI_M2M_WAKE_PIN_SetDigitalInput() do { PORTF_DIRCLR = 0x10; } while(0)
#define CONF_WIFI_M2M_WAKE_PIN_SetDigitalOutput() do { PORTF_DIRSET = 0x10; } while(0)
#define CONF_WIFI_M2M_WAKE_PIN_SetPullUp() do { PORTF_PIN4CTRL  |= PORT_PULLUPEN_bm; } while(0)
#define CONF_WIFI_M2M_WAKE_PIN_ResetPullUp() do { PORTF_PIN4CTRL  &= ~PORT_PULLUPEN_bm; } while(0)
#define CONF_WIFI_M2M_WAKE_PIN_SetInverted() do { PORTF_PIN4CTRL  |= PORT_INVEN_bm; } while(0)
#define CONF_WIFI_M2M_WAKE_PIN_ResetInverted() do { PORTF_PIN4CTRL  &= ~PORT_INVEN_bm; } while(0)
#define CONF_WIFI_M2M_WAKE_PIN_DisableInterruptOnChange() do { PORTF.PIN4CTRL = (PORTF.PIN4CTRL & ~PORT_ISC_gm) | 0x0 ; } while(0)
#define CONF_WIFI_M2M_WAKE_PIN_EnableInterruptForBothEdges() do { PORTF.PIN4CTRL = (PORTF.PIN4CTRL & ~PORT_ISC_gm) | 0x1 ; } while(0)
#define CONF_WIFI_M2M_WAKE_PIN_EnableInterruptForRisingEdge() do { PORTF.PIN4CTRL = (PORTF.PIN4CTRL & ~PORT_ISC_gm) | 0x2 ; } while(0)
#define CONF_WIFI_M2M_WAKE_PIN_EnableInterruptForFallingEdge() do { PORTF.PIN4CTRL = (PORTF.PIN4CTRL & ~PORT_ISC_gm) | 0x3 ; } while(0)
#define CONF_WIFI_M2M_WAKE_PIN_DisableDigitalInputBuffer() do { PORTF.PIN4CTRL = (PORTF.PIN4CTRL & ~PORT_ISC_gm) | 0x4 ; } while(0)
#define CONF_WIFI_M2M_WAKE_PIN_EnableInterruptForLowLevelSensing() do { PORTF.PIN4CTRL = (PORTF.PIN4CTRL & ~PORT_ISC_gm) | 0x5 ; } while(0)

//get/set LED_CONN aliases
#define LED_CONN_SetHigh() do { PORTD_OUTSET = 0x4; } while(0)
#define LED_CONN_SetLow() do { PORTD_OUTCLR = 0x4; } while(0)
#define LED_CONN_Toggle() do { PORTD_OUTTGL = 0x4; } while(0)
#define LED_CONN_GetValue() (VPORTD.IN & (0x1 << 2))
#define LED_CONN_SetDigitalInput() do { PORTD_DIRCLR = 0x4; } while(0)
#define LED_CONN_SetDigitalOutput() do { PORTD_DIRSET = 0x4; } while(0)
#define LED_CONN_SetPullUp() do { PORTD_PIN2CTRL  |= PORT_PULLUPEN_bm; } while(0)
#define LED_CONN_ResetPullUp() do { PORTD_PIN2CTRL  &= ~PORT_PULLUPEN_bm; } while(0)
#define LED_CONN_SetInverted() do { PORTD_PIN2CTRL  |= PORT_INVEN_bm; } while(0)
#define LED_CONN_ResetInverted() do { PORTD_PIN2CTRL  &= ~PORT_INVEN_bm; } while(0)
#define LED_CONN_DisableInterruptOnChange() do { PORTD.PIN2CTRL = (PORTD.PIN2CTRL & ~PORT_ISC_gm) | 0x0 ; } while(0)
#define LED_CONN_EnableInterruptForBothEdges() do { PORTD.PIN2CTRL = (PORTD.PIN2CTRL & ~PORT_ISC_gm) | 0x1 ; } while(0)
#define LED_CONN_EnableInterruptForRisingEdge() do { PORTD.PIN2CTRL = (PORTD.PIN2CTRL & ~PORT_ISC_gm) | 0x2 ; } while(0)
#define LED_CONN_EnableInterruptForFallingEdge() do { PORTD.PIN2CTRL = (PORTD.PIN2CTRL & ~PORT_ISC_gm) | 0x3 ; } while(0)
#define LED_CONN_DisableDigitalInputBuffer() do { PORTD.PIN2CTRL = (PORTD.PIN2CTRL & ~PORT_ISC_gm) | 0x4 ; } while(0)
#define LED_CONN_EnableInterruptForLowLevelSensing() do { PORTD.PIN2CTRL = (PORTD.PIN2CTRL & ~PORT_ISC_gm) | 0x5 ; } while(0)

void PIN_MANAGER_Initialize();
void PORTD_PD5_DefaultInterruptHandler(void);
void PORTD_PD5_SetInterruptHandler(void (* interruptHandler)(void)) ;
void PORTA_PA2_DefaultInterruptHandler(void);
void PORTA_PA2_SetInterruptHandler(void (* interruptHandler)(void)) ;
void PORTF_SW0_DefaultInterruptHandler(void);
void PORTF_SW0_SetInterruptHandler(void (* interruptHandler)(void)) ;
void PORTA_CONF_WIFI_M2M_RESET_PIN_DefaultInterruptHandler(void);
void PORTA_CONF_WIFI_M2M_RESET_PIN_SetInterruptHandler(void (* interruptHandler)(void)) ;
void PORTA_PA3_DefaultInterruptHandler(void);
void PORTA_PA3_SetInterruptHandler(void (* interruptHandler)(void)) ;
void PORTF_PF1_DefaultInterruptHandler(void);
void PORTF_PF1_SetInterruptHandler(void (* interruptHandler)(void)) ;
void PORTF_PF0_DefaultInterruptHandler(void);
void PORTF_PF0_SetInterruptHandler(void (* interruptHandler)(void)) ;
void PORTF_CONF_WIFI_M2M_CHIP_ENABLE_PIN_DefaultInterruptHandler(void);
void PORTF_CONF_WIFI_M2M_CHIP_ENABLE_PIN_SetInterruptHandler(void (* interruptHandler)(void)) ;
void PORTD_LED_DATA_DefaultInterruptHandler(void);
void PORTD_LED_DATA_SetInterruptHandler(void (* interruptHandler)(void)) ;
void PORTF_CONF_WIFI_M2M_INT_PIN_DefaultInterruptHandler(void);
void PORTF_CONF_WIFI_M2M_INT_PIN_SetInterruptHandler(void (* interruptHandler)(void)) ;
void PORTD_LED_ERR_DefaultInterruptHandler(void);
void PORTD_LED_ERR_SetInterruptHandler(void (* interruptHandler)(void)) ;
void PORTF_SW1_DefaultInterruptHandler(void);
void PORTF_SW1_SetInterruptHandler(void (* interruptHandler)(void)) ;
void PORTD_LED_WIFI_DefaultInterruptHandler(void);
void PORTD_LED_WIFI_SetInterruptHandler(void (* interruptHandler)(void)) ;
void PORTF_CONF_WIFI_M2M_WAKE_PIN_DefaultInterruptHandler(void);
void PORTF_CONF_WIFI_M2M_WAKE_PIN_SetInterruptHandler(void (* interruptHandler)(void)) ;
void PORTD_LED_CONN_DefaultInterruptHandler(void);
void PORTD_LED_CONN_SetInterruptHandler(void (* interruptHandler)(void)) ;
#endif /* PINS_H_INCLUDED */
