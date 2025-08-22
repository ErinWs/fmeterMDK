/***************************************************************************//**
 *   @file   AD7799.h
 *   @brief  Header file of AD7799 Driver.
 *   @author Bancisor Mihai
********************************************************************************
 * Copyright 2012(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
********************************************************************************
 *   SVN Revision: 580
*******************************************************************************/
#ifndef ADX_H
#define ADX_H
/******************************************************************************/
/* AD7799                                                                   */
/******************************************************************************/

typedef struct _ADX_COMPONENTS
{
	char  *desc;
	const int16_t init_channel;
	int16_t current_channel;
	int16_t gain;
	int16_t rate;
	union 
    {
    	uint8_t All;
    	struct
    	{
                volatile uint8_t  adc_updated		    :1;
                volatile uint8_t  running 	            :1;       
    	 }_bit;
    }sw;
    
	int16_t (*const Init )(int16_t channel_num,int16_t gain,int16_t rate); 
    void (*const stop)(void);
    void (*const restart)(uint32_t CHx,uint32_t Gain,uint32_t Rate);// 
    void (*const disale_eoc_interrup)(void);
    void (*const enable_eoc_interrupt)(void);
    void (*const task_handle)(void);//point to device_comps_task_handle
	
}adx_comps_t;
extern adx_comps_t adx_comps;


#endif	// _AD7799_H_


