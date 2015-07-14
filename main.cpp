/*
 * PSP Software Development Kit - http://www.pspdev.org
 * -----------------------------------------------------------------------
 * Licensed under the BSD license, see LICENSE in PSPSDK root for details.
 *
 * main.c - Basic Input demo -- reads from control pad and indicates button
 *          presses.
 *
 * Copyright (c) 2005 Marcus R. Brown <mrbrown@ocgnet.org>
 * Copyright (c) 2005 James Forshaw <tyranid@gmail.com>
 * Copyright (c) 2005 John Kelley <ps2dev@kelley.ca>
 * Copyright (c) 2005 Donour Sizemore <donour@uchicago.edu>
 *
 * $Id: main.c 1095 2005-09-27 21:02:16Z jim $
 */
#include <pspkernel.h>
#include <pspdebug.h>
#include <pspctrl.h>
#include <psputils.h>
#include <pspdisplay.h>
#include <pspge.h>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "nc1020.h"

/* Define the module info section */
PSP_MODULE_INFO("NC1020EMU", 0, 1, 1);

/* Define the main thread's attribute value (optional) */
PSP_MAIN_THREAD_ATTR(THREAD_ATTR_USER | THREAD_ATTR_VFPU);

#define FRAME_RATE 30
#define FRAME_INTERVAL (1000/FRAME_RATE)

#define PSP_SCREEN_WIDTH 480
#define PSP_SCREEN_HEIGHT 272
#define PSP_LINE_SIZE 512

#define GetMicroSec() sceKernelLibcClock()
//#define printf sio_printf

int done = 0;
static void* s_vram = (u32 *) 0x04000000;
static u32 s_bg = 0xFF57B090, s_fg = 0;
static u8 lcd_buf[1600];

/* Exit callback */
/*int exit_callback(int arg1, int arg2, void *common)
{
	wqxSaveNC1020();
	wqxQuitNC1020();
	return 0;
}*/

/* Exit callback */
int exit_callback(int arg1, int arg2, void *common)
{
	done = 1;
	return 0;
}

/* Callback thread */
int CallbackThread(SceSize args, void *argp)
{
	int cbid;

	cbid = sceKernelCreateCallback("Exit Callback", exit_callback, NULL);
	sceKernelRegisterExitCallback(cbid);
	sceKernelSleepThreadCB();

	return 0;
}

/* Sets up the callback thread and returns its thread id */
int SetupCallbacks(void)
{
	int thid = 0;

	thid = sceKernelCreateThread("update_thread", CallbackThread,
				     0x11, 0xFA0, 0, 0);
	if(thid >= 0)
	{
		sceKernelStartThread(thid, 0, 0);
	}

	return thid;
}

static void lcd_render()
{
	int i, j, k;
	u32* vram = (u32*) s_vram;
	
	if (!wqx::CopyLcdBuffer(lcd_buf))
		return;

	//int bufferwidth;
	//int pixelformat;
	//sceDisplayWaitVblankStart();  // if framebuf was set with PSP_DISPLAY_SETBUF_NEXTFRAME, wait until it is changed
	//sceDisplayGetFrameBuf((void**)&vram, &bufferwidth, &pixelformat, 1);
	vram += 56*PSP_LINE_SIZE;
	for (j=0; j<80; j++) {
		for (i=0; i<20; i++) {
			u8 val = lcd_buf[j*20+i];
			if (i == 0)	val &= 0x7F;	/* for left dots */
			for (k=0; k<8; k++) {
				u32 base = (PSP_LINE_SIZE*j + i*8 + k)*2 + 80;
				vram[base+1] = vram[base] = val&(1<<(7-k)) ? s_fg : s_bg;
			}
		}
		memcpy(&vram[PSP_LINE_SIZE*(2*j+1)], &vram[PSP_LINE_SIZE*j*2], PSP_LINE_SIZE*4);
	}
}

int main(void)
{
	int i = 0;
	unsigned int pre_buttons = 0;
	unsigned long cur_time = 0, pre_time = 0;
	SetupCallbacks();

	s_vram = (void*) (0x40000000 | (u32) sceGeEdramGetAddr());
	sceDisplaySetMode(0, PSP_SCREEN_WIDTH, PSP_SCREEN_HEIGHT);
	sceDisplaySetFrameBuf((void *) s_vram, PSP_LINE_SIZE, PSP_DISPLAY_PIXEL_FORMAT_8888, 1);
	for (i=0; i<PSP_LINE_SIZE*PSP_SCREEN_HEIGHT; i++)
		((u32*)s_vram)[i] = s_bg;

	printf("after set display framebuf \n");

	/* nc1020 init */
	wqx::Initialize(".");
	wqx::LoadNC1020();

	sceCtrlSetSamplingCycle(0);
	sceCtrlSetSamplingMode(PSP_CTRL_MODE_ANALOG);

	while(!done) {
		SceCtrlData pad;

		cur_time = GetMicroSec();
		if ((cur_time-pre_time) < FRAME_INTERVAL*1000)
			continue;
		pre_time = cur_time;
		wqx::RunTimeSlice((float)FRAME_INTERVAL, false);

		/* key event */
		sceCtrlReadBufferPositive(&pad, 1);
		if (pad.Buttons != 0 || pre_buttons != 0){
			printf("current time: %lums \n", GetMicroSec()/1000);
			if (pad.Buttons & PSP_CTRL_SQUARE){
				//printf("Square pressed \n");
				wqx::SetKey(0x10, true);	/* F1 */
			} else if (pre_buttons & PSP_CTRL_SQUARE){
				//printf("Square released \n");
				wqx::SetKey(0x10, false);
			}
			if (pad.Buttons & PSP_CTRL_TRIANGLE){
				//printf("Triangle pressed \n");
				wqx::SetKey(0x13, true);	/* F4 */
			} else if (pre_buttons & PSP_CTRL_TRIANGLE){
				//printf("Triangle released \n");
				wqx::SetKey(0x13, false);
			}
			if (pad.Buttons & PSP_CTRL_CIRCLE){
				//printf("Cicle pressed \n");
				wqx::SetKey(0x1D, true);	/* enter */
			} else if (pre_buttons & PSP_CTRL_CIRCLE){
				//printf("Cicle released \n");
				wqx::SetKey(0x1D, false);
			}
			if (pad.Buttons & PSP_CTRL_CROSS){
				//printf("Cross pressed \n");
				wqx::SetKey(0x3B, true);	/* esc */
			} else if (pre_buttons & PSP_CTRL_CROSS){
				//printf("Cross released \n");
				wqx::SetKey(0x3B, false);
			}

			if (pad.Buttons & PSP_CTRL_UP){
				//printf("Up pressed \n");
				wqx::SetKey(0x1A, true);
			} else if (pre_buttons & PSP_CTRL_UP){
				//printf("Up released \n");
				wqx::SetKey(0x1A, false);
			}
			if (pad.Buttons & PSP_CTRL_DOWN){
				//printf("Down pressed \n");
				wqx::SetKey(0x1B, true);
			} else if (pre_buttons & PSP_CTRL_DOWN){
				//printf("Down released \n");
				wqx::SetKey(0x1B, false);
			}
			if (pad.Buttons & PSP_CTRL_LEFT){
				//printf("Left pressed \n");
				wqx::SetKey(0x3F, true);
			} else if (pre_buttons & PSP_CTRL_LEFT){
				//printf("Left released \n");
				wqx::SetKey(0x3F, false);
			}
			if (pad.Buttons & PSP_CTRL_RIGHT){
				//printf("Right pressed \n");
				wqx::SetKey(0x1F, true);
			} else if (pre_buttons & PSP_CTRL_RIGHT){
				//printf("Right released \n");
				wqx::SetKey(0x1F, false);
			}

			if (pad.Buttons & PSP_CTRL_START){
				//printf("Start pressed \n");
				wqx::SetKey(0x08, true);	/* F10 */
			} else if (pre_buttons & PSP_CTRL_START){
				//printf("Start released \n");
				wqx::SetKey(0x08, false);
			}
			if (pad.Buttons & PSP_CTRL_SELECT){
				//printf("Select pressed \n");
				wqx::SetKey(0x0E, true);	/* F11 */
			} else if (pre_buttons & PSP_CTRL_SELECT){
				//printf("Select released \n");
				wqx::SetKey(0x0E, false);
			}
			if (pad.Buttons & PSP_CTRL_LTRIGGER){
				//printf("L-trigger pressed \n");
				wqx::SetKey(0x37, true);	/* PageUp */
			} else if (pre_buttons & PSP_CTRL_LTRIGGER){
				//printf("L-trigger released \n");
				wqx::SetKey(0x37, false);
			}
			if (pad.Buttons & PSP_CTRL_RTRIGGER){
				//printf("R-trigger pressed \n");
				wqx::SetKey(0x1E, true);	/* PageDown */
			} else if (pre_buttons & PSP_CTRL_RTRIGGER){
				//printf("R-trigger released \n");
				wqx::SetKey(0x1E, false);
			}
		}
		pre_buttons = pad.Buttons;

		/* copy lcd */
		lcd_render();
	}
	wqx::QuitNC1020();

	sceKernelExitGame();
	return 0;
}
