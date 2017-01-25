#pragma once

#include <windows.h>
#include <mmsystem.h>
#include <dsound.h>
#include <math.h>
#include "graph.h"

#define LEFT_CHANNEL  1
#define RIGHT_CHANNEL 2
#define Pi 3.14159265

#define GRAPH_SQUARE	0
#define GRAPH_TRIANGLE	1
#define GRAPH_SIN		2
/*
Period = 1 / Freq    ( Sec - Hz,  milSec - KHz )
Smpl in Period = Period * Smpl Freq
Rad in Smpl = Smpl in Period / (2 * Pi)
*/

#define MAX_GEN_CH 2

class GRAPH_GEN : public GRAPH {
public:
	DWORD  nSamplesPerSec;
	double dbPeriod[MAX_GEN_CH], dbFreq[MAX_GEN_CH], dbCurrTime[MAX_GEN_CH], dbShiftTime[MAX_GEN_CH]; // , dbSmpl_Time
	int    Amplitude[MAX_GEN_CH];
	int    GraphType[MAX_GEN_CH];

	GRAPH_GEN(COLORREF *clr, DWORD dwSampls=44100);
	void SetFreq(int ch, double dbF);
//	void FillBlock(void *pvAudioPtr, DWORD dwAudioBytes);
	int	 Next(int ch);
};


class GenerateSound
{
public:
	HWND   hwndPage;
	int    dwBufferBytes;
	DWORD  nSamplesPerSec;
	BOOL   bStopThread;
	HANDLE hThredGen;
	GRAPH_GEN  *gr;

	bool   Initialize();
	void   Shutdown();
	bool   SetChannels(BYTE ch);
	bool   Stop();
	static DWORD WINAPI threadGenerate( LPVOID lpParam );

private:
	IDirectSound8*		DirectSound;
	IDirectSoundBuffer*	SndBuffer;
};


class CaptureSound
{
public:
	HWND   hwndPage;
	DWORD  nSamplesPerSec;
	BOOL   bStopThread;
	HANDLE hThredCap;
	GRAPH  *gr;

	bool   Initialize();
	void   Shutdown();
	bool   SetChannels(BYTE ch);
	bool   Stop();
	static DWORD WINAPI threadCapture( LPVOID lpParam );

private:
	IDirectSoundCapture8*		DirectSoundCap;
	IDirectSoundCaptureBuffer*	CapBuffer;
};

extern CaptureSound*		CapSound;
