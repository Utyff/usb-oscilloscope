#pragma once

#include <windows.h>
#include <mmsystem.h>
#include <math.h>
#include "graph.h"

#define MAX_PACKET_SIZE 64
#define MAX_CONTROL_PACKETS 10

extern BYTE cmd[MAX_PACKET_SIZE];
extern BYTE Response[MAX_CONTROL_PACKETS][MAX_PACKET_SIZE];

bool SendCommand(HANDLE hPort, DWORD size, bool answer=0);

extern DWORD  dwSPSBeginTime, dwSPSReceivedBytes, dwSPS;  // for sample rate calculation

class CaptureSerial
{
public:
	HWND   hwndPage, hwndGenPage;
	DWORD  nSamplesPerSec;
	HANDLE hPort;
	BOOL   bStopThread;
	HANDLE hThredCap;
	GRAPH  *graph;
	DWORD  Freq1, Freq2, Impulse1, Impulse2;

	CaptureSerial();
	bool   Initialize();
	void   Shutdown();
	bool   SetChannels(BYTE ch);
	bool   Stop();
	bool   SetGenerator(int GenNum, int Freq, int Imp);
	static DWORD WINAPI threadCapture( LPVOID lpParam );
};

extern CaptureSerial		CapSerial;
