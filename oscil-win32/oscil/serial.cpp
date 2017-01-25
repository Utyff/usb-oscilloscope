#include "stdafx.h"
#include <math.h>
#include <winbase.h>
#include <stdlib.h>
#include "serial.h"
#include "oscil.h"

#define MAX_STRING  151
#define MAX_DEVICES 10

BYTE cmd[MAX_PACKET_SIZE];
BYTE Response[10][MAX_PACKET_SIZE];
DWORD  dwSPSBeginTime, dwSPSReceivedBytes, dwSPS;  // for sample rate calculation


bool SendCommand(HANDLE hPort, DWORD size, bool answer)
{
	DWORD		dwBytes, PacketCount=0, dwTicks;

	if( !hPort ) return false;

	if( !WriteFile( hPort, cmd, size, &dwBytes, NULL ) )
	{
		MsgError(GetLastError(), "Can`t write to COM-port");
		return false;
	}

	if( !answer ) return true;

	PurgeComm( hPort, PURGE_RXABORT ); // Clear read queue and wait for response

	dwTicks = GetSysTicks();
	while( dwTicks+20 > GetSysTicks() )
	{
		do
		{
			if( !ReadFile( hPort, Response[PacketCount], MAX_PACKET_SIZE, &dwBytes, NULL ) )  // Read packet
				{  MsgError(GetLastError(), "Can`t read from COM-port");	return false;	}
			if( dwBytes )
			{
				if( dwBytes != MAX_PACKET_SIZE )               return false;  // check packet size
				if( Response[PacketCount][0] != 1 )            return false;  // check packet type
				if( PacketCount != Response[PacketCount][1] )  return false;  // check packet number
				if( PacketCount >= Response[PacketCount][2] )  return false;  // check packet number
				if( ++PacketCount >= MAX_CONTROL_PACKETS )     return false;  // check packet number
				if( PacketCount = Response[PacketCount][2] )   return true;   // all packet received
			}
		}
		while( dwBytes );
		Sleep(1);
	} // while(dwTicks)

	return false; // answer was not received
}

// ----------- CaptureSerial ---------------
CaptureSerial::CaptureSerial()
{
	Freq1=1000; Freq2=5000; Impulse1=40; Impulse2=60;
	hPort=0;
}

INT_PTR CALLBACK  COMWndProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)  // Configuration window
{
	DWORD		dwBytes;

	switch (uMsg)
    {
	case WM_INITDIALOG:
		SendDlgItemMessage(hWnd,IDC_ENABLE_CH1,BM_SETCHECK,BST_CHECKED,0);
		SendDlgItemMessage(hWnd,IDC_ENABLE_CH2,BM_SETCHECK,BST_CHECKED,0);
		SendDlgItemMessage(hWnd,IDC_ENABLE_CH3,BM_SETCHECK,BST_CHECKED,0);
		SendDlgItemMessage(hWnd,IDC_ENABLE_CH4,BM_SETCHECK,BST_CHECKED,0);

		SendDlgItemMessage(hWnd,IDC_TRG1,BM_SETCHECK,BST_CHECKED,0); // set channel 1 as trigger

		return TRUE;

	case WM_COMMAND:
		switch (LOWORD(wParam) )
		{
		case IDC_TRG1:	if( HIWORD(wParam)==BN_CLICKED )  CS.graph->trg_ch=0;	break; // set channel 1 as trigger
		case IDC_TRG2:	if( HIWORD(wParam)==BN_CLICKED )  CS.graph->trg_ch=1;	break; // set channel 2 as trigger
		case IDC_TRG3:	if( HIWORD(wParam)==BN_CLICKED )  CS.graph->trg_ch=2;	break; // set channel 3 as trigger
		case IDC_TRG4:	if( HIWORD(wParam)==BN_CLICKED )  CS.graph->trg_ch=3;	break; // set channel 4 as trigger

		case IDC_LED:
			if( HIWORD(wParam)==BN_CLICKED )
			{
				if( SendDlgItemMessage(hWnd,IDC_LED,BM_GETCHECK,0,0)==BST_CHECKED )	{
					if( !WriteFile( CS.hPort, "ATL11", 5, &dwBytes, NULL ) )
						MsgError(GetLastError(), "Can`t write to COM-port");
				}
				else	{
					if( !WriteFile( CS.hPort, "ATL10", 5, &dwBytes, NULL ) )
						MsgError(GetLastError(), "Can`t write to COM-port");
				}
			}
			break;

		}
		break; // case WM_COMMAND:

	}
	return FALSE;
}

INT_PTR CALLBACK  COMGenWndProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)  // Configuration window
{
	BOOL	bl;
	int		i;

	switch (uMsg)
    {
	case WM_INITDIALOG:
		SetDlgItemInt( hWnd, IDC_FRQ1, (int)CS.Freq1, FALSE );
		SetDlgItemInt( hWnd, IDC_FRQ2, (int)CS.Freq2, FALSE );
		SetDlgItemInt( hWnd, IDC_IMPULSE1, (int)CS.Impulse1, FALSE );
		SetDlgItemInt( hWnd, IDC_IMPULSE2, (int)CS.Impulse2, FALSE );

		return TRUE;

	case WM_COMMAND:
		switch (LOWORD(wParam) )
		{
		case IDC_FRQ1:
			if( HIWORD(wParam)==EN_UPDATE )
			{
				i = GetDlgItemInt( hWnd, IDC_FRQ1, &bl, FALSE);
				if( bl && i>0 && i<1000000 )	CS.Freq1 = i;
				else	SetDlgItemInt( hWnd, IDC_FRQ1, CS.Freq1, FALSE );
			}
			break;

		case IDC_FRQ2:
			if( HIWORD(wParam)==EN_UPDATE )
			{
				i = GetDlgItemInt( hWnd, IDC_FRQ2, &bl, FALSE);
				if( bl && i>0 && i<1000000 )	CS.Freq2 = i;
				else	SetDlgItemInt( hWnd, IDC_FRQ2, CS.Freq2, FALSE );
			}
			break;

		case IDC_IMPULSE1:
			if( HIWORD(wParam)==EN_UPDATE )
			{
				i = GetDlgItemInt( hWnd, IDC_IMPULSE1, &bl, FALSE);
				if( bl && i>0 && i<90 )	CS.Impulse1 = i;
				else	SetDlgItemInt( hWnd, IDC_IMPULSE1, CS.Impulse1, FALSE );
			}
			break;

		case IDC_IMPULSE2:
			if( HIWORD(wParam)==EN_UPDATE )
			{
				i = GetDlgItemInt( hWnd, IDC_IMPULSE2, &bl, FALSE);
				if( bl && i>0 && i<90 )	CS.Impulse2 = i;
				else	SetDlgItemInt( hWnd, IDC_IMPULSE2, CS.Impulse2, FALSE );
			}
			break;

		}
		break; // case WM_COMMAND:

	}
	return FALSE;
}


bool CaptureSerial::Initialize()
{
	BOOL	bl;
	nSamplesPerSec=48800;

	hPort = CreateFile(wzCOMport, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
	if (hPort == INVALID_HANDLE_VALUE)
	{
		MsgError( GetLastError(), "Unable to open COM-port" );
		hPort = 0;
		return false;
	}

 	bl=SetCommMask(hPort, EV_RXCHAR);
	if( !bl )
	{
		MsgError( GetLastError(), "SetCommMask" );
		CloseHandle(hPort);
 		hPort = 0;
		return false;
 	}

 	bl=SetupComm(hPort, 64, 64); // set recomended IN/OUT queue size
	if( !bl )
	{
		MsgError( GetLastError() );
		CloseHandle(hPort);
 		hPort = 0;
		return false;
 	}

/*	COMMPROP *pr;
	pr = (COMMPROP*)HeapAlloc(GetProcessHeap(), HEAP_ZERO_MEMORY, sizeof(COMMPROP));
	bl = GetCommProperties(hPort, pr);
	DWORD dw = GetLastError();
	if (pr->wPacketLength != sizeof(COMMPROP))
	{
		pr=(COMMPROP*)HeapReAlloc(GetProcessHeap(), HEAP_ZERO_MEMORY, pr, pr->wPacketLength);
		pr->dwProvSpec1=COMMPROP_INITIALIZED;
		bl = GetCommProperties(hPort, pr);
		dw = GetLastError();
	}
	HeapFree(GetProcessHeap(),0,pr);  //*/

#define TIMEOUT 0

 	COMMTIMEOUTS CommTimeOuts;
 	CommTimeOuts.ReadIntervalTimeout = 0; //xFFFFFFFF;
 	CommTimeOuts.ReadTotalTimeoutMultiplier = 0;
 	CommTimeOuts.ReadTotalTimeoutConstant = TIMEOUT;
 	CommTimeOuts.WriteTotalTimeoutMultiplier = 0;
 	CommTimeOuts.WriteTotalTimeoutConstant = TIMEOUT;
 
 	if(!SetCommTimeouts(hPort, &CommTimeOuts))
	{
		MsgError( GetLastError() );
		CloseHandle(hPort);
 		hPort = 0;
		return false;
 	}
 
 	DCB ComDCM;

	memset(&ComDCM,0,sizeof(ComDCM));
	ComDCM.DCBlength = sizeof(DCB);
	if( !GetCommState(hPort, &ComDCM) )
	{
		MsgError( GetLastError() );
 		CloseHandle(hPort);
 		hPort = 0;
		return false;
 	}

//dwMaxBaud			268435456
//dwSettableBaud	268856176
#define baudrate	  2000999

	ComDCM.BaudRate = DWORD(baudrate);
	ComDCM.ByteSize = 8;
	ComDCM.Parity = NOPARITY;
	ComDCM.StopBits = ONESTOPBIT;
	ComDCM.fAbortOnError = TRUE;
	ComDCM.fDtrControl = DTR_CONTROL_DISABLE;
	ComDCM.fRtsControl = RTS_CONTROL_DISABLE;
	ComDCM.fBinary = TRUE;
	ComDCM.fParity = FALSE;
	ComDCM.fInX = FALSE;
	ComDCM.fOutX = FALSE;
	ComDCM.XonChar = 0;
	ComDCM.XoffChar = (unsigned char)0xFF;
	ComDCM.fErrorChar = FALSE;
	ComDCM.fNull = FALSE;
	ComDCM.fOutxCtsFlow = FALSE;
	ComDCM.fOutxDsrFlow = FALSE;
	ComDCM.XonLim = 128;
	ComDCM.XoffLim = 128;
 
 	if(!SetCommState(hPort, &ComDCM))
	{
		MsgError( GetLastError(), "SetCommState" );
 		CloseHandle(hPort);
 		hPort = 0;
		return false;
 	}

	return true;
}

bool CaptureSerial::SetGenerator(int GenNum, int Freq, int Imp)
{
	if( GenNum=0 )
	{
		Freq1 = Freq; Impulse1=Imp;
		// WriteFile();
	}
	else if( GenNum==1 )
	{
		Freq2 = Freq; Impulse2=Imp;
		// WriteFile();
	}
	else return false;

	return true;
}

void CaptureSerial::Shutdown()
{
	Stop();

	if( hPort )
	{
		CloseHandle(hPort);
		hPort=0;
	}

	return;
}

bool CaptureSerial::SetChannels(BYTE ch)
{
	if( hThredCap )  Stop();
	if( !ch )        return true;
	if( hPort == INVALID_HANDLE_VALUE ) return false;

	COLORREF clr[4]={RGB(10,80,255),RGB(10,180,225),RGB(10,200,105),RGB(10,230,55)};
	dwSPSBeginTime = dwSPSReceivedBytes = dwSPS = 0;

	graph = new GRAPH( clr, nSamplesPerSec, 4 );
	GL.AddGraph( graph );

	bStopThread=false;
	hThredCap=CreateThread(NULL, 0, threadCapture, this, 0, 0);

	SetThreadPriority(hThredCap, THREAD_PRIORITY_ABOVE_NORMAL);
	SetThreadPriority(hThredCap, THREAD_PRIORITY_HIGHEST);

	hwndPage = CfgTab.AddTab( wzCOMport+4, IDD_COM_CAPTURE, COMWndProc );
	TCHAR		wz[100];
	wcscpy_s(wz, sizeof(wz)/sizeof(TCHAR), wzCOMport+4);
	wcscat_s(wz, sizeof(wz)/sizeof(TCHAR), L" Generator");
	hwndGenPage = CfgTab.AddTab( wz, IDD_COM_GENERATOR, COMGenWndProc );

 	return true;
}

bool CaptureSerial::Stop()
{
	BOOL  b;
	DWORD dw;

	CfgTab.DeleteTab(hwndPage);
	CfgTab.DeleteTab(hwndGenPage);

	if ( hThredCap )        // Stop capture thread if it runned
	{
		b=GetExitCodeThread(hThredCap,&dw);
		if( dw != 259 ) MsgError(GetLastError(),"it stopped");  // it should never happen
		bStopThread=true;
		dw=WaitForSingleObject(hThredCap,100);
		b=GetExitCodeThread(hThredCap,&dw);
		if( dw == 259 ) MsgError(GetLastError(),"can`t stop");  // it should never happen
		CloseHandle(hThredCap);
		hThredCap=0;
	}

	if( graph )
	{
		GL.DeleteGraph( graph );
		delete graph;
		graph = 0;
	}
	
	return true;
}


DWORD WINAPI CaptureSerial::threadCapture( LPVOID lpParam ) 
{
#define CAP_BUFFER_SIZE 64*1024
	BYTE		btBuf[CAP_BUFFER_SIZE];
	DWORD		dwBytes, dwTime;
	struct _COMSTAT status;
	unsigned long etat;
	GRAPH			*gr;
	CaptureSerial	*capser=(CaptureSerial*)lpParam;

//	static int Arr1[1000], Arr2[1000], Arr3[1000], Arr4[1000], Arr5[1000], Arr6[1000], jj=0;

	if( !capser || !capser->hPort || !capser->graph ) return 13;
	gr = capser->graph;
	gr->previous = gr->current = 0;
	gr->iSweep = gr->iPrevSweep = -1;
 
// -------- Main capting circle
	while( !capser->bStopThread )  // Capture circle until exit signal.
	{
//		Arr2[jj]=GetSysTicks();
		if( !ClearCommError(capser->hPort, &etat, &status) )   // Get the number of bytes in the read queue
			{	MsgError(etat, "COM port error");	return 13;	}

//		Arr3[jj]=GetSysTicks();
		if( status.cbInQue ) {
			if( !ReadFile( capser->hPort, btBuf, status.cbInQue, &dwBytes, NULL ) )  // Need to set limit bytes for read
				{  MsgError(GetLastError(), "Can`t read from COM-port");	return 13;	}
		}
		else dwBytes=0;

//		if( dwBytes!=status.cbInQue )
//			ququ

		if( dwBytes ) // calculate sample per second 428366 428793 428580
		{
			dwTime = GetSysTicks();
			dwSPSReceivedBytes += dwBytes;
			if( (dwTime-dwSPSBeginTime) >= 1000 )
			{
				if( dwSPSBeginTime )
					dwSPS = (DWORD)( (double)dwSPSReceivedBytes / (dwTime-dwSPSBeginTime) * 1000.0);
				dwSPSReceivedBytes = 0;
				dwSPSBeginTime = dwTime;
			}
		}

		if( dwBytes )	gr->NewBlock(btBuf, dwBytes);	// load new data to graph

//		Arr1[jj]=status.cbInQue; Arr4[jj]=GetSysTicks(); Arr5[jj]=dwBytes;
		Sleep(1);
//		Arr6[jj]=GetSysTicks();
//		if( ++jj>999 ) jj=0;
	} // ----------- Main caturing circle

	return 25; // Exit from capture thread
} // CaptureSerial::threadCapture()
