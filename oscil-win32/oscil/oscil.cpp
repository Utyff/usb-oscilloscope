#include "stdafx.h"
#include <windowsx.h>
#include <commctrl.h>
#include <stdlib.h>
#include <stdio.h>
#include "oscil.h"

#define MAX_LOADSTRING 100

// --- Global Variables:
HINSTANCE	hInst;									// current instance
HWND		hWnd, hwndGraph=0, hwndConf=0, hwndTab=0;
TCHAR		szWindowClass[MAX_LOADSTRING];			// the main window class name
TCHAR		wzCOMport[20]=L"\\\\.\\COM";
int			iConfWndSize=260;
DWORD		HPT_freq = 0;				// HP timer frequency in counts per second


// --- Global objects
GRAPH_LIST		GL;
CaptureSerial	CS;
CaptureSound	CL;
GenerateSound	GenSnd;
TabControl		CfgTab;

// Forward declarations of functions included in this code module:
ATOM				MyRegisterClass(HINSTANCE hInstance);
BOOL				InitInstance(HINSTANCE, int);
LRESULT CALLBACK	MainWndProc(HWND, UINT, WPARAM, LPARAM);
INT_PTR CALLBACK	About(HWND, UINT, WPARAM, LPARAM);
BOOL				EnumSerialPorts();
void				CheckSysTimer();
void				EnumerateCOMports();


int APIENTRY _tWinMain(HINSTANCE hInstance,
                     HINSTANCE hPrevInstance,
                     LPTSTR    lpCmdLine,
                     int       nCmdShow)
{
	UNREFERENCED_PARAMETER(hPrevInstance);
	UNREFERENCED_PARAMETER(lpCmdLine);

 	// TODO: Place code here.
	MSG		msg;
	HACCEL	hAccelTable;

	// Initialize global strings
	LoadString(hInstance, IDC_OSCIL, szWindowClass, MAX_LOADSTRING);
	MyRegisterClass(hInstance);

	CheckSysTimer();   // set system timer for 1 msec and get HPT frequency
	EnumerateCOMports();

	// Perform application initialization:
	if (!InitInstance (hInstance, nCmdShow))
	{
		return FALSE;
	}

	hAccelTable = LoadAccelerators(hInstance, MAKEINTRESOURCE(IDC_OSCIL));

	// Main message loop:
	while (GetMessage(&msg, NULL, 0, 0))
	{
		if (!TranslateAccelerator(msg.hwnd, hAccelTable, &msg))
		{
			TranslateMessage(&msg);
			DispatchMessage(&msg);
		}
	}

	return (int) msg.wParam;
}


ATOM MyRegisterClass(HINSTANCE hInstance)
{
	WNDCLASSEX wcex;

	// Initialize common controls.
    INITCOMMONCONTROLSEX icex;
	icex.dwSize = sizeof(INITCOMMONCONTROLSEX);
    icex.dwICC = ICC_TAB_CLASSES;
    InitCommonControlsEx(&icex);

	wcex.cbSize = sizeof(WNDCLASSEX);

	wcex.style			= CS_HREDRAW | CS_VREDRAW ; //| WS_CLIPCHILDREN;
	wcex.lpfnWndProc	= MainWndProc;
	wcex.cbClsExtra		= 0;
	wcex.cbWndExtra		= 0;
	wcex.hInstance		= hInstance;
	wcex.hIcon			= LoadIcon(hInstance, MAKEINTRESOURCE(IDI_OSCIL));
	wcex.hIconSm		= LoadIcon(wcex.hInstance, MAKEINTRESOURCE(IDI_SMALL));
	wcex.hCursor		= LoadCursor(NULL, IDC_ARROW);
	wcex.hbrBackground  = CreateSolidBrush(RGB(255,244,217)); //46, 86, 131));
	wcex.lpszMenuName	= MAKEINTRESOURCE(IDC_OSCIL);
	wcex.lpszClassName	= szWindowClass;
	if ( !RegisterClassEx(&wcex) ) return 0;

	wcex.style			= CS_HREDRAW | CS_VREDRAW;
	wcex.lpfnWndProc	= GraphWndProc;
	wcex.cbClsExtra		= 0;
	wcex.cbWndExtra		= 0;
	wcex.hInstance		= hInstance;
	wcex.hIcon			= NULL;
	wcex.hIconSm		= NULL;
	wcex.hCursor		= LoadCursor(NULL, IDC_ARROW);
	wcex.hbrBackground  = CreateSolidBrush(RGB(255,244,217));
	wcex.lpszMenuName	= NULL;
	wcex.lpszClassName	= TEXT("GraphWindow");

	if ( !RegisterClassEx(&wcex) ) return 0;

	wcex.style			= CS_HREDRAW | CS_VREDRAW;
	wcex.lpfnWndProc	= ConfWndProc;
	wcex.cbClsExtra		= 0;
	wcex.cbWndExtra		= 0;
	wcex.hInstance		= hInstance;
	wcex.hIcon			= NULL;
	wcex.hIconSm		= NULL;
	wcex.hCursor		= LoadCursor(NULL, IDC_ARROW);
	wcex.hbrBackground  = CreateSolidBrush(RGB(255,244,217));
	wcex.lpszMenuName	= NULL;
	wcex.lpszClassName	= TEXT("ConfWindow");

	ATOM aa = RegisterClassEx(&wcex);
	// DWORD dw=GetLastError();
	return aa;

}


BOOL InitInstance(HINSTANCE hInstance, int nCmdShow)
{
//   HWND hWnd;

   hInst = hInstance; // Store instance handle in our global variable

   hWnd = CreateWindow(szWindowClass, L"Oscilograph", WS_OVERLAPPEDWINDOW,
      CW_USEDEFAULT, 0, CW_USEDEFAULT, 0, NULL, NULL, hInstance, NULL);

   if (!hWnd)
   {
      return FALSE;
   }

   ShowWindow(hWnd, nCmdShow);
   UpdateWindow(hWnd);

//   GetSoundDevs();

   return TRUE;
}

LRESULT CALLBACK MainWndProc(HWND hWnd, UINT message, WPARAM wParam, LPARAM lParam)
{
	int wmId, wmEvent;

	switch (message)
	{
	case WM_CREATE:
		RECT rcSize;   GetClientRect(hWnd, &rcSize);
		hwndGraph = CreateWindowEx(0, TEXT("GraphWindow"), NULL, WS_CHILD | WS_VISIBLE | WS_HSCROLL,
			0, 0, rcSize.right-iConfWndSize, rcSize.bottom, hWnd, 0, GetModuleHandle(NULL), NULL);
		hwndConf  = CreateWindow(TEXT("ConfWindow" ), NULL, WS_CHILD | WS_VISIBLE,
			rcSize.right-iConfWndSize, 0, iConfWndSize, rcSize.bottom, hWnd, 0, GetModuleHandle(NULL), NULL);
		break;

	case WM_COMMAND:
		wmId    = LOWORD(wParam);
		wmEvent = HIWORD(wParam);
		// Parse the menu selections:
		switch (wmId)
		{
		case ID_GENERATE_ON:
			DWORD dwState;
			HMENU hmenu;

			hmenu = GetMenu(hWnd);
			dwState = GetMenuState(hmenu, ID_GENERATE_ON, MF_BYCOMMAND);

			if (!(dwState & MF_CHECKED))   {
				CheckMenuItem(hmenu, ID_GENERATE_ON, MF_BYCOMMAND | MF_CHECKED);
				GenSnd.Initialize();
				GenSnd.SetChannels(1|2);
            }
            else    {
				CheckMenuItem(hmenu, ID_GENERATE_ON, MF_BYCOMMAND | MF_UNCHECKED);
				GenSnd.Shutdown();
            }
			break;  // case ID_GENERATE_ON:

		case ID_OSCILOGRAPH_AIN:
			hmenu = GetMenu(hWnd); 
			dwState = GetMenuState(hmenu, ID_OSCILOGRAPH_AIN, MF_BYCOMMAND);

			if (!(dwState & MF_CHECKED))   {
				CheckMenuItem(hmenu, ID_OSCILOGRAPH_AIN, MF_BYCOMMAND | MF_CHECKED);
				CL.Initialize();
				CL.SetChannels(1|2);
            }
            else	{
				CheckMenuItem(hmenu, ID_OSCILOGRAPH_AIN, MF_BYCOMMAND | MF_UNCHECKED);
				CL.Shutdown();
			}
			break;

		case ID_OSC_SERIAL:
			hmenu = GetMenu(hWnd); 
			dwState = GetMenuState(hmenu, ID_OSC_SERIAL, MF_BYCOMMAND);

			if (!(dwState & MF_CHECKED))    {
				CheckMenuItem(hmenu, ID_OSC_SERIAL, MF_BYCOMMAND | MF_CHECKED);
				CS.Initialize();
				CS.SetChannels( 1|2|4|8 );
            }
            else	{
				CheckMenuItem(hmenu, ID_OSC_SERIAL, MF_BYCOMMAND | MF_UNCHECKED);
				CS.Shutdown();
			}
			break;

		case IDM_ABOUT:
			DialogBox(hInst, MAKEINTRESOURCE(IDD_ABOUTBOX), hWnd, About);
			break;
		case IDM_EXIT:
			DestroyWindow(hWnd);
			break;
		default:
			return DefWindowProc(hWnd, message, wParam, lParam);
		}         // switch (wmId)
		break;    // case WM_COMMAND:
/*
	case WM_KEYDOWN:
		if ( wParam == 'C' )
		{
		}
		break;

	case WM_PAINT:
		PAINTSTRUCT ps;
		HDC hdc;

		hdc = BeginPaint(hWnd, &ps);
		// TODO: Add any drawing code here...

		RECT rcSize;
        GetClientRect(hWnd, &rcSize);

		//обновляем окно
		ValidateRect(hWnd, NULL);
		EndPaint(hWnd, &ps);
		break;
*/
	case WM_ERASEBKGND:
		return 1;

	case WM_MOUSEWHEEL:
		GL.SetScale(HIWORD(wParam));
		return 0;

/*	case WM_MOUSEMOVE:
		RECT rc;
		int xPos;
		HCURSOR hcur;

        GetClientRect(hWnd, &rc);
		xPos = GET_X_LPARAM(lParam);
		if( xPos > rc.right-iConfWndSize-3 && xPos < rc.right-iConfWndSize+3 )
		   { hcur = LoadCursor(NULL,IDC_SIZEWE); SetCursor(hcur); }
		return 0;
*/
	case WM_SIZE:
		if( hwndGraph )
			MoveWindow(hwndGraph, 0,0, LOWORD(lParam)-iConfWndSize,HIWORD(lParam), true);
		if( hwndConf )
			MoveWindow(hwndConf, LOWORD(lParam)-iConfWndSize,0, iConfWndSize,HIWORD(lParam), true);
		break;

	case WM_DESTROY:
		if( hwndGraph ) DestroyWindow( hwndGraph );
		if( hwndConf  ) DestroyWindow( hwndConf  );
		PostQuitMessage(0);
		break;

	default:
		return DefWindowProc(hWnd, message, wParam, lParam);
	}
	return 0;
}

void GetTimerParam(HWND hDlg)	// Get timer specs
{
	char		sz[50];
	long		dw1, dw2, CurrentTickSize;
	double		dbl;
	TIMECAPS	tc={0};
	SYSTEMTIME  SysTime;
	LARGE_INTEGER HPT, li;

	timeGetDevCaps(&tc, sizeof(tc)); // Min/Max timer

	if( !QueryPerformanceFrequency(&HPT) )
	{
		HPT.LowPart = 0;
		dbl = 0;
	}

	Sleep(1); // for sleep next start from begin

	if( HPT.LowPart )
	{
		QueryPerformanceCounter(&li);
		dw1 = li.LowPart;
	}
	GetSystemTime(&SysTime);
	dw2=SysTime.wMilliseconds;

	Sleep(1);

	if( HPT.LowPart )
	{
		QueryPerformanceCounter (&li);
		dbl = (li.LowPart - dw1) / ((double)HPT.LowPart / 1000.0);
		_gcvt_s( sz, sizeof(sz), dbl, 5 );			SetDlgItemTextA( hDlg,IDC_STATIC_CURHP,sz );
	}

	GetSystemTime(&SysTime);
	CurrentTickSize = SysTime.wMilliseconds-dw2;
	if( CurrentTickSize<0 )  CurrentTickSize += 1000;  // Check for seconds tick
	_itoa_s( CurrentTickSize,sz,sizeof(sz),10 );	SetDlgItemTextA( hDlg,IDC_STATIC_CUR,sz );

	_itoa_s( HPT.LowPart,    sz,sizeof(sz),10 );	SetDlgItemTextA( hDlg,IDC_HPT,sz );
	_itoa_s( tc.wPeriodMin,  sz,sizeof(sz),10 );	SetDlgItemTextA( hDlg,IDC_STATIC_MIN,sz );
	_itoa_s( tc.wPeriodMax,  sz,sizeof(sz),10 );	SetDlgItemTextA( hDlg,IDC_STATIC_MAX,sz );

//	timeBeginPeriod(1); // set minimal time
}


// Message handler for about box.
INT_PTR CALLBACK About(HWND hDlg, UINT message, WPARAM wParam, LPARAM lParam)
{
	char  sz[200];

	switch (message)
	{
	case WM_INITDIALOG:
		GetTimerParam(hDlg);
		return (INT_PTR)TRUE;

	case WM_LBUTTONDOWN:
		GetTimerParam(hDlg);
		return 0;

	case WM_RBUTTONDOWN:
		sprintf_s(sz, sizeof(sz), "Compilation date:  %s %s", __DATE__, __TIME__ );
		SetDlgItemTextA( hDlg,IDC_STATIC_DATE,sz );
		return 0;

	case WM_COMMAND:
		if (LOWORD(wParam) == IDOK || LOWORD(wParam) == IDCANCEL)
		{
			EndDialog(hDlg, LOWORD(wParam));
			return (INT_PTR)TRUE;
		}
		break;
	}
	return (INT_PTR)FALSE;
}

int MsgError(int err, char *szInfo)
{
	char sznum[1000], *sz, *ErrMsg;
	int  i;

	if( !FormatMessageA(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM, NULL,
			err, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), (LPSTR)&ErrMsg, 0, NULL) )
		return err;

	i = strlen(ErrMsg)+1000;
	sz = new char[i];

	_itoa_s( err, sznum, sizeof(sznum), 16 );

	sz[0]=0;
	if( szInfo ) strcat_s(sz, 150, szInfo);
	strcat_s(sz, i, "\n\nError code - ");
	strcat_s(sz, i, sznum);
	strcat_s(sz, i, "\n\nError message:\n");
	strcat_s(sz, i, ErrMsg);

	MessageBoxA(hWnd,sz,"Error",MB_ICONERROR|MB_OK);

	LocalFree(ErrMsg);
	delete sz;

	return err;
}

void CheckSysTimer()
{
	LARGE_INTEGER HPT;

	timeBeginPeriod(1);

	if( QueryPerformanceFrequency(&HPT) )	HPT_freq = HPT.LowPart;
}

DWORD GetSysTicks()  // return system miliseconds from machine start  or  from 1-st day of month
{
	LARGE_INTEGER	li;
	SYSTEMTIME		SysTime;

	if( HPT_freq )
	{
		QueryPerformanceCounter(&li);
		return	(DWORD)(li.QuadPart * 1000 / HPT_freq);
	}
	else
	{
		GetSystemTime(&SysTime);
		return	SysTime.wMilliseconds + 1000 * (SysTime.wSecond + 60 * 
				(SysTime.wMinute + 60 * (SysTime.wHour + 24*SysTime.wDay)));   // miliseconds from 1-st day of month
	}
}

// SetupDiGetClassDevs()
// http://www.codeproject.com/Articles/14412/Enumerating-windows-device

void EnumerateCOMports()
{
	DWORD	dw, i, max=0;
	char	sz[1000], sz2[1000];
	TCHAR	wz[1000];
	HANDLE	hPort;
	COMMTIMEOUTS CommTimeOuts={0};

	for( i=1; i<20; i++)
	{
		sprintf_s(sz, sizeof(sz), "COM%i", i);
		dw = QueryDosDeviceA( sz, sz2, sizeof(sz2) );
		if( dw )
		{
			swprintf( wz, sizeof(wz)/sizeof(TCHAR),L"\\\\.\\COM%i", i );
			hPort = CreateFile(wz, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, 0, NULL);
			if (hPort != INVALID_HANDLE_VALUE)
			{
				max=i;
 				if(!SetCommTimeouts(hPort, &CommTimeOuts))	MsgError(GetLastError());
				// Get version from oscilograph
// Oscilograph 32 (M3)
// Copyright (C) TUSUR, Zavodovskiy G.A. 2013
// ID: XXXXXXXX-XXXXXXXX-XXXXXXXX
// HW rev: 01.01.00
// SW ver: 01.01.00
// Compilation date: %s %s
				cmd[0]='A'; cmd[1]='T'; cmd[2]='I'; // command - ATI
				/*if( SendCommand( hPort,3,true ) )
				{
					i=20; // for break circle
					sprintf_s(sz, sizeof(sz), "%s", "Oscilograph 32 (M3)"); //&Response[0][2]);
					SetDlgItemTextA( CfgTab.hTabPages[0],IDC_STATIC_COMINFO,sz );
				} // */

				CloseHandle(hPort);
			}
		}
	} // for(i)

	if( max ) swprintf( wzCOMport, sizeof(wzCOMport)/sizeof(wchar_t),L"\\\\.\\COM%i", max );
}
