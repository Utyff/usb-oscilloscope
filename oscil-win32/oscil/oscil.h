#pragma once

#include <windows.h>
#include <mmsystem.h>
#include "resource.h"
#include "sound.h"
#include "serial.h"

#define MAX_CFG_TABS   5
#define MAX_TAB_NAME   50

class TabControl
{
public:
	HWND	hTab;
	HWND	hVisiblePage;
	HWND	hTabPages[MAX_CFG_TABS];
//	LPSTR	TabNames[MAX_CFG_TABS][MAX_TAB_NAME];
	int		iPageCount;
	HFONT	hFont;   // holds font for Tab window

	TabControl();
	int		Create(HWND hWnd);
	void	Destroy();
	HWND	AddTab(wchar_t *Name, WORD RES_ID, DLGPROC lpDialogFunc);
	void	DeleteTab(HWND hPage);
	void	DeleteTab(int iNum);
	void	SelectTab();
	void	Resize(int cx, int cy);

//	BOOL (CALLBACK *TabProc)(HWND, UINT, WPARAM, LPARAM);
//	void (*TabPage_OnSize)(HWND hwnd, UINT state, int cx, int cy);
};

extern HINSTANCE		hInst;
extern HWND				hWnd,hwndGraph,hwndConf,hwndTab;
extern TCHAR			wzCOMport[20];
extern GRAPH_LIST		GL;
extern CaptureSerial	CS;
extern CaptureSound		CL;
extern GenerateSound	GenSnd;
extern TabControl		CfgTab;

int   MsgError(int err, char *sz=0);
BOOL  GetSoundDevs();
DWORD GetSysTicks();    // return system miliseconds from start or from 1-st day of month
LRESULT CALLBACK GraphWndProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);
LRESULT CALLBACK ConfWndProc (HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam);
