#include "stdafx.h"
#include <windows.h>
#include <commctrl.h>
#include "oscil.h"

#define CFG_BORDER 5
#define TAB_BORDER 2
#define TAB_HEADER 22

HWND	hTabGnrl;
// HFONT	hFont=0;   // holds font in all window lifetime


INT_PTR CALLBACK  GnrlWndProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
	int		i;
	BOOL	bl;

	switch (uMsg)
    {
	case WM_INITDIALOG:  // Set initial state for all items

		SetDlgItemInt( hWnd, IDC_EDIT_TRSH, GL.TriggerThreshold, FALSE );

		switch( GL.TriggerMode )
		{
		case TRG_FREE_RUN:	SendDlgItemMessage(hWnd,IDC_FREERUN,BM_SETCHECK,BST_CHECKED,0); 	break;
		case TRG_FRONT:		SendDlgItemMessage(hWnd,IDC_FRONT,  BM_SETCHECK,BST_CHECKED,0);		break;
		case TRG_BACK:		SendDlgItemMessage(hWnd,IDC_BACK,   BM_SETCHECK,BST_CHECKED,0);		break;
		}

		if( GL.CircleRun )	SendDlgItemMessage(hWnd,IDC_SWEEP_CIRCLE,BM_SETCHECK,BST_CHECKED,0);

		SetDlgItemInt( hWnd, IDC_SWP_TIME, (int)GL.dbSweepTime, FALSE );

		SetDlgItemText( hWnd, IDC_COM_PORT, wzCOMport+4 );

		return TRUE;

	case WM_COMMAND:
		switch( LOWORD(wParam) )
		{
		case IDC_FREERUN:	if( HIWORD(wParam)==BN_CLICKED )  GL.TriggerMode=TRG_FREE_RUN;	break;
		case IDC_FRONT:		if( HIWORD(wParam)==BN_CLICKED )  GL.TriggerMode=TRG_FRONT;		break;
		case IDC_BACK:		if( HIWORD(wParam)==BN_CLICKED )  GL.TriggerMode=TRG_BACK;		break;

		case IDC_EDIT_TRSH:
			if( HIWORD(wParam)==EN_UPDATE )
			{
				i = GetDlgItemInt( hWnd, IDC_EDIT_TRSH, &bl, FALSE);
				if( bl && i>0 && i<256 )	GL.TriggerThreshold = i;
				else	SetDlgItemInt( hWnd, IDC_EDIT_TRSH, GL.TriggerThreshold, FALSE );
			}
			break;

		case IDC_SWP_RUN:
			if( HIWORD(wParam)==BN_CLICKED )  GL.Run=true;
			break;

		case IDC_SWEEP_CIRCLE:
			if( HIWORD(wParam)==BN_CLICKED )
			{
				if( SendDlgItemMessage(hWnd,IDC_SWEEP_CIRCLE,BM_GETCHECK,0,0)==BST_CHECKED )
						GL.CircleRun=true;
				else	GL.CircleRun=false;
			}
			break;

		case IDC_SWP_TIME:
			if( HIWORD(wParam)==EN_UPDATE )
			{
				i = GetDlgItemInt( hWnd, IDC_SWP_TIME, &bl, FALSE);
				if( bl && i>10 && i<1000 )	GL.dbSweepTime = i;
				else	SetDlgItemInt( hWnd, IDC_SWP_TIME, (int)GL.dbSweepTime, FALSE );
			}
			break;

		case IDC_COM_PORT:
			if( HIWORD(wParam)==EN_UPDATE )
				GetDlgItemText( hWnd, IDC_COM_PORT, wzCOMport+4, 8 );
			break;
		} // switch( LOWORD(wParam) )
		break;  // case WM_COMMAND:
	}
	return (INT_PTR)FALSE;
}

TabControl::TabControl()
{
	hTab = hVisiblePage = 0;
	ZeroMemory( hTabPages, sizeof(hTabPages) );
//	ZeroMemory( TabNames, sizeof(TabNames) );
	iPageCount = 0;
}

int TabControl::Create(HWND hWnd)
{
	RECT rc;   GetClientRect(hWnd, &rc);

	hTab=CreateWindow(WC_TABCONTROL, NULL, WS_CHILD | WS_VISIBLE | UDS_ARROWKEYS,
						CFG_BORDER, CFG_BORDER, rc.right-CFG_BORDER*2, rc.bottom-CFG_BORDER*2, hWnd, NULL, hInst, NULL);
	if( !hTab )   MsgError(GetLastError(), "create tab control");

	hFont=CreateFont (14, 0, 0, 0, FW_DONTCARE, FALSE, FALSE, FALSE, ANSI_CHARSET,
		OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, DEFAULT_QUALITY, DEFAULT_PITCH | FF_SWISS, TEXT("MS Shell Dlg"));
	SendMessage(hTab, WM_SETFONT, WPARAM (hFont), FALSE);

	AddTab( TEXT("General"), IDD_GENERAL, GnrlWndProc );

	TabCtrl_SetCurSel( hTab, 0 );
	SelectTab();

	return 0;
}

void TabControl::Destroy()
{
	for( int i=iPageCount-1; i>0; i-- ) 	DeleteTab(i);

	DestroyWindow(hTab);
	DeleteObject(hFont);
	return;
}

HWND TabControl::AddTab(wchar_t *Name, WORD RES_ID, DLGPROC lpDialogFunc)
{
	TCITEM tie={0};

	if( iPageCount >= MAX_CFG_TABS ) return 0;

	tie.mask = TCIF_TEXT;
	tie.iImage = -1;
	tie.pszText = Name;
	TabCtrl_InsertItem(hTab, iPageCount, &tie);

	hTabPages[iPageCount] = CreateDialog( hInst, MAKEINTRESOURCE(RES_ID), hTab, lpDialogFunc );
	ShowWindow( hTabPages[iPageCount], SW_HIDE );

	iPageCount++;

	RECT rc;
	GetClientRect(hwndConf, &rc);
	Resize(rc.right, rc.bottom);

//	TabCtrl_SetCurSel( hTab, iPageCount-1 );
	SelectTab();

	return hTabPages[iPageCount-1];
}

void TabControl::DeleteTab(HWND hPage)
{
	for( int i=0; i<iPageCount; i++ )
		if( hTabPages[i] == hPage )
		{
			DeleteTab(i);
			return;
		}
}

void TabControl::DeleteTab(int iNum)
{
	if( iNum >= iPageCount )  return;

	if( !DestroyWindow(hTabPages[iNum]) ) MsgError(GetLastError(), "Can`t destroy window");
	if( !TabCtrl_DeleteItem(hTab,iNum)  ) MsgError(iNum, "Can`t delete tab");

	for( int i=iNum; i < iPageCount; i++ )	hTabPages[i] = hTabPages[i+1];

	--iPageCount;

	TabCtrl_SetCurSel( hTab, 0 );
	SelectTab();
}

void TabControl::SelectTab()
{
	ShowWindow(hVisiblePage,SW_HIDE);
	hVisiblePage = hTabPages[TabCtrl_GetCurSel(hTab)];

	ShowWindow(hVisiblePage,SW_SHOW);
	SetFocus(hVisiblePage);
}

void TabControl::Resize(int cx, int cy)
{
	if( hTab )
		MoveWindow(hTab, CFG_BORDER, CFG_BORDER, cx-CFG_BORDER*2, cy-CFG_BORDER*2, true);

	RECT  rc;
	GetClientRect(hTab, &rc);

	for( int i=0; i<iPageCount; i++ )
		if( hTabPages[i] )
			MoveWindow(hTabPages[i], TAB_BORDER,TAB_HEADER, rc.right-TAB_BORDER*2,rc.bottom-TAB_HEADER-TAB_BORDER, true);
}

LRESULT CALLBACK ConfWndProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
	TCITEM tie={0};

	switch (uMsg)
    {
    case WM_CREATE:
		CfgTab.Create(hWnd);
		return 0;

	case WM_DESTROY:
		CfgTab.Destroy();
		return 0;

	case WM_SIZE:
		CfgTab.Resize( LOWORD(lParam), HIWORD(lParam) );
		break;

	case WM_NCHITTEST:
		return HTTRANSPARENT;

        case WM_NOTIFY:
            switch (((LPNMHDR)lParam)->code)
            {
            case TCN_SELCHANGE:
				CfgTab.SelectTab();
				break;
            }
	        break;

    default:
        return DefWindowProc(hWnd, uMsg, wParam, lParam);
    }
	return 0;
}
