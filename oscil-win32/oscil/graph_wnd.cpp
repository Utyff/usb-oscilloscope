#include "stdafx.h"
#include "oscil.h"

static bool	bStopRfrshThread;

DWORD WINAPI threadRefresh( LPVOID lpParam ) 
{
	while( !bStopRfrshThread )  // Generation circle until exit signal.
	{
		timeBeginPeriod(1);     // to be absolutely sure

		Sleep(100);             // 10 times per second
		InvalidateRect( (HWND)lpParam, NULL, FALSE );
	}
	return 117;
}

void SetWinSize(HWND hWnd, int cx)
{
	if( GL.iWinSizeX < cx  )  GL.iWinSizeX = cx-1;

	SCROLLINFO si;
	si.cbSize = sizeof(si);
	si.fMask  = SIF_ALL;
	si.nMin   = 0;
	si.nMax   = GL.iWinSizeX;
	si.nPage  = cx;
	si.nPos = si.nTrackPos = 0;
	SetScrollInfo(hWnd, SB_HORZ, &si, true);
}

LRESULT CALLBACK GraphWndProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
	static HANDLE	hRfrshThred;

	switch (uMsg)
    {
    case WM_CREATE:
		bStopRfrshThread=false;
		hRfrshThred=CreateThread(NULL, 0, threadRefresh, hWnd, 0, 0);

		SetWinSize(hWnd, ((LPCREATESTRUCT)lParam)->cx);
//		EnableScrollBar(hWnd, SB_HORZ, ESB_ENABLE_BOTH);
		return 0;

    case WM_DESTROY:
		if ( hRfrshThred )      // Stop refresh thread
		{
			bStopRfrshThread=true;
			WaitForSingleObject(hRfrshThred,1000);
			CloseHandle(hRfrshThred);
			hRfrshThred=0;
		}
		return 0;

    case WM_PAINT:
        {
			PAINTSTRUCT ps;
			HBITMAP		hbmp;

			RECT		rcSize;
			GetClientRect( hWnd, &rcSize );

			HDC	hdc=BeginPaint( hWnd, &ps );
			HDC memDC = CreateCompatibleDC ( hdc );
			hbmp = CreateCompatibleBitmap( hdc, rcSize.right, rcSize.bottom ); // all draw in the memory,  not screen
			SelectObject( memDC, hbmp );

			HPEN	hPen, hPenDot;
			HBRUSH	hBrush;
// draw frame
			hBrush  = CreateSolidBrush( RGB(255,244,217) );
			hPen    = CreatePen( PS_SOLID, 1, RGB(180,170,150) );
			hPenDot = CreatePen( PS_DOT,   1, RGB(180,170,150) );

			SetBkMode( memDC,TRANSPARENT );
			SelectObject( memDC, hBrush );
			SelectObject( memDC, hPen );
			Rectangle( memDC, 0,0,rcSize.right, rcSize.bottom );
// draw grid
			SelectObject( memDC, hPenDot );
			int i;
			int Step = rcSize.bottom/6;
			if( Step<10 ) Step=10;

			for( i=Step; i<rcSize.bottom; i+=Step )  // horizontal line
				{  MoveToEx( memDC, 0,i, NULL );    LineTo( memDC, rcSize.right, i );   }
			for( i=Step; i<rcSize.right; i+=Step )   // vertical line
				{  MoveToEx( memDC, i,0, NULL );	LineTo( memDC, i, rcSize.bottom );	}
// draw graphs
			for( i=0; i<GL.iGraphNums; i++ )   GL.gl[i]->Draw( memDC, rcSize );

			BitBlt(hdc, 0,0, rcSize.right, rcSize.bottom, memDC, 0,0, SRCCOPY);  // copy memory to screen

			DeleteObject(hPen);  DeleteObject(hPenDot);   DeleteObject(hBrush);
			DeleteDC(memDC);
			DeleteObject(hbmp);

			EndPaint(hWnd, &ps);
        }
        return 0;

    case WM_ERASEBKGND:
        return 1;

    case WM_SIZE:
		SetWinSize( hWnd, LOWORD(lParam) );
        InvalidateRect(hWnd, NULL, FALSE);
		//UpdateWindow(hWnd);
        return 0;

	case WM_HSCROLL:
		int iCurPos;
		iCurPos = GetScrollPos(hWnd, SB_HORZ);
		return 0;

	case WM_NCHITTEST:
		return HTTRANSPARENT;  // transit mouse messages to parent window

    default:
        return DefWindowProc(hWnd, uMsg, wParam, lParam);
    }
	return 0;
}
