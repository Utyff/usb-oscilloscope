#include "stdafx.h"
#include "graph.h"
#include "oscil.h"

void SetWinSize(HWND hWnd, int cx);

GRAPH::GRAPH(COLORREF *clr, DWORD dwSampls, int ch_Numbers)
{
	DWORD i;

	if( ch_Numbers>MAX_GRAPHS || ch_Numbers<0)
		{ MessageBoxA(0,"Wrong channel numbers.","Error", MB_ICONERROR); return; }

	current = previous = 0;    // blocks start
	iSweep = iPrevSweep = -1;  // no sweeps start
	TriggerPrevSign = 0;

	ZeroMemory(yShift, sizeof(yShift));
	szName[0][0] = szName[1][0] = szName[2][0] = szName[3][0] = 0;
	Enabled[0] = Enabled[1] = Enabled[2] = Enabled[3] = TRUE;
	size = GRAPH_BUFFER_SIZE;

	ch_nums = ch_Numbers;
	trg_ch = 0;
	ZeroMemory(arr, sizeof(arr));
	for( i=0; i<ch_nums; i++ )  arr[i] = new BYTE[size];
	if( clr )	for( i=0; i<ch_nums; i++ )  color[i] = clr[i];
	else		for( i=0; i<ch_nums; i++ )  color[i] = 0;

	SetSampleRate( dwSampls );
}

GRAPH::~GRAPH()
{
	int i;

	for( i=0; i<(int)ch_nums; i++ )
		if( arr[i]!=0 )	{ delete arr[i]; arr[i]=0; }
}

void GRAPH::SetSampleRate(DWORD dwSmplsPerSec)
{
	nSamplesPerSec = dwSmplsPerSec;
	iSamplsPerSweep = (DWORD)(GL.dbSweepTime * (double)nSamplesPerSec / 1000.0);
	dbSmpl_Time = (1.0/(double)dwSmplsPerSec) * 1000;   // Sampl time in milisec */
}

void GRAPH::NewBlock(BYTE *pbBuf, DWORD dwBytes)
{
	DWORD i, j;

	if( dwBytes%ch_nums ) { MessageBoxA(0,"Wrong bytes number", "Error", 0);  return; } // for debug

/*	if ( current==iSweep && !GL.CircleRun && !GL.Run )  // if buffer is full
	   return;             // and sweep not circling and new sweep not alowed - stop loading data */

	previous = current;

	for( i=0; i<dwBytes;  )  // copy data to graph buffer
	{
		if( current == iSweep )  // if reach to sweep begin
		{
			if ( !GL.CircleRun && !GL.Run )		// if sweep not circling and new run not alowed
				break;							// stop loading data
			iSweep = -1;            // or delete sweep
		}
		if( current == iPrevSweep )	iPrevSweep = -1;	// reach to previos sweep begin

		for( j=0; j<ch_nums; j++)	arr[j][current] = pbBuf[i++]; // copy data to graph

		if( ++current >= (int)size ) current=0; // circle buffer
	} // for(i)

	RunTrigger();   // run trigger for new sweep

} // NewBlock

void	GRAPH::RunTrigger()
{
	int i, iDiff, iSweepEnd;
//	static DWORD Arr1[1000], Arr2[1000], Arr3[1000], jj=0;

	if( iSweep >= 0 ) // if sweep exist - check for end of the sweep
	{
		iDiff = current - iSweep;    // sweep lenght
		if( iDiff<0 ) iDiff+=size;   // if cross 0
		if( iDiff<iSamplsPerSweep )    // if the sweep isn't over - end
			return;
		if( !GL.CircleRun && !GL.Run ) // if not circle and new run not alowed -
			return;                    // do not start new sweep
	}

	switch ( GL.TriggerMode )
	{
	case TRG_FREE_RUN:  //    !!! need to to fix iSweepEnd calculation
		GL.Run = false;			// disable next sweep
		if( iSweep<0 )	iSweepEnd=previous;
		else  {
			iSweepEnd = iSweep+iSamplsPerSweep;         // sweep end  !!! tofix - iSamplsPerSweep < current-previous
			if( iSweepEnd>(int)size ) iSweepEnd-=size;  // if cross 0
		}
		iPrevSweep = iSweep;
		iSweep = iSweepEnd+1;
		if( iSweep>(int)size ) iSweep-=size;   // if cross 0
		break;

	case TRG_FRONT:
		for( i=previous; i!=current; i++ )  // check each sample in the new block
		{
			if( i >= (int)size )         // circle buffer
			{
				i=0;                     // from end to degin
				if( i==current ) break;  // extra check for end of new block
			}

			if( TriggerPrevSign >= 0 )   // wait for front
				{	if( arr[trg_ch][i] < GL.TriggerThreshold )	TriggerPrevSign=-1;	}
			else
				if( arr[trg_ch][i] >= GL.TriggerThreshold ) // start new sweep
				{
					TriggerPrevSign=1;
					GL.Run=false;			// disable next sweep
					
					iPrevSweep = iSweep;
					iSweep = i;
//					Arr1[jj]=dwTicks*100; Arr2[jj]=(DWORD)GL.dbSweepStart*100; Arr3[jj]=arr[i];
//					if( ++jj>999 ) jj=0;
					break;  // for()
				}
		} // for(i)
		break;  // switch( GL.TriggerMode )

	case TRG_BACK:
		for( i=previous; i!=current; i++ )  // check each sample in the new block
		{
			if( i >= (int)size )          // circle buffer
			{
				i=0;                      // from end to degin
				if( i == current ) break; // extra check for end of new block
			}

			if( TriggerPrevSign <= 0 )  // wait for back
				{	if( arr[trg_ch][i] > GL.TriggerThreshold )	TriggerPrevSign=1;	}
			else
				if( arr[trg_ch][i] <= GL.TriggerThreshold ) // start new sweep
				{
					TriggerPrevSign=-1;
					GL.Run=false;			// disable next sweep
					
					iPrevSweep = iSweep;
					iSweep = i;

					break;  // for()
				}
		} // for(i)
		break;  // switch( GL.TriggerMode )

	}  // switch( GL.TriggerMode )

}  // RunTrigger()

// GDI+ http://msdn.microsoft.com/en-us/library/windows/desktop/ms533895(v=vs.85).aspx

void GRAPH::Draw( HDC hdc, RECT rc )
{
	bool	bPrevSweep=false;
	int		x, y, iStart, iEnd, iDiff;
	DWORD   i, j;
	HPEN	hPen, hPenDot;
	HBRUSH	hBr;
	HGDIOBJ hOld1, hOld2;

/*	static DWORD Arr1[1000], Arr2[1000], Arr3[1000], jj=0;
	Arr1[jj]=iSweep; Arr2[jj]=iPrevSweep; Arr3[jj]=arr[iSweep];
	if( ++jj>999 ) jj=0; // */

	for( j=0; j<ch_nums; j++ ) // draw limits line
	{
		hPenDot = CreatePen( PS_DOT,   1, color[j] );  // draw up and down line
		hOld1 = SelectObject( hdc, hPenDot );
		MoveToEx( hdc, 0,yShift[j],     NULL );    LineTo( hdc, rc.right, yShift[j] );
		MoveToEx( hdc, 0,yShift[j]+255, NULL );    LineTo( hdc, rc.right, yShift[j]+255 );
		SelectObject( hdc, hOld1 );
		DeleteObject( hPenDot );
	}

	char sz[100];

	_itoa_s( dwSPS, sz,sizeof(sz), 10 );
	DrawTextA(hdc, sz, -1, &rc, DT_SINGLELINE|DT_BOTTOM|DT_RIGHT); 

	iStart = 0;
	if( iSweep >= 0 ) // is sweep ready ?
	{
		iDiff = current - iSweep;	if( iDiff <= 0 ) iDiff += size;
		if( iDiff >= (int)iSamplsPerSweep )	iStart = iSweep;
	}
	if( !iStart && iPrevSweep >= 0 ) // is prevsweep kept ?   !!!!!!!!!!!!  
		iStart = iPrevSweep;


	if( !iStart )  // is sweep ready ?
	{
#define  SPOT_RADIUS 5     // sweep not ready - draw only spoot
		for( j=0; j<ch_nums; j++ )
		{
			hPen = CreatePen( PS_SOLID, 2, color[j] );
			hOld1 = SelectObject( hdc, hPen );
			hBr = CreateSolidBrush( color[j] );
			hOld2 = SelectObject( hdc, hBr );

			Ellipse(hdc, 0, 0x7f+yShift[j]-SPOT_RADIUS, SPOT_RADIUS*2, 0x7F+yShift[j]+SPOT_RADIUS);

			SelectObject( hdc, hOld2 );
			SelectObject( hdc, hOld1 );
			DeleteObject( hBr );
			DeleteObject( hPen );
		} // for( ch_nums )
		return;
	}

	double dbSamplInDot, dbNextDot;

	for( j=0; j<ch_nums; j++ )
	{
		hPen = CreatePen( PS_SOLID, 2, color[j] );
		hOld1 = SelectObject( hdc, hPen );

		dbSamplInDot = (double)iSamplsPerSweep / GL.iWinSizeX;
		if( dbSamplInDot<1 ) dbSamplInDot=1;

		iEnd = iStart + iSamplsPerSweep;
		if( iEnd >= (int)size ) iEnd -= size;

		dbNextDot = iStart + dbSamplInDot;

		MoveToEx(hdc, 0,arr[j][iStart] + yShift[j], NULL);
		x=1; y=0;
		for( i=iStart+1;  i!=iEnd;  i++ )  // till end of window
		{
			if( i >= size )     i-=size, dbNextDot -= size;
			y = y ? (y+arr[j][i])/2 : arr[j][i] ;

			if( i >= (DWORD)dbNextDot )
			{
				LineTo(hdc, x, y + yShift[j]);
				x++;  y=0;
				if( x>=rc.right ) break;    // end of window
				dbNextDot += dbSamplInDot;
			}
		}

		SelectObject( hdc, hOld1 );
		DeleteObject( hPen );
	} // for( ch_nums )
}

//  --------------  GRAPH_LIST -------------

GRAPH_LIST::GRAPH_LIST()
{
	iWinSizeX=0;
	iGraphNums=0;
	dbSweepTime=60;
	Run=true;
	CircleRun=true;
//	Trigger=0;
	TriggerThreshold=0x77;
//	TriggerMode=TRG_FREE_RUN;
	TriggerMode=TRG_FRONT;
//	TriggerMode=TRG_BACK;
}

/*bool	GRAPH_LIST::SetTrigger(GRAPH *g)  // set graph as trigger
{
	int i=0;
	while( i<iGraphNums )   // check for graph in list
		if( gl[i++] == g )
			{ Trigger=g; break;	}

	if( i>=iGraphNums ) return false;

	return true;
} */

bool	GRAPH_LIST::AddGraph(GRAPH *g)
{
//	dbSweepStart = dbPrevSweepStart = 0;

	if( iGraphNums >= MAX_GRAPH_NUMS-1 )	return false;
//	if( !iGraphNums ) Trigger=g;    // if it fisrt graph - set as trigger
	gl[iGraphNums++] = g;

	int     i;
	DWORD	j, sh=0;
	for( i=0; i < iGraphNums; i++ )  // set vertical shift graph on screen
		for( j=0; j<gl[i]->ch_nums; j++ )
			gl[i]->yShift[j] = (sh++)*100;

	return true;
}

bool	GRAPH_LIST::DeleteGraph(GRAPH *g)
{
	int i=0;
	while( i<iGraphNums )
	{
		if( gl[i] == g ) break;
		i++;
	}
	if( i>=iGraphNums ) return false;

	return	DeleteGraph(i);
}

bool	GRAPH_LIST::DeleteGraph(int iNum)
{
//	dbSweepStart = dbPrevSweepStart = 0;

	if( iGraphNums<=0 || iNum>=iGraphNums ) return false;

//	if( gl[iNum]==Trigger ) Trigger=0;   // if delete trigger - set first as trigger

	for( int i=iNum; i < iGraphNums-1; i++ )
		gl[i]=gl[i+1];

//	if( !Trigger ) Trigger=gl[0];

	iGraphNums--;

	int    i;
	DWORD  j, sh=0;
	for( i=0; i < iGraphNums; i++ )  // set vertical shift graph on screen
		for( j=0; j<gl[i]->ch_nums; j++ )
			gl[i]->yShift[j] = (sh++)*100;

	return true;
}

void	GRAPH_LIST::SetScale(short iStep)
{
	if( iStep > 0 )	iWinSizeX += iWinSizeX / 10;
	else			iWinSizeX -= iWinSizeX / 10;

	RECT  rc;
	GetClientRect(hwndGraph, &rc);
	SetWinSize( hwndGraph, rc.right);
}