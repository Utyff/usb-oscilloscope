#pragma once
#include <windows.h>

#define GRAPH_BUFFER_SIZE 64*1024
#define MAX_GRAPHS   4

class GRAPH {
public:
	DWORD		size;
	int			current, previous, iSweep, iSweepEnd, iPrevSweep;  // positions in the buffer
//	double		dbSweepStart, dbPrevSweepStart, dbTickCurr, dbTickPrev;  // miliseconds
	int			yShift[MAX_GRAPHS];  // position on the screen
	DWORD		ch_nums, trg_ch;
	int			TriggerPrevSign;
	BYTE		*arr[MAX_GRAPHS];
	BOOL		Enabled[MAX_GRAPHS];
	COLORREF	color[MAX_GRAPHS];
	char		szName[MAX_GRAPHS][100];
	DWORD		nSamplesPerSec;
	int			iSamplsPerSweep;
	double		dbSmpl_Time;

	// GRAPH();
	GRAPH(COLORREF *clr=0, DWORD dwSampls=44100, int ch_Numbers=1);
	~GRAPH();

	void SetSampleRate( DWORD dwSmpls );
	void RunTrigger();
	void NewBlock( BYTE *pbBuf, DWORD dwBytes );
	void Draw( HDC hdc, RECT rc );
};

class GRAPH_LIST
{

#define	MAX_GRAPH_NUMS	10
//  Trigger modes
#define TRG_FREE_RUN	0
#define TRG_FRONT		1
#define TRG_BACK		2

public:
	double	dbSweepTime;    // miliseconds
	int		iGraphNums;
	bool	CircleRun, Run;
	int		TriggerMode;
	int		TriggerThreshold;
	int		iWinSizeX;
//	GRAPH	*Trigger;
	GRAPH	*gl[MAX_GRAPH_NUMS];

	GRAPH_LIST();
//	void	RunTrigger(DWORD dwTicks);
//	bool	SetTrigger(GRAPH *gr);
	bool	AddGraph(GRAPH *gr);
	bool	DeleteGraph(GRAPH *gr);
	bool	DeleteGraph(int iNum);
	void	SetScale(short Step);

};  // class GRAPH_LIST
