/*
    DirectSound work
*/
#include "stdafx.h"
#include <math.h>
#include <winbase.h>
#include <stdlib.h>
#include "sound.h"
#include "oscil.h"


#define MAX_STRING  151
#define MAX_DEVICES 10
#define SND_BUFFER_SIZE 32*1024
#define CAP_BUFFER_SIZE 32*1024

typedef struct _SoundDev {
	GUID  GGUID;
	WCHAR Description[MAX_STRING];
	WCHAR Module[MAX_STRING];
} SoundDev;

SoundDev stPlayDevs[MAX_DEVICES], stRecordDevs[MAX_DEVICES];


HRESULT DSError( HRESULT hRes ) {
	char *str, sznum[MAX_STRING];

	if ( hRes==DS_OK ) return hRes;

	switch(hRes) {
		case DSERR_ALLOCATED: str="ALLOCATED\n"; break;
		case DSERR_INVALIDPARAM: str="INVALIDPARAM\n"; break;
		case DSERR_OUTOFMEMORY: str="OUTOFMEMORY\n"; break;
		case DSERR_UNSUPPORTED: str="UNSUPPORTED\n"; break;
		case DSERR_NOAGGREGATION: str="NOAGGREGATION\n"; break;
		case DSERR_UNINITIALIZED: str="UNINITIALIZED\n"; break;
		case DSERR_BADFORMAT: str="BADFORMAT\n"; break;
		case DSERR_ALREADYINITIALIZED: str="ALREADYINITIALIZED\n"; break;
		case DSERR_BUFFERLOST: str="BUFFERLOST\n"; break;
		case DSERR_CONTROLUNAVAIL: str="CONTROLUNAVAIL\n"; break;
		case DSERR_GENERIC: str="GENERIC\n"; break;
		case DSERR_INVALIDCALL: str="INVALIDCALL\n"; break;
		case DSERR_OTHERAPPHASPRIO: str="OTHERAPPHASPRIO\n"; break;
		case DSERR_PRIOLEVELNEEDED: str="PRIOLEVELNEEDED\n"; break;
		default:
			{ _itoa_s( (int)(hRes), sznum, MAX_STRING, 16 );  str=sznum;  break; }
	}
	MessageBoxA(NULL,str,"DS Error",MB_OK);
	return hRes;
}


//  ---------- GRAPH_GEN ----------------

GRAPH_GEN::GRAPH_GEN(COLORREF *clr, DWORD dwSampls):GRAPH(clr, dwSampls, 2)
{
	dbCurrTime[0] = dbShiftTime[0] = dbCurrTime[1] = dbShiftTime[1] = 0;
	Amplitude[0] = Amplitude[1] = 80;  // max - 0x7F ( 127 )
	SetFreq( 0, 530 ); SetFreq( 1, 530 );
//	GraphType = GRAPH_SQUARE;
//	GraphType = GRAPH_TRIANGLE;
	GraphType[0] = GraphType[1] = GRAPH_SIN;
}

void GRAPH_GEN::SetFreq(int ch, DOUBLE dbF)   // in Hertz
{
	dbFreq[ch] = dbF;            // Hz
	dbPeriod[ch] = 1/dbFreq[ch]; // sec
}

int GRAPH_GEN::Next(int ch)   // generate next sample
{
	dbCurrTime[ch] += dbSmpl_Time/1000;   // Miliseconds to seconds
	if( dbCurrTime[ch] > dbPeriod[ch] ) dbCurrTime[ch] -= dbPeriod[ch];

	double dbShifted = dbCurrTime[ch] + dbShiftTime[ch];
	if( dbShifted > dbPeriod[ch] ) dbShifted -= dbPeriod[ch];

	static int Arr1[1000], Arr2[1000], Arr3[1000], jj=0;
	Arr1[jj]=(int)dbCurrTime[ch]*10000; Arr2[jj]=GraphType[ch]; Arr3[jj]=ch;
	if( ++jj>999 ) jj=0; // */

	int res;
	switch( GraphType[ch] )
	{
	case  GRAPH_SQUARE:
		res=dbShifted < dbPeriod[ch]/2 ? 0x7f + Amplitude[ch] : 0x7f - Amplitude[ch] ;
		return res;

	case  GRAPH_TRIANGLE:
		dbShifted += dbPeriod[ch]/4;    // for align with other
		if( dbShifted > dbPeriod[ch] ) dbShifted -= dbPeriod[ch];
		return (int)( dbShifted < dbPeriod[ch]/2 ? 2* Amplitude[ch] *               dbShifted /(dbPeriod[ch]/2) + (0x7f - Amplitude[ch])
			                                     : 2* Amplitude[ch] * (dbPeriod[ch]-dbShifted)/(dbPeriod[ch]/2) + (0x7f - Amplitude[ch]) ) ;
	case  GRAPH_SIN:
		double dbCurrRad = ( (dbShifted) / dbPeriod[ch] ) * (2*Pi);
		return (int)( 0x7f + Amplitude[ch] * sin(dbCurrRad) );
	}
	return 0;
}


// ----------- GenerateSound --------------

// ----- Configuration window
INT_PTR CALLBACK  GenWndProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)
{
	switch (uMsg)
    {
	case WM_INITDIALOG:  // set initial values
		switch (GenSnd.gr->GraphType[0] )
		{
			case GRAPH_SIN:			SendDlgItemMessage(hWnd,IDC_L_SIN, BM_SETCHECK,BST_CHECKED,0); 	break;
			case GRAPH_TRIANGLE:	SendDlgItemMessage(hWnd,IDC_L_TRI, BM_SETCHECK,BST_CHECKED,0);	break;
			case GRAPH_SQUARE:		SendDlgItemMessage(hWnd,IDC_L_SQR, BM_SETCHECK,BST_CHECKED,0);	break;
		}
		switch( GenSnd.gr->GraphType[1] )
		{
			case GRAPH_SIN:			SendDlgItemMessage(hWnd,IDC_R_SIN, BM_SETCHECK,BST_CHECKED,0); 	break;
			case GRAPH_TRIANGLE:	SendDlgItemMessage(hWnd,IDC_R_TRI, BM_SETCHECK,BST_CHECKED,0);	break;
			case GRAPH_SQUARE:		SendDlgItemMessage(hWnd,IDC_R_SQR, BM_SETCHECK,BST_CHECKED,0);	break;
		}
		return TRUE;

	case WM_COMMAND:
		switch( LOWORD(wParam) )
		{
			case IDC_L_SIN:	if( HIWORD(wParam)==BN_CLICKED )  GenSnd.gr->GraphType[0]=GRAPH_SIN;		break;
			case IDC_L_TRI:	if( HIWORD(wParam)==BN_CLICKED )  GenSnd.gr->GraphType[0]=GRAPH_TRIANGLE;	break;
			case IDC_L_SQR:	if( HIWORD(wParam)==BN_CLICKED )  GenSnd.gr->GraphType[0]=GRAPH_SQUARE;		break;
			case IDC_R_SIN:	if( HIWORD(wParam)==BN_CLICKED )  GenSnd.gr->GraphType[1]=GRAPH_SIN;		break;
			case IDC_R_TRI:	if( HIWORD(wParam)==BN_CLICKED )  GenSnd.gr->GraphType[1]=GRAPH_TRIANGLE;	break;
			case IDC_R_SQR:	if( HIWORD(wParam)==BN_CLICKED )  GenSnd.gr->GraphType[1]=GRAPH_SQUARE;		break;
		}
		break; // case WM_COMMAND:

	}
	return FALSE;
}

bool GenerateSound::Initialize()
{
	HRESULT result;
	DSBUFFERDESC bufferDesc;
	WAVEFORMATEX waveFormat;

	nSamplesPerSec = 44100;

// Initialize the direct sound interface pointer for the default sound device.
	result = DirectSoundCreate8(NULL, &DirectSound, NULL);
	if( DSError(result) != DS_OK )
		return false;

// Set the cooperative level to priority so the format of the primary sound buffer can be modified.
	result = DirectSound->SetCooperativeLevel(hWnd, DSSCL_WRITEPRIMARY);
	if( DSError(result) != DS_OK )
		return false;

// Setup the primary buffer description.
	bufferDesc.dwSize = sizeof(DSBUFFERDESC);
	bufferDesc.dwFlags = DSBCAPS_PRIMARYBUFFER;
	bufferDesc.dwBufferBytes = 0;
	bufferDesc.dwReserved = 0;
	bufferDesc.lpwfxFormat = NULL;
	bufferDesc.guid3DAlgorithm = GUID_NULL;

// Get control of the primary sound buffer on the default sound device.
	result = DirectSound->CreateSoundBuffer(&bufferDesc, &SndBuffer, NULL);
	if( DSError(result) != DS_OK )
		return false;

// Setup the format of the primary sound bufffer.
	waveFormat.wFormatTag = WAVE_FORMAT_PCM;
	waveFormat.nSamplesPerSec = nSamplesPerSec;
	waveFormat.wBitsPerSample = 8;
	waveFormat.nChannels = 2;
	waveFormat.nBlockAlign = (waveFormat.wBitsPerSample / 8) * waveFormat.nChannels;
	waveFormat.nAvgBytesPerSec = waveFormat.nSamplesPerSec * waveFormat.nBlockAlign;
	waveFormat.cbSize = 0;

// Set the primary buffer to be the wave format specified.
	result = SndBuffer->SetFormat(&waveFormat);
	if( DSError(result) != DS_OK )
		return false;

	DSBCAPS stDSBCAPS;
	stDSBCAPS.dwSize=sizeof(stDSBCAPS);
	result = SndBuffer->GetCaps(&stDSBCAPS);
	dwBufferBytes=stDSBCAPS.dwBufferBytes;

	return true;
}

void GenerateSound::Shutdown()
{
	Stop();

	// Release the primary sound buffer pointer.
	if(SndBuffer)
	{
		SndBuffer->Release();
		SndBuffer = 0;
	}

	// Release the direct sound interface pointer.
	if(DirectSound)
	{
		DirectSound->Release();
		DirectSound = 0;
	}

	return;
}

bool GenerateSound::SetChannels(BYTE ch)
{
	HRESULT		result;

	if( hThredGen )  Stop();
	if( !ch )        return true;
	if( !SndBuffer ) return false;

	COLORREF clr[2]={RGB(186,0,21),RGB(234,182,0)};

	gr = new GRAPH_GEN( clr, nSamplesPerSec );
	GL.AddGraph( gr );

	result = SndBuffer->Play( 0, 0, DSBPLAY_LOOPING );
	if( DSError(result) != DS_OK )
		return false;

	bStopThread=false;
	hThredGen=CreateThread(NULL, 0, threadGenerate, this, 0, 0);

	SetThreadPriority(hThredGen, THREAD_PRIORITY_ABOVE_NORMAL);
	SetThreadPriority(hThredGen, THREAD_PRIORITY_HIGHEST);

	hwndPage = CfgTab.AddTab( TEXT("Generation"), IDD_GENERATOR, GenWndProc );

	return true;
}

bool GenerateSound::Stop()
{
	BOOL  b;
	DWORD dw;

	CfgTab.DeleteTab(hwndPage);

	if ( hThredGen )      // Stop generate thread if it runned
	{
		b=GetExitCodeThread(hThredGen,&dw);
		if( dw != 259 ) DSError(8);  // it should never happen
		bStopThread=true;
		dw=WaitForSingleObject(hThredGen,1000);
		b=GetExitCodeThread(hThredGen,&dw);
		if( dw == 259 ) DSError(9);  // it should never happen
		CloseHandle(hThredGen);
		hThredGen=0;
	}

	if( SndBuffer ) SndBuffer->Stop();   // stop generate

	if( gr )
	{
		GL.DeleteGraph( gr );
		delete gr;
		gr = 0;
	}
	
	return true;
}

DWORD WINAPI GenerateSound::threadGenerate( LPVOID lpParam ) 
{
	DWORD		i;
	DWORD		dwCurrentPlayCursor, dwCurrentWriteCursor, dwPrevPlayCursor, dwBlockSize;
	HRESULT		result;

	VOID		*pvAudioPtr1, *pvAudioPtr2;
	DWORD		dwAudioBytes1, dwAudioBytes2, dwTicks, dwRedrawTicks=0;
	SYSTEMTIME  SysTime;

	GRAPH_GEN		*Gr;
	GenerateSound	*gensnd=(GenerateSound*)lpParam;

	if( !gensnd ) return 1;
	Gr=gensnd->gr;
	if( !Gr )  return 1;

	dwPrevPlayCursor = dwCurrentPlayCursor = 0;
	Gr->current = 0;

// --------- Initial filling sound buffer
	result = gensnd->SndBuffer->Lock(0,0, &pvAudioPtr1, &dwAudioBytes1, 0, 0, DSBLOCK_ENTIREBUFFER);
	DSError(result);

	for( i=0; i<dwAudioBytes1; i++ )
	{
		((BYTE*)pvAudioPtr1)[i] = Gr->Next(0);
		i++;
		((BYTE*)pvAudioPtr1)[i] = Gr->Next(1);
	}
	result = gensnd->SndBuffer->Unlock(pvAudioPtr1, dwAudioBytes1, 0, 0);
	DSError(result);

// --------- Main generation circle
	while( !gensnd->bStopThread )  // Generation circle until exit signal.
	{
		GetSystemTime(&SysTime);
		dwTicks = SysTime.wMilliseconds + 1000 * (SysTime.wSecond + 60 * (SysTime.wMinute + 60 * (SysTime.wHour + 24*SysTime.wDay)));   // miliseconds from 1-st day of month

		result = gensnd->SndBuffer->GetCurrentPosition(&dwCurrentPlayCursor, &dwCurrentWriteCursor);
		DSError(result);

		if( dwPrevPlayCursor != dwCurrentPlayCursor )   // are there new space for data ?
		{
			// Get size of block for fill.  Check for end of capture buffer
			dwBlockSize = dwPrevPlayCursor < dwCurrentPlayCursor ?
				dwCurrentPlayCursor - dwPrevPlayCursor : gensnd->dwBufferBytes - dwPrevPlayCursor;

			result = gensnd->SndBuffer->Lock(dwPrevPlayCursor,dwBlockSize,
				         &pvAudioPtr1, &dwAudioBytes1, &pvAudioPtr2, &dwAudioBytes2, 0);
			DSError(result);
			if( dwAudioBytes2 ) DSError(7); // this should never happen

			//  Get new data for graphs
			Gr->NewBlock((BYTE*)pvAudioPtr1,   dwAudioBytes1);

			//  Fill sound buffer
			for( i=0; i<dwAudioBytes1; i++ )
			{
				((BYTE*)pvAudioPtr1)[i] = Gr->Next(0);
				i++;
				((BYTE*)pvAudioPtr1)[i] = Gr->Next(1);
			}

			result = gensnd->SndBuffer->Unlock(pvAudioPtr1, dwAudioBytes1, 0, 0);  // Release sound buffer
			DSError(result);

			// Set new filled position
			dwPrevPlayCursor = dwPrevPlayCursor < dwCurrentPlayCursor ? dwCurrentPlayCursor : 0;
		}

		Sleep(1);
	}   // --------- Main generation circle

	return 10;
}   // GenerateSound::threadGenerate()


// ----------- CaptureSound ---------------

INT_PTR CALLBACK  SndWndProc(HWND hWnd, UINT uMsg, WPARAM wParam, LPARAM lParam)  // Configuration window
{
	switch (uMsg)
    {
	case WM_INITDIALOG:
		SendDlgItemMessage(hWnd,IDC_TRG_L,BM_SETCHECK,BST_CHECKED,0); // set Left channel as trigger
		return TRUE;

	case WM_COMMAND:
		switch (LOWORD(wParam) )
		{
		case IDC_TRG_L:	if( HIWORD(wParam)==BN_CLICKED )  CL.gr->trg_ch=0;	break; // set Left  channel as trigger
		case IDC_TRG_R:	if( HIWORD(wParam)==BN_CLICKED )  CL.gr->trg_ch=1;	break; // set Right channel as trigger
		}
		break; // case WM_COMMAND:

	}
	return FALSE;
}

bool CaptureSound::Initialize()
{
	HRESULT			result;
	DSCBUFFERDESC	bufferCapDesc;
	WAVEFORMATEX	waveFormat;

	nSamplesPerSec=44100;

 	// Initialize the direct sound interface pointer for the default sound device.
	result = DirectSoundCaptureCreate8(NULL, &DirectSoundCap, NULL);
	if( DSError(result) != DS_OK )
		return false;
 
/*	DSCCAPS stDSCCAPS;
	stDSCCAPS.dwSize=sizeof(stDSCCAPS);
	result = DirectSoundCap->GetCaps(&stDSCCAPS);  //  Format None-1048575
*/
	// Setup the format of the primary sound bufffer.
	waveFormat.wFormatTag = WAVE_FORMAT_PCM;
	waveFormat.nSamplesPerSec = nSamplesPerSec;
	waveFormat.wBitsPerSample = 8;
	waveFormat.nChannels = 2;
	waveFormat.nBlockAlign = (waveFormat.wBitsPerSample / 8) * waveFormat.nChannels;
	waveFormat.nAvgBytesPerSec = waveFormat.nSamplesPerSec * waveFormat.nBlockAlign;
	waveFormat.cbSize = 0;

	// Setup the primary buffer description.
	ZeroMemory( &bufferCapDesc, sizeof(bufferCapDesc) );
	bufferCapDesc.dwSize = sizeof(DSCBUFFERDESC);
	bufferCapDesc.dwBufferBytes = CAP_BUFFER_SIZE;   // buffer size
	bufferCapDesc.lpwfxFormat = &waveFormat;

	// Get control of the primary sound buffer on the default sound device.
	result = DirectSoundCap->CreateCaptureBuffer(&bufferCapDesc, &CapBuffer, NULL);
	if( DSError(result) != DS_OK )
		return false;

	return true;
}

void CaptureSound::Shutdown()
{
	Stop();

	// Release the primary sound buffer pointer.
	if(CapBuffer)
	{
		CapBuffer->Release();
		CapBuffer = 0;
	}
 
	// Release the direct sound interface pointer.
	if(DirectSoundCap)
	{
		DirectSoundCap->Release();
		DirectSoundCap = 0;
	}
 
	return;
}

bool CaptureSound::SetChannels(BYTE ch)
{
	HRESULT		result;

	if( hThredCap )  Stop();
	if( !ch )        return true;
	if( !CapBuffer ) return false;

	COLORREF clr[2]={RGB(0,100,255),RGB(34,177,76)};

	gr = new GRAPH( clr, nSamplesPerSec, 2 );
	GL.AddGraph( gr );

	result = CapBuffer->Start(DSCBSTART_LOOPING);  //  DSCBSTART_LOOPING
	if( DSError(result)!=DS_OK )
		return false;

	bStopThread=false;
	hThredCap=CreateThread(NULL, 0, threadCapture, this, 0, 0);

	SetThreadPriority(hThredCap, THREAD_PRIORITY_ABOVE_NORMAL);
	SetThreadPriority(hThredCap, THREAD_PRIORITY_HIGHEST);

	hwndPage = CfgTab.AddTab( TEXT("Sound In."), IDD_SND_CAPTURE, SndWndProc );

	return true;
}

bool CaptureSound::Stop()
{
	BOOL  b;
	DWORD dw;

	CfgTab.DeleteTab(hwndPage);

	if ( hThredCap )        // Stop capture thread if it runned
	{
		b=GetExitCodeThread(hThredCap,&dw);
		if( dw != 259 ) DSError(8);  // it should never happen
		bStopThread=true;
		dw=WaitForSingleObject(hThredCap,100);
		b=GetExitCodeThread(hThredCap,&dw);
		if( dw == 259 ) DSError(9);  // it should never happen
		CloseHandle(hThredCap);
		hThredCap=0;
	}

	if( CapBuffer ) CapBuffer->Stop();   // stop capturing

	if( gr )
	{
		GL.DeleteGraph( gr );
		delete gr;
		gr = 0;
	}
	
	return true;
}

DWORD WINAPI CaptureSound::threadCapture( LPVOID lpParam ) 
{
	DWORD		dwCapturePosition, dwPrevCapturePosition, dwBlockSize;
	HRESULT		result;

	VOID		*pvAudioPtr1, *pvAudioPtr2;
	DWORD		dwAudioBytes1, dwAudioBytes2, dwTicks, dwRedrawTicks=0;
	SYSTEMTIME  SysTime;

	GRAPH		*Gr;
	CaptureSound	*capsnd=(CaptureSound*)lpParam;

//	DWORD	Arr1[1000], Arr2[1000], jj=0;

	if( !capsnd ) return 1;
	Gr=capsnd->gr;
	if( !Gr )  return 1;

	dwPrevCapturePosition = dwCapturePosition = 0;
	Gr->current = 0;
 
// -------- Main capting circle
	while( !capsnd->bStopThread )  // Capture circle until exit signal.
	{
		GetSystemTime(&SysTime);
		dwTicks = SysTime.wMilliseconds + 1000 * (SysTime.wSecond + 60 * (SysTime.wMinute + 60 * (SysTime.wHour + 24*SysTime.wDay)));   // miliseconds from 1-st day of month

		result = capsnd->CapBuffer->GetCurrentPosition(&dwCapturePosition, 0); // &dwReadPosition);
		DSError(result);

		if( dwPrevCapturePosition != dwCapturePosition ) // are there new data ?
		{
			// Get size of block for copy.  Check for end of capture buffer
			dwBlockSize = dwPrevCapturePosition < dwCapturePosition ?
				dwCapturePosition - dwPrevCapturePosition : CAP_BUFFER_SIZE - dwPrevCapturePosition;

			result = capsnd->CapBuffer->Lock(dwPrevCapturePosition,dwBlockSize,
				         &pvAudioPtr1, &dwAudioBytes1, &pvAudioPtr2, &dwAudioBytes2, 0);
			DSError(result);
			if( dwAudioBytes2 ) DSError(7); // it should never happen

			//  Get new data for graph
			Gr->NewBlock((BYTE*)pvAudioPtr1,   dwAudioBytes1);

			result = capsnd->CapBuffer->Unlock(pvAudioPtr1, dwAudioBytes1, 0, 0);
			DSError(result);

			// Set new readed position
			dwPrevCapturePosition = dwPrevCapturePosition < dwCapturePosition ? dwCapturePosition : 0;
		}

//		Arr1[jj]=dwTicks; Arr2[jj]=dwCapturePosition;
//		if( ++jj>999 ) jj=0;

		Sleep(1);
	} // ----------- Main caturing circle

	return 15; // Exit from capture thread
} // CaptureSound::threadCapture()


//  ------------ Enumerate sound devices -----------

BOOL CALLBACK DSPlayEnumCallback (LPGUID pGUID, LPCWSTR Description, LPCWSTR Module, VOID *Context)
{
	if ( !pGUID ) return true;

	int i=0;

	while( stPlayDevs[i].GGUID.Data1 != 0 )	i++;
	memcpy(&stPlayDevs[i].GGUID, pGUID, sizeof GUID);
	wcscpy_s(stPlayDevs[i].Description, MAX_STRING, Description);
	wcscpy_s(stPlayDevs[i].Module,      MAX_STRING, Module);

	return true;
}

BOOL CALLBACK DSRecordEnumCallback (LPGUID pGUID, LPCWSTR Description, LPCWSTR Module, VOID *Context)
{
	if ( !pGUID ) return true;

	int i=0;

	while( stRecordDevs[i].GGUID.Data1 != 0 )	i++;
	memcpy(&stRecordDevs[i].GGUID, pGUID, sizeof GUID);
	wcscpy_s(stRecordDevs[i].Description, MAX_STRING, Description);
	wcscpy_s(stRecordDevs[i].Module,      MAX_STRING, Module);

	return true;
}

BOOL GetSoundDevs()
{
	memset(&stPlayDevs,   0, sizeof stPlayDevs);
	memset(&stRecordDevs, 0, sizeof stRecordDevs);

	if ( (DirectSoundEnumerate((LPDSENUMCALLBACK)DSPlayEnumCallback, (LPVOID)"PlayDev")) != DS_OK )   return false;
	if ( (DirectSoundCaptureEnumerate((LPDSENUMCALLBACK)DSRecordEnumCallback, (LPVOID)"RecordDev")) != DS_OK )   return false;

	return true;
}
