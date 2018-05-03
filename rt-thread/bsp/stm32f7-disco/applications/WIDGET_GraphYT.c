/*********************************************************************
*                SEGGER Microcontroller GmbH & Co. KG                *
*        Solutions for real time microcontroller applications        *
**********************************************************************
*                                                                    *
*        (c) 1996 - 2013  SEGGER Microcontroller GmbH & Co. KG       *
*                                                                    *
*        Internet: www.segger.com    Support:  support@segger.com    *
*                                                                    *
**********************************************************************

** emWin V5.22 - Graphical user interface for embedded applications **
emWin is protected by international copyright laws.   Knowledge of the
source code may not be used to write a similar product.  This file may
only be used in accordance with a license and should not be re-
distributed in any way. We appreciate your understanding and fairness.
----------------------------------------------------------------------
File        : WIDGET_Graph.c
Purpose     : Demonstrates the use of the GRAPH widget
----------------------------------------------------------------------
*/
#include <rtthread.h>
#ifdef RT_USING_STEMWIN

#include <stdlib.h>
#include <string.h>

#include "DIALOG.h"
#include "GRAPH.h"
#include <drv_ADC.h>

/*********************************************************************
*
*       Defines
*
**********************************************************************
*/
#define MAX_VALUE 180

/*********************************************************************
*
*       Static data
*
**********************************************************************
*/
#define A_CURRENT_NUM 0
#define B_CURRENT_NUM 1
#define C_CURRENT_NUM 2
#define A_VOLTAGE_NUM 3
#define B_VOLTAGE_NUM 4
#define C_VOLTAGE_NUM 5
#define R_VOLTAGE_NUM 6
#define S_VOLTAGE_NUM 7
#define T_VOLTAGE_NUM 8
#define BUS_I_NUM 9
#define BUS_V_NUM 10
#define TEMP_NUM 11
#define GRAPHX 675
#define GRAPHY 350
#define A_CURRENT_PROGBAR GUI_ID_PROGBAR0
#define B_CURRENT_PROGBAR GUI_ID_PROGBAR1
#define C_CURRENT_PROGBAR GUI_ID_PROGBAR2
#define A_VOLTAGE_PROGBAR GUI_ID_PROGBAR3
#define B_VOLTAGE_PROGBAR GUI_ID_PROGBAR4
#define C_VOLTAGE_PROGBAR GUI_ID_PROGBAR5
#define R_VOLTAGE_PROGBAR GUI_ID_PROGBAR6
#define S_VOLTAGE_PROGBAR GUI_ID_PROGBAR7
#define T_VOLTAGE_PROGBAR GUI_ID_PROGBAR8
#define BUSI_PROGBAR GUI_ID_PROGBAR9
#define BUSV_PROGBAR GUI_ID_PROGBAR10
#define TEMP_PROGBAR GUI_ID_PROGBAR11
#define A_CURRENT_CHECK GUI_ID_CHECK2
#define A_VOLTAGE_CHECK GUI_ID_CHECK3
#define R_VOLTAGE_CHECK GUI_ID_CHECK4
#define BUSI_CHECK GUI_ID_CHECK5


static	WM_HWIN hDlg, hGraph = 0;
/* Maximum ranges  */
static int progMin = 0, progmax = 500;
static GRAPH_DATA_Handle  _ahData[12]; /* Array of handles for the GRAPH_DATA objects */
static GRAPH_SCALE_Handle _hScaleV;   /* Handle of vertical scale */
static GRAPH_SCALE_Handle _hScaleH;   /* Handle of horizontal scale */

typedef struct 
{
	I16 A_Current[GRAPHX];
	I16 B_Current[GRAPHX];
	I16 C_Current[GRAPHX];

	I16 A_Voltage[GRAPHX];
	I16 B_Voltage[GRAPHX];
	I16 C_Voltage[GRAPHX];

	I16 R_Voltage[GRAPHX];
	I16 S_Voltage[GRAPHX];
	I16 T_Voltage[GRAPHX];

	I16 BUS_I[GRAPHX];
	I16 BUS_V[GRAPHX];
	I16 TEMP[GRAPHX];
}  GraphValuetypedef;
static 	GraphValuetypedef GraphValue,GraphValueDisp;


static I16 _aValue[3];
static int _Stop;
static int A_Current_clicked = 1;
static int A_Voltage_clicked = 0;
static int R_Voltage_clicked = 0;
static int BUSI_clicked = 0;
static int CurrentGraphDeal = 0;
static int A_VoltageGraphDeal = 0;
static int R_VoltageGraphDeal = 0;
static int BUSIGraphDeal = 0;

static GUI_COLOR _aColor[] = {GUI_BLUE, GUI_GREEN,GUI_RED,
								GUI_YELLOW,	GUI_CYAN,GUI_MAGENTA,
								GUI_WHITE,GUI_BROWN,GUI_ORANGE,
								GUI_DARKGRAY,GUI_LIGHTBLUE,GUI_LIGHTRED}; /* Array of colors for the GRAPH_DATA objects */

/* Dialog ressource */
static const GUI_WIDGET_CREATE_INFO _aDialogCreate[] = {
  { FRAMEWIN_CreateIndirect, "Motor control system by QIEM",  0     ,   0,   0, 800, 480, 0},
  	#if 1
  { GRAPH_CreateIndirect,     0,                   GUI_ID_GRAPH0    ,   5,   5, 675, 350 },
  #endif
  { TEXT_CreateIndirect,      "Spacing X:",        0                ,  700, 40,  80,  24 },
  { TEXT_CreateIndirect,      "Spacing Y:",        0                ,  700, 120,  80,  24 },
  { SLIDER_CreateIndirect,    0,                   GUI_ID_SLIDER0   ,  692, 70,  95,  38 },
  { SLIDER_CreateIndirect,    0,                   GUI_ID_SLIDER1   ,  692, 150, 95,  38 },
  { CHECKBOX_CreateIndirect,  0,                   GUI_ID_CHECK0    , 700, 200,  125,   0 },
  { CHECKBOX_CreateIndirect,  0,                   GUI_ID_CHECK1    , 700, 225,  125,   0 },
#if 0
  { TEXT_CreateIndirect,      "Border",            0                , 280,   5,  35,  15 },
  { CHECKBOX_CreateIndirect,  0,                   GUI_ID_CHECK2    , 280,  20,  35,   0 },
  { CHECKBOX_CreateIndirect,  0,                   GUI_ID_CHECK3    , 280,  40,  35,   0 },
  { CHECKBOX_CreateIndirect,  0,                   GUI_ID_CHECK4    , 280,  60,  35,   0 },
  { CHECKBOX_CreateIndirect,  0,                   GUI_ID_CHECK5    , 280,  80,  35,   0 },

  { TEXT_CreateIndirect,      "Effect",            0                , 280, 100,  35,  15 },
  { RADIO_CreateIndirect,     0,                   GUI_ID_RADIO0    , 275, 115,  35,   0, 0, 3 },
#endif
  { CHECKBOX_CreateIndirect,  0,                   GUI_ID_CHECK6    , 700, 250,  125,   0 },
  { CHECKBOX_CreateIndirect,  0,                   GUI_ID_CHECK7    , 700, 275,  125,   0 },
  { BUTTON_CreateIndirect,    "Full Screen",       GUI_ID_BUTTON0   , 700, 300,  80,  36 },
  
  { CHECKBOX_CreateIndirect,  0,                            A_CURRENT_CHECK, 10, 360,  180,   0 },
  { PROGBAR_CreateIndirect,   "A Current",                    GUI_ID_PROGBAR0,   90, 360, 100,  18},
  { TEXT_CreateIndirect,		"B Current",		 0				  ,  10, 390,	80,  18 },
  { PROGBAR_CreateIndirect,   "B Current",                    GUI_ID_PROGBAR1,   90, 390, 100,  18},
  { TEXT_CreateIndirect,		  "C Current", 	   0				,  10, 420,   80,  18 },
  { PROGBAR_CreateIndirect,   "C Current",                    GUI_ID_PROGBAR2,   90, 420, 100,  18},

  { CHECKBOX_CreateIndirect,  0,                            A_VOLTAGE_CHECK    , 220, 360,  180,   0 },
  { PROGBAR_CreateIndirect,   "A Voltage",                    GUI_ID_PROGBAR3,   300, 360, 100,  18},
	{ TEXT_CreateIndirect,		  "B Voltage",		   0				,  220, 390,   80,  18 },
  { PROGBAR_CreateIndirect,   "B Voltage",                    GUI_ID_PROGBAR4,   300, 390, 100,  18},
	{ TEXT_CreateIndirect,		  "C Voltage",		   0				,  220, 420,   80,  18 },
  { PROGBAR_CreateIndirect,   "C Voltage",                    GUI_ID_PROGBAR5,   300, 420, 100,  18},

  { CHECKBOX_CreateIndirect,  0,                            R_VOLTAGE_CHECK    , 420, 360,  180,   0 },
  { PROGBAR_CreateIndirect,   "R Voltage",                    GUI_ID_PROGBAR6,   500, 360, 100,  18},
	{ TEXT_CreateIndirect,		  "S Voltage",		   0				,  420, 390,   80,  18 },
  { PROGBAR_CreateIndirect,   "S Voltage",                    GUI_ID_PROGBAR7,   500, 390, 100,  18},
   { TEXT_CreateIndirect,		"T Voltage",		 0				  ,  420, 420,	80,  18 },
  { PROGBAR_CreateIndirect,   "T Voltage",                    GUI_ID_PROGBAR8,   500, 420, 100,  18},

  { CHECKBOX_CreateIndirect,  0,                            BUSI_CHECK    , 620, 360,  180,   0 },
  { PROGBAR_CreateIndirect,   "BUS I",                    GUI_ID_PROGBAR9,   680, 360, 100,  18},
   { TEXT_CreateIndirect,		"BUS V",		 0				  ,  620, 390,	80,  18 },
  { PROGBAR_CreateIndirect,   "BUS V",                    GUI_ID_PROGBAR10,   680, 390, 100,  18},
   { TEXT_CreateIndirect,		"TEMP",		 0				  ,  620, 420,	80,  18 },
  { PROGBAR_CreateIndirect,   "TEMP",                    GUI_ID_PROGBAR11,   680, 420, 100,  18},



  //{ PROGBAR_CreateIndirect,   "",                    GUI_ID_PROGBAR2,   300, 370, 100,  18},
  //{ PROGBAR_CreateIndirect,   "",                    GUI_ID_PROGBAR3,   300, 410, 100,  18},
#if 0
  { CHECKBOX_CreateIndirect,  0,                   GUI_ID_CHECK8    , 235, 205,  70,   0 },
#endif
};

/*********************************************************************
*
*       Static code
*
**********************************************************************


*/
/*********************************************************************
*
*       _
*
* Purpose:
*   This routine calculates new random values in dependence of the previous added values
*   and adds them to the GRAPH_DATA objects
*/
void ArrayDeal(void) {
	I16 i = GRAPHX-1;

	if((A_Current_clicked == 1) && (CurrentGraphDeal == 0))
	{
		for(i = GRAPHX-1;i > 0;i--)
		{
			
			GraphValue.A_Current[i] = GraphValue.A_Current[i-1];
			GraphValue.B_Current[i] = GraphValue.B_Current[i-1];
			GraphValue.C_Current[i] = GraphValue.C_Current[i-1];
			
		}

		GraphValue.A_Current[i] = Global_User_ADC.PhaseA.qI_value;
		GraphValue.B_Current[i] = Global_User_ADC.PhaseB.qI_value;
		GraphValue.C_Current[i] = Global_User_ADC.PhaseC.qI_value;

	}

    if((A_Voltage_clicked == 1) && (A_VoltageGraphDeal == 0))
    {
		for(i = GRAPHX-1;i > 0;i--)
		{

			GraphValue.A_Voltage[i] = GraphValue.A_Voltage[i-1];
			GraphValue.B_Voltage[i] = GraphValue.B_Voltage[i-1];
			GraphValue.C_Voltage[i] = GraphValue.C_Voltage[i-1];
		}
		GraphValue.A_Voltage[i] = Global_User_ADC.PhaseA.qV_value;
		GraphValue.B_Voltage[i] = Global_User_ADC.PhaseB.qV_value;
		GraphValue.C_Voltage[i] = Global_User_ADC.PhaseC.qV_value;
    }
	
	if((R_Voltage_clicked == 1) && (R_VoltageGraphDeal == 0))
	{
		for(i = GRAPHX-1;i > 0;i--)
		{
			GraphValue.R_Voltage[i] = GraphValue.R_Voltage[i-1];
			GraphValue.S_Voltage[i] = GraphValue.S_Voltage[i-1];
			GraphValue.T_Voltage[i] = GraphValue.T_Voltage[i-1];
		}
		GraphValue.R_Voltage[i] = Global_User_ADC.UR.qV_value;
		GraphValue.S_Voltage[i] = Global_User_ADC.US.qV_value;
		GraphValue.T_Voltage[i] = Global_User_ADC.UT.qV_value;
	}
	if((BUSI_clicked == 1) && (BUSIGraphDeal == 0))
	{
		for(i = GRAPHX-1;i > 0;i--)
		{
			GraphValue.BUS_I[i] = GraphValue.BUS_I[i-1];
			GraphValue.BUS_V[i] = GraphValue.BUS_V[i-1];
			GraphValue.TEMP[i] = GraphValue.TEMP[i-1];
		}
		GraphValue.BUS_I[i] = Global_User_ADC.BUS.qI_value;
		GraphValue.BUS_V[i] = Global_User_ADC.BUS.qV_value;
		GraphValue.TEMP[i] = Global_User_ADC.Temp.TEMP_value;
	}
}


/*********************************************************************
*
*       _AddValues
*
* Purpose:
*   This routine calculates new random values in dependence of the previous added values
*   and adds them to the GRAPH_DATA objects
*/
static void _AddValues(WM_HWIN hGraph) {
	int i;
	
	if(A_Current_clicked)
	{				
		CurrentGraphDeal = 1;
		for(i = 0;i < GRAPHX;i++)
		{
			GraphValueDisp.A_Current[i] = GraphValue.A_Current[i];
			GraphValueDisp.B_Current[i] = GraphValue.B_Current[i];
			GraphValueDisp.C_Current[i] = GraphValue.C_Current[i];
		}
		CurrentGraphDeal = 0;
		for(i = GRAPHX-1;i > 0;i--)
		{
			//rt_kprintf("GraphValue.A_Currrent = %d\r\n", GraphValue.A_Current[i]);
			GRAPH_DATA_YT_AddValue(_ahData[A_CURRENT_NUM], GraphValueDisp.A_Current[i]);
			GRAPH_DATA_YT_AddValue(_ahData[B_CURRENT_NUM], GraphValueDisp.B_Current[i]);
			GRAPH_DATA_YT_AddValue(_ahData[C_CURRENT_NUM], GraphValueDisp.C_Current[i]);
		}
		GRAPH_DATA_YT_AddValue(_ahData[A_CURRENT_NUM], GraphValueDisp.A_Current[i]);
		GRAPH_DATA_YT_AddValue(_ahData[B_CURRENT_NUM], GraphValueDisp.B_Current[i]);
		GRAPH_DATA_YT_AddValue(_ahData[C_CURRENT_NUM], GraphValueDisp.C_Current[i]);
	}
	if(A_Voltage_clicked)
	{

		A_VoltageGraphDeal = 1;
		for(i = 0;i < GRAPHX;i++)
		{
			GraphValueDisp.A_Voltage[i] = GraphValue.A_Voltage[i];
			GraphValueDisp.B_Voltage[i] = GraphValue.B_Voltage[i];
			GraphValueDisp.C_Voltage[i] = GraphValue.C_Voltage[i];
		}
		A_VoltageGraphDeal = 0;

		for(i = GRAPHX-1;i > 0;i--)
		{
			GRAPH_DATA_YT_AddValue(_ahData[A_VOLTAGE_NUM], GraphValueDisp.A_Voltage[i]);
			GRAPH_DATA_YT_AddValue(_ahData[B_VOLTAGE_NUM], GraphValueDisp.B_Voltage[i]);
			GRAPH_DATA_YT_AddValue(_ahData[C_VOLTAGE_NUM], GraphValueDisp.C_Voltage[i]);

		}
		GRAPH_DATA_YT_AddValue(_ahData[A_VOLTAGE_NUM], GraphValueDisp.A_Voltage[i]);
		GRAPH_DATA_YT_AddValue(_ahData[B_VOLTAGE_NUM], GraphValueDisp.B_Voltage[i]);
		GRAPH_DATA_YT_AddValue(_ahData[C_VOLTAGE_NUM], GraphValueDisp.C_Voltage[i]);
		
	}
	
	if(R_Voltage_clicked)
	{
		R_VoltageGraphDeal = 1;
		for(i = 0;i < GRAPHX;i++)
		{
			GraphValueDisp.R_Voltage[i] = GraphValue.R_Voltage[i];
			GraphValueDisp.S_Voltage[i] = GraphValue.S_Voltage[i];
			GraphValueDisp.T_Voltage[i] = GraphValue.T_Voltage[i];
		}
		R_VoltageGraphDeal = 0;
		
		for(i = GRAPHX-1;i > 0;i--)
		{

			GRAPH_DATA_YT_AddValue(_ahData[R_VOLTAGE_NUM], GraphValueDisp.R_Voltage[i]);
			GRAPH_DATA_YT_AddValue(_ahData[S_VOLTAGE_NUM], GraphValueDisp.S_Voltage[i]);
			GRAPH_DATA_YT_AddValue(_ahData[T_VOLTAGE_NUM], GraphValueDisp.T_Voltage[i]);
		}
		GRAPH_DATA_YT_AddValue(_ahData[R_VOLTAGE_NUM], GraphValueDisp.R_Voltage[i]);
		GRAPH_DATA_YT_AddValue(_ahData[S_VOLTAGE_NUM], GraphValueDisp.S_Voltage[i]);
		GRAPH_DATA_YT_AddValue(_ahData[T_VOLTAGE_NUM], GraphValueDisp.T_Voltage[i]);
				
	}
	if(BUSI_clicked)
	{


		BUSIGraphDeal = 1;
		for(i = 0;i < GRAPHX;i++)
		{
			GraphValueDisp.BUS_I[i] = GraphValue.BUS_I[i];
			GraphValueDisp.BUS_V[i] = GraphValue.BUS_V[i];
			GraphValueDisp.TEMP[i] = GraphValue.TEMP[i];
		}
		BUSIGraphDeal = 0;

		for(i = GRAPHX-1;i > 0;i--)
		{
			GRAPH_DATA_YT_AddValue(_ahData[BUS_I_NUM], GraphValue.BUS_I[i]);
			GRAPH_DATA_YT_AddValue(_ahData[BUS_V_NUM], GraphValue.BUS_V[i]);
			GRAPH_DATA_YT_AddValue(_ahData[TEMP_NUM], GraphValue.TEMP[i]);
		}
		GRAPH_DATA_YT_AddValue(_ahData[BUS_I_NUM], GraphValue.BUS_I[i]);
		GRAPH_DATA_YT_AddValue(_ahData[BUS_V_NUM], GraphValue.BUS_V[i]);
		GRAPH_DATA_YT_AddValue(_ahData[TEMP_NUM], GraphValue.TEMP[i]);

	}
}

/*********************************************************************
*
*       _UserDraw
*
* Purpose:
*   This routine is called by the GRAPH object before anything is drawn
*   and after the last drawing operation.
*/
static void _UserDraw(WM_HWIN hWin, int Stage) {
  if (Stage == GRAPH_DRAW_LAST) {
    char acText[] = "voltage";
    GUI_RECT Rect, RectInvalid;
    int FontSizeY;
    GUI_SetFont(&GUI_Font13_ASCII);
    FontSizeY = GUI_GetFontSizeY();
    WM_GetInsideRect(&Rect);
    WM_GetInvalidRect(hWin, &RectInvalid);
    Rect.x1 = Rect.x0 + FontSizeY;
    GUI_SetColor(GUI_YELLOW);
    GUI_DispStringInRectEx(acText, &Rect, GUI_TA_HCENTER, strlen(acText), GUI_ROTATE_CCW);
  }
}

/*********************************************************************
*
*       _ForEach
*
* Purpose:
*   This routine hides/shows all windows except the button, graph and scroll bar widgets
*/
static void _ForEach(WM_HWIN hWin, void * pData) {
  int Id, FullScreenMode;
  FullScreenMode = *(int *)pData;
  Id = WM_GetId(hWin);
  if ((Id == GUI_ID_GRAPH0) || (Id == GUI_ID_BUTTON0) || (Id == GUI_ID_VSCROLL) || (Id == GUI_ID_HSCROLL)) {
    return;
  }
  if (FullScreenMode) {
    WM_HideWindow(hWin);
  } else {
    WM_ShowWindow(hWin);
  }
}

/*********************************************************************
*
*       _ToggleFullScreenMode
*
* Purpose:
*   This routine switches between full screen mode and normal mode by hiding or showing the
*   widgets of the dialog, enlarging/shrinking the graph widget and modifying some other
*   attributes of the dialog widgets.
*/
static void _ToggleFullScreenMode(WM_HWIN hDlg) {
  static int FullScreenMode;
  static GUI_RECT Rect;
  static unsigned ScalePos;
  WM_HWIN hGraph, hButton;
  hGraph  = WM_GetDialogItem(hDlg, GUI_ID_GRAPH0);
  hButton = WM_GetDialogItem(hDlg, GUI_ID_BUTTON0);
  FullScreenMode ^= 1;
  if (FullScreenMode) {
    /* Enter the full screen mode */
    WM_HWIN hClient;
    GUI_RECT RectInside;
    hClient = WM_GetClientWindow(hDlg);
    BUTTON_SetText(hButton, "Back");
    WM_MoveWindow(hButton, 0, 11);
    FRAMEWIN_SetTitleVis(hDlg, 0);
    WM_GetInsideRectEx(hClient, &RectInside);
    WM_GetWindowRectEx(hGraph, &Rect);
    WM_ForEachDesc(hClient, _ForEach, &FullScreenMode); /* Hide all descendants */
    WM_SetWindowPos(hGraph, WM_GetWindowOrgX(hClient), WM_GetWindowOrgX(hClient), RectInside.x1, RectInside.y1);
    ScalePos = GRAPH_SCALE_SetPos(_hScaleH, RectInside.y1 - 20);
  } else {
    /* Return to normal mode */
    BUTTON_SetText(hButton, "Full Screen");
    WM_MoveWindow(hButton, 0, -11);
    WM_ForEachDesc(WM_GetClientWindow(hDlg), _ForEach, &FullScreenMode); /* Show all descendants */
    WM_SetWindowPos(hGraph, Rect.x0, Rect.y0, Rect.x1 - Rect.x0 + 1, Rect.y1 - Rect.y0 + 1);
    FRAMEWIN_SetTitleVis(hDlg, 1);
    GRAPH_SCALE_SetPos(_hScaleH, ScalePos);
  }
}

/*********************************************************************
*
*       _SetProgbarValue
*/
static void _SetProgbarValue(int Id, I16 Value) {
  char acBuffer[6] = {"   "};
  WM_HWIN hItem;
  hItem = WM_GetDialogItem(hDlg, Id);
  PROGBAR_SetValue(hItem, Value);
  Value = Value + progMin;
  acBuffer[2] = '0' + Value % 10;
  acBuffer[1] = (Value >=  10) ? '0' + (Value % 100) /  10 : ' ';
  acBuffer[0] = (Value >= 100) ? '0' + Value / 100 : ' ';
  PROGBAR_SetText(hItem, acBuffer);
}

static void SetProgbar(void)
{
	if(A_Current_clicked)
	{
		_SetProgbarValue(A_CURRENT_PROGBAR,Global_User_ADC.PhaseA.qI_value);
		_SetProgbarValue(B_CURRENT_PROGBAR,Global_User_ADC.PhaseB.qI_value);
		_SetProgbarValue(C_CURRENT_PROGBAR,Global_User_ADC.PhaseC.qI_value);
	}
	if(A_Voltage_clicked)
	{
		_SetProgbarValue(A_VOLTAGE_PROGBAR,Global_User_ADC.PhaseA.qV_value);
		_SetProgbarValue(B_VOLTAGE_PROGBAR,Global_User_ADC.PhaseB.qV_value);
		_SetProgbarValue(C_VOLTAGE_PROGBAR,Global_User_ADC.PhaseC.qV_value);
	}
	if(R_Voltage_clicked)
	{
		_SetProgbarValue(R_VOLTAGE_PROGBAR,Global_User_ADC.UR.qV_value);
		_SetProgbarValue(S_VOLTAGE_PROGBAR,Global_User_ADC.US.qV_value);
		_SetProgbarValue(T_VOLTAGE_PROGBAR,Global_User_ADC.UT.qV_value);
	}
	if(BUSI_clicked)
	{
		_SetProgbarValue(BUSI_PROGBAR,Global_User_ADC.BUS.qI_value);
		_SetProgbarValue(BUSV_PROGBAR,Global_User_ADC.BUS.qV_value);
		_SetProgbarValue(TEMP_PROGBAR,Global_User_ADC.Temp.TEMP_value);
	}

}

static void _cbCpuWindow(WM_MESSAGE * pMsg) {
 
  WM_HTIMER hTimer;
  static int16_t i;
  switch (pMsg->MsgId) 
  {
  case WM_CREATE:
    /* Create timer */
	hTimer = WM_CreateTimer(pMsg->hWin, 0, 1, 0);
    break;
    
  case WM_TIMER:
	WM_SelectWindow(hDlg);
	GUI_SetPenSize(5);
	GUI_SetColor(GUI_GREEN);
	GUI_DrawGraph(GraphValue.A_Current,GRAPHX,0,0);
    WM_InvalidateWindow(pMsg->hWin);
	WM_RestartTimer(pMsg->Data.v, 1);
    break; 
    
  case WM_DELETE:
    WM_DeleteTimer(hTimer);
    break;
    
  default:
    WM_DefaultProc(pMsg);
  }
}




/*********************************************************************
*
*       _cbCallback
*
* Purpose:
*   Callback function of the dialog
*/
static void _cbCallback(WM_MESSAGE * pMsg) {
  int i, NCode, Id, Value;
  WM_HWIN hDlg, hItem;
  hDlg = pMsg->hWin;
  switch (pMsg->MsgId) {
  case WM_INIT_DIALOG:
    hItem = WM_GetDialogItem(hDlg, GUI_ID_GRAPH0);
	#if 0
	WM_CreateWindowAsChild(0, 0, 800, 360, pMsg->hWin, WM_CF_SHOW | WM_CF_HASTRANS, _cbCpuWindow , 0); 
	#endif
    /* Add graphs */
    for (i = 0; i < GUI_COUNTOF(_aColor); i++) {
      _ahData[i] = GRAPH_DATA_YT_Create(_aColor[i], 675, 0, 0);
      GRAPH_AttachData(hItem, _ahData[i]);
    }
    /* Set graph attributes */
    GRAPH_SetGridDistY(hItem, 25);
    GRAPH_SetGridVis(hItem, 1);
    GRAPH_SetGridFixedX(hItem, 1);
    GRAPH_SetUserDraw(hItem, _UserDraw);
    /* Create and add vertical scale */
    _hScaleV = GRAPH_SCALE_Create( 35, GUI_TA_RIGHT, GRAPH_SCALE_CF_VERTICAL, 25);
    GRAPH_SCALE_SetTextColor(_hScaleV, GUI_YELLOW);
    GRAPH_AttachScale(hItem, _hScaleV);
    /* Create and add horizontal scale */
    _hScaleH = GRAPH_SCALE_Create(325, GUI_TA_HCENTER, GRAPH_SCALE_CF_HORIZONTAL, 50);
    GRAPH_SCALE_SetTextColor(_hScaleH, GUI_DARKGREEN);
    GRAPH_AttachScale(hItem, _hScaleH);
	/* Init progress bars */
	hItem = WM_GetDialogItem(hDlg, A_CURRENT_PROGBAR);
    WIDGET_SetEffect(hItem, &WIDGET_Effect_3D);
	PROGBAR_SetMinMax(hItem,0,GRAPHY);
	PROGBAR_SetBarColor(hItem,0,_aColor[A_CURRENT_NUM]);
	PROGBAR_SetBarColor(hItem,1,_aColor[A_CURRENT_NUM]);
    _SetProgbarValue(A_CURRENT_PROGBAR, 100);
    hItem = WM_GetDialogItem(hDlg, B_CURRENT_PROGBAR);
    WIDGET_SetEffect(hItem, &WIDGET_Effect_3D);
	PROGBAR_SetMinMax(hItem,0,GRAPHY);
	PROGBAR_SetBarColor(hItem,0,_aColor[B_CURRENT_NUM]);
	PROGBAR_SetBarColor(hItem,1,_aColor[B_CURRENT_NUM]);
    _SetProgbarValue(B_CURRENT_PROGBAR,100);
	    hItem = WM_GetDialogItem(hDlg, C_CURRENT_PROGBAR);
    WIDGET_SetEffect(hItem, &WIDGET_Effect_3D);
	PROGBAR_SetMinMax(hItem,0,GRAPHY);
	PROGBAR_SetBarColor(hItem,0,_aColor[C_CURRENT_NUM]);
	PROGBAR_SetBarColor(hItem,1,_aColor[C_CURRENT_NUM]);
    _SetProgbarValue(C_CURRENT_PROGBAR,100);
	    hItem = WM_GetDialogItem(hDlg, A_VOLTAGE_PROGBAR);
    WIDGET_SetEffect(hItem, &WIDGET_Effect_3D);
	PROGBAR_SetMinMax(hItem,0,GRAPHY);
	PROGBAR_SetBarColor(hItem,0,_aColor[A_VOLTAGE_NUM]);
	PROGBAR_SetBarColor(hItem,1,_aColor[A_VOLTAGE_NUM]);
    _SetProgbarValue(A_VOLTAGE_PROGBAR,100);
	    hItem = WM_GetDialogItem(hDlg, B_VOLTAGE_PROGBAR);
    WIDGET_SetEffect(hItem, &WIDGET_Effect_3D);
	PROGBAR_SetMinMax(hItem,0,GRAPHY);
	PROGBAR_SetBarColor(hItem,0,_aColor[B_VOLTAGE_NUM]);
	PROGBAR_SetBarColor(hItem,1,_aColor[B_VOLTAGE_NUM]);
    _SetProgbarValue(B_VOLTAGE_PROGBAR,100);
	    hItem = WM_GetDialogItem(hDlg, C_VOLTAGE_PROGBAR);
    WIDGET_SetEffect(hItem, &WIDGET_Effect_3D);
	PROGBAR_SetMinMax(hItem,0,GRAPHY);
	PROGBAR_SetBarColor(hItem,0,_aColor[C_VOLTAGE_NUM]);
	PROGBAR_SetBarColor(hItem,1,_aColor[C_VOLTAGE_NUM]);
    _SetProgbarValue(C_VOLTAGE_PROGBAR,100);
	    hItem = WM_GetDialogItem(hDlg, R_VOLTAGE_PROGBAR);
    WIDGET_SetEffect(hItem, &WIDGET_Effect_3D);
	PROGBAR_SetMinMax(hItem,0,GRAPHY);
	PROGBAR_SetBarColor(hItem,0,_aColor[R_VOLTAGE_NUM]);
	PROGBAR_SetBarColor(hItem,1,_aColor[R_VOLTAGE_NUM]);
    _SetProgbarValue(R_VOLTAGE_PROGBAR,100);
	    hItem = WM_GetDialogItem(hDlg, S_VOLTAGE_PROGBAR);
    WIDGET_SetEffect(hItem, &WIDGET_Effect_3D);
	PROGBAR_SetMinMax(hItem,0,GRAPHY);
	PROGBAR_SetBarColor(hItem,0,_aColor[S_VOLTAGE_NUM]);
	PROGBAR_SetBarColor(hItem,1,_aColor[S_VOLTAGE_NUM]);
    _SetProgbarValue(S_VOLTAGE_PROGBAR,100);
	    hItem = WM_GetDialogItem(hDlg, T_VOLTAGE_PROGBAR);
    WIDGET_SetEffect(hItem, &WIDGET_Effect_3D);
	PROGBAR_SetMinMax(hItem,0,GRAPHY);
	PROGBAR_SetBarColor(hItem,0,_aColor[T_VOLTAGE_NUM]);
	PROGBAR_SetBarColor(hItem,1,_aColor[T_VOLTAGE_NUM]);
    _SetProgbarValue(T_VOLTAGE_PROGBAR,100);
	    hItem = WM_GetDialogItem(hDlg, BUSI_PROGBAR);
    WIDGET_SetEffect(hItem, &WIDGET_Effect_3D);
	PROGBAR_SetMinMax(hItem,0,GRAPHY);
	PROGBAR_SetBarColor(hItem,0,_aColor[BUS_I_NUM]);
	PROGBAR_SetBarColor(hItem,1,_aColor[BUS_I_NUM]);
    _SetProgbarValue(BUSI_PROGBAR,100);
	    hItem = WM_GetDialogItem(hDlg, BUSV_PROGBAR);
    WIDGET_SetEffect(hItem, &WIDGET_Effect_3D);
	PROGBAR_SetMinMax(hItem,0,GRAPHY);
	PROGBAR_SetBarColor(hItem,0,_aColor[BUS_V_NUM]);
	PROGBAR_SetBarColor(hItem,1,_aColor[BUS_V_NUM]);
    _SetProgbarValue(BUSV_PROGBAR,100);
	    hItem = WM_GetDialogItem(hDlg, TEMP_PROGBAR);
    WIDGET_SetEffect(hItem, &WIDGET_Effect_3D);
	PROGBAR_SetMinMax(hItem,0,GRAPHY);
	PROGBAR_SetBarColor(hItem,0,_aColor[TEMP_NUM]);
	PROGBAR_SetBarColor(hItem,1,_aColor[TEMP_NUM]);
    _SetProgbarValue(TEMP_PROGBAR,100);
	
    /* Init check boxes */
#if 0
    hItem = WM_GetDialogItem(hDlg, GUI_ID_CHECK2);
    CHECKBOX_SetText(hItem, "L");
    hItem = WM_GetDialogItem(hDlg, GUI_ID_CHECK3);
    CHECKBOX_SetText(hItem, "T");
    hItem = WM_GetDialogItem(hDlg, GUI_ID_CHECK4);
    CHECKBOX_SetText(hItem, "R");
    hItem = WM_GetDialogItem(hDlg, GUI_ID_CHECK5);
    CHECKBOX_SetText(hItem, "B");
#endif
    hItem = WM_GetDialogItem(hDlg, GUI_ID_CHECK0);
    CHECKBOX_SetText(hItem, "Stop");
    hItem = WM_GetDialogItem(hDlg, GUI_ID_CHECK1);
    CHECKBOX_SetText(hItem, "Grid");
    CHECKBOX_SetState(hItem, 1);
    hItem = WM_GetDialogItem(hDlg, A_CURRENT_CHECK);
    CHECKBOX_SetText(hItem, "A Current");
	CHECKBOX_SetState(hItem,1);
	hItem = WM_GetDialogItem(hDlg, A_VOLTAGE_CHECK);
    CHECKBOX_SetText(hItem, "A Voltage");
	hItem = WM_GetDialogItem(hDlg, R_VOLTAGE_CHECK);
    CHECKBOX_SetText(hItem, "R Voltage");
	hItem = WM_GetDialogItem(hDlg, BUSI_CHECK);
    CHECKBOX_SetText(hItem, "BUS I");
	hItem = WM_GetDialogItem(hDlg, GUI_ID_CHECK6);
    CHECKBOX_SetText(hItem, "HScroll");
    CHECKBOX_SetState(hItem, 1);
    hItem = WM_GetDialogItem(hDlg, GUI_ID_CHECK7);
    CHECKBOX_SetText(hItem, "VScroll");
#if 0
    hItem = WM_GetDialogItem(hDlg, GUI_ID_CHECK8);
    CHECKBOX_SetText(hItem, "MirrorX");
#endif
    /* Init slider widgets */
    hItem = WM_GetDialogItem(hDlg, GUI_ID_SLIDER0);
    SLIDER_SetRange(hItem, 0, 10);
    SLIDER_SetValue(hItem, 5);
    SLIDER_SetNumTicks(hItem, 10);
    hItem = WM_GetDialogItem(hDlg, GUI_ID_SLIDER1);
    SLIDER_SetRange(hItem, 0, 20);
    SLIDER_SetValue(hItem, 5);
    SLIDER_SetNumTicks(hItem, 10);
    /* Init radio widget */
#if 0
    hItem = WM_GetDialogItem(hDlg, GUI_ID_RADIO0);
    RADIO_SetText(hItem, "3D", 0);
    RADIO_SetText(hItem, "flat", 1);
    RADIO_SetText(hItem, "-", 2);
#endif
    /* Init button widget */
    hItem = WM_GetDialogItem(hDlg, GUI_ID_BUTTON0);
    WM_SetStayOnTop(hItem, 1);
    break;
  case WM_NOTIFY_PARENT:
    Id    = WM_GetId(pMsg->hWinSrc);      /* Id of widget */
    NCode = pMsg->Data.v;                 /* Notification code */
    switch (NCode) {
    case WM_NOTIFICATION_CLICKED:
      switch (Id) {
      case GUI_ID_BUTTON0:
        _ToggleFullScreenMode(hDlg);
        break;
      }
      break;
    case WM_NOTIFICATION_VALUE_CHANGED:
      switch (Id) {
      case GUI_ID_CHECK0:
        /* Toggle stop mode */
        _Stop ^= 1;
        break;
      case GUI_ID_CHECK1:
        /* Toggle grid */
        hItem = WM_GetDialogItem(hDlg, GUI_ID_GRAPH0);
        GRAPH_SetGridVis(hItem, CHECKBOX_IsChecked(WM_GetDialogItem(hDlg, GUI_ID_CHECK1)));
        break;
#if 0
      case GUI_ID_CHECK2:
      case GUI_ID_CHECK3:
      case GUI_ID_CHECK4:
      case GUI_ID_CHECK5:
        /* Toggle border */
        hItem = WM_GetDialogItem(hDlg, GUI_ID_GRAPH0);
        GRAPH_SetBorder(hItem, 
                        CHECKBOX_IsChecked(WM_GetDialogItem(hDlg, GUI_ID_CHECK2)) * 40, 
                        CHECKBOX_IsChecked(WM_GetDialogItem(hDlg, GUI_ID_CHECK3)) * 5, 
                        CHECKBOX_IsChecked(WM_GetDialogItem(hDlg, GUI_ID_CHECK4)) * 5, 
                        CHECKBOX_IsChecked(WM_GetDialogItem(hDlg, GUI_ID_CHECK5)) * 5);
        break;
#endif
      case GUI_ID_SLIDER0:
        /* Set horizontal grid spacing */
        hItem = WM_GetDialogItem(hDlg, GUI_ID_GRAPH0);
        Value = SLIDER_GetValue(pMsg->hWinSrc) * 10;
        GRAPH_SetGridDistX(hItem, Value);
        GRAPH_SCALE_SetTickDist(_hScaleH, Value);
        break;
      case GUI_ID_SLIDER1:
        /* Set vertical grid spacing */
        hItem = WM_GetDialogItem(hDlg, GUI_ID_GRAPH0);
        Value = SLIDER_GetValue(pMsg->hWinSrc) * 5;
        GRAPH_SetGridDistY(hItem, Value);
        GRAPH_SCALE_SetTickDist(_hScaleV, Value);
        break;
#if 0
      case GUI_ID_RADIO0:
        /* Set the widget effect */
        hItem = WM_GetDialogItem(hDlg, GUI_ID_GRAPH0);
        switch (RADIO_GetValue(pMsg->hWinSrc)) {
        case 0:
          WIDGET_SetEffect(hItem, &WIDGET_Effect_3D);
          break;
        case 1:
          WIDGET_SetEffect(hItem, &WIDGET_Effect_Simple);
          break;
        case 2:
          WIDGET_SetEffect(hItem, &WIDGET_Effect_None);
          break;
        }
        break;
#endif
      case GUI_ID_CHECK6:
        /* Toggle horizontal scroll bar */
        hItem = WM_GetDialogItem(hDlg, GUI_ID_GRAPH0);
        if (CHECKBOX_IsChecked(WM_GetDialogItem(hDlg, GUI_ID_CHECK6))) {
          GRAPH_SetVSizeX(hItem, 500);
        } else {
          GRAPH_SetVSizeX(hItem, 0);
        }
        break;
      case GUI_ID_CHECK7:
        /* Toggle vertical scroll bar */
        hItem = WM_GetDialogItem(hDlg, GUI_ID_GRAPH0);
        if (CHECKBOX_IsChecked(WM_GetDialogItem(hDlg, GUI_ID_CHECK7))) {
          GRAPH_SetVSizeY(hItem, 300);
        } else {
          GRAPH_SetVSizeY(hItem, 0);
        }
        break;
		case A_CURRENT_CHECK:
		if (CHECKBOX_IsChecked(WM_GetDialogItem(hDlg,A_CURRENT_CHECK))){
			A_Current_clicked = 1;
			} else {
			A_Current_clicked = 0;
			GRAPH_DATA_YT_Clear(_ahData[A_CURRENT_NUM]);
			GRAPH_DATA_YT_Clear(_ahData[B_CURRENT_NUM]);
			GRAPH_DATA_YT_Clear(_ahData[C_CURRENT_NUM]);
			_SetProgbarValue(A_CURRENT_PROGBAR, 0);
			_SetProgbarValue(B_CURRENT_PROGBAR,0);
			_SetProgbarValue(C_CURRENT_PROGBAR,0);
				}
			break;
		case A_VOLTAGE_CHECK:
		if (CHECKBOX_IsChecked(WM_GetDialogItem(hDlg,A_VOLTAGE_CHECK))){
			A_Voltage_clicked = 1;
			} else {
			A_Voltage_clicked = 0;
			GRAPH_DATA_YT_Clear(_ahData[A_VOLTAGE_NUM]);
			GRAPH_DATA_YT_Clear(_ahData[B_VOLTAGE_NUM]);
			GRAPH_DATA_YT_Clear(_ahData[C_VOLTAGE_NUM]);
			_SetProgbarValue(A_VOLTAGE_PROGBAR, 0);
			_SetProgbarValue(B_VOLTAGE_PROGBAR,0);
			_SetProgbarValue(C_VOLTAGE_PROGBAR,0);
				}
			break;	
		case R_VOLTAGE_CHECK:
		if (CHECKBOX_IsChecked(WM_GetDialogItem(hDlg,R_VOLTAGE_CHECK))){
		R_Voltage_clicked = 1;
		} else {
		R_Voltage_clicked = 0;
			GRAPH_DATA_YT_Clear(_ahData[R_VOLTAGE_NUM]);
			GRAPH_DATA_YT_Clear(_ahData[S_VOLTAGE_NUM]);
			GRAPH_DATA_YT_Clear(_ahData[T_VOLTAGE_NUM]);
			_SetProgbarValue(R_VOLTAGE_PROGBAR, 0);
			_SetProgbarValue(S_VOLTAGE_PROGBAR,0);
			_SetProgbarValue(T_VOLTAGE_PROGBAR,0);
			}
		break;
		case BUSI_CHECK:
		if (CHECKBOX_IsChecked(WM_GetDialogItem(hDlg,BUSI_CHECK))){
		BUSI_clicked = 1;
		} else {
		BUSI_clicked = 0;
			GRAPH_DATA_YT_Clear(_ahData[BUS_I_NUM]);
			GRAPH_DATA_YT_Clear(_ahData[BUS_V_NUM]);
			GRAPH_DATA_YT_Clear(_ahData[TEMP_NUM]);
			_SetProgbarValue(BUSI_PROGBAR, 0);
			_SetProgbarValue(BUSV_PROGBAR,0);
			_SetProgbarValue(TEMP_PROGBAR,0);
			}
		break;
			
#if 0
      case GUI_ID_CHECK8:
        /* Toggle alignment */
        hItem = WM_GetDialogItem(hDlg, GUI_ID_GRAPH0);
        for (i = 0; i < GUI_COUNTOF(_aColor); i++) {
          if (CHECKBOX_IsChecked(WM_GetDialogItem(hDlg, GUI_ID_CHECK8))) {
            GRAPH_DATA_YT_SetAlign(_ahData[i], GRAPH_ALIGN_LEFT);
            GRAPH_DATA_YT_MirrorX (_ahData[i], 1);
          } else {
            GRAPH_DATA_YT_SetAlign(_ahData[i], GRAPH_ALIGN_RIGHT);
            GRAPH_DATA_YT_MirrorX (_ahData[i], 0);
          }
        }
        break;
#endif
      }
      break;
    }
    break;
  default:
    WM_DefaultProc(pMsg);
  }
}

/*********************************************************************
*
*       Public code
*
**********************************************************************
*/
/*********************************************************************
*
*       MainTask
*/
#include "fcntl.h"
#ifdef RT_USING_DFS_ELMFAT
#include "ff.h"
#endif
void DlgTask(void);
void DlgTask(void) {
 int fd;
 
 #ifdef RT_USING_DFS_ELMFAT
 FIL MyFile;
 FRESULT res;								/* FatFs function common result code */
#endif

 uint32_t byteswritten;					   /* File write counts */
 uint8_t wtext[] = "1234567890";			/* File write buffer */
	
  GUI_CURSOR_Show();
  WM_SetDesktopColor(GUI_BLACK);
  #if GUI_SUPPORT_MEMDEV
    WM_SetCreateFlags(WM_CF_MEMDEV);
  #endif
  hDlg = GUI_CreateDialogBox(_aDialogCreate, GUI_COUNTOF(_aDialogCreate), &_cbCallback, 0, 0, 0);
  while (1) {
    #ifdef WIN32
      GUI_Delay(10);
    #endif
	#if 0
	if ((fd = open("adc",O_CREAT | O_RDWR, 0)) >0 )
		{
		rt_kprintf("open file secess\r\n");
		close(fd);
  	}
	#endif
	#if 0
    if(f_open(&MyFile, "MY.TXT", FA_CREATE_NEW | FA_WRITE) != FR_OK)
    {
      /* 'STM32.TXT' file Open for write Error */
	  
	  if(f_open(&MyFile, "MY.TXT", FA_WRITE) != FR_OK)
	  {
		/* 'STM32.TXT' file Open for read Error */
		rt_kprintf("open file fail!\r\n");
	  }
	  else
	  {
		res = f_lseek(&MyFile,MyFile.fsize);
		/*##-8- Read data from the text file ###########################*/
		res = f_write(&MyFile, wtext, sizeof(wtext), (UINT*)&byteswritten);
		
		if((byteswritten == 0) || (res != FR_OK))
		{
		  /* 'STM32.TXT' file Write or EOF Error */
		  rt_kprintf("write file fail!\r\n");
		  f_close(&MyFile);
		}
		else
		{
		  /*##-6- Close the open text file #################################*/
		  rt_kprintf("write file success!\r\n");
		  f_close(&MyFile);
	  	}
      }
    }
	else
	{
		rt_kprintf("file created success!\r\n");
		f_close(&MyFile);
	}
	#endif
    if (!_Stop) {
      if (!hGraph) {
        hGraph = WM_GetDialogItem(hDlg, GUI_ID_GRAPH0);
      }
	  //WM_SelectWindow(hGraph);
	  //GUI_SetPenSize(5); 
		_AddValues(hGraph);
	  //GUI_SetColor(GUI_GREEN);
	  //GUI_DrawGraph(GraphValue.A_Current,GRAPHX,0,0);
	  SetProgbar();
    }
    GUI_Exec();
  }
}
#endif
/*************************** End of file ****************************/

