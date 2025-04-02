/*
 * exppack_ctrl.c
 *
 *  Created on: Dec 31, 2024
 *      Author: INVINCIBLENESS
 */

#include "exppack_ctrl.h"

/**
 *-----------------------------------------------------------
 *              TYPE DEFINITIONS AND ENUMERATIONS
 *-----------------------------------------------------------
 * @brief Enumerated types and structures central to this module.
 */



/**
 *------------------------------------------------------------
 *                           VARIABLES
 *------------------------------------------------------------
 * @brief Variables accessible throughout the application.
 */

TaskObj_t exppackCtrlTask;


Exppack_Data_t ExpPackDataObj;
ScaledData_t ScaledDataObj;


// Loop Time Count //
static uint32_t exppackCtrlLoopCnt;
static float exppackCtrlTimeElap;


/* -------------- ADC buffer for DMA ------------- */
static uint16_t* adc1buff = {0};
static uint16_t* adc2buff = {0};
static uint16_t* adc3buff = {0};
/* ----------------------------------------------- */


/* ------------ Individual PMMG (1~8) ------------ */
IOIF_pMMG_Obj_t pMMGObj_1A;
IOIF_pMMG_Obj_t pMMGObj_1B;
IOIF_pMMG_Obj_t pMMGObj_2A;
IOIF_pMMG_Obj_t pMMGObj_2B;
IOIF_pMMG_Obj_t pMMGObj_3A;
IOIF_pMMG_Obj_t pMMGObj_3B;
IOIF_pMMG_Obj_t pMMGObj_4A;
IOIF_pMMG_Obj_t pMMGObj_4B;

uint8_t state_1A = 0;
uint8_t state_1B = 0;
uint8_t state_2A = 0;
uint8_t state_2B = 0;
uint8_t state_3A = 0;
uint8_t state_3B = 0;
uint8_t state_4A = 0;
uint8_t state_4B = 0;
/* ----------------------------------------------- */


/* ------------- USB CDC FS Transmit ------------- */
uint8_t usbTxBuf[100];
char usbTxBufChar[100] = "";
uint8_t usbTxBufSize = 0;
char splitString[2] = ",";
char newLine[2] = "\n";
char strBuf_8bit[4];
char strBuf_12bit[5];
char strBuf_16bit[6];
char strBuf_32bit[11];
uint32_t timeUSBCDC = 0;
uint8_t usbTxUpdate = 0;
/* ----------------------------------------------- */

/* ---------------- For GPIO EXTI ---------------- */
uint8_t GPIO_EXTI_FLAG = 0;
uint32_t DEBOUNCE_CNT = 0;
uint8_t USB_CDC_STOP_FLAG = 0;
/* ----------------------------------------------- */


/* ---------------- For Code Time Check ---------------- */
static uint32_t EXTDEVcodeStartTick = 0;
static uint32_t EXTDEVcodeEndTick = 0;
/* ----------------------------------------------------- */


/* ----------------- For KW University ----------------- */
StudentsData_t StudentsDataObj;
ReceivedDataFromCM_t ReceivedDataObj;
P_Vector_Decoder pvectorObj_RH;
P_Vector_Decoder pvectorObj_LH;
PIDObject posCtrl_RH;
PIDObject posCtrl_LH;
MotionMapFileInfo MotionMap_File;
/* ----------------------------------------------------- */

/**
 *------------------------------------------------------------
 *                 STATIC FUNCTION PROTOTYPES
 *------------------------------------------------------------
 * @brief Static Function prototypes for this module.
 */

/* ------------------- STATE FUNCTION ------------------- */
static void StateOff_Run(void);

static void StateStandby_Run(void);

static void StateEnable_Ent(void);
static void StateEnable_Run(void);
static void StateEnable_Ext(void);

static void StateError_Run(void);

/* ------------------- INITIALIZATION ------------------- */
static void ScalingForPDO(ScaledData_t* Scaled2ByteData, Exppack_Data_t* exppackDataObj);
/* ------------------- ROUTINE ------------------- */
static int Ent_P_Vector_Decoder_RH();
static int Run_P_Vector_Decoder_RH();
static int Ext_P_Vector_Decoder_RH();
static int Ent_P_Vector_Decoder_LH();
static int Run_P_Vector_Decoder_LH();
static int Ext_P_Vector_Decoder_LH();

/* ------------------- STATIC FUNCTIONS ------------------- */
static int InitPMMG(void);
static int GetRawPMMG_A(Exppack_Data_t* exppackDataObj);
static int GetRawPMMG_B(Exppack_Data_t* exppackDataObj);
static int GetRawFSR(Exppack_Data_t* exppackDataObj);
static int16_t signingEMG(uint16_t input);
static int GetRawEMG(Exppack_Data_t* exppackDataObj);

static float RectificationEMG(float EMGval);
static void RectifyEMG_ALL(Exppack_Data_t* exppackDataObj);
static float NormalizeEMG(float EMG_rect_val);
static void NormalizeEMG_ALL(Exppack_Data_t* exppackDataObj);
static float LowPassFilter(uint8_t* isInitialized, float* prevOutput, float currInput);
static float BandPassFilter(uint8_t* isInitialized, float* prevOutput, float* prev_prevOutput, int16_t currInput, int16_t prevInput, int16_t prev_prevInput);
static void LowPassFilteringEMG_ALL(Exppack_Data_t* exppackDataObj);
static void BandPassFilteringEMG_ALL(Exppack_Data_t* exppackDataObj);
static float MovingAverage(uint16_t* MA_Buffer, uint8_t* ind, uint8_t* cnt, float* sum, uint16_t newVal);
static void MovingAverageEMG_ALL(Exppack_Data_t* exppackDataObj);
static void ProcessEMG(Exppack_Data_t* exppackDataObj);

#ifdef BUTTON_MODE
static void PrepareUSBCDCTxData(Exppack_Data_t* exppackDataObj);
static void GPIO_EXTI_8_CALLBACK(uint16_t gpioPins);

static void PrepareUSBCDCTxData_DEMO4(Exppack_Data_t* exppackDataObj);
static void TerminateUSBCDC_DEMO4(void);
#endif
/* ------------------- SDO CALLBACK ------------------- */
static void GetThetaRH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void GetThetaLH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void GetAccXRH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void GetAccXLH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void GetAccYRH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void GetAccYLH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_P_Vector_MotionMap(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_P_Vector_Yd_RH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_P_Vector_L_RH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_P_Vector_S0_RH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_P_Vector_Sd_RH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_P_Vector_Yd_LH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_P_Vector_L_LH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_P_Vector_S0_LH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);
static void Set_P_Vector_Sd_LH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res);


/**
 *------------------------------------------------------------
 *                      PUBLIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions that interface with this module.
 */

DOP_COMMON_SDO_CB(exppackCtrlTask)

void InitExppackCtrl(void)
{
    InitTask(&exppackCtrlTask);

	/* Start DMA ADC1 for FSR Sensor */
	if(IOIF_StartADCDMA(IOIF_ADC1, &adc1buff, IOIF_ADC1_BUFFER_LENGTH)) {
		//TODO: Error Process
	}
	if(IOIF_StartADCDMA(IOIF_ADC2, &adc2buff, IOIF_ADC2_BUFFER_LENGTH)) {
		//TODO: Error Process
	}
	if(IOIF_StartADCDMA(IOIF_ADC3, &adc3buff, IOIF_ADC3_BUFFER_LENGTH)) {
		//TODO: Error Process
	}

	/* Initialize pMMG Sensor */
	InitPMMG();

#ifdef BUTTON_MODE
	/* GPIO EXTI Setting */
	IOIF_GPIOCBPtr_t Button_EXTI_Callback = GPIO_EXTI_8_CALLBACK;
	IOIF_SetGPIOCB(GPIO_PIN_8, IOIF_GPIO_EXTI_CALLBACK, Button_EXTI_Callback);
#endif

	/* State Definition */
	TASK_CREATE_STATE(&exppackCtrlTask, TASK_STATE_OFF,      NULL,				StateOff_Run,       NULL,         		 true);
	TASK_CREATE_STATE(&exppackCtrlTask, TASK_STATE_STANDBY,  NULL,				StateStandby_Run,	NULL,         		 false);
	TASK_CREATE_STATE(&exppackCtrlTask, TASK_STATE_ENABLE,   StateEnable_Ent,	StateEnable_Run, 	StateEnable_Ext,	 false);
	TASK_CREATE_STATE(&exppackCtrlTask, TASK_STATE_ERROR,    NULL,				StateError_Run,    	NULL,				 false);

	/* Routine Definition */
	TASK_CREATE_ROUTINE(&exppackCtrlTask,  ROUTINE_ID_EXTDEV_P_VECTOR_DECODER_RH, Ent_P_Vector_Decoder_RH, Run_P_Vector_Decoder_RH, Ext_P_Vector_Decoder_RH);
	TASK_CREATE_ROUTINE(&exppackCtrlTask,  ROUTINE_ID_EXTDEV_P_VECTOR_DECODER_LH, Ent_P_Vector_Decoder_LH, Run_P_Vector_Decoder_LH, Ext_P_Vector_Decoder_LH);

	/* DOD Definition */
	// DOD
	DOP_CreateDOD(TASK_ID_EXTDEV);

	// PDO
	/* For PDO setting */
	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_PMMG_1A,				DOP_UINT16,	    	1,    &ScaledDataObj.pMMG_1A_scaled);
	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_PMMG_1B,				DOP_UINT16,	    	1,    &ScaledDataObj.pMMG_1B_scaled);
	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_PMMG_2A,				DOP_UINT16,	    	1,    &ScaledDataObj.pMMG_2A_scaled);
	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_PMMG_2B,				DOP_UINT16,	    	1,    &ScaledDataObj.pMMG_2B_scaled);
	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_PMMG_3A,				DOP_UINT16,	    	1,    &ScaledDataObj.pMMG_3A_scaled);
	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_PMMG_3B,				DOP_UINT16,	    	1,    &ScaledDataObj.pMMG_3B_scaled);
	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_PMMG_4A,				DOP_UINT16,	    	1,    &ScaledDataObj.pMMG_4A_scaled);
	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_PMMG_4B,				DOP_UINT16,	    	1,    &ScaledDataObj.pMMG_4B_scaled);

	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_EMG_L1,				DOP_UINT16,	    	1,    &ScaledDataObj.EMG_L1_scaled);
	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_EMG_L2,				DOP_UINT16,	    	1,    &ScaledDataObj.EMG_L2_scaled);
	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_EMG_L3,				DOP_UINT16,	    	1,    &ScaledDataObj.EMG_L3_scaled);
	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_EMG_L4,				DOP_UINT16,	    	1,    &ScaledDataObj.EMG_L4_scaled);
	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_EMG_R1,				DOP_UINT16,	    	1,    &ScaledDataObj.EMG_R1_scaled);
	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_EMG_R2,				DOP_UINT16,	    	1,    &ScaledDataObj.EMG_R2_scaled);
	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_EMG_R3,				DOP_UINT16,	    	1,    &ScaledDataObj.EMG_R3_scaled);
	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_EMG_R4,				DOP_UINT16,	    	1,    &ScaledDataObj.EMG_R4_scaled);

	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_FSR_L1,				DOP_UINT16,	    	1,    &ScaledDataObj.FSR_L1_scaled);
	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_FSR_L2,				DOP_UINT16,	    	1,    &ScaledDataObj.FSR_L2_scaled);
	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_FSR_L3,				DOP_UINT16,	    	1,    &ScaledDataObj.FSR_L3_scaled);
	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_FSR_L4,				DOP_UINT16,	    	1,    &ScaledDataObj.FSR_L4_scaled);
	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_FSR_R1,				DOP_UINT16,	    	1,    &ScaledDataObj.FSR_R1_scaled);
	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_FSR_R2,				DOP_UINT16,	    	1,    &ScaledDataObj.FSR_R2_scaled);
	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_FSR_R3,				DOP_UINT16,	    	1,    &ScaledDataObj.FSR_R3_scaled);
	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_FSR_R4,				DOP_UINT16,	    	1,    &ScaledDataObj.FSR_R4_scaled);

	/* For HAR_Demo_4 */
	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_EMG_HARDEMO_1,		DOP_INT16,	    	1,    &ScaledDataObj.EMG_RAWSIGN);
	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_EMG_HARDEMO_2,		DOP_UINT16,	    	1,    &ScaledDataObj.EMG_ENVELOPE);
//	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_EMG_HARDEMO_3,		DOP_UINT16,	    	1,    &ScaledDataObj.EMG_MA);

	/* For KW_Univ */
	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_CONTROL_INPUT_RH,	DOP_INT16,	    	1,    &ScaledDataObj.control_input_RH);
	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_CONTROL_INPUT_LH,	DOP_INT16,	    	1,    &ScaledDataObj.control_input_LH);
	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_POSITION_REF_RH,		DOP_INT16,	    	1,    &ScaledDataObj.theta_ref_RH);
	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_POSITION_REF_LH,		DOP_INT16,	    	1,    &ScaledDataObj.theta_ref_LH);
	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_EMG_RAW,				DOP_INT16,	    	1,    &ScaledDataObj.EMG_RAW);
	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_EMG_PROCESSED,		DOP_UINT16,	    	1,    &ScaledDataObj.EMG_PROCESSED);

	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_FVECTOR_TRIGGER_1,	DOP_UINT8,	    	1,    &ScaledDataObj.Fvector_trigger_1);
	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_FVECTOR_TRIGGER_2,	DOP_UINT8,	    	1,    &ScaledDataObj.Fvector_trigger_2);
	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_FVECTOR_TRIGGER_3,	DOP_UINT8,	    	1,    &ScaledDataObj.Fvector_trigger_3);
	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_FVECTOR_TRIGGER_4,	DOP_UINT8,	    	1,    &ScaledDataObj.Fvector_trigger_4);
	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_FVECTOR_TRIGGER_5,	DOP_UINT8,	    	1,    &ScaledDataObj.Fvector_trigger_5);

	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_PVECTOR_RH_TEST,		DOP_FLOAT32,	    1,    &ScaledDataObj.Pvector_test_RH);
	DOP_CreatePDO(TASK_ID_EXTDEV, 	 PDO_ID_EXTDEV_PVECTOR_LH_TEST,		DOP_FLOAT32,	    1,    &ScaledDataObj.Pvector_test_LH);



	// SDO
	DOP_COMMON_SDO_CREATE(TASK_ID_EXTDEV)
	DOP_CreateSDO(TASK_ID_EXTDEV,	SDO_ID_EXTDEV_GET_THETA_RH,	DOP_INT16,	GetThetaRH);
	DOP_CreateSDO(TASK_ID_EXTDEV,	SDO_ID_EXTDEV_GET_THETA_LH,	DOP_INT16,	GetThetaLH);
	DOP_CreateSDO(TASK_ID_EXTDEV,	SDO_ID_EXTDEV_GET_ACCX_RH,	DOP_INT16,	GetAccXRH);
	DOP_CreateSDO(TASK_ID_EXTDEV,	SDO_ID_EXTDEV_GET_ACCX_LH,	DOP_INT16,	GetAccXLH);
	DOP_CreateSDO(TASK_ID_EXTDEV,	SDO_ID_EXTDEV_GET_ACCY_RH,	DOP_INT16,	GetAccYRH);
	DOP_CreateSDO(TASK_ID_EXTDEV,	SDO_ID_EXTDEV_GET_ACCY_LH,	DOP_INT16,	GetAccYLH);

	DOP_CreateSDO(TASK_ID_EXTDEV,	SDO_ID_EXTDEV_MOTIONMAP_ID,    DOP_UINT8,	Set_P_Vector_MotionMap);
	DOP_CreateSDO(TASK_ID_EXTDEV,	SDO_ID_EXTDEV_P_VECTOR_YD_RH,  DOP_INT16,	Set_P_Vector_Yd_RH);
	DOP_CreateSDO(TASK_ID_EXTDEV,	SDO_ID_EXTDEV_P_VECTOR_L_RH,   DOP_UINT16,	Set_P_Vector_L_RH);
	DOP_CreateSDO(TASK_ID_EXTDEV,	SDO_ID_EXTDEV_P_VECTOR_S0_RH,  DOP_UINT8,	Set_P_Vector_S0_RH);
	DOP_CreateSDO(TASK_ID_EXTDEV,	SDO_ID_EXTDEV_P_VECTOR_SD_RH,  DOP_UINT8,	Set_P_Vector_Sd_RH);
	DOP_CreateSDO(TASK_ID_EXTDEV,	SDO_ID_EXTDEV_P_VECTOR_YD_LH,  DOP_INT16,	Set_P_Vector_Yd_LH);
	DOP_CreateSDO(TASK_ID_EXTDEV,	SDO_ID_EXTDEV_P_VECTOR_L_LH,   DOP_UINT16,	Set_P_Vector_L_LH);
	DOP_CreateSDO(TASK_ID_EXTDEV,	SDO_ID_EXTDEV_P_VECTOR_S0_LH,  DOP_UINT8,	Set_P_Vector_S0_LH);
	DOP_CreateSDO(TASK_ID_EXTDEV,	SDO_ID_EXTDEV_P_VECTOR_SD_LH,  DOP_UINT8,	Set_P_Vector_Sd_LH);

	/* Timer Callback Allocation */
	if (IOIF_StartTimIT(IOIF_TIM2) > 0) {
		//TODO: ERROR PROCESS
	}
	IOIF_SetTimCB(IOIF_TIM2, IOIF_TIM_PERIOD_ELAPSED_CALLBACK, RunExppackCtrl, NULL);
}

void RunExppackCtrl(void* params)
{
	/* Loop Start Time Check */							// pMMG DAQ에서 DWT->CYCCNT 사용하므로, 이 부분 사용 금지
//	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
//	DWT->CYCCNT = 0;
//	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	EXTDEVcodeStartTick = DWT->CYCCNT;

	/* Run Device */
	RunTask(&exppackCtrlTask);

	/* Elapsed Time Check */
	EXTDEVcodeEndTick = DWT->CYCCNT;
	if (EXTDEVcodeEndTick < EXTDEVcodeStartTick) {
		exppackCtrlTimeElap = ((4294967295 - EXTDEVcodeStartTick) + EXTDEVcodeEndTick) / 480;	// in microsecond (Roll-over)
	}
	else {
		exppackCtrlTimeElap = (DWT->CYCCNT - EXTDEVcodeStartTick) / 480;				// in microsecond
	}
}



/**
 *------------------------------------------------------------
 *                      STATIC FUNCTIONS
 *------------------------------------------------------------
 * @brief Functions intended for internal use within this module.
 */

static void StateOff_Run(void)
{
	StateTransition(&exppackCtrlTask.stateMachine, TASK_STATE_STANDBY);
}

static void StateStandby_Run(void)
{
#ifdef BUTTON_MODE
	if (GPIO_EXTI_FLAG == 1) {
		StateTransition(&exppackCtrlTask.stateMachine, TASK_STATE_ENABLE);
	}

	if (USB_CDC_STOP_FLAG == 1) {
		TerminateUSBCDC_DEMO4();
		USB_CDC_STOP_FLAG = 0;
	}

	DEBOUNCE_CNT++;
#endif

//	StateTransition(&exppackCtrlTask.stateMachine, TASK_STATE_ENABLE);
}

static void StateEnable_Ent(void)
{
	EntRoutines(&exppackCtrlTask.routine);

	exppackCtrlLoopCnt = 0;
}

static void StateEnable_Run(void)
{
	if (startPvector_decoding == 1) {
		startPvector_decoding = 0;			// Reset

		if (motionMap_selection >= 0 && motionMap_selection < 40) {
			pvectorObj_RH = MotionMap_File.MS[motionMap_selection].MD[0].p_vector_decoder;		// Assign
			pvectorObj_LH = MotionMap_File.MS[motionMap_selection].MD[1].p_vector_decoder;		// Assign
			PushRoutine(&exppackCtrlTask.routine, ROUTINE_ID_EXTDEV_P_VECTOR_DECODER_RH);
			PushRoutine(&exppackCtrlTask.routine, ROUTINE_ID_EXTDEV_P_VECTOR_DECODER_LH);

			motionMap_selection = 99;		// Reset
		}
	}

	RunRoutines(&exppackCtrlTask.routine);

	/* Get pMMG data (1) A,B are activated(about 2.6ms), (2) A or B is activated(about 1.3ms) */
	if ( (ExpPackDataObj.pMMG_activated_A == 1) && (ExpPackDataObj.pMMG_activated_B == 1) ) {
		if (exppackCtrlLoopCnt % 3 == 0) {		// Sampling with 3ms
			GetRawPMMG_A(&ExpPackDataObj);
			GetRawPMMG_B(&ExpPackDataObj);
		}
	}
	else if ( (ExpPackDataObj.pMMG_activated_A == 1) && (ExpPackDataObj.pMMG_activated_B == 0) ) {
		if (exppackCtrlLoopCnt % 2 == 0) {		// Sampling with 2ms
			GetRawPMMG_A(&ExpPackDataObj);
		}
	}
	else if ( (ExpPackDataObj.pMMG_activated_A == 0) && (ExpPackDataObj.pMMG_activated_B == 1) ) {
		if (exppackCtrlLoopCnt % 2 == 0) {		// Sampling with 2ms
			GetRawPMMG_B(&ExpPackDataObj);
		}
	}

	GetRawFSR(&ExpPackDataObj);
	GetRawEMG(&ExpPackDataObj);

	/* Processing Part */
	ProcessEMG(&ExpPackDataObj);
	ExpPackDataObj.emg_data.emg_R2_LPF[0] = 2 * ExpPackDataObj.emg_data.emg_R2_LPF[0];	// For Demo_4

	/* For KW_Univ */



	/* Scaling for PDO sending */
	ScalingForPDO(&ScaledDataObj, &ExpPackDataObj);


#ifdef BUTTON_MODE
	/* Data saving process for PC */
//#ifdef USB_CDC_ACTIVATE
//	if ((exppackCtrlLoopCnt+1) % PERIOD_USB_CDC == 0) {
//		PrepareUSBCDCTxData(&ExpPackDataObj);
//
//		timeUSBCDC = timeUSBCDC + PERIOD_USB_CDC;
//	}
//#endif

#ifdef USB_CDC_ACTIVATE
//	PrepareUSBCDCTxData_DEMO4(&ExpPackDataObj);			// For HAR_DEMO_4 -> 1ms
//	timeUSBCDC = timeUSBCDC + 1;

	if ((exppackCtrlLoopCnt+1) % PERIOD_USB_CDC == 0) {
		PrepareUSBCDCTxData_DEMO4(&ExpPackDataObj);

		timeUSBCDC = timeUSBCDC + PERIOD_USB_CDC;
	}
#endif

	DEBOUNCE_CNT++;
#endif

	exppackCtrlLoopCnt++;
}

static void StateEnable_Ext(void)
{
    ExtRoutines(&exppackCtrlTask.routine);
}

static void StateError_Run(void)
{

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static int InitPMMG(void)
{
	uint8_t pMMG_activate_A = 0;
	uint8_t pMMG_activate_B = 0;

#ifdef PMMG_1A_ENABLE
	state_1A = IOIF_pMMG_Init(&pMMGObj_1A, IOIF_SPI1, IOIF_GPIO_PORT_G, IOIF_GPIO_PIN_10);
	pMMG_activate_A = 1;
#endif
#ifdef PMMG_1B_ENABLE
	state_1B = IOIF_pMMG_Init(&pMMGObj_1B, IOIF_SPI1, IOIF_GPIO_PORT_G, IOIF_GPIO_PIN_8);
	pMMG_activate_B = 1;
#endif
#ifdef PMMG_2A_ENABLE
	state_2A = IOIF_pMMG_Init(&pMMGObj_2A, IOIF_SPI2, IOIF_GPIO_PORT_B, IOIF_GPIO_PIN_12);
	pMMG_activate_A = 1;
#endif
#ifdef PMMG_2B_ENABLE
	state_2B = IOIF_pMMG_Init(&pMMGObj_2B, IOIF_SPI2, IOIF_GPIO_PORT_H, IOIF_GPIO_PIN_5);
	pMMG_activate_B = 1;
#endif
#ifdef PMMG_3A_ENABLE
	state_3A = IOIF_pMMG_Init(&pMMGObj_3A, IOIF_SPI3, IOIF_GPIO_PORT_A, IOIF_GPIO_PIN_15);
	pMMG_activate_A = 1;
#endif
#ifdef PMMG_3B_ENABLE
	state_3B = IOIF_pMMG_Init(&pMMGObj_3B, IOIF_SPI3, IOIF_GPIO_PORT_D, IOIF_GPIO_PIN_14);
	pMMG_activate_B = 1;
#endif
#ifdef PMMG_4A_ENABLE
	state_4A = IOIF_pMMG_Init(&pMMGObj_4A, IOIF_SPI4, IOIF_GPIO_PORT_E, IOIF_GPIO_PIN_4);
	pMMG_activate_A = 1;
#endif
#ifdef PMMG_4B_ENABLE
	state_4B = IOIF_pMMG_Init(&pMMGObj_4B, IOIF_SPI4, IOIF_GPIO_PORT_D, IOIF_GPIO_PIN_15);
	pMMG_activate_B = 1;
#endif

	ExpPackDataObj.pMMG_activated_A = pMMG_activate_A;
	ExpPackDataObj.pMMG_activated_B = pMMG_activate_B;

	return 0;
}


static int GetRawPMMG_A(Exppack_Data_t* exppackDataObj)
{
	IOIF_pMMG_Update_multiple_2(&pMMGObj_1A, &pMMGObj_2A);

	// Mapping //
#ifdef PMMG_1A_ENABLE
	exppackDataObj->pMMG_press.pMMG1A_press = (float)pMMGObj_1A.pMMGData.pressureKPa;
	exppackDataObj->pMMG_temp.pMMG1A_temp = (float)pMMGObj_1A.pMMGData.temperatureC;
#endif
#ifdef PMMG_2A_ENABLE
	exppackDataObj->pMMG_press.pMMG2A_press = (float)pMMGObj_2A.pMMGData.pressureKPa;
	exppackDataObj->pMMG_temp.pMMG2A_temp = (float)pMMGObj_2A.pMMGData.temperatureC;
#endif
#ifdef PMMG_3A_ENABLE
	exppackDataObj->pMMG_press.pMMG3A_press = (float)pMMGObj_3A.pMMGData.pressureKPa;
	exppackDataObj->pMMG_temp.pMMG3A_temp = (float)pMMGObj_3A.pMMGData.temperatureC;
#endif
#ifdef PMMG_4A_ENABLE
	exppackDataObj->pMMG_press.pMMG4A_press = (float)pMMGObj_4A.pMMGData.pressureKPa;
	exppackDataObj->pMMG_temp.pMMG4A_temp = (float)pMMGObj_4A.pMMGData.temperatureC;
#endif

	return 0;
}

static int GetRawPMMG_B(Exppack_Data_t* exppackDataObj)
{
	IOIF_pMMG_Update_multiple_2(&pMMGObj_1B, &pMMGObj_2B);

	// Mapping //
#ifdef PMMG_1B_ENABLE
	exppackDataObj->pMMG_press.pMMG1B_press = (float)pMMGObj_1B.pMMGData.pressureKPa;
	exppackDataObj->pMMG_temp.pMMG1B_temp = (float)pMMGObj_1B.pMMGData.temperatureC;
#endif
#ifdef PMMG_2B_ENABLE
	exppackDataObj->pMMG_press.pMMG2B_press = (float)pMMGObj_2B.pMMGData.pressureKPa;
	exppackDataObj->pMMG_temp.pMMG2B_temp = (float)pMMGObj_2B.pMMGData.temperatureC;
#endif
#ifdef PMMG_3B_ENABLE
	exppackDataObj->pMMG_press.pMMG3B_press = (float)pMMGObj_3B.pMMGData.pressureKPa;
	exppackDataObj->pMMG_temp.pMMG3B_temp = (float)pMMGObj_3B.pMMGData.temperatureC;
#endif
#ifdef PMMG_4B_ENABLE
	exppackDataObj->pMMG_press.pMMG4B_press = (float)pMMGObj_4B.pMMGData.pressureKPa;
	exppackDataObj->pMMG_temp.pMMG4B_temp = (float)pMMGObj_4B.pMMGData.temperatureC;
#endif

	return 0;
}


static int GetRawFSR(Exppack_Data_t* exppackDataObj)
{
	/* FSR */
#ifdef FSR_L1_ENABLE
	exppackDataObj->fsr_data.fsr_L1_raw = adc1buff[2];
#endif
#ifdef FSR_L2_ENABLE
	exppackDataObj->fsr_data.fsr_L2_raw = adc1buff[3];
#endif
#ifdef FSR_L3_ENABLE
	exppackDataObj->fsr_data.fsr_L3_raw = adc3buff[0];
#endif
#ifdef FSR_L4_ENABLE
	exppackDataObj->fsr_data.fsr_L4_raw = adc3buff[1];
#endif
#ifdef FSR_R1_ENABLE
	exppackDataObj->fsr_data.fsr_R1_raw = adc3buff[5];
#endif
#ifdef FSR_R2_ENABLE
	exppackDataObj->fsr_data.fsr_R2_raw = adc3buff[9];
#endif
#ifdef FSR_R3_ENABLE
	exppackDataObj->fsr_data.fsr_R3_raw = adc3buff[4];
#endif
#ifdef FSR_R4_ENABLE
	exppackDataObj->fsr_data.fsr_R4_raw = adc3buff[8];
#endif

	return 0;
}


static int16_t signingEMG(uint16_t input)
{
	/* Consider the sign of EMG signal (zero mean) */
	int16_t output = 0;
	if (input > 2048) {
		output = input - 2048;
	}
	else {
		output = input - 2048;
	}

	return output;
}

static int GetRawEMG(Exppack_Data_t* exppackDataObj)
{
	/* EMG */
#ifdef EMG_L1_ENABLE
	exppackDataObj->emg_data.emg_L1_raw = adc3buff[3];
	exppackDataObj->emg_data.emg_L1_rawSign[2] = exppackDataObj->emg_data.emg_L1_rawSign[1];
	exppackDataObj->emg_data.emg_L1_rawSign[1] = exppackDataObj->emg_data.emg_L1_rawSign[0];
	exppackDataObj->emg_data.emg_L1_rawSign[0] = signingEMG(exppackDataObj->emg_data.emg_L1_raw);
#endif
#ifdef EMG_L2_ENABLE
	exppackDataObj->emg_data.emg_L2_raw = adc3buff[7];
	exppackDataObj->emg_data.emg_L2_rawSign[2] = exppackDataObj->emg_data.emg_L2_rawSign[1];
	exppackDataObj->emg_data.emg_L2_rawSign[1] = exppackDataObj->emg_data.emg_L2_rawSign[0];
	exppackDataObj->emg_data.emg_L2_rawSign[0] = signingEMG(exppackDataObj->emg_data.emg_L2_raw);
#endif
#ifdef EMG_L3_ENABLE
	exppackDataObj->emg_data.emg_L3_raw = adc3buff[2];
	exppackDataObj->emg_data.emg_L3_rawSign[2] = exppackDataObj->emg_data.emg_L3_rawSign[1];
	exppackDataObj->emg_data.emg_L3_rawSign[1] = exppackDataObj->emg_data.emg_L3_rawSign[0];
	exppackDataObj->emg_data.emg_L3_rawSign[0] = signingEMG(exppackDataObj->emg_data.emg_L3_raw);
#endif
#ifdef EMG_L4_ENABLE
	exppackDataObj->emg_data.emg_L4_raw = adc3buff[6];
	exppackDataObj->emg_data.emg_L4_rawSign[2] = exppackDataObj->emg_data.emg_L4_rawSign[1];
	exppackDataObj->emg_data.emg_L4_rawSign[1] = exppackDataObj->emg_data.emg_L4_rawSign[0];
	exppackDataObj->emg_data.emg_L4_rawSign[0] = signingEMG(exppackDataObj->emg_data.emg_L4_raw);
#endif
#ifdef EMG_R1_ENABLE
	exppackDataObj->emg_data.emg_R1_raw = adc1buff[0];
	exppackDataObj->emg_data.emg_R1_rawSign[2] = exppackDataObj->emg_data.emg_R1_rawSign[1];
	exppackDataObj->emg_data.emg_R1_rawSign[1] = exppackDataObj->emg_data.emg_R1_rawSign[0];
	exppackDataObj->emg_data.emg_R1_rawSign[0] = signingEMG(exppackDataObj->emg_data.emg_R1_raw);
#endif
#ifdef EMG_R2_ENABLE
	exppackDataObj->emg_data.emg_R2_raw = adc1buff[1];
	exppackDataObj->emg_data.emg_R2_rawSign[2] = exppackDataObj->emg_data.emg_R2_rawSign[1];
	exppackDataObj->emg_data.emg_R2_rawSign[1] = exppackDataObj->emg_data.emg_R2_rawSign[0];
	exppackDataObj->emg_data.emg_R2_rawSign[0] = signingEMG(exppackDataObj->emg_data.emg_R2_raw);
#endif
#ifdef EMG_R3_ENABLE
	exppackDataObj->emg_data.emg_R3_raw = adc2buff[0];
	exppackDataObj->emg_data.emg_R3_rawSign[2] = exppackDataObj->emg_data.emg_R3_rawSign[1];
	exppackDataObj->emg_data.emg_R3_rawSign[1] = exppackDataObj->emg_data.emg_R3_rawSign[0];
	exppackDataObj->emg_data.emg_R3_rawSign[0] = signingEMG(exppackDataObj->emg_data.emg_R3_raw);
#endif
#ifdef EMG_R4_ENABLE
	exppackDataObj->emg_data.emg_R4_raw = adc2buff[1];
	exppackDataObj->emg_data.emg_R4_rawSign[2] = exppackDataObj->emg_data.emg_R4_rawSign[1];
	exppackDataObj->emg_data.emg_R4_rawSign[1] = exppackDataObj->emg_data.emg_R4_rawSign[0];
	exppackDataObj->emg_data.emg_R4_rawSign[0] = signingEMG(exppackDataObj->emg_data.emg_R4_raw);
#endif

	return 0;
}


#ifdef BUTTON_MODE
static void PrepareUSBCDCTxData(Exppack_Data_t* exppackDataObj)
{
	/* Reset the buffers */
	memset(usbTxBuf, 0, sizeof(usbTxBuf));
	memset(usbTxBufChar, 0, sizeof(usbTxBufChar));

	/* Assign the Data to send through USB CDC FS */
	uint32_t data1 = timeUSBCDC;
	float data2 = (float)exppackDataObj->pMMG_press.pMMG1A_press;
	float data3 = (float)exppackDataObj->pMMG_press.pMMG1B_press;
	float data4 = (float)exppackDataObj->pMMG_press.pMMG2A_press;
	float data5 = (float)exppackDataObj->pMMG_press.pMMG2B_press;
	float data6 = (float)exppackDataObj->pMMG_press.pMMG3A_press;
	float data7 = (float)exppackDataObj->pMMG_press.pMMG3B_press;
	float data8 = (float)exppackDataObj->pMMG_press.pMMG4A_press;
	float data9 = (float)exppackDataObj->pMMG_press.pMMG4B_press;
	uint16_t data10 = exppackDataObj->fsr_data.fsr_L1_raw;
	uint16_t data11 = exppackDataObj->fsr_data.fsr_L2_raw;

	/* Append 1st component to sent */
	sprintf(usbTxBufChar, "%lu", data1);

	/* Append 2nd component to sent */
	strcat(usbTxBufChar, splitString);
	memset(strBuf_32bit, '\0', sizeof(strBuf_32bit));
	sprintf(strBuf_32bit, "%.2f", data2);
	strcat(usbTxBufChar, strBuf_32bit);

	/* Append 3rd component to sent */
	strcat(usbTxBufChar, splitString);
	memset(strBuf_32bit, '\0', sizeof(strBuf_32bit));
	sprintf(strBuf_32bit, "%.2f", data3);
	strcat(usbTxBufChar, strBuf_32bit);

	/* Append 4th component to sent */
	strcat(usbTxBufChar, splitString);
	memset(strBuf_32bit, '\0', sizeof(strBuf_32bit));
	sprintf(strBuf_32bit, "%.2f", data4);
	strcat(usbTxBufChar, strBuf_32bit);

	/* Append 5th component to sent */
	strcat(usbTxBufChar, splitString);
	memset(strBuf_32bit, '\0', sizeof(strBuf_32bit));
	sprintf(strBuf_32bit, "%.2f", data5);
	strcat(usbTxBufChar, strBuf_32bit);

	/* Append 6th component to sent */
	strcat(usbTxBufChar, splitString);
	memset(strBuf_32bit, '\0', sizeof(strBuf_32bit));
	sprintf(strBuf_32bit, "%.2f", data6);
	strcat(usbTxBufChar, strBuf_32bit);

	/* Append 7th component to sent */
	strcat(usbTxBufChar, splitString);
	memset(strBuf_32bit, '\0', sizeof(strBuf_32bit));
	sprintf(strBuf_32bit, "%.2f", data7);
	strcat(usbTxBufChar, strBuf_32bit);

	/* Append 8th component to sent */
	strcat(usbTxBufChar, splitString);
	memset(strBuf_32bit, '\0', sizeof(strBuf_32bit));
	sprintf(strBuf_32bit, "%.2f", data8);
	strcat(usbTxBufChar, strBuf_32bit);

	/* Append 9th component to sent */
	strcat(usbTxBufChar, splitString);
	memset(strBuf_32bit, '\0', sizeof(strBuf_32bit));
	sprintf(strBuf_32bit, "%.2f", data9);
	strcat(usbTxBufChar, strBuf_32bit);

	/* Append 10th component to sent */
	strcat(usbTxBufChar, splitString);
	memset(strBuf_16bit, '\0', sizeof(strBuf_16bit));
	sprintf(strBuf_16bit, "%u", data10);
	strcat(usbTxBufChar, strBuf_16bit);

	/* Append 11th component to sent */
	strcat(usbTxBufChar, splitString);
	memset(strBuf_16bit, '\0', sizeof(strBuf_16bit));
	sprintf(strBuf_16bit, "%u", data11);
	strcat(usbTxBufChar, strBuf_16bit);

	/* Add "\n" in the end of data */
	strcat(usbTxBufChar, newLine);

	sprintf((char*)usbTxBuf, usbTxBufChar);
	usbTxBufSize = strlen(usbTxBufChar);

	usbTxUpdate = 1;
}


static void GPIO_EXTI_8_CALLBACK(uint16_t gpioPins)
{
	/* EXTI operation */
	if (gpioPins == GPIO_PIN_8 && GPIO_EXTI_FLAG == 0 && DEBOUNCE_CNT > 500) {
		timeUSBCDC = 0;
		GPIO_EXTI_FLAG = 1;
		DEBOUNCE_CNT = 0;
	}
	else if (gpioPins == GPIO_PIN_8 && GPIO_EXTI_FLAG == 1 && DEBOUNCE_CNT > 500) {
		timeUSBCDC = 0;
		GPIO_EXTI_FLAG = 0;
		exppackCtrlLoopCnt = 0;
		DEBOUNCE_CNT = 0;

		USB_CDC_STOP_FLAG = 1;
		StateTransition(&exppackCtrlTask.stateMachine, TASK_STATE_STANDBY);
	}
}
#endif





/* ------------------- SCALING DATA FOR PDO COMMUNICATION ------------------- */
// Function to scale a float to a 16-bit integer type(int16)
int16_t ScaleFloatToInt16(float value, float scaleFactor)
{
	// Scale the float value
	int16_t scaledInt16Value = (int16_t)(value * DATA_CONV_CONST_INT16 / scaleFactor);

	return scaledInt16Value;
}

// Function to scale a float to a 16-bit integer type(uint16)
uint16_t ScaleFloatToUInt16(float value, float scaleFactor)
{
	// Scale the float value
	uint16_t scaledUint16Value = (uint16_t)(value * DATA_CONV_CONST_UINT16 / scaleFactor);

	return scaledUint16Value;
}

// Function to scale int16 to a float type
float ScaleInt16ToFloat(int16_t value, float scaleFactor)
{
    // Scale the float value
    float scaledValue = (float)(value * scaleFactor / DATA_CONV_CONST_INT16);

    return scaledValue;
}



static void ScalingForPDO(ScaledData_t* Scaled2ByteData, Exppack_Data_t* exppackDataObj)
{
	// Scaling for PDO Data(2Byte)
//	Scaled2ByteData->pMMG_1A_scaled = ScaleFloatToUInt16(exppackDataObj->pMMG_press.pMMG1A_press, PMMG_SCALING_FACTOR);
//	Scaled2ByteData->pMMG_1B_scaled = ScaleFloatToUInt16(exppackDataObj->pMMG_press.pMMG1B_press, PMMG_SCALING_FACTOR);
//	Scaled2ByteData->pMMG_2A_scaled = ScaleFloatToUInt16(exppackDataObj->pMMG_press.pMMG2A_press, PMMG_SCALING_FACTOR);
//	Scaled2ByteData->pMMG_2B_scaled = ScaleFloatToUInt16(exppackDataObj->pMMG_press.pMMG2B_press, PMMG_SCALING_FACTOR);
//	Scaled2ByteData->pMMG_3A_scaled = ScaleFloatToUInt16(exppackDataObj->pMMG_press.pMMG3A_press, PMMG_SCALING_FACTOR);
//	Scaled2ByteData->pMMG_3B_scaled = ScaleFloatToUInt16(exppackDataObj->pMMG_press.pMMG3B_press, PMMG_SCALING_FACTOR);
//	Scaled2ByteData->pMMG_4A_scaled = ScaleFloatToUInt16(exppackDataObj->pMMG_press.pMMG4A_press, PMMG_SCALING_FACTOR);
//	Scaled2ByteData->pMMG_4B_scaled = ScaleFloatToUInt16(exppackDataObj->pMMG_press.pMMG4B_press, PMMG_SCALING_FACTOR);

//	Scaled2ByteData->EMG_L1_scaled = exppackDataObj->emg_data.emg_L1_raw;
//	Scaled2ByteData->EMG_L2_scaled = exppackDataObj->emg_data.emg_L2_raw;
//	Scaled2ByteData->EMG_L3_scaled = exppackDataObj->emg_data.emg_L3_raw;
//	Scaled2ByteData->EMG_L4_scaled = exppackDataObj->emg_data.emg_L4_raw;
//	Scaled2ByteData->EMG_R1_scaled = exppackDataObj->emg_data.emg_R1_raw;
//	Scaled2ByteData->EMG_R2_scaled = exppackDataObj->emg_data.emg_R2_raw;
//	Scaled2ByteData->EMG_R3_scaled = exppackDataObj->emg_data.emg_R3_raw;
//	Scaled2ByteData->EMG_R4_scaled = exppackDataObj->emg_data.emg_R4_raw;

//	Scaled2ByteData->FSR_L1_scaled = exppackDataObj->fsr_data.fsr_L1_raw;
//	Scaled2ByteData->FSR_L2_scaled = exppackDataObj->fsr_data.fsr_L2_raw;
//	Scaled2ByteData->FSR_L3_scaled = exppackDataObj->fsr_data.fsr_L3_raw;
//	Scaled2ByteData->FSR_L4_scaled = exppackDataObj->fsr_data.fsr_L4_raw;
//	Scaled2ByteData->FSR_R1_scaled = exppackDataObj->fsr_data.fsr_R1_raw;
//	Scaled2ByteData->FSR_R2_scaled = exppackDataObj->fsr_data.fsr_R2_raw;
//	Scaled2ByteData->FSR_R3_scaled = exppackDataObj->fsr_data.fsr_R3_raw;
//	Scaled2ByteData->FSR_R4_scaled = exppackDataObj->fsr_data.fsr_R4_raw;

	/* [0,1] normalized & rectified */
//	Scaled2ByteData->EMG_L1_scaled = ScaleFloatToUInt16(exppackDataObj->emg_data.emg_L1_norm, EMG_NORM_SCALING_FACTOR);
//	Scaled2ByteData->EMG_L2_scaled = ScaleFloatToUInt16(exppackDataObj->emg_data.emg_L2_norm, EMG_NORM_SCALING_FACTOR);
//	Scaled2ByteData->EMG_L3_scaled = ScaleFloatToUInt16(exppackDataObj->emg_data.emg_L3_norm, EMG_NORM_SCALING_FACTOR);
//	Scaled2ByteData->EMG_L4_scaled = ScaleFloatToUInt16(exppackDataObj->emg_data.emg_L4_norm, EMG_NORM_SCALING_FACTOR);
//	Scaled2ByteData->EMG_R1_scaled = ScaleFloatToUInt16(exppackDataObj->emg_data.emg_R1_norm, EMG_NORM_SCALING_FACTOR);
//	Scaled2ByteData->EMG_R2_scaled = ScaleFloatToUInt16(exppackDataObj->emg_data.emg_R2_norm, EMG_NORM_SCALING_FACTOR);
//	Scaled2ByteData->EMG_R3_scaled = ScaleFloatToUInt16(exppackDataObj->emg_data.emg_R3_norm, EMG_NORM_SCALING_FACTOR);
//	Scaled2ByteData->EMG_R4_scaled = ScaleFloatToUInt16(exppackDataObj->emg_data.emg_R4_norm, EMG_NORM_SCALING_FACTOR);

	/* [-1,1] normalized & signing */
//	Scaled2ByteData->EMG_L1_scaled = ScaleFloatToInt16((float)(exppackDataObj->emg_data.emg_L1_rawSign / 2048.0f), EMG_RAWSIGN_SCALING_FACTOR);
//	Scaled2ByteData->EMG_L2_scaled = ScaleFloatToInt16((float)(exppackDataObj->emg_data.emg_L2_rawSign / 2048.0f), EMG_RAWSIGN_SCALING_FACTOR);
//	Scaled2ByteData->EMG_L3_scaled = ScaleFloatToInt16((float)(exppackDataObj->emg_data.emg_L3_rawSign / 2048.0f), EMG_RAWSIGN_SCALING_FACTOR);
//	Scaled2ByteData->EMG_L4_scaled = ScaleFloatToInt16((float)(exppackDataObj->emg_data.emg_L4_rawSign / 2048.0f), EMG_RAWSIGN_SCALING_FACTOR);
//	Scaled2ByteData->EMG_R1_scaled = ScaleFloatToInt16((float)(exppackDataObj->emg_data.emg_R1_rawSign / 2048.0f), EMG_RAWSIGN_SCALING_FACTOR);
//	Scaled2ByteData->EMG_R2_scaled = ScaleFloatToInt16((float)(exppackDataObj->emg_data.emg_R2_rawSign / 2048.0f), EMG_RAWSIGN_SCALING_FACTOR);
//	Scaled2ByteData->EMG_R3_scaled = ScaleFloatToInt16((float)(exppackDataObj->emg_data.emg_R3_rawSign / 2048.0f), EMG_RAWSIGN_SCALING_FACTOR);
//	Scaled2ByteData->EMG_R4_scaled = ScaleFloatToInt16((float)(exppackDataObj->emg_data.emg_R4_rawSign / 2048.0f), EMG_RAWSIGN_SCALING_FACTOR);

	/* Send raw & processed EMG data (for Demo4) */
//	Scaled2ByteData->EMG_RAWSIGN = ScaleFloatToInt16((float)(exppackDataObj->emg_data.emg_R2_rawSign[0] / 2048.0f), EMG_RAWSIGN_SCALING_FACTOR);	// [-1,1]
//	Scaled2ByteData->EMG_ENVELOPE = ScaleFloatToUInt16(exppackDataObj->emg_data.emg_R2_LPF[0], EMG_NORM_SCALING_FACTOR);							// [0, 1]
//	Scaled2ByteData->EMG_MA = ScaleFloatToUInt16(exppackDataObj->emg_data.emg_R2_MA, EMG_NORM_SCALING_FACTOR);										// [0, 1]

	/* Send PDO data (for KW_Univ) */
	Scaled2ByteData->control_input_RH = ScaleFloatToInt16((float)(totalDataObj.u_RH), CONTROL_INPUT_SCALING_FACTOR);
	Scaled2ByteData->control_input_LH = ScaleFloatToInt16((float)(totalDataObj.u_LH), CONTROL_INPUT_SCALING_FACTOR);
	Scaled2ByteData->theta_ref_RH = ScaleFloatToInt16((float)(totalDataObj.theta_RH_ref), DEG_SCALING_FACTOR);
	Scaled2ByteData->theta_ref_LH = ScaleFloatToInt16((float)(totalDataObj.theta_LH_ref), DEG_SCALING_FACTOR);
	Scaled2ByteData->EMG_RAW = ScaleFloatToInt16((float)(totalDataObj.EMG_raw), EMG_RAWSIGN_SCALING_FACTOR);					// [-1,1]
	Scaled2ByteData->EMG_PROCESSED = ScaleFloatToUInt16((float)(totalDataObj.EMG_processed), EMG_NORM_SCALING_FACTOR);			// [0,1]

	Scaled2ByteData->Fvector_trigger_1 = totalDataObj.fvector_trigger_1;
	Scaled2ByteData->Fvector_trigger_2 = totalDataObj.fvector_trigger_2;
	Scaled2ByteData->Fvector_trigger_3 = totalDataObj.fvector_trigger_3;
	Scaled2ByteData->Fvector_trigger_4 = totalDataObj.fvector_trigger_4;
	Scaled2ByteData->Fvector_trigger_5 = totalDataObj.fvector_trigger_5;

//	Scaled2ByteData->Pvector_test_RH = ScaleFloatToInt16((float)(posCtrl_RH.ref), DEG_SCALING_FACTOR);
//	Scaled2ByteData->Pvector_test_LH = ScaleFloatToInt16((float)(posCtrl_LH.ref), DEG_SCALING_FACTOR);
	Scaled2ByteData->Pvector_test_RH = posCtrl_RH.ref;
	Scaled2ByteData->Pvector_test_LH = posCtrl_LH.ref;
}



static float RectificationEMG(float EMGval)
{
	/* Rectification */
	float rectifiedSignal = 0;
	if (EMGval >= 0) {
		rectifiedSignal = EMGval;
	}
	else {
		rectifiedSignal = (-1)*EMGval;
	}

	return rectifiedSignal;
}

static void RectifyEMG_ALL(Exppack_Data_t* exppackDataObj)
{
	/* EMG */
#ifdef EMG_L1_ENABLE
	exppackDataObj->emg_data.emg_L1_rect = RectificationEMG(exppackDataObj->emg_data.emg_L1_BPF[0]);
#endif
#ifdef EMG_L2_ENABLE
	exppackDataObj->emg_data.emg_L2_rect = RectificationEMG(exppackDataObj->emg_data.emg_L2_BPF[0]);
#endif
#ifdef EMG_L3_ENABLE
	exppackDataObj->emg_data.emg_L3_rect = RectificationEMG(exppackDataObj->emg_data.emg_L3_BPF[0]);
#endif
#ifdef EMG_L4_ENABLE
	exppackDataObj->emg_data.emg_L4_rect = RectificationEMG(exppackDataObj->emg_data.emg_L4_BPF[0]);
#endif
#ifdef EMG_R1_ENABLE
	exppackDataObj->emg_data.emg_R1_rect = RectificationEMG(exppackDataObj->emg_data.emg_R1_BPF[0]);
#endif
#ifdef EMG_R2_ENABLE
	exppackDataObj->emg_data.emg_R2_rect = RectificationEMG(exppackDataObj->emg_data.emg_R2_BPF[0]);
#endif
#ifdef EMG_R3_ENABLE
	exppackDataObj->emg_data.emg_R3_rect = RectificationEMG(exppackDataObj->emg_data.emg_R3_BPF[0]);
#endif
#ifdef EMG_R4_ENABLE
	exppackDataObj->emg_data.emg_R4_rect = RectificationEMG(exppackDataObj->emg_data.emg_R4_BPF[0]);
#endif
}


static float NormalizeEMG(float EMG_rect_val)
{
	/* Normalization */
	float normalizedSignal = EMG_rect_val/2048.0f;

	return normalizedSignal;
}

static void NormalizeEMG_ALL(Exppack_Data_t* exppackDataObj)
{
	/* EMG */
#ifdef EMG_L1_ENABLE
	exppackDataObj->emg_data.emg_L1_norm = NormalizeEMG(exppackDataObj->emg_data.emg_L1_rect);
#endif
#ifdef EMG_L2_ENABLE
	exppackDataObj->emg_data.emg_L2_norm = NormalizeEMG(exppackDataObj->emg_data.emg_L2_rect);
#endif
#ifdef EMG_L3_ENABLE
	exppackDataObj->emg_data.emg_L3_norm = NormalizeEMG(exppackDataObj->emg_data.emg_L3_rect);
#endif
#ifdef EMG_L4_ENABLE
	exppackDataObj->emg_data.emg_L4_norm = NormalizeEMG(exppackDataObj->emg_data.emg_L4_rect);
#endif
#ifdef EMG_R1_ENABLE
	exppackDataObj->emg_data.emg_R1_norm = NormalizeEMG(exppackDataObj->emg_data.emg_R1_rect);
#endif
#ifdef EMG_R2_ENABLE
	exppackDataObj->emg_data.emg_R2_norm = NormalizeEMG(exppackDataObj->emg_data.emg_R2_rect);
#endif
#ifdef EMG_R3_ENABLE
	exppackDataObj->emg_data.emg_R3_norm = NormalizeEMG(exppackDataObj->emg_data.emg_R3_rect);
#endif
#ifdef EMG_R4_ENABLE
	exppackDataObj->emg_data.emg_R4_norm = NormalizeEMG(exppackDataObj->emg_data.emg_R4_rect);
#endif
}


static float LowPassFilter(uint8_t* isInitialized, float* prevOutput, float currInput)
{
	if (*isInitialized == 0) {
		*prevOutput = currInput;
		*isInitialized = 1;
	}

//	float currOutput = 0.644150443975408*(*prevOutput) + 0.355849556024592*(currInput);		// 70Hz
//	float currOutput = 0.828204181306860*(*prevOutput) + 0.171795818693140*(currInput);		// 30Hz
	float currOutput = 0.969072426304811*(*prevOutput) + 0.030927573695189*(currInput);		// 5Hz

	*prevOutput = currOutput;

	return currOutput;
}


static float BandPassFilter(uint8_t* isInitialized, float* prevOutput, float* prev_prevOutput, int16_t currInput, int16_t prevInput, int16_t prev_prevInput)
{
	if (*isInitialized == 0) {
		*prevOutput = currInput;
		*prev_prevOutput = currInput;

		*isInitialized = 1;
	}

//	float currOutput = 0.710362863191271*(*prevOutput) + 0.185761652639405*(*prev_prevOutput) + (0.0001949032802507652 * currInput) - (0.0001949032802507652 * prev_prevInput);
	float currOutput = 0.941075889592254*(*prevOutput) - 0.052177855701698*(*prev_prevOutput) + (0.861014163143824 * prevInput) - (0.861014163143824 * prev_prevInput);

	*prev_prevOutput = *prevOutput;
	*prevOutput = currOutput;

	return currOutput;
}


static void LowPassFilteringEMG_ALL(Exppack_Data_t* exppackDataObj)
{
#ifdef EMG_L1_ENABLE
	exppackDataObj->emg_data.emg_L1_LPF[0] = LowPassFilter(&exppackDataObj->emg_data.emg_L1_LPFstart, &exppackDataObj->emg_data.emg_L1_LPF[1], exppackDataObj->emg_data.emg_L1_norm);
#endif
#ifdef EMG_L2_ENABLE
	exppackDataObj->emg_data.emg_L2_LPF[0] = LowPassFilter(&exppackDataObj->emg_data.emg_L2_LPFstart, &exppackDataObj->emg_data.emg_L2_LPF[1], exppackDataObj->emg_data.emg_L2_norm);
#endif
#ifdef EMG_L3_ENABLE
	exppackDataObj->emg_data.emg_L3_LPF[0] = LowPassFilter(&exppackDataObj->emg_data.emg_L3_LPFstart, &exppackDataObj->emg_data.emg_L3_LPF[1], exppackDataObj->emg_data.emg_L3_norm);
#endif
#ifdef EMG_L4_ENABLE
	exppackDataObj->emg_data.emg_L4_LPF[0] = LowPassFilter(&exppackDataObj->emg_data.emg_L4_LPFstart, &exppackDataObj->emg_data.emg_L4_LPF[1], exppackDataObj->emg_data.emg_L4_norm);
#endif
#ifdef EMG_R1_ENABLE
	exppackDataObj->emg_data.emg_R1_LPF[0] = LowPassFilter(&exppackDataObj->emg_data.emg_R1_LPFstart, &exppackDataObj->emg_data.emg_R1_LPF[1], exppackDataObj->emg_data.emg_R1_norm);
#endif
#ifdef EMG_R2_ENABLE
	exppackDataObj->emg_data.emg_R2_LPF[0] = LowPassFilter(&exppackDataObj->emg_data.emg_R2_LPFstart, &exppackDataObj->emg_data.emg_R2_LPF[1], exppackDataObj->emg_data.emg_R2_norm);
#endif
#ifdef EMG_R3_ENABLE
	exppackDataObj->emg_data.emg_R3_LPF[0] = LowPassFilter(&exppackDataObj->emg_data.emg_R3_LPFstart, &exppackDataObj->emg_data.emg_R3_LPF[1], exppackDataObj->emg_data.emg_R3_norm);
#endif
#ifdef EMG_R4_ENABLE
	exppackDataObj->emg_data.emg_R4_LPF[0] = LowPassFilter(&exppackDataObj->emg_data.emg_R4_LPFstart, &exppackDataObj->emg_data.emg_R4_LPF[1], exppackDataObj->emg_data.emg_R4_norm);
#endif
}

static void BandPassFilteringEMG_ALL(Exppack_Data_t* exppackDataObj)
{
#ifdef EMG_L1_ENABLE
	exppackDataObj->emg_data.emg_L1_BPF[0] = BandPassFilter(&exppackDataObj->emg_data.emg_L1_BPFstart, &exppackDataObj->emg_data.emg_L1_BPF[1], &exppackDataObj->emg_data.emg_L1_BPF[2], exppackDataObj->emg_data.emg_L1_rawSign[0], exppackDataObj->emg_data.emg_L1_rawSign[1], exppackDataObj->emg_data.emg_L1_rawSign[2]);
#endif
#ifdef EMG_L2_ENABLE
	exppackDataObj->emg_data.emg_L2_BPF[0] = BandPassFilter(&exppackDataObj->emg_data.emg_L2_BPFstart, &exppackDataObj->emg_data.emg_L2_BPF[1], &exppackDataObj->emg_data.emg_L2_BPF[2], exppackDataObj->emg_data.emg_L2_rawSign[0], exppackDataObj->emg_data.emg_L2_rawSign[1], exppackDataObj->emg_data.emg_L2_rawSign[2]);
#endif
#ifdef EMG_L3_ENABLE
	exppackDataObj->emg_data.emg_L3_BPF[0] = BandPassFilter(&exppackDataObj->emg_data.emg_L3_BPFstart, &exppackDataObj->emg_data.emg_L3_BPF[1], &exppackDataObj->emg_data.emg_L3_BPF[2], exppackDataObj->emg_data.emg_L3_rawSign[0], exppackDataObj->emg_data.emg_L3_rawSign[1], exppackDataObj->emg_data.emg_L3_rawSign[2]);
#endif
#ifdef EMG_L4_ENABLE
	exppackDataObj->emg_data.emg_L4_BPF[0] = BandPassFilter(&exppackDataObj->emg_data.emg_L4_BPFstart, &exppackDataObj->emg_data.emg_L4_BPF[1], &exppackDataObj->emg_data.emg_L4_BPF[2], exppackDataObj->emg_data.emg_L4_rawSign[0], exppackDataObj->emg_data.emg_L4_rawSign[1], exppackDataObj->emg_data.emg_L4_rawSign[2]);
#endif
#ifdef EMG_R1_ENABLE
	exppackDataObj->emg_data.emg_R1_BPF[0] = BandPassFilter(&exppackDataObj->emg_data.emg_R1_BPFstart, &exppackDataObj->emg_data.emg_R1_BPF[1], &exppackDataObj->emg_data.emg_R1_BPF[2], exppackDataObj->emg_data.emg_R1_rawSign[0], exppackDataObj->emg_data.emg_R1_rawSign[1], exppackDataObj->emg_data.emg_R1_rawSign[2]);
#endif
#ifdef EMG_R2_ENABLE
	exppackDataObj->emg_data.emg_R2_BPF[0] = BandPassFilter(&exppackDataObj->emg_data.emg_R2_BPFstart, &exppackDataObj->emg_data.emg_R2_BPF[1], &exppackDataObj->emg_data.emg_R2_BPF[2], exppackDataObj->emg_data.emg_R2_rawSign[0], exppackDataObj->emg_data.emg_R2_rawSign[1], exppackDataObj->emg_data.emg_R2_rawSign[2]);
#endif
#ifdef EMG_R3_ENABLE
	exppackDataObj->emg_data.emg_R3_BPF[0] = BandPassFilter(&exppackDataObj->emg_data.emg_R3_BPFstart, &exppackDataObj->emg_data.emg_R3_BPF[1], &exppackDataObj->emg_data.emg_R3_BPF[2], exppackDataObj->emg_data.emg_R3_rawSign[0], exppackDataObj->emg_data.emg_R3_rawSign[1], exppackDataObj->emg_data.emg_R3_rawSign[2]);
#endif
#ifdef EMG_R4_ENABLE
	exppackDataObj->emg_data.emg_R4_BPF[0] = BandPassFilter(&exppackDataObj->emg_data.emg_R4_BPFstart, &exppackDataObj->emg_data.emg_R4_BPF[1], &exppackDataObj->emg_data.emg_R4_BPF[2], exppackDataObj->emg_data.emg_R4_rawSign[0], exppackDataObj->emg_data.emg_R4_rawSign[1], exppackDataObj->emg_data.emg_R4_rawSign[2]);
#endif
}


/* newVal = EMG_rect */
static float MovingAverage(uint16_t* MA_Buffer, uint8_t* ind, uint8_t* cnt, float* sum, uint16_t newVal)
{
	float resultedMAval = 0.0;

	*sum = *sum - MA_Buffer[*ind] / (EMG_SCALING_FACTOR / 2.0f);
	MA_Buffer[*ind] = newVal;
	*sum = *sum + ( newVal / (EMG_SCALING_FACTOR / 2.0f) );

	*ind = (*ind + 1) % EMG_MA_BUFF_SIZE;

	if (*cnt < EMG_MA_BUFF_SIZE) {
		*cnt = *cnt + 1;
		resultedMAval = *sum / *cnt;
	}
	else {
		resultedMAval = *sum / EMG_MA_BUFF_SIZE;
	}

	return resultedMAval;
}


static void MovingAverageEMG_ALL(Exppack_Data_t* exppackDataObj)
{
	/* EMG for 70Hz*/
#ifdef EMG_L1_ENABLE
	exppackDataObj->emg_data.emg_L1_MA = MovingAverage(exppackDataObj->emg_data.emg_L1_MA_Buff, &exppackDataObj->emg_data.emg_L1_MA_index, &exppackDataObj->emg_data.emg_L1_MA_count, &exppackDataObj->emg_data.emg_L1_MA_sum, exppackDataObj->emg_data.emg_L1_rect);
#endif
#ifdef EMG_L2_ENABLE
	exppackDataObj->emg_data.emg_L2_MA = MovingAverage(exppackDataObj->emg_data.emg_L2_MA_Buff, &exppackDataObj->emg_data.emg_L2_MA_index, &exppackDataObj->emg_data.emg_L2_MA_count, &exppackDataObj->emg_data.emg_L2_MA_sum, exppackDataObj->emg_data.emg_L2_rect);
#endif
#ifdef EMG_L3_ENABLE
	exppackDataObj->emg_data.emg_L3_MA = MovingAverage(exppackDataObj->emg_data.emg_L3_MA_Buff, &exppackDataObj->emg_data.emg_L3_MA_index, &exppackDataObj->emg_data.emg_L3_MA_count, &exppackDataObj->emg_data.emg_L3_MA_sum, exppackDataObj->emg_data.emg_L3_rect);
#endif
#ifdef EMG_L4_ENABLE
	exppackDataObj->emg_data.emg_L4_MA = MovingAverage(exppackDataObj->emg_data.emg_L4_MA_Buff, &exppackDataObj->emg_data.emg_L4_MA_index, &exppackDataObj->emg_data.emg_L4_MA_count, &exppackDataObj->emg_data.emg_L4_MA_sum, exppackDataObj->emg_data.emg_L4_rect);
#endif
#ifdef EMG_R1_ENABLE
	exppackDataObj->emg_data.emg_R1_MA = MovingAverage(exppackDataObj->emg_data.emg_R1_MA_Buff, &exppackDataObj->emg_data.emg_R1_MA_index, &exppackDataObj->emg_data.emg_R1_MA_count, &exppackDataObj->emg_data.emg_R1_MA_sum, exppackDataObj->emg_data.emg_R1_rect);
#endif
#ifdef EMG_R2_ENABLE
	exppackDataObj->emg_data.emg_R2_MA = MovingAverage(exppackDataObj->emg_data.emg_R2_MA_Buff, &exppackDataObj->emg_data.emg_R2_MA_index, &exppackDataObj->emg_data.emg_R2_MA_count, &exppackDataObj->emg_data.emg_R2_MA_sum, exppackDataObj->emg_data.emg_R2_rect);
#endif
#ifdef EMG_R3_ENABLE
	exppackDataObj->emg_data.emg_R3_MA = MovingAverage(exppackDataObj->emg_data.emg_R3_MA_Buff, &exppackDataObj->emg_data.emg_R3_MA_index, &exppackDataObj->emg_data.emg_R3_MA_count, &exppackDataObj->emg_data.emg_R3_MA_sum, exppackDataObj->emg_data.emg_R3_rect);
#endif
#ifdef EMG_R4_ENABLE
	exppackDataObj->emg_data.emg_R4_MA = MovingAverage(exppackDataObj->emg_data.emg_R4_MA_Buff, &exppackDataObj->emg_data.emg_R4_MA_index, &exppackDataObj->emg_data.emg_R4_MA_count, &exppackDataObj->emg_data.emg_R4_MA_sum, exppackDataObj->emg_data.emg_R4_rect);
#endif
}



static void ProcessEMG(Exppack_Data_t* exppackDataObj)
{
	BandPassFilteringEMG_ALL(exppackDataObj);		// 20~450Hz
	RectifyEMG_ALL(exppackDataObj);					// Absolute
	NormalizeEMG_ALL(exppackDataObj);				// Normalize [0,1]
	LowPassFilteringEMG_ALL(exppackDataObj);		// Enveloping
//	MovingAverageEMG_ALL(exppackDataObj);
}




#ifdef BUTTON_MODE
/* For HAR_Demo_4 [EMG] */
static void PrepareUSBCDCTxData_DEMO4(Exppack_Data_t* exppackDataObj)
{
	/* Reset the buffers */
	memset(usbTxBuf, 0, sizeof(usbTxBuf));
	memset(usbTxBufChar, 0, sizeof(usbTxBufChar));

	/* Assign the Data to send through USB CDC FS */
	uint32_t data1 = timeUSBCDC;
	float data2 = (float)exppackDataObj->pMMG_press.pMMG1A_press;
	float data3 = (float)exppackDataObj->pMMG_press.pMMG1B_press;

	/* Append 1st component to sent */
	sprintf(usbTxBufChar, "%lu", data1);

	/* Append 2nd component to sent */
	strcat(usbTxBufChar, splitString);
	memset(strBuf_32bit, '\0', sizeof(strBuf_32bit));
	sprintf(strBuf_32bit, "%.2f", data2);
	strcat(usbTxBufChar, strBuf_32bit);

	/* Append 3rd component to sent */
	strcat(usbTxBufChar, splitString);
	memset(strBuf_32bit, '\0', sizeof(strBuf_32bit));
	sprintf(strBuf_32bit, "%.2f", data3);
	strcat(usbTxBufChar, strBuf_32bit);


	/* Add "\n" in the end of data */
	strcat(usbTxBufChar, newLine);

	sprintf((char*)usbTxBuf, usbTxBufChar);
	usbTxBufSize = strlen(usbTxBufChar);

	usbTxUpdate = 1;
}

static void TerminateUSBCDC_DEMO4(void)
{
	/* Reset the buffers */
	memset(usbTxBuf, 0, sizeof(usbTxBuf));
	memset(usbTxBufChar, 0, sizeof(usbTxBufChar));

	/* Assign the Terminate Data to send through USB CDC FS */
	uint8_t terminateData = USB_CDC_TERMINATE_PYTHON;

	/* Append 1st component to sent */
	sprintf(usbTxBufChar, "%u", terminateData);

	/* Add "\n" in the end of data */
	strcat(usbTxBufChar, newLine);

	sprintf((char*)usbTxBuf, usbTxBufChar);
	usbTxBufSize = strlen(usbTxBufChar);

	usbTxUpdate = 1;
}
#endif





/*--------------------------------------------------------------- SDO CALLBACK ---------------------------------------------------------------*/

static void GetThetaRH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&ReceivedDataObj.theta_RH_scaled, req->data, 2);

	StudentsDataObj.theta_RH = ScaleInt16ToFloat(ReceivedDataObj.theta_RH_scaled, DEG_SCALING_FACTOR);

    res->dataSize = 0;
    res->status = DOP_SDO_SUCC;
}

static void GetThetaLH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&ReceivedDataObj.theta_LH_scaled, req->data, 2);

	StudentsDataObj.theta_LH = ScaleInt16ToFloat(ReceivedDataObj.theta_LH_scaled, DEG_SCALING_FACTOR);

    res->dataSize = 0;
    res->status = DOP_SDO_SUCC;
}

static void GetAccXRH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&ReceivedDataObj.accX_RH_scaled, req->data, 2);

	StudentsDataObj.accX_RH = ScaleInt16ToFloat(ReceivedDataObj.accX_RH_scaled, ACC_SCALING_FACTOR);

    res->dataSize = 0;
    res->status = DOP_SDO_SUCC;
}

static void GetAccXLH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&ReceivedDataObj.accX_LH_scaled, req->data, 2);

	StudentsDataObj.accX_LH = ScaleInt16ToFloat(ReceivedDataObj.accX_LH_scaled, ACC_SCALING_FACTOR);

    res->dataSize = 0;
    res->status = DOP_SDO_SUCC;
}

static void GetAccYRH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&ReceivedDataObj.accY_RH_scaled, req->data, 2);

	StudentsDataObj.accY_RH = ScaleInt16ToFloat(ReceivedDataObj.accY_RH_scaled, ACC_SCALING_FACTOR);

    res->dataSize = 0;
    res->status = DOP_SDO_SUCC;
}

static void GetAccYLH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&ReceivedDataObj.accY_LH_scaled, req->data, 2);

	StudentsDataObj.accY_LH = ScaleInt16ToFloat(ReceivedDataObj.accY_LH_scaled, ACC_SCALING_FACTOR);

    res->dataSize = 0;
    res->status = DOP_SDO_SUCC;
}




/*----------------------------------------------------- P Vector Setting -----------------------------------------------------*/
static void Set_P_Vector_MotionMap(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&MotionMap_File.MM_idx , req->data, 1);		// cnt means "(MS_ID)-1"

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_P_Vector_Yd_RH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&MotionMap_File.MS[MotionMap_File.MM_idx].MD[0].p_vector_decoder.p_buffer[MotionMap_File.MS[MotionMap_File.MM_idx].MD[0].p_vector_decoder.N].yd , req->data, 2);

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_P_Vector_Yd_LH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&MotionMap_File.MS[MotionMap_File.MM_idx].MD[1].p_vector_decoder.p_buffer[MotionMap_File.MS[MotionMap_File.MM_idx].MD[1].p_vector_decoder.N].yd , req->data, 2);

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_P_Vector_L_RH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&MotionMap_File.MS[MotionMap_File.MM_idx].MD[0].p_vector_decoder.p_buffer[MotionMap_File.MS[MotionMap_File.MM_idx].MD[0].p_vector_decoder.N].L , req->data, 2);

	MotionMap_File.MS[MotionMap_File.MM_idx].MD[0].p_vector_decoder.durationCompleted = 0;

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_P_Vector_L_LH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&MotionMap_File.MS[MotionMap_File.MM_idx].MD[1].p_vector_decoder.p_buffer[MotionMap_File.MS[MotionMap_File.MM_idx].MD[1].p_vector_decoder.N].L , req->data, 2);

	MotionMap_File.MS[MotionMap_File.MM_idx].MD[1].p_vector_decoder.durationCompleted = 0;

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_P_Vector_S0_RH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&MotionMap_File.MS[MotionMap_File.MM_idx].MD[0].p_vector_decoder.p_buffer[MotionMap_File.MS[MotionMap_File.MM_idx].MD[0].p_vector_decoder.N].s0 , req->data, 1);

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_P_Vector_S0_LH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&MotionMap_File.MS[MotionMap_File.MM_idx].MD[1].p_vector_decoder.p_buffer[MotionMap_File.MS[MotionMap_File.MM_idx].MD[1].p_vector_decoder.N].s0 , req->data, 1);

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_P_Vector_Sd_RH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&MotionMap_File.MS[MotionMap_File.MM_idx].MD[0].p_vector_decoder.p_buffer[MotionMap_File.MS[MotionMap_File.MM_idx].MD[0].p_vector_decoder.N].sd , req->data, 1);

	MotionMap_File.MS[MotionMap_File.MM_idx].MD[0].p_vector_decoder.N++;

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}

static void Set_P_Vector_Sd_LH(DOP_SDOArgs_t* req, DOP_SDOArgs_t* res)
{
	memcpy(&MotionMap_File.MS[MotionMap_File.MM_idx].MD[1].p_vector_decoder.p_buffer[MotionMap_File.MS[MotionMap_File.MM_idx].MD[1].p_vector_decoder.N].sd , req->data, 1);

	MotionMap_File.MS[MotionMap_File.MM_idx].MD[1].p_vector_decoder.N++;

	res->dataSize = 0;
	res->status = DOP_SDO_SUCC;
}





/*--------------------------------------------------------------- P_vector Decoder ---------------------------------------------------------------*/
static int Ent_P_Vector_Decoder_RH()
{
	memset(&pvectorObj_RH, 0, sizeof(P_Vector_Decoder));
	pvectorObj_RH.yd_f = StudentsDataObj.theta_RH;

	return 0;
}

static int Run_P_Vector_Decoder_RH()
{
	if ((pvectorObj_RH.N > 0) && (pvectorObj_RH.ON == 0)) {
		// FIFO process
		// double yd = (double)pvectorObj_RH.p_buffer[0].yd*0.002;
		// double y0 = pvectorObj_RH.yd_f;
		// double s0 =  (double)pvectorObj_RH.p_buffer[0].s0;
		// double sd = -(double)pvectorObj_RH.p_buffer[0].sd;
		double yd = (double)pvectorObj_RH.p_buffer[0].yd*0.002*0.1;
		double y0 = (double)pvectorObj_RH.yd_f;
		double s0 = (double)pvectorObj_RH.p_buffer[0].s0*0.1;
		double sd = -(double)pvectorObj_RH.p_buffer[0].sd*0.1;
		pvectorObj_RH.yd_f = yd;

		double e  = yd - y0;

		pvectorObj_RH.a0 = y0;
		pvectorObj_RH.a2 =                 0.5*s0 * e;
		pvectorObj_RH.a3 = (10 - 1.5*s0 + 0.5*sd) * e;
		pvectorObj_RH.a4 =    (-15 + 1.5*s0 - sd) * e;
		pvectorObj_RH.a5 =  (6 - 0.5*s0 + 0.5*sd) * e;

		pvectorObj_RH.L     = (float)pvectorObj_RH.p_buffer[0].L;
		if (pvectorObj_RH.L != 0) {
			pvectorObj_RH.L_inv = 1/pvectorObj_RH.L;
		} else {
			// TODO: Handle division by zero
			pvectorObj_RH.L_inv = 0.0;
		}

		pvectorObj_RH.count = 0;
		pvectorObj_RH.ON    = 1;

		if (pvectorObj_RH.N == 1) {
			y0 = (double)pvectorObj_RH.p_buffer[0].yd * 0.002;
			pvectorObj_RH.b0 = y0;
			pvectorObj_RH.b2 = 0;
			pvectorObj_RH.b3 = 0;
			pvectorObj_RH.b4 = 0;
			pvectorObj_RH.b5 = 0;

		} else if (pvectorObj_RH.N > 1) {
			yd = (double)pvectorObj_RH.p_buffer[1].yd * 0.002;
			y0 = (double)pvectorObj_RH.p_buffer[0].yd * 0.002;
			s0 =  (double)pvectorObj_RH.p_buffer[1].s0;
			sd = -(double)pvectorObj_RH.p_buffer[1].sd;
			e  = yd - y0;

			pvectorObj_RH.b0 = y0;
			pvectorObj_RH.b2 =                 0.5*s0 * e;
			pvectorObj_RH.b3 = (10 - 1.5*s0 + 0.5*sd) * e;
			pvectorObj_RH.b4 =    (-15 + 1.5*s0 - sd) * e;
			pvectorObj_RH.b5 =  (6 - 0.5*s0 + 0.5*sd) * e;
		}
	}

	if (pvectorObj_RH.ON == 1) {
		pvectorObj_RH.t1 = (float)pvectorObj_RH.count * pvectorObj_RH.L_inv;	// tau^1
		if (pvectorObj_RH.t1 > 1) pvectorObj_RH.t1 = 1;					// saturation for safety
		pvectorObj_RH.t2 = pvectorObj_RH.t1 * pvectorObj_RH.t1;               // tau^2
		pvectorObj_RH.t3 = pvectorObj_RH.t2 * pvectorObj_RH.t1;               // tau^3
		pvectorObj_RH.t4 = pvectorObj_RH.t3 * pvectorObj_RH.t1;               // tau^4
		pvectorObj_RH.t5 = pvectorObj_RH.t4 * pvectorObj_RH.t1;               // tau^5

		posCtrl_RH.ref = pvectorObj_RH.a0 + //
				pvectorObj_RH.a2*pvectorObj_RH.t2 + //
				pvectorObj_RH.a3*pvectorObj_RH.t3 + //
				pvectorObj_RH.a4*pvectorObj_RH.t4 + //
				pvectorObj_RH.a5*pvectorObj_RH.t5;  //

		if (pvectorObj_RH.count < (pvectorObj_RH.L - 2)) {
			double t_t1 = 0.0, t_t2 = 0.0, t_t3 = 0.0, t_t4 = 0.0, t_t5 = 0.0;
			t_t1 = ((double)pvectorObj_RH.count + 1) * pvectorObj_RH.L_inv;
			if (t_t1 > 1) t_t1 = 1;
			t_t2 = t_t1 * t_t1;
			t_t3 = t_t2 * t_t1;
			t_t4 = t_t3 * t_t1;
			t_t5 = t_t4 * t_t1;

			posCtrl_RH.ref1 = pvectorObj_RH.a0 + //
					pvectorObj_RH.a2 * t_t2 + //
					pvectorObj_RH.a3 * t_t3 + //
					pvectorObj_RH.a4 * t_t4 + //
					pvectorObj_RH.a5 * t_t5;  //

			t_t1 = ((double)pvectorObj_RH.count + 2) * pvectorObj_RH.L_inv;
			if (t_t1 > 1) t_t1 = 1;
			t_t2 = t_t1 * t_t1;
			t_t3 = t_t2 * t_t1;
			t_t4 = t_t3 * t_t1;
			t_t5 = t_t4 * t_t1;

			posCtrl_RH.ref2 = pvectorObj_RH.a0 + //
					pvectorObj_RH.a2 * t_t2 + //
					pvectorObj_RH.a3 * t_t3 + //
					pvectorObj_RH.a4 * t_t4 + //
					pvectorObj_RH.a5 * t_t5;  //
		} else if (pvectorObj_RH.count == (pvectorObj_RH.L - 2)) {
			double t_t1 = 0.0, t_t2 = 0.0, t_t3 = 0.0, t_t4 = 0.0, t_t5 = 0.0;
			t_t1 = ((double)pvectorObj_RH.count + 1) * pvectorObj_RH.L_inv;
			if (t_t1 > 1) t_t1 = 1;
			t_t2 = t_t1 * t_t1;
			t_t3 = t_t2 * t_t1;
			t_t4 = t_t3 * t_t1;
			t_t5 = t_t4 * t_t1;

			posCtrl_RH.ref1 = pvectorObj_RH.a0 + //
					pvectorObj_RH.a2 * t_t2 + //
					pvectorObj_RH.a3 * t_t3 + //
					pvectorObj_RH.a4 * t_t4 + //
					pvectorObj_RH.a5 * t_t5;  //

			posCtrl_RH.ref2 = (double)pvectorObj_RH.p_buffer[0].yd * 0.002;

		} else if (pvectorObj_RH.count == (pvectorObj_RH.L - 1)) {
			posCtrl_RH.ref1 = (double)pvectorObj_RH.p_buffer[0].yd * 0.002;

			double t_t1 = 0.0, t_t2 = 0.0, t_t3 = 0.0, t_t4 = 0.0, t_t5 = 0.0;
			if (pvectorObj_RH.p_buffer[1].L != 0) {
				t_t1 = (double)1 / pvectorObj_RH.p_buffer[1].L;
			} else {
				// TODO: Handle division by zero
				t_t1 = 0.0;
			}

			if (t_t1 > 1) t_t1 = 1;
			t_t2 = t_t1 * t_t1;
			t_t3 = t_t2 * t_t1;
			t_t4 = t_t3 * t_t1;
			t_t5 = t_t4 * t_t1;

			posCtrl_RH.ref2 = pvectorObj_RH.b0 + //
					pvectorObj_RH.b2 * t_t2 + //
					pvectorObj_RH.b3 * t_t3 + //
					pvectorObj_RH.b4 * t_t4 + //
					pvectorObj_RH.b5 * t_t5;  //
		}

		pvectorObj_RH.count++;

		if (pvectorObj_RH.count >= pvectorObj_RH.L) {
			pvectorObj_RH.ON = 0;
			pvectorObj_RH.count  = 0;

			// FIFO shifting
			for (int i = 0; i < pvectorObj_RH.N; i++) {
				memcpy(&pvectorObj_RH.p_buffer[i], &pvectorObj_RH.p_buffer[i+1], sizeof(P_Vector));
			}
			pvectorObj_RH.N--;

			// TODO : Send Msg Duration Completed!!
			pvectorObj_RH.durationCompleted = 1;
			SendDurationCompletedFlag_RH();
		}
	}

	return 0;
}

static int Ext_P_Vector_Decoder_RH()
{
	for (int i = 0; i < P_VECTOR_BUFF_SIZE; ++i) {
		pvectorObj_RH.p_buffer[i].yd = 0;
		pvectorObj_RH.p_buffer[i].L = 0;
		pvectorObj_RH.p_buffer[i].s0 = 0;
		pvectorObj_RH.p_buffer[i].sd = 0;
	}
	return 0;
}




static int Ent_P_Vector_Decoder_LH()
{
	totalDataObj.fvector_trigger_1 = 1;

	memset(&pvectorObj_LH, 0, sizeof(P_Vector_Decoder));
	pvectorObj_LH.yd_f = StudentsDataObj.theta_LH;

	return 0;
}

static int Run_P_Vector_Decoder_LH()
{
	if ((pvectorObj_LH.N > 0) && (pvectorObj_LH.ON == 0)) {
		// FIFO process
		// double yd = (double)pvectorObj_LH.p_buffer[0].yd*0.002;
		// double y0 = pvectorObj_LH.yd_f;
		// double s0 =  (double)pvectorObj_LH.p_buffer[0].s0;
		// double sd = -(double)pvectorObj_LH.p_buffer[0].sd;
		double yd = (double)pvectorObj_LH.p_buffer[0].yd*0.002*0.1;
		double y0 = (double)pvectorObj_LH.yd_f;
		double s0 = (double)pvectorObj_LH.p_buffer[0].s0*0.1;
		double sd = -(double)pvectorObj_LH.p_buffer[0].sd*0.1;
		pvectorObj_LH.yd_f = yd;

		double e  = yd - y0;

		pvectorObj_LH.a0 = y0;
		pvectorObj_LH.a2 =                 0.5*s0 * e;
		pvectorObj_LH.a3 = (10 - 1.5*s0 + 0.5*sd) * e;
		pvectorObj_LH.a4 =    (-15 + 1.5*s0 - sd) * e;
		pvectorObj_LH.a5 =  (6 - 0.5*s0 + 0.5*sd) * e;

		pvectorObj_LH.L     = (float)pvectorObj_LH.p_buffer[0].L;
		if (pvectorObj_LH.L != 0) {
			pvectorObj_LH.L_inv = 1/pvectorObj_LH.L;
		} else {
			// TODO: Handle division by zero
			pvectorObj_LH.L_inv = 0.0;
		}

		pvectorObj_LH.count = 0;
		pvectorObj_LH.ON    = 1;

		if (pvectorObj_LH.N == 1) {
			y0 = (double)pvectorObj_LH.p_buffer[0].yd * 0.002;
			pvectorObj_LH.b0 = y0;
			pvectorObj_LH.b2 = 0;
			pvectorObj_LH.b3 = 0;
			pvectorObj_LH.b4 = 0;
			pvectorObj_LH.b5 = 0;

		} else if (pvectorObj_LH.N > 1) {
			yd = (double)pvectorObj_LH.p_buffer[1].yd * 0.002;
			y0 = (double)pvectorObj_LH.p_buffer[0].yd * 0.002;
			s0 =  (double)pvectorObj_LH.p_buffer[1].s0;
			sd = -(double)pvectorObj_LH.p_buffer[1].sd;
			e  = yd - y0;

			pvectorObj_LH.b0 = y0;
			pvectorObj_LH.b2 =                 0.5*s0 * e;
			pvectorObj_LH.b3 = (10 - 1.5*s0 + 0.5*sd) * e;
			pvectorObj_LH.b4 =    (-15 + 1.5*s0 - sd) * e;
			pvectorObj_LH.b5 =  (6 - 0.5*s0 + 0.5*sd) * e;
		}
	}

	if (pvectorObj_LH.ON == 1) {
		pvectorObj_LH.t1 = (float)pvectorObj_LH.count * pvectorObj_LH.L_inv;	// tau^1
		if (pvectorObj_LH.t1 > 1) pvectorObj_LH.t1 = 1;					// saturation for safety
		pvectorObj_LH.t2 = pvectorObj_LH.t1 * pvectorObj_LH.t1;               // tau^2
		pvectorObj_LH.t3 = pvectorObj_LH.t2 * pvectorObj_LH.t1;               // tau^3
		pvectorObj_LH.t4 = pvectorObj_LH.t3 * pvectorObj_LH.t1;               // tau^4
		pvectorObj_LH.t5 = pvectorObj_LH.t4 * pvectorObj_LH.t1;               // tau^5

		posCtrl_LH.ref = pvectorObj_LH.a0 + //
				pvectorObj_LH.a2*pvectorObj_LH.t2 + //
				pvectorObj_LH.a3*pvectorObj_LH.t3 + //
				pvectorObj_LH.a4*pvectorObj_LH.t4 + //
				pvectorObj_LH.a5*pvectorObj_LH.t5;  //

		if (pvectorObj_LH.count < (pvectorObj_LH.L - 2)) {
			double t_t1 = 0.0, t_t2 = 0.0, t_t3 = 0.0, t_t4 = 0.0, t_t5 = 0.0;
			t_t1 = ((double)pvectorObj_LH.count + 1) * pvectorObj_LH.L_inv;
			if (t_t1 > 1) t_t1 = 1;
			t_t2 = t_t1 * t_t1;
			t_t3 = t_t2 * t_t1;
			t_t4 = t_t3 * t_t1;
			t_t5 = t_t4 * t_t1;

			posCtrl_LH.ref1 = pvectorObj_LH.a0 + //
					pvectorObj_LH.a2 * t_t2 + //
					pvectorObj_LH.a3 * t_t3 + //
					pvectorObj_LH.a4 * t_t4 + //
					pvectorObj_LH.a5 * t_t5;  //

			t_t1 = ((double)pvectorObj_LH.count + 2) * pvectorObj_LH.L_inv;
			if (t_t1 > 1) t_t1 = 1;
			t_t2 = t_t1 * t_t1;
			t_t3 = t_t2 * t_t1;
			t_t4 = t_t3 * t_t1;
			t_t5 = t_t4 * t_t1;

			posCtrl_LH.ref2 = pvectorObj_LH.a0 + //
					pvectorObj_LH.a2 * t_t2 + //
					pvectorObj_LH.a3 * t_t3 + //
					pvectorObj_LH.a4 * t_t4 + //
					pvectorObj_LH.a5 * t_t5;  //
		} else if (pvectorObj_LH.count == (pvectorObj_LH.L - 2)) {
			double t_t1 = 0.0, t_t2 = 0.0, t_t3 = 0.0, t_t4 = 0.0, t_t5 = 0.0;
			t_t1 = ((double)pvectorObj_LH.count + 1) * pvectorObj_LH.L_inv;
			if (t_t1 > 1) t_t1 = 1;
			t_t2 = t_t1 * t_t1;
			t_t3 = t_t2 * t_t1;
			t_t4 = t_t3 * t_t1;
			t_t5 = t_t4 * t_t1;

			posCtrl_LH.ref1 = pvectorObj_LH.a0 + //
					pvectorObj_LH.a2 * t_t2 + //
					pvectorObj_LH.a3 * t_t3 + //
					pvectorObj_LH.a4 * t_t4 + //
					pvectorObj_LH.a5 * t_t5;  //

			posCtrl_LH.ref2 = (double)pvectorObj_LH.p_buffer[0].yd * 0.002;

		} else if (pvectorObj_LH.count == (pvectorObj_LH.L - 1)) {
			posCtrl_LH.ref1 = (double)pvectorObj_LH.p_buffer[0].yd * 0.002;

			double t_t1 = 0.0, t_t2 = 0.0, t_t3 = 0.0, t_t4 = 0.0, t_t5 = 0.0;
			if (pvectorObj_LH.p_buffer[1].L != 0) {
				t_t1 = (double)1 / pvectorObj_LH.p_buffer[1].L;
			} else {
				// TODO: Handle division by zero
				t_t1 = 0.0;
			}

			if (t_t1 > 1) t_t1 = 1;
			t_t2 = t_t1 * t_t1;
			t_t3 = t_t2 * t_t1;
			t_t4 = t_t3 * t_t1;
			t_t5 = t_t4 * t_t1;

			posCtrl_LH.ref2 = pvectorObj_LH.b0 + //
					pvectorObj_LH.b2 * t_t2 + //
					pvectorObj_LH.b3 * t_t3 + //
					pvectorObj_LH.b4 * t_t4 + //
					pvectorObj_LH.b5 * t_t5;  //
		}

		pvectorObj_LH.count++;

		if (pvectorObj_LH.count >= pvectorObj_LH.L) {
			pvectorObj_LH.ON = 0;
			pvectorObj_LH.count  = 0;

			// FIFO shifting
			for (int i = 0; i < pvectorObj_LH.N; i++) {
				memcpy(&pvectorObj_LH.p_buffer[i], &pvectorObj_LH.p_buffer[i+1], sizeof(P_Vector));
			}
			pvectorObj_LH.N--;

			// TODO : Send Msg Duration Completed!!
			pvectorObj_LH.durationCompleted = 1;
			SendDurationCompletedFlag_LH();
		}
	}

	return 0;
}

static int Ext_P_Vector_Decoder_LH()
{
	totalDataObj.fvector_trigger_1 = 0;

	for (int i = 0; i < P_VECTOR_BUFF_SIZE; ++i) {
		pvectorObj_LH.p_buffer[i].yd = 0;
		pvectorObj_LH.p_buffer[i].L = 0;
		pvectorObj_LH.p_buffer[i].s0 = 0;
		pvectorObj_LH.p_buffer[i].sd = 0;
	}

	return 0;
}


