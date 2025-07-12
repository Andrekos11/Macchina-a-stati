/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * if no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "String.h"
#include "math.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CAN_RX_BUFFER_SIZE 16
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim11;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM11_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
typedef struct
{
	uint32_t BASE_ID_NODE;
	//qui vanno definiti una serie di header(sia in scrittura che in lettura) che si identificano con ogni tipo di messaggio canopen base
	CAN_TxHeaderTypeDef Tx_STD_Header_NMT;//header di un generico messaggio inviato
	//CAN_TxHeaderTypeDef Tx_STD_Header_SDO;
	CAN_TxHeaderTypeDef Tx_STD_Header_PDO[4];
	//lista dei corpi messaggio per ogni classe base di messaggio canopen
	uint8_t TxData_PDO[4][8];
	uint8_t TxData_NMT[8];
	//contenitori in cui butto i dati ricevuti, in base ovviamente ai contenitori fifo di ricezione che seguono lo smistamento dato dai filtri
	CAN_RxHeaderTypeDef RxHeader_node[CAN_RX_BUFFER_SIZE];
	uint8_t RxData_node[CAN_RX_BUFFER_SIZE][8];

	uint8_t buffer_shift_refister;//scorre dentro la interrupt di ricezione dei messaggi can
	uint8_t buffer_shift_refister_main;//scorre nel main
	int16_t NMT_slave_feadback;//feedback  0:boot-up,7f:preoperational,05:operational,04:stopped
	int16_t NMT_slave_state;//-1:per transizioni, poi ...  0:boot-up,7f:preoperational,05:operational,04:stopped
	int16_t NMT_slave_state_requested;//0:boot-up,7f:preoperational,05:operational,04:stopped
	int16_t NMT_Writer;//output al motore    80,81,82,1,2

	int8_t Hight_Level_State;//:SINT =-1-1:stato di setup mio iniziale,0:disbled,1:power enabled,2:fault

	int8_t SubStates_Power_Disabled;//:SINT=-1 -1:not in this state(/gate on entry e on exit),0:switch on disabled, 1:ready to switch on

	int8_t SubStates_Fault;//:SINT=-1 -1:not in this state,0:Fault reaction active,1:fault

	int8_t SubStates_Power_Enabled;//:SINT=-1 -1:not in this state,0:switched on,1:operation enabled,2:quick stop active

	char state_Drive_string[50];
	char state_NMT_string[50];
	uint16_t StatusWord_app;
	bool StatusWord_bool[16];
	uint16_t ControlWord_app;
	bool ControlWord_bool[16];
	uint32_t Encoder_actual_position;
	uint8_t Touch_probe_status;//valgono solo i primi 3 bit, sono delle bandiere che mi dicono se sono state memorizzate delle posizioni nei touch probe, ovvero nei lumiti dx e sx
	uint8_t Mode_of_operation;
	uint32_t Actual_Position_encoder;
	uint32_t Actual_Current;
	uint32_t Actual_Velocity_encoder;
	uint32_t Speed_search_for_zero;
	uint32_t Memorized_position_positive;
	uint32_t Memorized_position_negative;

	uint16_t Error_code;
	uint32_t Digital_inputs;
	//mancano gli errori......solo se hai tempo.
	//bool Node_rx_msg_pending;

	//ora la sezione dedicata agli eventi
	//NMT Master-Slave
	//ogniuno di questi comandi verrà preceduto da un catattere maiuscolo S o T che insicherà a quale dei due motori è diretto,
	bool Enter_Pre_Operational;//a e a
	bool Reset_Communication;//b
	bool Reset_Node;//c
	bool Start_Remote_Node;//d
	bool Stop_Remote_Node;//e
	bool Transition_Drive_requested;//not a comand


	bool Shutdown;//f
	bool Switch_On;//g
	bool Disable_voltage;//h
	bool Quick_Stop;//i
	bool Disable_operation;//j
	bool Enable_Operation;//k
	bool Fault_Reset;//l
	uint8_t TxBuffer[8];//buffer di ricezione per i messaggi da usart2
	uint32_t TxBuffer_Priority[8];

}NODE;

NODE Node[2];

uint8_t RxBuffer[100]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};//


uint32_t TxMailbox[3];//buffer di eleborazione di invio messaggi...sono tre fissi per uscita can
//la TxMailbox0 è destinanta allo sparo degli nmt e del pdo1, il pdo1 potrà essere sparato dal main e in via eccezionale anche dalla callback della usart2, il che vuol dire priorità 1(massima) e 3(minima)

int counter=0;


//contenitori in cui butto i dati ricevuti, in base ovviamente ai contenitori fifo di ricezione che seguono lo smistamento dato dai filtri


 void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1);
 void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan1);
 void HAL_CAN_RxFifo0FullCallback(CAN_HandleTypeDef *hcan1);
 void HAL_CAN_RxFifo1FullCallback(CAN_HandleTypeDef *hcan1);
 void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim11);

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_CAN1_Init();
  MX_TIM11_Init();
  /* USER CODE BEGIN 2 */
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ResetError(&hcan1);
  HAL_UART_Receive_DMA(&huart2, RxBuffer, sizeof(RxBuffer));
      HAL_UART_Receive_IT(&huart2, RxBuffer, sizeof(RxBuffer));
  HAL_TIM_Base_Init(&htim11);
  if (HAL_TIM_Base_Start_IT(&htim11) != HAL_OK)
  {
      Error_Handler();
  }
  Node[0].BASE_ID_NODE=2;//a questo punto si imposta un ciclo che si aspetta che vengano settati i valori dei due nodi o da usart2 o da wifi, poi seguirà un setting automatico di tutti i messaggi predefiniti e valori da azzerare....
  Node[1].BASE_ID_NODE=3;//battezzo pari lo sterzo e dispari la trazione

  for(int i=0;i<2 ;i++){
	  //config nmt
	  //Node[i].TxBuffer={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	  //Node[i].state_Drive_string='stato_non_definito';
	  //Node[i].state_NMT_string='stato_non_definito';
	  Node[i].Speed_search_for_zero=1000;
	  Node[i].Tx_STD_Header_NMT.StdId=0;
	  Node[i].Tx_STD_Header_NMT.ExtId=0;
	  Node[i].Tx_STD_Header_NMT.IDE=CAN_ID_STD;
	  Node[i].Tx_STD_Header_NMT.TransmitGlobalTime = DISABLE;
	  Node[i].Tx_STD_Header_NMT.RTR = CAN_RTR_DATA;
	  Node[i].Tx_STD_Header_NMT.DLC=2;
	  Node[i].TxData_NMT[1]=Node[i].BASE_ID_NODE;
	  Node[i].Tx_STD_Header_PDO[0].StdId=0x200+Node[i].BASE_ID_NODE;
	  Node[i].Tx_STD_Header_PDO[1].StdId=0x300+Node[i].BASE_ID_NODE;
	  Node[i].Tx_STD_Header_PDO[2].StdId=0x400+Node[i].BASE_ID_NODE;
	  Node[i].Tx_STD_Header_PDO[3].StdId=0x500+Node[i].BASE_ID_NODE;

	  Node[i].buffer_shift_refister=0;
	  Node[i].buffer_shift_refister_main=0;

	  	  Node[i].Tx_STD_Header_PDO[0].ExtId=Node[i].Tx_STD_Header_PDO[1].ExtId=Node[i].Tx_STD_Header_PDO[2].ExtId=0;
	  	  Node[i].Tx_STD_Header_PDO[0].IDE=Node[i].Tx_STD_Header_PDO[1].IDE=Node[i].Tx_STD_Header_PDO[2].IDE=CAN_ID_STD;
	  	  Node[i].Tx_STD_Header_PDO[0].TransmitGlobalTime = Node[i].Tx_STD_Header_PDO[1].TransmitGlobalTime=Node[i].Tx_STD_Header_PDO[2].TransmitGlobalTime=DISABLE;
	  	  Node[i].Tx_STD_Header_PDO[0].RTR =Node[i].Tx_STD_Header_PDO[1].RTR =Node[i].Tx_STD_Header_PDO[2].RTR = CAN_RTR_DATA;
	  	  Node[i].Tx_STD_Header_PDO[0].DLC=7;
	  	  Node[i].Tx_STD_Header_PDO[1].DLC=8;
	  	  Node[i].Tx_STD_Header_PDO[2].DLC=8;
	  Node[i].NMT_slave_feadback=0;
	  Node[i].NMT_slave_state=127;
	  Node[i].NMT_Writer=0;
	  Node[i].NMT_slave_state_requested=127;

	  Node[i].Enter_Pre_Operational=false;
	  Node[i].Reset_Communication=false;
	  Node[i].Reset_Node=false;
	  Node[i].Start_Remote_Node=false;
	  Node[i].Stop_Remote_Node=false;

	  Node[i].Shutdown=false;
	  Node[i].Switch_On=false;
	  Node[i].Disable_voltage=false;
	  Node[i].Quick_Stop=false;
	  Node[i].Disable_operation=false;
	  Node[i].Enable_Operation=false;
	  Node[i].Fault_Reset=false;
	  Node[i].Transition_Drive_requested=false;
	  Node[i].Transition_Drive_requested=false;

	  Node[i].Hight_Level_State=-1;
	  Node[i].SubStates_Power_Disabled=-1;
	  Node[i].SubStates_Fault=-1;
	  Node[i].SubStates_Power_Enabled=-1;

	  Node[i].Encoder_actual_position=0;
	  Node[i].Actual_Current=0;
	  for (int j = 0; j < 16; j++){
		  Node[i].StatusWord_bool[j]=0;
		  Node[i].ControlWord_bool[j]=0;
	  }
	  //finire......

  }
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
  //HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FifO0_FULL);
  //HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FifO0_OVERRUN);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);
  //HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FifO1_FULL);
  //HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FifO1_OVERRUN);

  __HAL_RCC_TIM11_CLK_ENABLE();

uint8_t k=0;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  //for(int k=0;k<2;k++){
	  //if i due buffer sono di valore diverso allora autoincremento elaboro il prossimo
	  if(Node[k].buffer_shift_refister!=Node[k].buffer_shift_refister_main){
		  switch (Node[k].RxHeader_node[Node[k].buffer_shift_refister].StdId-(uint32_t)Node[k].BASE_ID_NODE) {
		  		case 0x700://Heartbeat nmt state
		  			Node[k].NMT_slave_feadback=Node[k].RxData_node[Node[k].buffer_shift_refister][0];
		  			Node[k].TxBuffer_Priority[0]=(uint32_t)Node[k].RxData_node[Node[k].buffer_shift_refister][0];
		  			//questo valore viene sparato dal timer
		  			break;
		  		case 0x180://
		  			Node[k].StatusWord_app= ((uint32_t)Node[k].RxData_node[Node[k].buffer_shift_refister][1] << 8)|
		  							((uint32_t)Node[k].RxData_node[Node[k].buffer_shift_refister][0]);


		  			for (int j=0;j<=15;j++){
		  				Node[k].StatusWord_bool[j] = (Node[k].StatusWord_app % 2) == 1;
		  				Node[k].StatusWord_app = Node[k].StatusWord_app / 2;
		  			}
		  			Node[k].Mode_of_operation=(uint8_t)Node[k].RxData_node[Node[k].buffer_shift_refister][2];

		  			Node[k].Touch_probe_status= ((uint16_t)Node[k].RxData_node[Node[k].buffer_shift_refister][5] << 8)|
		  					  					  					((uint16_t)Node[k].RxData_node[Node[k].buffer_shift_refister][4]);
		  					  			Node[k].Error_code= ((uint16_t)Node[k].RxData_node[Node[k].buffer_shift_refister][7] << 8)|
		  					  					  			((uint16_t)Node[k].RxData_node[Node[k].buffer_shift_refister][6]);
		  			Node[k].TxBuffer[0]=Node[k].Hight_Level_State;
		  			Node[k].TxBuffer[1]=Node[k].SubStates_Power_Enabled;
		  			Node[k].TxBuffer[2]=Node[k].SubStates_Power_Disabled;
		  			Node[k].TxBuffer[3]=Node[k].SubStates_Fault;
		  			Node[k].TxBuffer[4]=Node[k].Mode_of_operation;
		  			Node[k].TxBuffer[5]=(uint8_t)Node[k].Touch_probe_status;
		  			Node[k].TxBuffer[6]=Node[k].RxData_node[Node[k].buffer_shift_refister][6];
		  			Node[k].TxBuffer[7]=Node[k].RxData_node[Node[k].buffer_shift_refister][7];//valutare se sparare in usart con DMA o normale
		  			break;
		  		case 0x280:
		  			Node[k].Actual_Position_encoder=((uint32_t)Node[k].RxData_node[Node[k].buffer_shift_refister][3] << 24) |
		                     	 	 	 	 	 	((uint32_t)Node[k].RxData_node[Node[k].buffer_shift_refister][2] << 16) |
													((uint32_t)Node[k].RxData_node[Node[k].buffer_shift_refister][1] << 8)  |
													((uint32_t)Node[k].RxData_node[Node[k].buffer_shift_refister][0]);
		  			Node[k].Actual_Velocity_encoder=((uint32_t)Node[k].RxData_node[Node[k].buffer_shift_refister][7] << 24) |
		  					                     	((uint32_t)Node[k].RxData_node[Node[k].buffer_shift_refister][6] << 16) |
		  											((uint32_t)Node[k].RxData_node[Node[k].buffer_shift_refister][5] << 8)  |
		  											((uint32_t)Node[k].RxData_node[Node[k].buffer_shift_refister][4]);
		  			Node[k].TxBuffer_Priority[1]=Node[k].Actual_Position_encoder;
		  			Node[k].TxBuffer_Priority[2]=Node[k].Actual_Velocity_encoder;
		  			// Ricostruzione del valore dell'encoder a 32 bit (little-endian)

		  			break;
		  		case 0x380://Heartbeat nmt state
		  			Node[k].Digital_inputs=((uint32_t)Node[k].RxData_node[Node[k].buffer_shift_refister][7] << 24) |
	 	 	 	 	 								((uint32_t)Node[k].RxData_node[Node[k].buffer_shift_refister][6] << 16) |
													((uint32_t)Node[k].RxData_node[Node[k].buffer_shift_refister][5] << 8)  |
													((uint32_t)Node[k].RxData_node[Node[k].buffer_shift_refister][4]);
		  			Node[k].Actual_Current=((uint32_t)Node[k].RxData_node[Node[k].buffer_shift_refister][3] << 24) |
		  			     	 	 	 	 	 					((uint32_t)Node[k].RxData_node[Node[k].buffer_shift_refister][2] << 16) |
		  														((uint32_t)Node[k].RxData_node[Node[k].buffer_shift_refister][1] << 8)  |
		  														((uint32_t)Node[k].RxData_node[Node[k].buffer_shift_refister][0]);
		  			Node[k].TxBuffer_Priority[3]=Node[k].Actual_Current;

		  			break;

		  		case 0x480://Heartbeat nmt state
		  			Node[k].Memorized_position_positive=((uint32_t)Node[k].RxData_node[Node[k].buffer_shift_refister][3] << 24) |
	 	 	 	 	 									((uint32_t)Node[k].RxData_node[Node[k].buffer_shift_refister][2] << 16) |
														((uint32_t)Node[k].RxData_node[Node[k].buffer_shift_refister][1] << 8)  |
														((uint32_t)Node[k].RxData_node[Node[k].buffer_shift_refister][0]);

		  			Node[k].Memorized_position_negative=((uint32_t)Node[k].RxData_node[Node[k].buffer_shift_refister][7] << 24) |
		                     							((uint32_t)Node[k].RxData_node[Node[k].buffer_shift_refister][6] << 16) |
														((uint32_t)Node[k].RxData_node[Node[k].buffer_shift_refister][5] << 8)  |
														((uint32_t)Node[k].RxData_node[Node[k].buffer_shift_refister][4]);
		  			Node[k].TxBuffer_Priority[4]=Node[k].Memorized_position_positive;
		  			Node[k].TxBuffer_Priority[5]=Node[k].Memorized_position_negative;
		  			break;

		  	}

		  Node[k].buffer_shift_refister_main++;
		  if(Node[k].buffer_shift_refister_main>=CAN_RX_BUFFER_SIZE) Node[k].buffer_shift_refister_main=0;

	  }
	  //e, sempre se è uguale al valore massimo allora riporta a zero....attento a nion mangiarti il valore 0






		if(Node[k].Shutdown==1 && Node[k].NMT_slave_state==5){
			Node[k].ControlWord_bool[0]=0;
			Node[k].ControlWord_bool[3]=0;
			Node[k].ControlWord_bool[2]=1;
			Node[k].ControlWord_bool[1]=1;
			Node[k].Transition_Drive_requested=1;
			Node[k].Shutdown=0;
		}//per eitare che uno alzi il boleano per sbaglio...metto un controllo non qui ma direttamente sulla callback, è piu codice ma rende piu leggero il main
		if(Node[k].Switch_On==1 && Node[k].NMT_slave_state==5){
			Node[k].ControlWord_bool[0]=1;
			Node[k].ControlWord_bool[1]=1;
			Node[k].ControlWord_bool[2]=1;
			Node[k].ControlWord_bool[3]=0;
			Node[k].Transition_Drive_requested=1;
			Node[k].Switch_On=0;
		}
		if(Node[k].Disable_voltage==1 && Node[k].NMT_slave_state==5){
			Node[k].Disable_voltage=0;
			Node[k].ControlWord_bool[0]=0;
			Node[k].ControlWord_bool[1]=0;
			Node[k].ControlWord_bool[2]=0;
			Node[k].ControlWord_bool[3]=0;
			Node[k].Transition_Drive_requested=1;
		}
		if(Node[k].Quick_Stop==1 && Node[k].NMT_slave_state==5){
			Node[k].ControlWord_bool[0]=0;
			Node[k].ControlWord_bool[1]=1;
			Node[k].ControlWord_bool[2]=0;
			Node[k].ControlWord_bool[3]=0;
			Node[k].Transition_Drive_requested=1;
			Node[k].Quick_Stop=0;
		}
		if(Node[k].Disable_operation==1 && Node[k].NMT_slave_state==5){
			Node[k].ControlWord_bool[0]=1;
			Node[k].ControlWord_bool[1]=1;
			Node[k].ControlWord_bool[2]=1;
			Node[k].ControlWord_bool[3]=0;
			Node[k].Transition_Drive_requested=1;
			Node[k].Disable_operation=0;
		}
		if(Node[k].Enable_Operation==1 && Node[k].NMT_slave_state==5){
			Node[k].ControlWord_bool[0]=1;
			Node[k].ControlWord_bool[1]=1;
			Node[k].ControlWord_bool[2]=1;
			Node[k].ControlWord_bool[3]=1;
			Node[k].Transition_Drive_requested=1;
			Node[k].Enable_Operation=0;
		}


		if(Node[k].Fault_Reset==1 && Node[k].NMT_slave_state==5){
			Node[k].ControlWord_bool[7]=1;
			Node[k].Fault_Reset=0;
		}else if(Node[k].Fault_Reset==0){
			Node[k].ControlWord_bool[7]=0;
		}







	  if(Node[k].Enter_Pre_Operational==1){
		  Node[k].NMT_slave_state_requested=0x7f;
	  	}


	  if(Node[k].Reset_Communication==1){
		  Node[k].NMT_slave_state_requested=0;
	  	}


	  if(Node[k].Reset_Node==1){
		  Node[k].NMT_slave_state_requested=0;

	  	}


	  if(Node[k].Start_Remote_Node==1){
		  Node[k].NMT_slave_state_requested=5;
	  	}


	  if(Node[k].Stop_Remote_Node==1){
		  Node[k].NMT_slave_state_requested=4;
	  	}


//o NMT_slave_feadback
	  if(Node[k].NMT_slave_feadback!=Node[k].NMT_slave_state_requested){
		  Node[k].NMT_slave_state=-1;
	  }


	  switch (Node[k].NMT_slave_state){

	  	case -1:
	  		if(Node[k].NMT_slave_state_requested==Node[k].NMT_slave_feadback){
	  			  			Node[k].NMT_slave_state=Node[k].NMT_slave_feadback;
	  			  			Node[k].NMT_slave_state_requested=Node[k].NMT_slave_state;
	  			  			continue;
	  			  		}
	  		switch (Node[k].NMT_slave_state_requested){
	  			case 0:

	  				switch (Node[k].NMT_slave_feadback){
	  					case 05://1:Reset Communication or Reset Node
	  						if(Node[k].Actual_Velocity_encoder==0 && (Node[k].Hight_Level_State==-1 || Node[k].Hight_Level_State==0 || Node[k].SubStates_Power_Disabled!=-1 || Node[k].Hight_Level_State==2) && Node[k].Reset_Communication==1){
	  							Node[k].NMT_Writer=0x82;
	  							Node[k].Reset_Communication=false;
	  						}
	  						if(Node[k].Actual_Velocity_encoder==0 && (Node[k].Hight_Level_State==-1 || Node[k].Hight_Level_State==0 || Node[k].SubStates_Power_Disabled!=-1 || Node[k].Hight_Level_State==2) && Node[k].Reset_Node==1){
	  							Node[k].NMT_Writer=0x81;
	  							Node[k].Reset_Node=false;
	  						}
	  						break;
	  					default:
	  						if(Node[k].Reset_Communication==1){
	  							Node[k].NMT_Writer=0x82;
	  							Node[k].Reset_Communication=false;
	  						}
	  						if(Node[k].Reset_Node==1){
	  							Node[k].NMT_Writer=0x81;
	  							Node[k].Reset_Node=false;
	  						}
	  				}
	  				break;

	  			case 0x7F://enter pre operational
	  				switch (Node[k].NMT_slave_feadback){

	  					case 05:
	  						if(Node[k].Actual_Velocity_encoder==0 && (Node[k].Hight_Level_State==-1 || Node[k].Hight_Level_State==0 || Node[k].SubStates_Power_Disabled!=-1 || Node[k].Hight_Level_State==2) && Node[k].Enter_Pre_Operational==1){//sta a significare che prima deve essere portato il motore in condizione di sicurezza
	  							Node[k].NMT_Writer=0x80;
	  							Node[k].Enter_Pre_Operational=false;
	  						}
	  						break;
	  					default:
	  						if(Node[k].Enter_Pre_Operational==1){
	  							Node[k].NMT_Writer=0x80;
	  							Node[k].Enter_Pre_Operational=false;
	  						}

	  				}
	  				break;
	  			case 5://enter operational
	  				//impostre controlword tutta uguale a 0
	  				//impostare a 0 la velocità targhet e la posizione targhet
	  				Node[k].SubStates_Power_Enabled=-1;
	  				Node[k].SubStates_Power_Disabled=-1;
	  				Node[k].SubStates_Fault=-1;
	  				Node[k].Hight_Level_State=-1;

	  				switch (Node[k].NMT_slave_feadback){
	  					case 0x7f:
	  						if(Node[k].Start_Remote_Node==1){
	  							Node[k].NMT_Writer=1;
	  							Node[k].Start_Remote_Node=false;
	  						}
	  						break;

	  					case 04:
	  						if(Node[k].Start_Remote_Node==1){
	  							Node[k].NMT_Writer=1;
	  							Node[k].Start_Remote_Node=false;
	  						}
	  						break;

	  				}
	  				break;
	  			case 4:

	  				if(Node[k].Stop_Remote_Node==1){
	  					Node[k].NMT_Writer=2;
	  					Node[k].Stop_Remote_Node=false;
	  				}
	  				break;

	  		}



	  	case 0://
	  		Node[k].NMT_slave_state=-1;
	  		//eccezione...perche in teoria il motore non permane in questo stato per molto e la transizione al preoperazionale è automatica e scatenata da un evento interno
	  		break;
	  	case 0x7f:
	  		//per sviluppi futuri qui si potrebbe impostare un meccanismo di invio e scrittura di messaggi sdo di configurazione, questo per avere un controllo ancora superiore e modificare eventuali parametri interini anche runtime, senza dover collegarci al motore e farlo tramite epos studio


	  		break;
	  	case 04:


	  		break;
	  	case 05:
	  		if((Node[k].StatusWord_bool[0]==1 && Node[k].StatusWord_bool[1]==1 && Node[k].StatusWord_bool[2]==1 && Node[k].StatusWord_bool[3]==1 && Node[k].StatusWord_bool[5]==0 && Node[k].StatusWord_bool[6]==0 && Node[k].Hight_Level_State!=2) || (Node[k].StatusWord_bool[0]==0 && Node[k].StatusWord_bool[1]==0 && Node[k].StatusWord_bool[2]==0 && Node[k].StatusWord_bool[3]==1 && Node[k].StatusWord_bool[5]==0 && Node[k].StatusWord_bool[6]==0 && Node[k].Hight_Level_State!=2)){
	  		switch (Node[k].Hight_Level_State) {
	  			case -1:
	  				Node[k].SubStates_Power_Enabled=-1;
	  				Node[k].SubStates_Power_Disabled=-1;
	  				Node[k].SubStates_Fault=0;
	  				Node[k].Hight_Level_State=2;
	  				break;
	  			case 0:
	  				Node[k].SubStates_Power_Disabled=-1;//se si era in power enabled si passa per il gatewat in uscita del power enabled
	  				Node[k].SubStates_Fault=0;// o 1 vediamo....
	  				break;
	  			case 1:
	  				Node[k].SubStates_Power_Enabled=-1;//se si era in power enabled si passa per il gatewat in uscita del power enabled
	  				Node[k].SubStates_Fault=0;// o 1 vediamo....
	  				break;
	  		}
	  	}
	  	//sate machine that follows the changings { the status word and track whether the transition required from the event is compatibke with the actual driver state and mode { operation
	  	switch (Node[k].Hight_Level_State) {
	  		case -1:

	  			//sfruttare questo stato per saltare alcune transizioni automatiche appena si entra in operazionale con gli NMT slave, torniamo qui infatti ogni volta che il motore deve RIENTRARE in operational_NMT
	  			//ci si aspetta direttamente che la status word assuma i valori aspettati con controlwort preimpostata di default a 0....0 ad ogni entry in operational, dopo di che si parte con il monitoring della macchina a stati dell'azionamento


	  			//if (StatusWord_bool[0]==FALSE && StatusWord_bool[1]==FALSE && StatusWord_bool[2]==FALSE && StatusWord_bool[3]==FALSE &&  StatusWord_bool[6]==TRUE){
	  				Node[k].Hight_Level_State=0;
	  				Node[k].SubStates_Power_Disabled=-1;
	  				Node[k].SubStates_Fault=-1;
	  				Node[k].SubStates_Power_Enabled=-1;

	  			//}
	  			break;

	  		case 0://SubStates_Power_Disabled
	  			switch (Node[k].SubStates_Power_Disabled) {
	  				case -1://on entry e on exit di power state disabled e condizione di not in this state

	  					if (Node[k].StatusWord_bool[0]==0 && Node[k].StatusWord_bool[1]==0 && Node[k].StatusWord_bool[2]==0 && Node[k].StatusWord_bool[3]==0 &&  Node[k].StatusWord_bool[6]==1) {//StatusWord_bool[5]=TRUE &&
	  						//terminazione a switch on disabled
	  						//Node[k].state_motor_string='switch_on_disabled';
	  						Node[k].SubStates_Power_Disabled=0;
	  					}else if(Node[k].StatusWord_bool[0]==1 && Node[k].StatusWord_bool[1]==0 && Node[k].StatusWord_bool[2]==0 && Node[k].StatusWord_bool[3]==0 && Node[k].StatusWord_bool[5]==1 && Node[k].StatusWord_bool[6]==0) {
	  						//terminazione a ready to switch on
	  						//Node[k].state_motor_string='ready_to_switch_on';
	  						Node[k].SubStates_Power_Disabled=1;
	  					}else{
	  						Node[k].Hight_Level_State=1;
	  						Node[k].SubStates_Power_Enabled=-1;
	  					}

	  					if(Node[k].SubStates_Fault!=-1){
	  						Node[k].Hight_Level_State=2;
	  						//e poi forse altre cose
	  					}
	  					break;

	  				case 0://switch on disabled si puo uscire solo cun uno shutdown che scatena la transizione 2 verso ready to switch on


	  					break;

	  				case 1:	//ready to switch on che puo transitare con 3-switch on e 7
	  					//7 transizione possibile sia con un coman{ di Disable_voltage  che di Quick_Stop


	  					break;


	  			}
	  			break;
	  		case 2://SubStates_Fault
	  			switch (Node[k].SubStates_Fault) {

	  				case 0:
	  					if(Node[k].StatusWord_bool[0]==0 && Node[k].StatusWord_bool[1]==0 && Node[k].StatusWord_bool[2]==0 && Node[k].StatusWord_bool[3]==1 && Node[k].StatusWord_bool[5]==0 && Node[k].StatusWord_bool[6]==0){
	  						Node[k].SubStates_Fault=1;
	  					}
	  					break;

	  				case 1:
	  					if (Node[k].StatusWord_bool[0]==0 && Node[k].StatusWord_bool[1]==0 && Node[k].StatusWord_bool[2]==0 && Node[k].StatusWord_bool[3]==0 && Node[k].StatusWord_bool[6]==1) {
	  						Node[k].Hight_Level_State=0;
	  						//e poi forse altre cose
	  						Node[k].SubStates_Power_Disabled=-1;
	  					}
	  					break;



	  			}
	  			break;

	  		case 1://SubStates_Power_Enabled
	  			switch (Node[k].SubStates_Power_Enabled) {
	  				case -1:
	  					if (Node[k].StatusWord_bool[0]==1 && Node[k].StatusWord_bool[1]==1 && Node[k].StatusWord_bool[2]==1 && Node[k].StatusWord_bool[3]==0 && Node[k].StatusWord_bool[5]==1 && Node[k].StatusWord_bool[6]==0) {
	  						//terminazione a operation enabled
	  						//Node[k].state_motor_string='operation enabled';
	  						Node[k].SubStates_Power_Enabled=1;
	  					}else if(Node[k].StatusWord_bool[0]==1 && Node[k].StatusWord_bool[1]==1 && Node[k].StatusWord_bool[2]==0 && Node[k].StatusWord_bool[3]==0 && Node[k].StatusWord_bool[5]==1 && Node[k].StatusWord_bool[6]==0) {
	  						Node[k].SubStates_Power_Enabled=0;//terminazione a switched on
	  						//Node[k].state_motor_string='switched_on';
	  					}else if(Node[k].StatusWord_bool[0]==1 && Node[k].StatusWord_bool[1]==1 && Node[k].StatusWord_bool[2]==1 && Node[k].StatusWord_bool[3]==0 && Node[k].StatusWord_bool[5]==0 && Node[k].StatusWord_bool[6]==0){
	  						Node[k].SubStates_Power_Enabled=2;//terminazione a quick stop active
	  						//Node[k].state_motor_string='quick_stop_active';
	  					}else{
	  						Node[k].Hight_Level_State=0;
	  						//e poi forse altre cose
	  						Node[k].SubStates_Power_Disabled=-1;
	  					}


	  					if(Node[k].SubStates_Fault!=-1){
	  						Node[k].Hight_Level_State=2;
	  						//e poi forse altre cose
	  						//state_motor_string='Power_disabled';
	  					}
	  					break;

	  					/*if (StatusWord_bool[0]=TRUE && StatusWord_bool[1]=TRUE && StatusWord_bool[2]=TRUE && StatusWord_bool[3]=FALSE && StatusWord_bool[5]=TRUE && StatusWord_bool[6]=FALSE) {

	  						switch Mode_Of_Operation_IN {
	  							case 8://csp
	  								if(fbMyTimer.Q=FALSE){
	  										//qualsiasi richiesta di scrittura di posizione la annullo
	  									ELSE
	  										fbMyTimer(IN=FALSE);
	  										SubStates_Power_Enabled=1;//terminazione a Operation Enabled
	  								}
	  							case 9:
	  								if(fbMyTimer.Q=FALSE){
	  										//qualsiasi richiesta di scrittura di velocità la annullo
	  									ELSE
	  										fbMyTimer(IN=FALSE);
	  										SubStates_Power_Enabled=1;//terminazione a Operation Enabled
	  								}
	  							case 0:
	  								SubStates_Power_Enabled=1;//terminazione a Operation Enabled
	  							case 1:
	  								SubStates_Power_Enabled=1;//terminazione a Operation Enabled
	  							case 6:
	  								SubStates_Power_Enabled=1;//terminazione a Operation Enabled
	  						}
	  						first_transition_to_operation_enabled=FALSE;
	  					}
	  					*/




	  				case 0://switched on , possibili le transizioni in uscita 4,6



	  					break;


	  				case 1://full operational state/ operation enable, possibili le transizioni in uscita 5,8,9,11



	  					break;

	  				case 2://quick stop active, possibili le transizioni in uscita 16(ci sono degli ulteriori congtrolli ma quelli si faranno in seguito)
	  					/*(*
	  					if (StatusWord_bool[0]=FALSE && StatusWord_bool[1]=FALSE && StatusWord_bool[2]=FALSE && StatusWord_bool[3]=FALSE &&  StatusWord_bool[6]=TRUE OR( StatusWord_bool[0]=FALSE && StatusWord_bool[1]=FALSE && StatusWord_bool[2]=FALSE && StatusWord_bool[3]=FALSE && StatusWord_bool[6]=TRUE)) {
	  						SubStates_Power_Enabled=-1;//terminazione a switch on disabled passan{ dal gate di uscita del power enabled
	  						SubStates_Power_Disabled=0;
	  					}*)*/

	  					/*if(StatusWord_bool[0]=FALSE && StatusWord_bool[1]=FALSE && StatusWord_bool[2]=FALSE && StatusWord_bool[3]=FALSE &&  StatusWord_bool[6]=TRUE){
	  						SubStates_Power_Enabled=-1;//terminazione prevista(solo in mode { operations 9 e 8 e se il motore è fermo prima del trigger) a switch on disabled direttamente bypassan{ il quick stop
	  						SubStates_Power_Disabled=1;
	  						state_motor_string='switch on disabled';
	  					}*/
	  					//da verificare questa transizione puo essere automatica in alcuni contesti
	  					break;

	  				}
	  				break;
	  	}






	  		break;



	  }
	  if (Node[k].Transition_Drive_requested==1) {
	  	  	  	Node[k].SubStates_Power_Enabled=-1;
	  	  	  	Node[k].SubStates_Power_Disabled=-1;
	  	  	  	Node[k].SubStates_Fault=-1;
	  	  	  	Node[k].Transition_Drive_requested=false;
	  	  	  	//da qui al prossimo commento in realtà è uno spezzon edi codice che devo rendere moooolto piu richiamabile da altre parti.....è qui coosi solo per essere testato
	  	  	  	Node[k].ControlWord_app=0;
	  	  	  	for(int j=0;j<=15;j++){
	  	  	  		if(Node[k].ControlWord_bool[j]==1){
	  	  	  		Node[k].ControlWord_app=Node[k].ControlWord_app+ pow(2, j);
	  	  	  		}
	  	  	  	}
	  	  	Node[k].TxData_PDO[0][0]=(Node[k].ControlWord_app >> 0)  & 0xFF;
	  	  	Node[k].TxData_PDO[0][1]=(Node[k].ControlWord_app >> 8)  & 0xFF;
	  	  	Node[k].TxData_PDO[0][2]=3;
	  	  	Node[k].TxData_PDO[0][3]=(Node[k].Speed_search_for_zero >> 0) && 0xFF;
	  	  	Node[k].TxData_PDO[0][4]=(Node[k].Speed_search_for_zero >> 8) && 0xFF;
	  	  	Node[k].TxData_PDO[0][5]=(Node[k].Speed_search_for_zero >> 16) && 0xFF;
	  	  	Node[k].TxData_PDO[0][6]=(Node[k].Speed_search_for_zero >> 24) && 0xFF;
	  	  	 if (HAL_CAN_AddTxMessage(&hcan1, &Node[k].Tx_STD_Header_PDO[0], Node[k].TxData_PDO[0], &TxMailbox[0]) != HAL_OK)
	  	  	               {
	  	  	             	  Error_Handler();
	  	  	               }
	  	  }

	  if(Node[k].NMT_Writer!=0){
		  Node[k].TxData_NMT[0]=Node[k].NMT_Writer;
	  	if (HAL_CAN_AddTxMessage(&hcan1, &Node[k].Tx_STD_Header_NMT, &Node[k].TxData_NMT[0], &TxMailbox[0]) != HAL_OK)
	                 {
	               	  Error_Handler();
	                 }
	  	Node[k].NMT_Writer=0;
	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLRCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_4TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  CAN_FilterTypeDef canfilterconfig0;

        canfilterconfig0.FilterActivation = CAN_FILTER_ENABLE;
        canfilterconfig0.FilterBank = 0;  // anything between 0 to SlaveStartFilterBank
        canfilterconfig0.FilterFIFOAssignment = CAN_RX_FIFO0;
        canfilterconfig0.FilterIdHigh     = 0 << 5; // 0x0020
        canfilterconfig0.FilterIdLow      = 0;
        canfilterconfig0.FilterMaskIdHigh = 1 << 5; // confronta solo il bit meno significativo
        canfilterconfig0.FilterMaskIdLow  = 0;
        canfilterconfig0.FilterMode = CAN_FILTERMODE_IDMASK;//due tipi di filtro
        canfilterconfig0.FilterScale = CAN_FILTERSCALE_32BIT;
        canfilterconfig0.SlaveStartFilterBank = 25;  // 13 to 27 are assigned to slave CAN (CAN 2) OR 0 to 12 are assgned to CAN1
                    //meglio farselo spiegare.....    canfilterconfig.SlaveStartFilterBank = 13;  // 13 to 27 are assigned to slave CAN (CAN 2) OR 0 to 12 are assgned to CAN1

        HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig0);
        CAN_FilterTypeDef canfilterconfig1;

              canfilterconfig1.FilterActivation = CAN_FILTER_ENABLE;
              canfilterconfig1.FilterBank = 1;  // anything between 0 to SlaveStartFilterBank
              canfilterconfig1.FilterFIFOAssignment = CAN_RX_FIFO1;
              canfilterconfig1.FilterIdHigh     = 1 << 5; // 0x0020
              canfilterconfig1.FilterIdLow      = 0;
              canfilterconfig1.FilterMaskIdHigh = 1 << 5; // confronta solo il bit meno significativo
              canfilterconfig1.FilterMaskIdLow  = 0;
              canfilterconfig1.FilterMode = CAN_FILTERMODE_IDMASK;//due tipi di filtro
              canfilterconfig1.FilterScale = CAN_FILTERSCALE_32BIT;
              canfilterconfig1.SlaveStartFilterBank = 25;  // 13 to 27 are assigned to slave CAN (CAN 2) OR 0 to 12 are assgned to CAN1
                          //meglio farselo spiegare.....    canfilterconfig.SlaveStartFilterBank = 13;  // 13 to 27 are assigned to slave CAN (CAN 2) OR 0 to 12 are assgned to CAN1

              HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig1);
  /* USER CODE END CAN1_Init 2 */

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 8;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 59999;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */

  /* USER CODE END TIM11_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1){

	HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &Node[0].RxHeader_node[Node[0].buffer_shift_refister], Node[0].RxData_node[Node[0].buffer_shift_refister]);
	Node[0].buffer_shift_refister++;
	if(Node[0].buffer_shift_refister>=16) Node[0].buffer_shift_refister=0;
}
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan1){
	HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO1, &Node[1].RxHeader_node[Node[1].buffer_shift_refister], Node[1].RxData_node[Node[1].buffer_shift_refister]);
	Node[1].buffer_shift_refister++;
		if(Node[1].buffer_shift_refister>=16) Node[0].buffer_shift_refister=0;

}
void RxFifo0FullCallback(CAN_HandleTypeDef *hcan1){

}
void RxFifo1FullCallback(CAN_HandleTypeDef *hcan1){

}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim11){
	//HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox1);
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
