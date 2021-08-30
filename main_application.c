/* Standard includes. */
#include <stdio.h>
#include <conio.h>
#include <string.h>
#include <stdlib.h>
/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "timers.h"
#include "extint.h"

/* Hardware simulator utility functions */
#include "HW_access.h"
//--------------------------------------------------------------------------------------------------//


//---------------------------------SERIAL SIMULATOR CHANNEL-----------------------------------------//
#define COM_CH0 (0)
#define COM_CH1 (1)
#define COM_CH2 (2)

//--------------------------------------------------------------------------------------------------//



//-------------------------------------TASK PRIORITIES----------------------------------------------//
#define	TASK_SERIAL_SEND_PRI		( tskIDLE_PRIORITY + 3 )
#define TASK_SERIAL_REC_PRI			( tskIDLE_PRIORITY + 4 )
#define TASK_SENSOR_SERIAL_REC_PRI	( tskIDLE_PRIORITY + 5 )
#define OBRADA_TASK_PRI				( tskIDLE_PRIORITY + 1 )
#define	SERVICE_TASK_PRI			( tskIDLE_PRIORITY + 2 )
//--------------------------------------------------------------------------------------------------//



//--------------------------------TASKS: FORWARD DECLARATIONS---------------------------------------//
void SerialSend_Task(void* pvParameters);
void SerialReceive_Task(void* pvParameters);
void Obrada_task(void* pvParameters);
void LED_bar_task(void* pvParameters);
void Lijevi_senzor_task(void* pvParameters);
void Desni_senzor_task(void* pvParameters);
void Seg7_task(void* pvParameters);
void Blink_1(void* pvParameters);
void Blink_2(void* pvParameters);
//--------------------------------------------------------------------------------------------------//


//-------------------------------------SEMAPHORE HANDLES--------------------------------------------//
SemaphoreHandle_t TBE_BinarySemaphore_0;
SemaphoreHandle_t TBE_BinarySemaphore_1;
SemaphoreHandle_t TBE_BinarySemaphore_2;
SemaphoreHandle_t RXC_BinarySemaphore_0;
SemaphoreHandle_t RXC_BinarySemaphore_1;
SemaphoreHandle_t RXC_BinarySemaphore_2;
SemaphoreHandle_t LED_INT_BinarySemaphore;
SemaphoreHandle_t mutex_serijska;
SemaphoreHandle_t obrada;
SemaphoreHandle_t flag;
SemaphoreHandle_t seg7;
SemaphoreHandle_t lijevi;
SemaphoreHandle_t desni;

//--------------------------------------------------------------------------------------------------//


//------------------------------------ QUEUE HANDLES------------------------------------------------//
static QueueHandle_t kanal_2_ispis_rijeci = NULL;
static QueueHandle_t kanal_2_ispis_duzine = NULL;

static QueueHandle_t komandna_rijec_niz = NULL;
static QueueHandle_t komandna_rijec_duzina = NULL;

static QueueHandle_t prekidac = NULL;
static QueueHandle_t stanje = NULL;

static QueueHandle_t lijevi_senzor_seg7 = NULL;
static QueueHandle_t desni_senzor_seg7 = NULL;

static QueueHandle_t lijevi_senzor_obrada = NULL;
static QueueHandle_t desni_senzor_obrada = NULL;

static QueueHandle_t blink_lijevi_1 = NULL;
static QueueHandle_t blink_desni_1 = NULL;

static QueueHandle_t blink_lijevi_2 = NULL;
static QueueHandle_t blink_desni_2 = NULL;
//--------------------------------------------------------------------------------------------------//


//--------------------------------------TIMER FUNKCIJA----------------------------------------------//
static void seg7_tajmer_callback(TimerHandle_t Tmh); //svakih 80ms ispisuje stanje sistema			//
static void lijevi_tajmer_callback(TimerHandle_t Tmh); //svakih 200ms ispisuje stanje sistema		//
static void desni_tajmer_callback(TimerHandle_t Tmh); //svakih 200ms ispisuje stanje sistema		//
static void ispis_tajmer_callback(TimerHandle_t Tmh); //svakih 5000ms ispisuje stanje sistema		//
//--------------------------------------------------------------------------------------------------//
																									//
																									//
//---------------------------------------TIMER HANDLES----------------------------------------------//																//
TimerHandle_t seg7_tajmer;																			//
TimerHandle_t lijevi_tajmer;																		//
TimerHandle_t desni_tajmer;																			//		PODACI	
TimerHandle_t ispis_tajmer;																			//
//--------------------------------------------------------------------------------------------------//		  ZA
																									//
																									//		 TIMER
//----------------------------------PERIODIC TIMER CALLBACK-----------------------------------------//
static void seg7_tajmer_callback(TimerHandle_t seg7_tajmer) {										//
	xSemaphoreGive(seg7); // predaj semafor svakih 80ms												//
}																									//
static void lijevi_tajmer_callback(TimerHandle_t lijevi_tajmer) {									//
	xSemaphoreGive(lijevi); // predaj semafor svakih 200ms											//
}																									//
static void desni_tajmer_callback(TimerHandle_t desni_tajmer) {										//
	xSemaphoreGive(desni); // predaj semafor svakih 200ms											//
}																									//
static void ispis_tajmer_callback(TimerHandle_t ispis_tajmer) {										//
	xSemaphoreGive(obrada); // predaj semafor svakih 5000ms											//
}																									//
//--------------------------------------------------------------------------------------------------//



//-------------------------------------TRASNMISSION DATA--------------------------------------------//
unsigned volatile t_point;
//--------------------------------------------------------------------------------------------------//



//---------------------------------------RECEPTION DATA---------------------------------------------//
#define R_BUF_SIZE (6)
unsigned volatile r_point;
unsigned volatile ls_point;
unsigned volatile ds_point;
//--------------------------------------------------------------------------------------------------//



//--------------------------7-SEG NUMBER DATABASE - ALL HEX DIGITS----------------------------------//
static const char hexnum[] = { 0x3F, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07,
								0x7F, 0x6F, 0x77, 0x7C, 0x39, 0x5E, 0x79, 0x71 };
//--------------------------------------------------------------------------------------------------//



//----------------------------------------INTERUPT--------------------------------------------------//

/* TBE - TRANSMISSION BUFFER EMPTY - INTERRUPT HANDLER */
static uint32_t prvProcessTBEInterrupt(void)
{
	BaseType_t xHigherPTW = pdFALSE;

	if (get_TBE_status(0) != 0)
	xSemaphoreGiveFromISR(TBE_BinarySemaphore_0, &xHigherPTW);

	if (get_TBE_status(1) != 0)
	xSemaphoreGiveFromISR(TBE_BinarySemaphore_1, &xHigherPTW);

	if (get_TBE_status(2) != 0)
	xSemaphoreGiveFromISR(TBE_BinarySemaphore_2, &xHigherPTW);

	portYIELD_FROM_ISR(xHigherPTW);
}

/* RXC - RECEPTION COMPLETE - INTERRUPT HANDLER */
static uint32_t prvProcessRXCInterrupt(void)
{
	BaseType_t xHigherPTW = pdFALSE;

	if (get_RXC_status(0) != 0)
	xSemaphoreGiveFromISR(RXC_BinarySemaphore_0, &xHigherPTW);

	if (get_RXC_status(1) != 0)
	xSemaphoreGiveFromISR(RXC_BinarySemaphore_1, &xHigherPTW);

	if (get_RXC_status(2) != 0)
	xSemaphoreGiveFromISR(RXC_BinarySemaphore_2, &xHigherPTW);

	portYIELD_FROM_ISR(xHigherPTW);
}
//--------------------------------------------------------------------------------------------------//



/* OPC - ON INPUT CHANGE - INTERRUPT HANDLER */
static uint32_t OnLED_ChangeInterrupt(void)
{
	BaseType_t xHigherPTW = pdFALSE;

	xSemaphoreGiveFromISR(LED_INT_BinarySemaphore, &xHigherPTW);

	portYIELD_FROM_ISR(xHigherPTW);
}
//--------------------------------------------------------------------------------------------------//


//-------------------------------------MAIN - SYSTEM STARTUP POINT--------------------------------------//
void main_demo(void)
{

	//--------------------------------------INICIJALIZACIJA---------------------------------------------//
	init_serial_uplink(COM_CH0);  // inicijalizacija serijske TX na kanalu 0
	init_serial_downlink(COM_CH0);// inicijalizacija serijske TX na kanalu 0
	init_serial_uplink(COM_CH1);  // inicijalizacija serijske TX na kanalu 1
	init_serial_downlink(COM_CH1);// inicijalizacija serijske TX na kanalu 1
	init_serial_uplink(COM_CH2);  // inicijalizacija serijske TX na kanalu 2
	init_serial_downlink(COM_CH2);// inicijalizacija serijske TX na kanalu 2
	init_LED_comm();
	init_7seg_comm();
	//--------------------------------------------------------------------------------------------------//


	//------------------------------------CREATE A TIMER TASK-------------------------------------------//
	seg7_tajmer = xTimerCreate("Timer", pdMS_TO_TICKS(80), pdTRUE, NULL, seg7_tajmer_callback); //callback 80ms
	xTimerStart(seg7_tajmer, 0);

	lijevi_tajmer = xTimerCreate("Timer1", pdMS_TO_TICKS(200), pdTRUE, NULL, lijevi_tajmer_callback); //callback 200ms
	xTimerStart(lijevi_tajmer, 0);

	desni_tajmer = xTimerCreate("Timer2", pdMS_TO_TICKS(200), pdTRUE, NULL, desni_tajmer_callback); //callback 200ms
	xTimerStart(desni_tajmer, 0);

	ispis_tajmer = xTimerCreate("Timer3", pdMS_TO_TICKS(5000), pdTRUE, NULL, ispis_tajmer_callback); //callback 5000ms
	xTimerStart(ispis_tajmer, 0);
	//--------------------------------------------------------------------------------------------------//

	
	//------------------------------------CREATE SEMAPHORE----------------------------------------------//
	/* Create TBE semaphore - serial transmit comm */
	TBE_BinarySemaphore_0 = xSemaphoreCreateBinary();
	TBE_BinarySemaphore_1 = xSemaphoreCreateBinary();
	TBE_BinarySemaphore_2 = xSemaphoreCreateBinary();

	/* Create TBE semaphore - serial transmit comm */
	RXC_BinarySemaphore_0 = xSemaphoreCreateBinary();
	RXC_BinarySemaphore_1 = xSemaphoreCreateBinary();
	RXC_BinarySemaphore_2 = xSemaphoreCreateBinary();

	// create LED interrapt semaphore
	LED_INT_BinarySemaphore = xSemaphoreCreateBinary();
	// create mutex semaphore
	mutex_serijska = xSemaphoreCreateMutex();
	// create obrada semaphore
	obrada = xSemaphoreCreateBinary();
	//  create flag semaphore
	flag = xSemaphoreCreateBinary();
	// create SEG7 semaphore
	seg7 = xSemaphoreCreateBinary();
	// create lijevi semaphore
	lijevi = xSemaphoreCreateBinary();
	// create desni semaphore
	desni = xSemaphoreCreateBinary();
	//--------------------------------------------------------------------------------------------------//


	//----------------------------------------INTERUPT--------------------------------------------------//
	/* SERIAL TRANSMISSION INTERRUPT HANDLER */
	vPortSetInterruptHandler(portINTERRUPT_SRL_TBE, prvProcessTBEInterrupt);

	/* SERIAL RECEPTION INTERRUPT HANDLER */
	vPortSetInterruptHandler(portINTERRUPT_SRL_RXC, prvProcessRXCInterrupt);

	/* ON INPUT CHANGE INTERRUPT HANDLER */
	vPortSetInterruptHandler(portINTERRUPT_SRL_OIC, OnLED_ChangeInterrupt);
	//--------------------------------------------------------------------------------------------------//


	//---------------------KREIRANJE REDOVA ZA KOMUNIKACIJU IZMEDJU TASKOVA-----------------------------//
	kanal_2_ispis_rijeci = xQueueCreate(3, sizeof(uint8_t[60])); //red za skladistenje poruke za ispis
	kanal_2_ispis_duzine = xQueueCreate(3, sizeof(uint8_t)); //red za skladistenje duzine rijeci

	komandna_rijec_niz = xQueueCreate(3, sizeof(uint8_t[6])); //red za skladistenje primljene rijeci
	komandna_rijec_duzina = xQueueCreate(3, sizeof(uint8_t)); //red za skladistenje duzine primljene rijeci

	prekidac = xQueueCreate(1, sizeof(uint8_t)); //red za primanje stanja prekidaca
	stanje = xQueueCreate(1, sizeof(uint8_t)); //red za primanje stanja sistema, tj. ukljucen/iskljucen

	lijevi_senzor_seg7 = xQueueCreate(2, sizeof(uint8_t[2])); //red za primanje vrijednosti sa lijevog senzora na displej
	desni_senzor_seg7 = xQueueCreate(2, sizeof(uint8_t[2])); //red za primanje vrijednosti sa desnog senzora na displej

	lijevi_senzor_obrada = xQueueCreate(2, sizeof(uint8_t[2])); //red za primanje vrijednosti sa lijevog senzora za obradu
	desni_senzor_obrada = xQueueCreate(2, sizeof(uint8_t[2])); //red za primanje vrijednosti sa desnog senzora za obradu

	blink_lijevi_1 = xQueueCreate(2, sizeof(uint8_t[2])); //red za primanje vrijednosti sa lijevog senzora za udaljenosti vece od 0%
	blink_desni_1 = xQueueCreate(2, sizeof(uint8_t[2])); //red za primanje vrijednosti sa desnog senzora za udaljenosti vece od 0%

	blink_lijevi_2 = xQueueCreate(2, sizeof(uint8_t[2])); //red za primanje vrijednosti sa lijevog senzora za udaljenosti manje i jednake 0%
	blink_desni_2 = xQueueCreate(2, sizeof(uint8_t[2])); //red za primanje vrijednosti sa lijevog senzora za udaljenosti manje i jednake od 0%
	//--------------------------------------------------------------------------------------------------//


	//----------------------------------------CREATE TASK-----------------------------------------------//
	// PC RECEIVE TASK
	xTaskCreate(SerialReceive_Task, "SRx", configMINIMAL_STACK_SIZE, NULL, TASK_SERIAL_REC_PRI, NULL);

	// PC SEND TASK
	xTaskCreate(SerialSend_Task, "STx", configMINIMAL_STACK_SIZE, NULL, TASK_SERIAL_SEND_PRI, NULL);

	// LIJEVI SENZOR TASK
	xTaskCreate(Lijevi_senzor_task, "Ls_SRx", configMINIMAL_STACK_SIZE, NULL, TASK_SENSOR_SERIAL_REC_PRI, NULL);

	// DESNI SENZPR TASK
	xTaskCreate(Desni_senzor_task, "Ds_SRx", configMINIMAL_STACK_SIZE, NULL, TASK_SENSOR_SERIAL_REC_PRI, NULL);

	// OBRADA PODATAKA TASK
	xTaskCreate(Obrada_task, "Obrada_task", configMINIMAL_STACK_SIZE, NULL, OBRADA_TASK_PRI, NULL);

	// LED BAR TASK
	xTaskCreate(LED_bar_task, "LED_bar_task", configMINIMAL_STACK_SIZE, NULL, SERVICE_TASK_PRI, NULL);
	
	// SEG7 TASK
	xTaskCreate(Seg7_task, "Seg7_task", configMINIMAL_STACK_SIZE, NULL, SERVICE_TASK_PRI, NULL);

	// BLINK TASK 1
	xTaskCreate(Blink_1, "Blink_1", configMINIMAL_STACK_SIZE, NULL, SERVICE_TASK_PRI, NULL);

	// BLINK TASK 2
	xTaskCreate(Blink_2, "Blink_2", configMINIMAL_STACK_SIZE, NULL, SERVICE_TASK_PRI, NULL);
	//--------------------------------------------------------------------------------------------------//


	vTaskStartScheduler();

	while (1);
}



void SerialSend_Task(void* pvParameters)
{
	//uint8_t t_point = 0;
	uint8_t ispis_niza[60] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
							   0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
							   0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
							   0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
							   0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
							   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, };
	uint8_t duzina_ispis_niza = 0;
	while (1)
	{

		xSemaphoreTake(TBE_BinarySemaphore_2, portMAX_DELAY);// ceka na serijski prijemni interapt

		xQueueReceive(kanal_2_ispis_rijeci, &ispis_niza, pdMS_TO_TICKS(10)); //prijem miza iz obrade podataka, brzina ispisivanja jednog karaktera 10ms
		xQueueReceive(kanal_2_ispis_duzine, &duzina_ispis_niza, pdMS_TO_TICKS(10)); //prijem duzine niza iz obrade podataka, brzina ispisivanja jednog karaktera 10ms

		if (t_point < duzina_ispis_niza) //dok nije ispisan posljednji karakter salji slovo po slovo 
			send_serial_character(COM_CH2, ispis_niza[t_point++]);//ucitava primljeni karakter u promenjivu cc

		else { //kada se ispise posljednji karakter, vrati brojac na nulu i predaj semafor da se mutex zatvori
			t_point = 0;
			duzina_ispis_niza = 0;
		}
	}
}

void SerialReceive_Task(void* pvParameters)
{
	//uint8_t r_point = 0;
	uint8_t cc = 0;
	uint8_t r_buffer[6] = { 0, 0, 0, 0, 0, 0 };
	uint8_t duzina_primljene_rijeci = 0;

	while (1)
	{
		xSemaphoreTake(RXC_BinarySemaphore_2, portMAX_DELAY);// ceka na serijski prijemni interapt
		get_serial_character(COM_CH2, &cc);//ucitava primljeni karakter u promenjivu cc

		if (cc == 0x0d)
		{
			duzina_primljene_rijeci = r_point;
			xQueueSend(komandna_rijec_niz, &r_buffer, 0U);// salje komandnu rijec tasku za obradu podataka
			xQueueSend(komandna_rijec_duzina, &duzina_primljene_rijeci, 0U);//salje duzinu konadne rijeci tasku za obradu podataka
			r_point = 0;
		}
		else if (r_point < R_BUF_SIZE)// pamti karaktere prije CR
		{
			r_buffer[r_point++] = cc;
			//printf("primio karakter: %u\n", (unsigned)cc);// prikazuje primljeni karakter u cmd prompt
		}

	}
}

void Obrada_task(void* pvParameters)
{
	uint8_t LED = 0x00;
	uint8_t r_buffer[6] = { 0, 0, 0, 0, 0, 0 };
	uint8_t duzina_komandne_rijeci = 0;
	uint8_t ispis_niza[60] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 
							   0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
							   0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
							   0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
							   0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
							   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, };
	uint8_t duzina_ispis_niza = 0;
	uint8_t ukljucen = 0;
	uint8_t ls_buffer[2] = { 0, 0 };
	uint8_t ds_buffer[2] = { 0, 0 };

	while (1)
	{
		xSemaphoreTake(obrada, portMAX_DELAY);
		
		xQueueReceive(lijevi_senzor_obrada, &ls_buffer, pdMS_TO_TICKS(10));//primi podatak od lijevog senzora
		xQueueReceive(desni_senzor_obrada, &ds_buffer, pdMS_TO_TICKS(10));//primi podatak od desnog senzora
		xQueueReceive(komandna_rijec_niz, &r_buffer, pdMS_TO_TICKS(10)); //primi komandnu rijec od PC-a
		xQueueReceive(komandna_rijec_duzina, &duzina_komandne_rijeci, pdMS_TO_TICKS(10));//primi duzinu komandne poruke od PC-a
		xQueueReceive(prekidac, &LED, pdMS_TO_TICKS(10));//primi stanje od prekidaca

		// provjeravam da li je primljena komandna rijec START
		if (((duzina_komandne_rijeci == sizeof("START") - 1) && (strncmp(r_buffer, ("START"), duzina_komandne_rijeci) == 0)) || (LED == 0x01))
		{
			ukljucen = 1;
			set_LED_BAR(1, 0x01);

			strcpy(ispis_niza, "Sistem: ");
			duzina_ispis_niza = sizeof("Sistem: ") - 1;
			strcat(ispis_niza, "UKLJUCEN\n");
			duzina_ispis_niza += sizeof("UKLJUCEN\n");
			strcat(ispis_niza, "	LS:	");
			duzina_ispis_niza += sizeof("	LS:	") - 1;

			if (ls_buffer[0] >= 100)
			{
				strcat(ispis_niza, "     100%\n");
				duzina_ispis_niza += sizeof("     100%\n") - 1;
			}
			else if (ls_buffer[0] >= 50 && ls_buffer[0] < 100)
			{
				//ispis_niza[duzina_ispis_niza++] = (unsigned)ls_buffer[0] / 10 + 48;
				//ispis_niza[duzina_ispis_niza++] = (unsigned)ls_buffer[0] % 10 + 48;
				strcat(ispis_niza, "50%-100%\n");
				duzina_ispis_niza += sizeof("50%-100%\n") - 1;
			}
			else if (ls_buffer[0] > 20 && ls_buffer[0] < 50)
			{
				//ispis_niza[duzina_ispis_niza++] = (unsigned)ls_buffer[0] / 10 + 48;
				//ispis_niza[duzina_ispis_niza++] = (unsigned)ls_buffer[0] % 10 + 48;
				strcat(ispis_niza, "  0%-50%\n");
				duzina_ispis_niza += sizeof("  0%-50%\n") - 1;
			}
			else if (ls_buffer[0] <= 20)
			{
				strcat(ispis_niza, "      0%\n");
				duzina_ispis_niza += sizeof("      0%\n") - 1;
			}



			strcat(ispis_niza, "	DS:	");
			duzina_ispis_niza += sizeof("	DS:	") - 1;
			if (ds_buffer[0] >= 100)
			{
				strcat(ispis_niza, "     100%\n");
				duzina_ispis_niza += sizeof("     100%\n") - 1;
			}
			else if (ds_buffer[0] >= 50 && ds_buffer[0] < 100)
			{
				//ispis_niza[duzina_ispis_niza++] = (unsigned)ds_buffer[0] / 10;
				//ispis_niza[duzina_ispis_niza++] = (unsigned)ds_buffer[0] % 10;
				strcat(ispis_niza, "50%-100%\n");
				duzina_ispis_niza += sizeof("50%-100%\n") - 1;
			}
			else if (ds_buffer[0] > 20 && ds_buffer[0] < 50)
			{
				//ispis_niza[duzina_ispis_niza++] = (unsigned)ds_buffer[0] / 10 + 48;
				//ispis_niza[duzina_ispis_niza++] = (unsigned)ds_buffer[0] % 10 + 48;
				strcat(ispis_niza, "  0%-50%\n");
				duzina_ispis_niza += sizeof("  0%-50%\n") - 1;
			}
			else if (ds_buffer[0] <= 20)
			{
				strcat(ispis_niza, "      0%\n");
				duzina_ispis_niza += sizeof("      0%\n") - 1;
			}

		}

		// provjeravam da li je primljena komandna rijec STOP
		else if (((duzina_komandne_rijeci == sizeof("STOP") - 1) && (strncmp(r_buffer, ("STOP"), duzina_komandne_rijeci) == 0)) || (LED == 0x00))
		{
			ukljucen = 2;
			set_LED_BAR(1, 0x00);
			strcpy(ispis_niza, "Sistem: ");
			duzina_ispis_niza = sizeof("Sistem: ");
			strcat(ispis_niza, "ISKLJUCEN\n");
			duzina_ispis_niza += sizeof("ISKLJUCEN\n")-1;
		}

		// provjeravam da li je primljeno nesto drugo
		else
		{
			strcpy(ispis_niza, "SISTEM NIJE AKTIVAN");
			duzina_ispis_niza = sizeof("SISTEM NIJE AKTIVAN") - 1;
		}

		send_serial_character(COM_CH2, 13);

		xQueueSend(kanal_2_ispis_rijeci, &ispis_niza, 0U);//salji niz tasku za ispis na PC
		xQueueSend(kanal_2_ispis_duzine, &duzina_ispis_niza, 0U);//salji duzinu niza tasku za isps na PC
		xQueueSend(stanje, &ukljucen, 0U);
	}
}

void LED_bar_task(void* pvParameters) //ocitati prekidace i reci da li je ukljuceno ili iskljuceno
{
	uint8_t d;
	uint8_t ukljucen;

	while (1)
	{
		xSemaphoreTake(LED_INT_BinarySemaphore, portMAX_DELAY);
		xQueueReceive(stanje, &ukljucen, pdMS_TO_TICKS(10));//prima podatak o komandnoj rijeci od taska za obradu podataka
		
		//printf("Lijevi senzor: %u\n", ls_buffer[0]);// prikazuje primljeni karakter u cmd prompt
		get_LED_BAR(0, &d); //ocitaj stanje prvog stubca, prve ledovke led bara
		
		xQueueSend(prekidac, &d, 0U);//posalji stanje prekidaca tasku za obradu podataka
	}
}

void Lijevi_senzor_task(void* pvParameters)//task za lijevi senzor, kanal 0
{
	uint8_t ll = 0;
	uint8_t ls_point = 0;
	uint8_t ls_buffer[2] = { 0, 0 };
	uint8_t duzina = 0;
	

	while (1)
	{
			//xSemaphoreTake(lijevi, portMAX_DELAY);// ceka na serijski prijemni interapt svakih 200ms
			xSemaphoreTake(RXC_BinarySemaphore_0, portMAX_DELAY);// ceka na serijski prijemni interapt
			get_serial_character(COM_CH0, &ll);//ucitava primljeni karakter u promenjivu cc
			
			if (ll == 0x0d)
			{
				duzina = ls_point;
				ls_point = 0;
				xQueueSend(lijevi_senzor_obrada, &ls_buffer, 0U);//salji podatak sa lijevog senzora tasku za obradi podataka
				xQueueSend(lijevi_senzor_seg7, &ls_buffer, 0U);//salji podatak sa lijevog senzora tasku seg7 (displej)
				printf("Lijevi senzor: %u\n", ls_buffer[0]);// prikazuje stanje lijevog senzora u cmd prompt
			}
			else if (ls_point < R_BUF_SIZE)// pamti karaktere prije CR
			{
				ls_buffer[ls_point++] = ll;
			}
	}
}

void Desni_senzor_task(void* pvParameters) //ocitati prekidace i reci da li je ukljuceno ili iskljuceno
{
	uint8_t dd = 0;
	uint8_t ds_point = 0;
	uint8_t ds_buffer[2] = { 0, 0 };
	uint8_t duzina = 0;

	while (1)
	{
			//xSemaphoreTake(desni, portMAX_DELAY);// ceka na serijski prijemni interapt svakih 200ms
			xSemaphoreTake(RXC_BinarySemaphore_1, portMAX_DELAY);// ceka na serijski prijemni interapt

			get_serial_character(COM_CH1, &dd);//ucitava primljeni karakter u promenjivu cc

			if (dd == 0x0d)
			{
				duzina = ds_point;
				ds_point = 0;
				xQueueSend(desni_senzor_obrada, &ds_buffer, 0U);//salji podatak sa desnog senzora tasku za obradu podataka
				xQueueSend(desni_senzor_seg7, &ds_buffer, 0U);//salji podatak sa desnog senzora tasku seg7 (displej)
				printf("Desni senzor: %u\n", ds_buffer[0]);// prikazuje primljeni karakter u cmd prompt
			}
			else if (ds_point < R_BUF_SIZE)// pamti karaktere prije CR
			{
				ds_buffer[ds_point++] = dd;
			}
	}
}

void Seg7_task(void* pvParameters)
{
	uint8_t ls_buffer[2] = { 0, 0 };
	uint8_t ds_buffer[2] = { 0, 0 };
	uint8_t ukljucen;
	uint8_t postotak_lijevi;
	uint8_t postotak_desni;
	uint8_t duzina_ispis_niza;
	while (1)
	{
		xSemaphoreTake(seg7, portMAX_DELAY);// ceka na serijski prijemni interapt

		xQueueReceive(stanje, &ukljucen, pdMS_TO_TICKS(10));//primi podatak o stanju sistema
		xQueueReceive(lijevi_senzor_seg7, &ls_buffer, pdMS_TO_TICKS(10));//primi podatak od taska za obradu podataka od lijevog senzora
		xQueueReceive(desni_senzor_seg7, &ds_buffer, pdMS_TO_TICKS(10));// primi podatak od taska za obradu podataka od desnog senzora

		if (ukljucen == 1)// sistem UKLJUCEN
		{

			if (ls_buffer[0] > 99)
			{
				postotak_lijevi = 0;

				select_7seg_digit(0);
				set_7seg_digit(hexnum[(uint8_t)1]);
				select_7seg_digit(1);
				set_7seg_digit(hexnum[(uint8_t)0]);
				select_7seg_digit(2);
				set_7seg_digit(hexnum[(uint8_t)0]);

				//xQueueSend(blink_lijevi, &postotak_lijevi, 0U);//salji podatak za blinkanje diode [100%]
			}

			else if (ls_buffer[0] <= 99 && ls_buffer[0] >= 50)
			{
				postotak_lijevi = 1;

				select_7seg_digit(0);
				set_7seg_digit(hexnum[(uint8_t)0]);
				select_7seg_digit(1);
				set_7seg_digit(hexnum[(uint8_t)ls_buffer[0] / 10]);
				select_7seg_digit(2);
				set_7seg_digit(hexnum[(uint8_t)ls_buffer[0] % 10]);

				//xQueueSend(blink_lijevi, &postotak_lijevi, 0U);//salji podatak za blinkanje diode [50%-100%] - lijevi senzor
			}

			else if (ls_buffer[0] <= 49 && ls_buffer[0] > 20)
			{
				postotak_lijevi = 2;

				select_7seg_digit(0);
				set_7seg_digit(hexnum[(uint8_t)0]);
				select_7seg_digit(1);
				set_7seg_digit(hexnum[(uint8_t)ls_buffer[0] / 10]);
				select_7seg_digit(2);
				set_7seg_digit(hexnum[(uint8_t)ls_buffer[0] % 10]);

				//xQueueSend(blink_lijevi, &postotak_lijevi, 0U);//salji podatak za blinkanje diode [0%-50%] - lijevi senzor
			}

			else if (ls_buffer[0] <= 20)
			{
				postotak_lijevi = 3;

				select_7seg_digit(0);
				set_7seg_digit(hexnum[(uint8_t)0]);
				select_7seg_digit(1);
				set_7seg_digit(hexnum[(uint8_t)2]);
				select_7seg_digit(2);
				set_7seg_digit(hexnum[(uint8_t)0]);

				//xQueueSend(blink_lijevi, &postotak_lijevi, 0U);//salji podatak za blinkanje diode [0%] - lijevi senzor
			}
				

			if (ds_buffer[0] > 99)
			{
				postotak_desni = 0;

				select_7seg_digit(4);
				set_7seg_digit(hexnum[(uint8_t)1]);
				select_7seg_digit(5);
				set_7seg_digit(hexnum[(uint8_t)0]);
				select_7seg_digit(6);
				set_7seg_digit(hexnum[(uint8_t)0]);

				//xQueueSend(blink_desni, &postotak_desni, 0U);//salji podatak za blinkanje diode [100%] - desni senzor
			}

			else if (ds_buffer[0] <= 99 && ds_buffer[0] >= 50)
			{
				postotak_desni = 1;

				select_7seg_digit(4);
				set_7seg_digit(hexnum[(uint8_t)0]);
				select_7seg_digit(5);
				set_7seg_digit(hexnum[(uint8_t)ds_buffer[0] / 10]);
				select_7seg_digit(6);
				set_7seg_digit(hexnum[(uint8_t)ds_buffer[0] % 10]);

				//xQueueSend(blink_desni, &postotak_desni, 0U);//salji podatak za blinkanje diode [50%-100%] - desni senzor
			}

			else if (ds_buffer[0] <= 49 && ds_buffer[0] > 20)
			{
				postotak_desni = 2;

				select_7seg_digit(4);
				set_7seg_digit(hexnum[(uint8_t)0]);
				select_7seg_digit(5);
				set_7seg_digit(hexnum[(uint8_t)ds_buffer[0] / 10]);
				select_7seg_digit(6);
				set_7seg_digit(hexnum[(uint8_t)ds_buffer[0] % 10]);

				//xQueueSend(blink_lijevi, &postotak_desni, 0U);//salji podatak za blinkanje diode [0%-50%] - desni senzor
			}

			else if (ds_buffer[0] <= 20)
			{
				postotak_desni = 3;

				select_7seg_digit(4);
				set_7seg_digit(hexnum[(uint8_t)0]);
				select_7seg_digit(5);
				set_7seg_digit(hexnum[(uint8_t)2]);
				select_7seg_digit(6);
				set_7seg_digit(hexnum[(uint8_t)0]);

				//xQueueSend(blink_desni, &postotak_desni, 0U);//salji podatak za blinkanje diode [0%] - desni senzor
			}
		}

		else if (ukljucen == 2)// sistem ISKLJUCEN
		{
			postotak_lijevi = 4;
			postotak_desni = 4;

			select_7seg_digit(0);
			set_7seg_digit(hexnum[(uint8_t)0]);
			select_7seg_digit(1);
			set_7seg_digit(hexnum[(uint8_t)0]);
			select_7seg_digit(2);
			set_7seg_digit(hexnum[(uint8_t)0]);
			select_7seg_digit(4);
			set_7seg_digit(hexnum[(uint8_t)0]);
			select_7seg_digit(5);
			set_7seg_digit(hexnum[(uint8_t)0]);
			select_7seg_digit(6);
			set_7seg_digit(hexnum[(uint8_t)0]);

			//xQueueSend(blink_lijevi, &postotak_lijevi, 0U);//salji podatak za gasenje svih dioda lijevog senzora
			//xQueueSend(blink_desni, &postotak_desni, 0U);//salji podatak za gasenje svih dioda desnog senzora
		}
		xQueueSend(blink_lijevi_1, &postotak_lijevi, 0U);//salji podatak za gasenje svih dioda lijevog senzora
		xQueueSend(blink_desni_1, &postotak_desni, 0U);//salji podatak za gasenje svih dioda desnog senzora
		xQueueSend(blink_lijevi_2, &postotak_lijevi, 0U);//salji podatak za gasenje svih dioda lijevog senzora
		xQueueSend(blink_desni_2, &postotak_desni, 0U);//salji podatak za gasenje svih dioda desnog senzora
	}
}

void Blink_1(void* pvParameters)
{
	uint8_t ls_buffer[2] = { 0, 0 };
	uint8_t ds_buffer[2] = { 0, 0 };
	uint8_t postotak_lijevi = 0;
	uint8_t postotak_desni = 0;
	while (1)
	{
		xQueueReceive(blink_lijevi_1, &postotak_lijevi, pdMS_TO_TICKS(10));//primi podatak za blinkanje dioda lijevog senzora
		xQueueReceive(blink_desni_1, &postotak_desni, pdMS_TO_TICKS(10));//primi podatak za blinkanje dioda desnog senzora
		
		if (postotak_lijevi == 0 || postotak_lijevi == 4)//udaljenost lijevog senzora 100% ili sistem ISKLJUCEN
		{
			set_LED_BAR(2, 0x00);
			set_LED_BAR(3, 0x00);
			set_LED_BAR(4, 0x00);
		}

		else if (postotak_lijevi == 1)//udaljenost lijevog senzora 50%-100%
		{
			set_LED_BAR(3, 0x00);
			set_LED_BAR(4, 0x00);

				set_LED_BAR(2, 0x01);
				vTaskDelay(pdMS_TO_TICKS(500));// kada se koristi vremenski delay
				set_LED_BAR(2, 0x00);
				vTaskDelay(pdMS_TO_TICKS(50));// kada se koristi vremenski delay
		}

		else if (postotak_lijevi == 2)//udaljenost lijevog senzora 0%-50%
		{
			set_LED_BAR(2, 0x00);
			set_LED_BAR(4, 0x00);

				set_LED_BAR(3, 0x01);
				vTaskDelay(pdMS_TO_TICKS(500));// kada se koristi vremenski delay
				set_LED_BAR(3, 0x00);
				vTaskDelay(pdMS_TO_TICKS(500));// kada se koristi vremenski delay
		}


		if (postotak_desni == 0 || postotak_desni == 4)//udaljenost desnog senzora 100% ili sistem ISKLJUCEN
		{
			set_LED_BAR(5, 0x00);
			set_LED_BAR(6, 0x00);
			set_LED_BAR(7, 0x00);
		}

		else if (postotak_desni == 1)//udaljenost desnog senzora 50%-100%
		{
			set_LED_BAR(6, 0x00);
			set_LED_BAR(7, 0x00);

			set_LED_BAR(5, 0x01);
			vTaskDelay(pdMS_TO_TICKS(500));// kada se koristi vremenski delay
			set_LED_BAR(5, 0x00);
			vTaskDelay(pdMS_TO_TICKS(500));// kada se koristi vremenski delay
		}

		else if (postotak_desni == 2)//udaljenost desnog senzora 0%-0%
		{
			set_LED_BAR(5, 0x00);
			set_LED_BAR(7, 0x00);

			set_LED_BAR(6, 0x01);
			vTaskDelay(pdMS_TO_TICKS(500));// kada se koristi vremenski delay
			set_LED_BAR(6, 0x00);
			vTaskDelay(pdMS_TO_TICKS(500));// kada se koristi vremenski delay


		}
	}
}

void Blink_2(void* pvParameters)
{
	uint8_t ls_buffer[2] = { 0, 0 };
	uint8_t ds_buffer[2] = { 0, 0 };
	uint8_t postotak_lijevi = 0;
	uint8_t postotak_desni = 0;
	while (1)
	{
		xQueueReceive(blink_lijevi_2, &postotak_lijevi, pdMS_TO_TICKS(10));//primi podatak za blinkanje dioda lijevog senzora
		xQueueReceive(blink_desni_2, &postotak_desni, pdMS_TO_TICKS(10));//primi podatak za blinkanje dioda desnog senzora

		if (postotak_lijevi == 3)//udaljenost lijevog senzora 0%
		{
			set_LED_BAR(2, 0x00);
			set_LED_BAR(3, 0x00);

			set_LED_BAR(4, 0x01);
			vTaskDelay(pdMS_TO_TICKS(50));// kada se koristi vremenski delay
			set_LED_BAR(4, 0x00);
			vTaskDelay(pdMS_TO_TICKS(5));// kada se koristi vremenski delay
		}

		if (postotak_desni == 3)//udaljenost desnog senzora 0%
		{
			set_LED_BAR(5, 0x00);
			set_LED_BAR(6, 0x00);

			set_LED_BAR(7, 0x01);
			vTaskDelay(pdMS_TO_TICKS(50));// kada se koristi vremenski delay
			set_LED_BAR(7, 0x00);
			vTaskDelay(pdMS_TO_TICKS(5));// kada se koristi vremenski delay
		}
	}
}