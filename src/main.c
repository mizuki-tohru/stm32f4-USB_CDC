/**
  ******************************************************************************
  * @file    main.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    19-September-2011
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 
/* Includes ------------------------------------------------------------------*/#include "systick.h"

//#include "main.h"
#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usbd_desc.h"
#include "usbd_cdc_vcp.h"

//Library config for this project!!!!!!!!!!!
#include "stm32f4xx.h"
#include "platform_config.h"
#include "hw_config.h"
#include "stm32f4xx_conf.h"
#include "i2c.h"
#include "math.h"

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"


/* Private typedef -----------------------------------------------------------*//* 重要な全体変数 */
typedef enum {
	STARTING		= 0,	// 起動時
	NORMAL			= 1,	// 定常時
	STANDINGSTILL	= 2,	// 静止時 仰角30度以上
	LAUNCHPREPARE	= 3,	// 打上げ準備完了
	FLIGHT			= 4		// 飛行状態
} CPUSTATUS;

CPUSTATUS cpustatus = STARTING;

/* Private define ------------------------------------------------------------*/

#define TESTRESULT_ADDRESS         0x080FFFFC
#define ALLTEST_PASS               0x00000000
#define ALLTEST_FAIL               0x55555555

#define	GREEN_LED_OFF	GPIO_ResetBits(GPIOA,GPIO_Pin_7)
#define	GREEN_LED_ON	GPIO_SetBits(GPIOA,GPIO_Pin_7)
#define	ORANGE_LED_OFF	GPIO_ResetBits(GPIOA,GPIO_Pin_6)
#define	ORANGE_LED_ON	GPIO_SetBits(GPIOA,GPIO_Pin_6)
#define	RED_LED_OFF		GPIO_ResetBits(GPIOA,GPIO_Pin_5)
#define	RED_LED_ON		GPIO_SetBits(GPIOA,GPIO_Pin_5)

#define	TRIM1 			10
#define	TRIM2 			-8
#define	TRIM3 			0
#define	TRIM4 			-15

#define	TSTEP			100		// タスクの周期 (msec)
#define	TIMING1			1000	// STATRTING～NORMAL (msec)
#define	TIMING2			5000	// STANDINGSTILLL～LAUNCHPREPARE (msec)
#define	TIMING21		3000	// ゆっくり点滅 (msec)
#define	TIMING22		4000	// 速く点滅 (msec)
#define	TIMING23		5000	// さらに速く点滅 (msec)
#define	TIMING3			100		// 飛行と判断された後、少し戻って平均化 (msec)
#define	TIMING4			6000	// FLIGHT時間 (msec)

#define	LED_ORANGE_T	1000	// ORANGE LEDの点滅 (msec)
#define	LED_RED_T		800		// RED LEDの点滅 (msec)
#define	LED_GREEN_T1	800		// GREEN LEDのゆっくり点滅 (msec)
#define	LED_GREEN_T2	600		// GREEN LEDのゆっくり点滅 (msec)
#define	LED_GREEN_T3	200		// GREEN LEDのゆっくり点滅 (msec)
#define	LIMIT_A			0.2f	// 静止判定の加速度のズレ上限(G)
#define	LIMIT_W			3.0f	// 静止判定の角速度のズレ上限(dps)
#define	LIMIT_EL		30.0	// 静止判定の仰角の下限(deg)
#define ERRORCOUNT		2		// STATRTING状態でのエラー許容の上限
#define	LIMIT_AZ		0.1f	// FLIGHTへ移行するための加速度下限値(G)
#define	LIMIT_AXY		0.8f	// FLIGHTへ移行する時の加速度で横方向の上限（相対値）

#define 	SERVO1(a)	setPWM3_2(150 + a)
#define 	SERVO2(a)	setPWM3_1(150 + a)
#define 	SERVO3(a)	setPWM12_2(150 + a)
#define 	SERVO4(a)	setPWM12_1(150 + a)

/* Private variables ---------------------------------------------------------*/
#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment = 4   
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN USB_OTG_CORE_HANDLE  USB_OTG_dev __ALIGN_END;

// __IO uint32_t TimingDelay;

/* 1号機用校正データ*/
const float ADXL345_Px = 9.500193;
const float ADXL345_Nx = -10.419566;
const float ADXL345_Py = 10.419566;
const float ADXL345_Ny = -9.500193;
const float ADXL345_Pz = 9.500193;
const float ADXL345_Nz = -9.80665;

const float compassN = 7.016667f;
// const float compassN = 8.55f;
			
// 松戸あたりでは、偏角7 °1′(西偏) 伏角49 °24′
// 大樹町では、    偏角8 °33′(西偏)伏角56 °31′
// 日本国内では、磁北は真北より約5 - 10度西へ偏っている。
// 射点位置はN42°30'15'' E143°27'24'' (参考：なつまつり報告)
// 着水目標はN42°29' 1'' E143°30'39'' (参考：いちごプレスリリース)
// 打上げ方向は、真北から時計回りに130度ほどか？
// 計算上、117.237081188807度

// LED用
uint8_t countLED_GREEN = 0;
uint8_t statusLED_GREEN = 0;
uint8_t countLED_ORANGE = 0;
uint8_t statusLED_ORANGE = 0;
uint8_t countLED_RED = 0;
uint8_t statusLED_RED = 0;

extern uint32_t SystemCoreClock;

GPIO_InitTypeDef  GPIO_InitStructure;
portTickType xLastWakeTime;

/* Private function prototypes -----------------------------------------------*/
static void vloopTask( void *pvParameters );


/* Private functions ---------------------------------------------------------*/
void initTIM()
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  TIM_OCInitTypeDef  TIM_OCInitStructure;
  /* TIM3 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  /* TIM12 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);
  /* GPIOC clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
  /* GPIOB clock enable */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

  /* GPIOC Configuration: TIM3 CH2 (PC7) */
  /* GPIOC Configuration: TIM3 CH1 (PC6) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6|GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOC, &GPIO_InitStructure); 
  /* Connect TIM Channels to AF1 */
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_TIM3);
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_TIM3);
  /* GPIOB Configuration: TIM12 CH2 (PB15) */
  /* GPIOB Configuration: TIM12 CH1 (PB14) */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14|GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure); 
  /* Connect TIM Channels to AF1 */
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource15, GPIO_AF_TIM12);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource14, GPIO_AF_TIM12);

  uint16_t PrescalerValue = (uint16_t) ((SystemCoreClock /2) / 100000) - 1;
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 999;
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  /* Output Compare Timing Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 150;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC1Init(TIM3, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Disable);
  /* Output Compare Timing Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 150;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC2Init(TIM3, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM3, TIM_OCPreload_Disable);
  /* Time base configuration */
  TIM_TimeBaseStructure.TIM_Period = 999;
  TIM_TimeBaseStructure.TIM_Prescaler = PrescalerValue;
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM12, &TIM_TimeBaseStructure);
  /* Output Compare Timing Mode configuration: Channel1 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 150;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC1Init(TIM12, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM12, TIM_OCPreload_Disable);
  /* Output Compare Timing Mode configuration: Channel2 */
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = 150;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC2Init(TIM12, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM12, TIM_OCPreload_Disable);

  /* TIM3 enable counter */
  TIM_Cmd(TIM3, ENABLE);
  /* TIM12 enable counter */
  TIM_Cmd(TIM12, ENABLE);
}

void setPWM3_1(uint32_t i)
{
  TIM_SetCompare1(TIM3,i);
}

void setPWM3_2(uint32_t i)
{
  TIM_SetCompare2(TIM3,i);
}

void setPWM12_1(uint32_t i)
{
  TIM_SetCompare1(TIM12,i);
}

void setPWM12_2(uint32_t i)
{
  TIM_SetCompare2(TIM12,i);
}

DWORD acc_size;
WORD acc_files, acc_dirs;
FILINFO Finfo;
DIR  dir;
FATFS Fatfs[_VOLUMES];
FIL fileR;
// BYTE Buff[BUFSIZE] __attribute__ ((aligned (4))) ;
__IO uint8_t Command_index = 0;
UINT nTextFile; // TXT ファイルの数
FATFS *fs;  /* Pointer to file system object */

#if _USE_LFN
char Lfname[512];
#endif

uint32_t get_fattime (void)
{
  uint32_t res;
  /* See rtc_support.h */
//  RTC_GetTime(RTC_Format_BIN, &RTC_TimeStructure);  
//  RTC_GetDate(RTC_Format_BIN, &RTC_DateStructure);
  
  res =  (( (uint32_t)ts_year + (2000 - 1980)) << 25)
      | ((uint32_t)(ts_mon) << 21)
      | ((uint32_t)ts_mday << 16)
      | (uint32_t)(ts_hour << 11)
      | (uint32_t)(ts_min << 5)
      | (uint32_t)(ts_sec);
  return res;
}

static void put_rc (FRESULT rc)
{
  const char *str =
    "OK\0" "DISK_ERR\0" "INT_ERR\0" "NOT_READY\0" "NO_FILE\0" "NO_PATH\0"
    "INVALID_NAME\0" "DENIED\0" "EXIST\0" "INVALID_OBJECT\0" "WRITE_PROTECTED\0"
    "INVALID_DRIVE\0" "NOT_ENABLED\0" "NO_FILE_SYSTEM\0" "MKFS_ABORTED\0" "TIMEOUT\0"
    "LOCKED\0" "NOT_ENOUGH_CORE\0" "TOO_MANY_OPEN_FILES\0";
  FRESULT i;

  for (i = 0; i != rc && *str; i++) {
    while (*str++) ;
  }
  printf("rc=%u FR_%s\n", (UINT)rc, str);
}


static FRESULT scan_files (
  char* path /* Pointer to the path name working buffer */
)
{
  DIR dirs;
  FRESULT res;
  BYTE i;
  char *fn;
  if ((res = f_opendir(&dirs, path)) == FR_OK) {
    i = strlen(path);
    while (((res = f_readdir(&dirs, &Finfo)) == FR_OK) && Finfo.fname[0]){
      if (_FS_RPATH && Finfo.fname[0] == '.') continue;
#if _USE_LFN
      fn = *Finfo.lfname ? Finfo.lfname : Finfo.fname;
#else
      fn = Finfo.fname;
#endif
      if (Finfo.fattrib & AM_DIR) {
        acc_dirs++;
        *(path+i) = '/';
        strcpy(path+i+1, fn);
        res = scan_files(path);
        *(path+i) = '\0';
        if (res != FR_OK) break;
      } else {
        acc_files++;
        acc_size += Finfo.fsize;
      }
    }
  }
  return res;
}

FRESULT initSD()
{
  BYTE res;
  long p2;
  static const BYTE ft[] = {0, 12, 16, 32};

#if _USE_LFN
  Finfo.lfname = Lfname;
  Finfo.lfsize = sizeof(Lfname);
#endif
  f_mount(0, &Fatfs[0]);
  printf("_MAX_SS : %d\n",_MAX_SS);
  printf("FatFs module test terminal for %s\n",MPU_SUBMODEL);
  cputs(_USE_LFN ? "LFN Enabled" : "LFN Disabled");
  printf(", Code page: %u\n", _CODE_PAGE);
  res = f_getfree("", (DWORD*)&p2, &fs);
  if (res) {
    put_rc(res);
  } else {
    printf("FAT type = FAT%u\nBytes/Cluster = %lu\nNumber of FATs = %u\n"
			"Root DIR entries = %u\nSectors/FAT = %lu\nNumber of clusters = %lu\n"
			"FAT start (lba) = %lu\nDIR start (lba,clustor) = %lu\nData start (lba) = %lu\n\n...",
			ft[fs->fs_type & 3], (DWORD)fs->csize * 512, (WORD)fs->n_fats,
			fs->n_rootdir, fs->fsize, (DWORD)fs->n_fatent - 2,
			fs->fatbase, fs->dirbase, fs->database
    );
    acc_size = acc_files = acc_dirs = 0;
    Finfo.lfname = Lfname;
    Finfo.lfsize = sizeof(Lfname);
    res = scan_files("");
    if (res) {
      put_rc(res);
    } else {
      printf("\r%u files, %lu bytes.\n%u folders.\n"
			  "%lu KB total disk space.\n%lu KB available.\n",
			  acc_files, acc_size, acc_dirs,
			  (fs->n_fatent - 2) * (fs->csize / 2), p2 * (fs->csize / 2)
      );
    }
  }
  return res;
}

char upper(char c)
{
  if ((c >= 'a') && (c <= 'z'))
    return c - 'a' + 'A';
  return c;
}

int isEx(char *filename,char * ex)
{
  int p1,p2;
  p1 = 0;
  while ((filename[p1] != 0) && (filename[p1] != '.')) {
    p1++;
  }
  if (filename[p1] == 0)
    return 0;
  p1++;
  if (filename[p1] == 0)
    return 0;
  p2 = 0;
  while ((filename[p1] != 0) && (ex[p2] != 0)) {
    if (upper(filename[p1]) != upper(ex[p2]))
      return 0;
    p1++;
    p2++;
  }
  if ((filename[p1] == 0) && (ex[p2] == 0))
    return 1;
  return 0;
}

FRESULT lsSD()
{
  BYTE res;
  long p1, p2;
  UINT s1, s2;
  res = f_getfree("", (DWORD*)&p2, &fs);// testcode
  res = f_opendir(& dir, "");
  if (res) {
    put_rc(res);
  } else {
    p1 = s1 = s2 = 0;
    nTextFile = 0;
    for(;;) {
      res = f_readdir(& dir, &Finfo);
      if ((res != FR_OK) || !Finfo.fname[0]) break;
      if (Finfo.fattrib & AM_DIR) {
        s2++;
      } else {
        s1++; p1 += Finfo.fsize;
        if (isEx(&(Finfo.fname[0]),"TXT")) {
          nTextFile++;
        }
      }
      printf("%c%c%c%c%c %u/%02u/%02u %02u:%02u %9lu  %s",
          (Finfo.fattrib & AM_DIR) ? 'D' : '-',
          (Finfo.fattrib & AM_RDO) ? 'R' : '-',
          (Finfo.fattrib & AM_HID) ? 'H' : '-',
          (Finfo.fattrib & AM_SYS) ? 'S' : '-',
          (Finfo.fattrib & AM_ARC) ? 'A' : '-',
          (Finfo.fdate >> 9) + 1980, (Finfo.fdate >> 5) & 15, Finfo.fdate & 31,
          (Finfo.ftime >> 11), (Finfo.ftime >> 5) & 63,
          Finfo.fsize, &(Finfo.fname[0]));
#if _USE_LFN
      for (p2 = strlen(Finfo.fname); p2 < 14; p2++)
        cputs(" ");
      printf("%s\n", Lfname);
#else
      cputs("\r\n");
#endif
      if (countLED_GREEN == 0) {
        if(statusLED_GREEN) {
		  GREEN_LED_OFF;
          statusLED_GREEN = 0;
        } else {
		  GREEN_LED_ON;
          statusLED_GREEN = 1;
        }
        countLED_GREEN = 3;
      } else {
        countLED_GREEN--;
      }
    }
    printf("%4u File(s),%10lu bytes total\n%4u  dir(s)", s1, p1, s2);
    res = f_getfree("", (DWORD*)&p1, &fs);
    if (res == FR_OK)
      printf(", %10lu bytes free\n", p1 * fs->csize * 512);
    else
      put_rc(res);
    printf("TXT File %u \n", nTextFile);
  }
  return res;
}

/**************************************************************************/
/*! 
    @brief  Main Program.
  @param  None.
    @retval None.
*/
/**************************************************************************/
FIL file;
#define FBUF_SIZE 512		// SDカードへ書き込むためのバッファサイズ
#define RING_BUF_SIZE 8
uint8_t ring_buffer[RING_BUF_SIZE][FBUF_SIZE];
volatile uint8_t ring_buffer_p_in  = 0;
volatile uint8_t ring_buffer_p_out = 0;
UINT pBuf = 0;
UINT flgSDcard = 0;

void putcSD(uint8_t data)
{
	ring_buffer[ring_buffer_p_in][pBuf] = data;
	pBuf++;
	if (pBuf >= FBUF_SIZE) {
		pBuf = 0;
		ring_buffer_p_in ++;
		if (ring_buffer_p_in >= RING_BUF_SIZE) {
			ring_buffer_p_in = 0;
		}
	}
}

void chechAndWriteSD()
{
	int i,n;
	UINT fbuflen;
	if (flgSDcard) {
		n = ring_buffer_p_in - ring_buffer_p_out;
		if (n < 0) {
			n += RING_BUF_SIZE;
		}
		if (n >= 2) {
			n -= 1;
		}
		if (n > 0) {
			f_open(&file,Lfname,FA_READ|FA_WRITE);
			f_lseek(&file,f_size(&file));
			for (i = 0;i < n;i++) {
				f_write (&file,ring_buffer[ring_buffer_p_out],FBUF_SIZE,&fbuflen);
				ring_buffer_p_out++;
				if (ring_buffer_p_out >= RING_BUF_SIZE) {
					ring_buffer_p_out = 0;
				}
			}
			f_close(&file);
		}
	}
}

// 外部I/F	内容	STM32F405のピン番号
// LED1		PA5		21
// LED2		PA6		22
// LED3		PA7		23


GPIO_InitTypeDef  GPIO_InitStructure;
/**************************************************************************/
int main(void)
{
	cpustatus = STARTING;
	__IO uint32_t i = 0;  
	uint8_t buf[255];
	uint8_t len;
	
	/* Set Basis System */
	Set_System();
	
 	USBD_Init(&USB_OTG_dev,     
            USB_OTG_FS_CORE_ID, 
            &USR_desc, 
            &USBD_CDC_cb, 
            &USR_cb);

	Delay(1000);
	
	/* GPIO Periph clock enable */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA|RCC_AHB1Periph_GPIOB
						  |RCC_AHB1Periph_GPIOC|RCC_AHB1Periph_GPIOD
						  |RCC_AHB1Periph_GPIOE|RCC_AHB1Periph_GPIOF
						  |RCC_AHB1Periph_GPIOG, ENABLE);
    
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0000);
	NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4 );
	
	/* Configure HCLK clock as SysTick clock source. */
	SysTick_CLKSourceConfig( SysTick_CLKSource_HCLK );
	/* Set SysTickCounter for _delay_ms(); */
	SysTickInit(INTERVAL);
	
	initTIM();
	SERVO1(0);
	SERVO2(0);
	SERVO3(0);
	SERVO4(0);
	I2Cs_Initialize();
	/* PA5/6/7 for LED1/2/3	*/ 
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GREEN_LED_ON;
	ORANGE_LED_ON;
	RED_LED_ON;

	VCP_send_str("USB serial Hello world\n");
            
	xTaskCreate( vloopTask, ( signed char * ) "VCP", 4096, NULL,1,NULL);
	vTaskStartScheduler();
	/* Main Loop */
	while (1)
	{

	}
}

static void vloopTask( void *pvParameters )
{
	xLastWakeTime = xTaskGetTickCount ();
	for( ;; ){
		vTaskDelayUntil(&xLastWakeTime, 1000/portTICK_RATE_MS);
		VCP_send_str("USB Hoge\n");
	}
}

/**************************************************************************/

void Fail_Handler(void)
{
	/* Erase last sector */ 
	FLASH_EraseSector(FLASH_Sector_11, VoltageRange_3);
	/* Write FAIL code at last word in the flash memory */
	FLASH_ProgramWord(TESTRESULT_ADDRESS, ALLTEST_FAIL);
	while(1){
	/* Toggle Red LED */
	//  STM_EVAL_LEDToggle(LED5);
		Delay(5);
	}
}


#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
	/* User can add his own implementation to report the file name and line
	number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* Infinite loop */
	while (1){
	}
}
#endif
/**************************************************************************/


