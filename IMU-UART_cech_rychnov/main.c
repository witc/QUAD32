#include "main.h"

#define DBG

FATFS FatFs;
FIL File;

void InitGPIO()
{
  STM_EVAL_LEDInit(LED4);
  STM_EVAL_LEDInit(LED5);
  STM_EVAL_LEDInit(LED7);
  STM_EVAL_LEDInit(LED10);
  STM_EVAL_PBInit(BUTTON_USER, BUTTON_MODE_GPIO);
  STM_EVAL_LEDOff(LED4);
  STM_EVAL_LEDOff(LED5);
  STM_EVAL_LEDOff(LED7);
  STM_EVAL_LEDOff(LED10);
}

FRESULT newFile() {
  char fileName[] = "data00.txt\0";
  int i = 0;

  disk_initialize(0);
  f_mount(&FatFs, "", 0);
  while (f_stat(fileName, NULL) == FR_OK) {
    i++;
    fileName[4] = i / 10 + '0';
    fileName[5] = i % 10 + '0';
  }
  return f_open(&File, fileName, FA_WRITE | FA_CREATE_ALWAYS);
}

float a[3], m[3], g[3];
float quaternion[4] = {1.0f, 0, 0, 0};
float samplePeriod = 1.0f/200.0f;

char text[128];

int main(void)
{
  int LogOn = 0;
  int ComOn = 0;
  int dT;
  int16_t data[32];
  uint32_t x;
  UINT cnt;

  InitSystemTick();
  InitGPIO();
  InitTimer();
  STM_EVAL_LEDOn(LED4);

  InitUART(115200);
  InitPressureSensor();
  InitFlowMeter();

  InitAccAndMag();
  InitGyro();

  STM_EVAL_LEDOn(LED7); // zapnem LED7 - zelena

  Delta_us();
  while(1)
  {
    // sample period is time elapsed since previous sampling of sensors
    sampleSensors(a,m,g);
    // update timebase for next time
    dT = Delta_us();
    samplePeriod = 0.000001f * dT;
    // convert gyro deg/s to rad/s
    imuDegToRadV3(g);
    // update AHRS
    MadgwickFullAHRSUpdate(g, a, m, samplePeriod, quaternion);

    if (LogOn) {
      sprintf(text, "%5d%6d%6d%6d%7d%7d%7d%5d%5d%5d\r\n",
              dT,
              aRawData[0], aRawData[1], aRawData[2],
              gRawData[0], gRawData[1], gRawData[2],
              mRawData[0], mRawData[1], mRawData[2]);
      f_write(&File, text, strlen(text), &cnt);
    }

    if(STM_EVAL_PBGetState(BUTTON_USER)) {   // zmena rezimu vystupu
      if (LogOn) {
        // Stop logdata
        LogOn = 0;
        STM_EVAL_LEDOff(LED5); // zapnem LED7 - zelena
        if (f_close(&File) == 0)
          xprintf("Stop login data.\n\r");
        else
          xprintf("Error write datafile.\n\r");
      } else {
        // Start logdata
        if (newFile() == 0)
          xprintf("Start login data.\n\r");
        else
          xprintf("Error create datafile.\n\r");
        STM_EVAL_LEDOn(LED5); // zapnem LED7 - zelena
        LogOn = 1;
      }
    }

    if (ComOn) {
      data[0] = dT;
      data[1] = aRawData[0];
      data[2] = aRawData[1];
      data[3] = aRawData[2];
      data[4] = gRawData[0];
      data[5] = gRawData[1];
      data[6] = gRawData[2];
      data[7] = mRawData[0];
      data[8] = mRawData[1];
      data[9] = mRawData[2];
      data[10] = 0x8080;
      SendDataUART((uint8_t*) data, 22);
    }

    if (IsReceiveUART()) {
      if (ReadByteUART() == 's') {
        STM_EVAL_LEDOn(LED10); // zapnem LED7 - zelena
        ComOn = 1;
      } else {
        STM_EVAL_LEDOff(LED10); // zapnem LED7 - zelena
        ComOn = 0;
      }
    }
/*
    if (x = GetFlowMeter()) {
      printf("Flow: %d\n\r", x);
    }
*/
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line) {
  while (1) {}
}
#endif
