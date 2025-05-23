// RAK3172-SIP - Analoges Zeugs
  // Beispiel C: c:\dokus\Lora\rak\rui3\cores\STM32WLE\component\service\battery\service_battery.c
  //  driver: C:\dokus\Lora\rak\rui3\cores\STM32WLE\component\core\mcu\stm32wle5xx\uhal\uhal_adc.c
  
  // RUI3 Doku: https://docs.rakwireless.com/product-categories/software-apis-and-libraries/rui3/arduino-api
  
  // Oversampling wohl nicht vorgesehen/implementiert in RUI3

#include "udrv_adc.h"

#define analogPin PB4 // - am SIP-Eval hinten!

int val = 0;  // variable to store the value read

void setup()
{
  Serial.begin(115200);
  Serial.printf("Analog\n");
  
  udrv_adc_set_resolution(UDRV_ADC_RESOLUTION_12BIT); // STM32 hat ADC12-Bit
}

void loop()
{
  uint32_t t0 = millis();
  val = analogRead(analogPin);  // read the input pin

  Serial.printf("T:%u ArdAD:%d ",t0,val);          // Eigtl.

  int16_t val16;
  udrv_adc_read(analogPin,&val16);  // val und val16 wohl beide das selbe

  int16_t bat16;
  udrv_adc_read(UDRV_ADC_CHANNEL_VBAT,&bat16);
  int16_t ref16;
  udrv_adc_read(UDRV_ADC_CHANNEL_VREFINT,&ref16);

  float vbatf; 
  service_battery_get_batt_level(&vbatf);
  float vbats = api.system.bat.get();
  Serial.printf(" -B:%.3f %.3f ",vbatf,vbats); // Messen wohl beide das selbe

  // Oversampling-Test mit Temperatur
  // Temperatursensor rauscht ordentlich, aber mit 10 Mittelungen/8msec kommt man auf +/- 0.1 stablie Werte
  #define ANZ_TAV  10   // x Mittelungen fuer Temperatur
  int32_t temp32=0;
  uint32_t tc0 = millis();
  for(uint16_t i=0;i<ANZ_TAV;i++){
  int16_t temp16;
    udrv_adc_read(UDRV_ADC_CHANNEL_TEMPSENSOR,&temp16); // ca. 800usec/Sample
    temp32+=temp16;
  }
  uint32_t tce = millis();
  float tempadf = (float)temp32/(float)ANZ_TAV;
  int16_t tcal30 = *(uint16_t*)0x1FFF75A8; // ca. 950 Ausm Datenblatt 3.18.1
  int16_t tcal130 = *(uint16_t*)0x1FFF75C8; // ca. 1200
  float tempf;
  if(tcal130>tcal30) tempf = (100.0 / (float)(tcal130-tcal30)) * (tempadf-(float)tcal30) + 30.0 ;
  else tempf= -99.0;

  Serial.printf(" Tf:%.2f M:%d", tempf, tce-tc0);


  Serial.printf(" --- UvalAD: %d Bat:%d Ref:%d\n",val16, bat16,ref16);  


  delay(500);
}