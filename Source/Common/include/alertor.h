#ifndef __ALERTOR_H
#define __ALERTOR_H

//------------------ Sensor -------------------------
//#define ENABLE_LIGHT_SENSOR
#define ENABLE_INFRARED_SENSOR
#define ENBALE_RANGEFINDER_SENSOR

#define INFRARED_SENSOR_1
//#define INFRARED_SENSOR_2
//-----------------------------------------------------

extern unsigned char alert_flag;
#define INFRARED1_ALERT 		(0x01)
#define INFRARED2_ALERT 		(0x02)
#define RANGEFINDER_ALERT		(0x04)
#define LIGHT_CONDITION			(0x80)

void vAlertorTask(void * pvParameters);

#endif
