#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"

#define IMAGE_BUFFER_SIZE		640
#define WIDTH_SLOPE				5
#define MIN_LINE_WIDTH			40
#define ROTATION_THRESHOLD		10
#define GOAL_DISTANCE 			8
#define MAX_DISTANCE 			25
#define MAX_SPEED 				400    //steps/s
#define ERROR_THRESHOLD			0.9f   //[cm] because of the noise of the camera
#define KP_1					200    //800
#define KD_1					50     //250
#define KI_1					3.5f   //50
#define MAX_SUM_ERROR 			(MAX_SPEED/KI_1)
#define KP_2					2
#define KD_2					0.01f
#define PI_CLOCK				10     // in ms
#define GO 						1
#define STOP 					0
#define NB_PX_LOCAL_MEAN   		200
#define FOLLOW_LINE				0
#define TRAFFIC_LIGHT			1
#define MIN_INT_THRESHOLD		30 	   //minimal intensity threshold
#define	MAX_INT_THRESHOLD		180	   //maximal intensity threshold
#define MIN_INT_NB_PX			1
#define MAX_INT_NB_PX			40
#define MODE_TRAFFIC			1
#define MODE_LINE				479

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

//à enlever
void SendUint8ToComputer(uint8_t* data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif
