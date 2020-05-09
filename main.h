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
#define MAX_SPEED 				400    //steps/s
#define ERROR_THRESHOLD			0.9f   //[cm] because of the noise of the camera
#define KP_1					200
#define KD_1					50
#define KI_1					3.5f
#define MAX_SUM_ERROR 		    (MAX_SPEED/KI_1)
#define KP_2					2
#define KD_2					0.01f
#define PI_CLOCK				10     // in ms
#define GO 						1
#define STOP 					0
#define NB_PX_LOCAL_SUM   		200
#define MIN_INT_THRESHOLD		30 	   //minimal intensity threshold
#define	MAX_INT_THRESHOLD		180	   //maximal intensity threshold

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

#ifdef __cplusplus
}
#endif

#endif
