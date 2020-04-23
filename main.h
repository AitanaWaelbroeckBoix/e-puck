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
#define ROTATION_COEFF			2
#define PXTOCM					1570.0f //experimental value
#define GOAL_DISTANCE 			10.0f
#define MAX_DISTANCE 			25.0f
#define ERROR_THRESHOLD			0.1f	//[cm] because of the noise of the camera#define KP_1					800.0f
#define KI						3.5f	//must not be zero
#define KD_1					1      // sans unit� ( � tuner )
#define KP_2					2
#define KD_2					1   // [m]  (� tuner)
#define PI_CLOCK				10    // ATTENTION en ms � prendre en compte pour les KD
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI)
#define GO 						1
#define STOP 					0

#define NB_PX_LOCAL_MEAN   		200
#define MEAN_COEFF				1.5
#define MIN_CONTRAST			2
#define CONTRAST_COEFF			0.9


/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

void SendUint8ToComputer(uint8_t* data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif
