#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"


//constants for the differents parts of the project
#define IMAGE_BUFFER_SIZE		640
#define WIDTH_SLOPE				5
#define MIN_LINE_WIDTH			20
#define ROTATION_COEFF			2 
#define MMTOCM					0.1f	//used to convert [mm] to [cm]
#define GOAL_OBSTICLE_DISTANCE 	15.0f	//[cm]
#define MAX_OBSTICLE_DISTANCE	30.0f	//[cm]
#define MIN_OBSTICLE_DISTANCE	7.0f	//[cm]
#define ROTATION_THRESHOLD		18
#define ERROR_THRESHOLD			0.6f	//[cm] because of the noise of the TOF sensor
#define KP1						1.2f
#define KP2						400.0f
#define KI2						2.0f
#define MAX_SUM_ERROR 			(MOTOR_SPEED_LIMIT/KI2)
#define MAX_SELECTOR			15
#define CRUISE_SPEED			600
#define SHIFT_R					11
#define SHIFT_G					5
#define MASK_R					0xF800
#define MASK_G					0x07E0
#define	MASK_B					0x001F

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

void SendUint8ToComputer(uint8_t* data, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif
