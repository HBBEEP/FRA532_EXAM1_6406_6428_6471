/*
 *
 *
 *
 */

#ifndef Motor_h
#define Motor_h

	// EEPROM AREA  ///////////////////////////////////////////////////////////
#define MOTOR_MODEL_NUMBER_L           0
#define MOTOR_MODEL_NUMBER_H           1
#define MOTOR_VERSION                  2
#define MOTOR_ID                       3
#define MOTOR_BAUD_RATE                4
#define MOTOR_RETURN_DELAY_TIME        5
#define MOTOR_CW_ANGLE_LIMIT_L         6
#define MOTOR_CW_ANGLE_LIMIT_H         7
#define MOTOR_CCW_ANGLE_LIMIT_L        8
#define MOTOR_CCW_ANGLE_LIMIT_H        9
#define MOTOR_SYSTEM_DATA2             10
#define MOTOR_LIMIT_TEMPERATURE        11
#define MOTOR_DOWN_LIMIT_VOLTAGE       12
#define MOTOR_UP_LIMIT_VOLTAGE         13
#define MOTOR_MOTOR_TORQUE_L          14
#define MOTOR_MOTOR_TORQUE_H          15
#define MOTOR_RETURN_LEVEL             16
#define MOTOR_ALARM_LED                17
#define MOTOR_ALARM_SHUTDOWN           18
#define MOTOR_OPERATING_MODE           19
#define MOTOR_DOWN_CALIBRATION_L       20
#define MOTOR_DOWN_CALIBRATION_H       21
#define MOTOR_UP_CALIBRATION_L         22
#define MOTOR_UP_CALIBRATION_H         23

	// RAM AREA  //////////////////////////////////////////////////////////////
#define MOTOR_TORQUE_ENABLE            24
#define MOTOR_LED                      25
#define MOTOR_CW_COMPLIANCE_MARGIN     26
#define MOTOR_CCW_COMPLIANCE_MARGIN    27
#define MOTOR_CW_COMPLIANCE_SLOPE      28
#define MOTOR_CCW_COMPLIANCE_SLOPE     29
#define MOTOR_GOAL_POSITION_L          30
#define MOTOR_GOAL_POSITION_H          31
#define MOTOR_GOAL_SPEED_L             32
#define MOTOR_GOAL_SPEED_H             33
#define MOTOR_TORQUE_LIMIT_L           34
#define MOTOR_TORQUE_LIMIT_H           35
#define MOTOR_PRESENT_POSITION_L       36
#define MOTOR_PRESENT_POSITION_H       37
#define MOTOR_PRESENT_SPEED_L          38
#define MOTOR_PRESENT_SPEED_H          39
#define MOTOR_PRESENT_LOAD_L           40
#define MOTOR_PRESENT_LOAD_H           41
#define MOTOR_PRESENT_VOLTAGE          42
#define MOTOR_PRESENT_TEMPERATURE      43
#define MOTOR_REGISTERED_INSTRUCTION   44
#define MOTOR_PAUSE_TIME               45
#define MOTOR_MOVING                   46
#define MOTOR_LOCK                     47
#define MOTOR_PUNCH_L                  48
#define MOTOR_PUNCH_H                  49
#define Goal_Acceleration				73

    // Status Return Levels ///////////////////////////////////////////////////////////////
#define MOTOR_RETURN_NONE              0
#define MOTOR_RETURN_READ              1
#define MOTOR_RETURN_ALL               2

    // Instruction Set ///////////////////////////////////////////////////////////////
#define MOTOR_PING                     1
#define MOTOR_READ_DATA                2
#define MOTOR_WRITE_DATA               3
#define MOTOR_REG_WRITE                4
#define MOTOR_ACTION                   5
#define MOTOR_RESET                    6
#define MOTOR_SYNC_WRITE               131

	// Specials ///////////////////////////////////////////////////////////////
#define OFF                         0
#define ON                          1
#define LEFT						0
#define RIGHT                       1
#define MOTOR_BYTE_READ                1
#define MOTOR_BYTE_READ_POS            2
#define MOTOR_RESET_LENGTH				2
#define MOTOR_ACTION_LENGTH			2
#define MOTOR_ID_LENGTH                4
#define MOTOR_LR_LENGTH                4
#define MOTOR_SRL_LENGTH               4
#define MOTOR_RDT_LENGTH               4
#define MOTOR_LEDALARM_LENGTH          4
#define MOTOR_SALARM_LENGTH            4
#define MOTOR_TL_LENGTH                4
#define MOTOR_VL_LENGTH                6
#define MOTOR_CM_LENGTH                6
#define MOTOR_CS_LENGTH                6
#define MOTOR_CCW_CW_LENGTH            8
#define MOTOR_BD_LENGTH                4
#define MOTOR_TEM_LENGTH               4
#define MOTOR_MOVING_LENGTH            4
#define MOTOR_ACC_LENGTH            	4
#define MOTOR_RWS_LENGTH               4
#define MOTOR_VOLT_LENGTH              4
#define MOTOR_LED_LENGTH               4
#define MOTOR_TORQUE_LENGTH            4
#define MOTOR_POS_LENGTH               4
#define MOTOR_GOAL_LENGTH              5
#define MOTOR_MT_LENGTH                5
#define MOTOR_PUNCH_LENGTH             5
#define MOTOR_SPEED_LENGTH             5
#define MOTOR_GOAL_SP_LENGTH           7
#define MOTOR_ACTION_CHECKSUM			250
#define BROADCAST_ID                254
#define MOTOR_START                    255
#define MOTOR_CCW_AL_L                 255 
#define MOTOR_CCW_AL_H                 3
#define TIME_OUT                    10
#define TX_MODE                     1
#define RX_MODE                     0
#define LOCK                        1

#include <inttypes.h>

class MOTOR {
private:
	
	unsigned char Checksum;
	unsigned char Direction_Pin;
	unsigned char Time_Counter;
	unsigned char Incoming_Byte;               
	unsigned char Position_High_Byte;
	unsigned char Position_Low_Byte;
	unsigned char Speed_High_Byte;
	unsigned char Speed_Low_Byte;
	unsigned char Load_High_Byte;
	unsigned char Load_Low_Byte;
	
	int Moving_Byte;
	int RWS_Byte;
	int Speed_Long_Byte;
	int Load_Long_Byte;
	int Position_Long_Byte;
	int Temperature_Byte;
	int Voltage_Byte;
	int Error_Byte; 

	int returned_Byte;

	int read_error(void); 

	HardwareSerial *varSerial;

public:
	void begin(long baud, unsigned char directionPin, HardwareSerial *srl);
	void end(void);
	
	int reset(unsigned char ID);
	int ping(unsigned char ID); 
	
	int setID(unsigned char ID, unsigned char newID);
	int setBD(unsigned char ID, long baud);
	
	int move(unsigned char ID, int Position);
	int moveSpeed(unsigned char ID, int Position, int Speed);
	int setEndless(unsigned char ID,bool Status);
	int turnWheel(unsigned char ID, bool SIDE, int Speed);
	int moveRW(unsigned char ID, int Position);
	int moveSpeedRW(unsigned char ID, int Position, int Speed);
	
	void action(void);
	
	int setTempLimit(unsigned char ID, unsigned char Temperature);
	int setAngleLimit(unsigned char ID, int CWLimit, int CCWLimit);
	int setVoltageLimit(unsigned char ID, unsigned char DVoltage, unsigned char UVoltage);
	int setMMOTORTorque(unsigned char ID, int MMOTORTorque);
	int setSRL(unsigned char ID, unsigned char SRL);
	int setRDT(unsigned char ID, unsigned char RDT);
	int setLEDAlarm(unsigned char ID, unsigned char LEDAlarm);
	int setShutdownAlarm(unsigned char ID, unsigned char SALARM);
	int setCMargin(unsigned char ID, unsigned char CWCMargin, unsigned char CCWCMargin);
	int setCSlope(unsigned char ID, unsigned char CWCSlope, unsigned char CCWCSlope);
	int setPunch(unsigned char ID, int Punch);
	
	int moving(unsigned char ID);
	int lockRegister(unsigned char ID);
	int RWStatus(unsigned char ID);
	
	int readTemperature(unsigned char ID);
	int readVoltage(unsigned char ID);
	int readPosition(unsigned char ID);
	int readSpeed(unsigned char ID);
	int readLoad(unsigned char ID);
	
	int torqueStatus(unsigned char ID, bool Status);
	int ledStatus(unsigned char ID, bool Status);

	int sendMotorPacket(unsigned char *packet, unsigned int length);
	void sendMotorPacketNoError(unsigned char *packet, unsigned int length);

	int readRegister(unsigned char ID, unsigned char reg, unsigned char reg_len);
	int setAcceleration(unsigned char ID, int Acceleration);
};

extern MOTOR Motor;

#endif
