#if defined(ARDUINO) && ARDUINO >= 100  // Arduino IDE Version
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "Motor.h"

// Macros /////////////////////////////////////////////////////////////////////

#define sendData(packet, length)  	(varSerial->write(packet, length))    	// Write Over Serial
#define flush()						(varSerial->flush())					// Wait until buffer empty
#define availableData() 			(varSerial->available())    			// Check Serial Data Available
#define readData()      			(varSerial->read())         			// Read Serial Data
#define peekData()      			(varSerial->peek())         			// Peek Serial Data
#define beginCom(args)  			(varSerial->begin(args))   				// Begin Serial Comunication
#define endCom()        			(varSerial->end())          			// End Serial Comunication

#define setDPin(DirPin,Mode)   		(pinMode(DirPin,Mode))       			// Select the Switch to TX/RX Mode Pin
#define switchCom(DirPin,Mode) 		(digitalWrite(DirPin,Mode))  			// Switch to TX/RX Mode

#define delayus(args) 				(delayMicroseconds(args))  				// Delay Microseconds

// Private Methods ////////////////////////////////////////////////////////////

int MOTOR::read_error(void)
{
	
	Time_Counter = 0;
	while((availableData() < 5) & (Time_Counter < TIME_OUT)) // Wait for Data
	{
		Time_Counter++;
		delayus(1000);
	}
	
	while (availableData() > 0)
	{
		Incoming_Byte = readData();
		if ( (Incoming_Byte == 255) & (peekData() == 255) )
		{
			readData();                                    // Start Bytes
			readData();                                    // MOTOR-12 ID
			readData();                                    // Length
			Error_Byte = readData();                       // Error
				return (Error_Byte);
		}
	}
	return (-1);											 // No MOTOR Response
}

// Public Methods /////////////////////////////////////////////////////////////

void MOTOR::begin(long baud, unsigned char directionPin, HardwareSerial *srl)
{
	varSerial = srl;
	Direction_Pin = directionPin;
	setDPin(Direction_Pin, OUTPUT);
	beginCom(baud);
}

void MOTOR::end()
{
	endCom();
}

int MOTOR::reset(unsigned char ID)
{
	const unsigned int length = 6;
	unsigned char packet[length];
	
	Checksum = (~(ID + MOTOR_RESET_LENGTH + MOTOR_RESET)) & 0xFF;

	packet[0] = MOTOR_START;
	packet[1] = MOTOR_START;
	packet[2] = ID;
	packet[3] = MOTOR_RESET_LENGTH;
	packet[4] = MOTOR_RESET;
	packet[5] = Checksum;

    return (sendMotorPacket(packet, length));
}

int MOTOR::ping(unsigned char ID)
{
	const unsigned int length = 6;
	unsigned char packet[length];

	Checksum = (~(ID + MOTOR_READ_DATA + MOTOR_PING)) & 0xFF;
	
	packet[0] = MOTOR_START;
	packet[1] = MOTOR_START;
	packet[2] = ID;
	packet[3] = MOTOR_READ_DATA;
	packet[4] = MOTOR_PING;
	packet[5] = Checksum;

	return (sendMotorPacket(packet, length));
}

int MOTOR::setID(unsigned char ID, unsigned char newID)
{    
	const unsigned int length = 8;
	unsigned char packet[length];

	Checksum = (~(ID + MOTOR_ID_LENGTH + MOTOR_WRITE_DATA + MOTOR_ID + newID)) & 0xFF;

	packet[0] = MOTOR_START;
	packet[1] = MOTOR_START;
	packet[2] = ID;
	packet[3] = MOTOR_ID_LENGTH;
	packet[4] = MOTOR_WRITE_DATA;
	packet[5] = MOTOR_ID;
	packet[6] = newID;
	packet[7] = Checksum;

    return (sendMotorPacket(packet, length));
}

int MOTOR::setBD(unsigned char ID, long baud)
{    
	const unsigned int length = 8;
	unsigned char packet[length];

	unsigned char Baud_Rate = (2000000/baud) - 1;
	Checksum = (~(ID + MOTOR_BD_LENGTH + MOTOR_WRITE_DATA + MOTOR_BAUD_RATE + Baud_Rate)) & 0xFF;

	packet[0] = MOTOR_START;
	packet[1] = MOTOR_START;
	packet[2] = ID;
	packet[3] = MOTOR_BD_LENGTH;
	packet[4] = MOTOR_WRITE_DATA;
	packet[5] = MOTOR_BAUD_RATE;
	packet[6] = Baud_Rate;
	packet[7] = Checksum;
    
	return (sendMotorPacket(packet, length));
}

int MOTOR::move(unsigned char ID, int Position)
{
    char Position_H,Position_L;
    Position_H = Position >> 8;           // 16 bits - 2 x 8 bits variables
    Position_L = Position;

    const unsigned int length = 9;
    unsigned char packet[length];

	Checksum = (~(ID + MOTOR_GOAL_LENGTH + MOTOR_WRITE_DATA + MOTOR_GOAL_POSITION_L + Position_L + Position_H)) & 0xFF;

    packet[0] = MOTOR_START;
    packet[1] = MOTOR_START;
    packet[2] = ID;
    packet[3] = MOTOR_GOAL_LENGTH;
    packet[4] = MOTOR_WRITE_DATA;
    packet[5] = MOTOR_GOAL_POSITION_L;
    packet[6] = Position_L;
    packet[7] = Position_H;
    packet[8] = Checksum;

    return (sendMotorPacket(packet, length));
}

int MOTOR::moveSpeed(unsigned char ID, int Position, int Speed)
{
    char Position_H,Position_L,Speed_H,Speed_L;
    Position_H = Position >> 8;    
    Position_L = Position;                // 16 bits - 2 x 8 bits variables
    Speed_H = Speed >> 8;
    Speed_L = Speed;                      // 16 bits - 2 x 8 bits variables

    const unsigned int length = 11;
    unsigned char packet[length];
    
    Checksum = (~(ID + MOTOR_GOAL_SP_LENGTH + MOTOR_WRITE_DATA + MOTOR_GOAL_POSITION_L + Position_L + Position_H + Speed_L + Speed_H)) & 0xFF;

    packet[0] = MOTOR_START;
    packet[1] = MOTOR_START;
    packet[2] = ID;
    packet[3] = MOTOR_GOAL_SP_LENGTH;
    packet[4] = MOTOR_WRITE_DATA;
    packet[5] = MOTOR_GOAL_POSITION_L;
    packet[6] = Position_L;
    packet[7] = Position_H;
    packet[8] = Speed_L;
    packet[9] = Speed_H;
    packet[10] = Checksum;

    return (sendMotorPacket(packet, length));
}

int MOTOR::setEndless(unsigned char ID, bool Status)
{
	if ( Status )
	{
		const unsigned int length = 9;
		unsigned char packet[length];

		Checksum = (~(ID + MOTOR_GOAL_LENGTH + MOTOR_WRITE_DATA + MOTOR_CCW_ANGLE_LIMIT_L)) & 0xFF;

	    packet[0] = MOTOR_START;
	    packet[1] = MOTOR_START;
	    packet[2] = ID;
	    packet[3] = MOTOR_GOAL_LENGTH;
	    packet[4] = MOTOR_WRITE_DATA;
	    packet[5] = MOTOR_CCW_ANGLE_LIMIT_L;
	    packet[6] = 0; 						// full rotation
	    packet[7] = 0;						// full rotation
	    packet[8] = Checksum;

	    return (sendMotorPacket(packet, length));
	}
	else
	{
		turnWheel(ID,0,0);

		const unsigned int length = 9;
		unsigned char packet[length];

		Checksum = (~(ID + MOTOR_GOAL_LENGTH + MOTOR_WRITE_DATA + MOTOR_CCW_ANGLE_LIMIT_L + MOTOR_CCW_AL_L + MOTOR_CCW_AL_H)) & 0xFF;

	    packet[0] = MOTOR_START;
	    packet[1] = MOTOR_START;
	    packet[2] = ID;
	    packet[3] = MOTOR_GOAL_LENGTH;
	    packet[4] = MOTOR_WRITE_DATA;
	    packet[5] = MOTOR_CCW_ANGLE_LIMIT_L;
	    packet[6] = MOTOR_CCW_AL_L;
	    packet[7] = MOTOR_CCW_AL_H;
	    packet[8] = Checksum;

	    return (sendMotorPacket(packet, length));
	}
}

int MOTOR::turnWheel(unsigned char ID, bool SIDE, int Speed)
{		
		Speed = Speed/0.916;
		if (SIDE == LEFT)
		{
			char Speed_H,Speed_L;
			Speed_H = Speed >> 8;
			Speed_L = Speed;                     // 16 bits - 2 x 8 bits variables
			
			const unsigned int length = 9;
			unsigned char packet[length];
			
			Checksum = (~(ID + MOTOR_SPEED_LENGTH + MOTOR_WRITE_DATA + MOTOR_GOAL_SPEED_L + Speed_L + Speed_H)) & 0xFF;

		    packet[0] = MOTOR_START;
		    packet[1] = MOTOR_START;
		    packet[2] = ID;
		    packet[3] = MOTOR_SPEED_LENGTH;
		    packet[4] = MOTOR_WRITE_DATA;
		    packet[5] = MOTOR_GOAL_SPEED_L;
		    packet[6] = Speed_L;
		    packet[7] = Speed_H;
		    packet[8] = Checksum;

		    return (sendMotorPacket(packet, length));
		}

		else
		{
			char Speed_H,Speed_L;
			Speed_H = (Speed >> 8) + 4;
			Speed_L = Speed;                     // 16 bits - 2 x 8 bits variables
			
			const unsigned int length = 9;
			unsigned char packet[length];

			Checksum = (~(ID + MOTOR_SPEED_LENGTH + MOTOR_WRITE_DATA + MOTOR_GOAL_SPEED_L + Speed_L + Speed_H)) & 0xFF;
			
		    packet[0] = MOTOR_START;
		    packet[1] = MOTOR_START;
		    packet[2] = ID;
		    packet[3] = MOTOR_SPEED_LENGTH;
		    packet[4] = MOTOR_WRITE_DATA;
		    packet[5] = MOTOR_GOAL_SPEED_L;
		    packet[6] = Speed_L;
		    packet[7] = Speed_H;
		    packet[8] = Checksum;

		    return (sendMotorPacket(packet, length));
		}
}

int MOTOR::moveRW(unsigned char ID, int Position)
{
    char Position_H,Position_L;
    Position_H = Position >> 8;           // 16 bits - 2 x 8 bits variables
    Position_L = Position;

	const unsigned int length = 9;
	unsigned char packet[length];

    Checksum = (~(ID + MOTOR_GOAL_LENGTH + MOTOR_REG_WRITE + MOTOR_GOAL_POSITION_L + Position_L + Position_H)) & 0xFF;

    packet[0] = MOTOR_START;
    packet[1] = MOTOR_START;
    packet[2] = ID;
    packet[3] = MOTOR_GOAL_LENGTH;
    packet[4] = MOTOR_REG_WRITE;
    packet[5] = MOTOR_GOAL_POSITION_L;
    packet[6] = Position_L;
    packet[7] = Position_H;
    packet[8] = Checksum;
	
    return (sendMotorPacket(packet, length));
}

int MOTOR::moveSpeedRW(unsigned char ID, int Position, int Speed)
{
    char Position_H,Position_L,Speed_H,Speed_L;
    Position_H = Position >> 8;    
    Position_L = Position;                // 16 bits - 2 x 8 bits variables
    Speed_H = Speed >> 8;
    Speed_L = Speed;                      // 16 bits - 2 x 8 bits variables

	const unsigned int length = 11;
	unsigned char packet[length];

    Checksum = (~(ID + MOTOR_GOAL_SP_LENGTH + MOTOR_REG_WRITE + MOTOR_GOAL_POSITION_L + Position_L + Position_H + Speed_L + Speed_H)) & 0xFF;

    packet[0] = MOTOR_START;
    packet[1] = MOTOR_START;
    packet[2] = ID;
    packet[3] = MOTOR_GOAL_SP_LENGTH;
    packet[4] = MOTOR_REG_WRITE;
    packet[5] = MOTOR_GOAL_POSITION_L;
    packet[6] = Position_L;
    packet[7] = Position_H;
    packet[8] = Speed_L;
    packet[9] = Speed_H;
    packet[10] = Checksum;
	
    return (sendMotorPacket(packet, length));
}

void MOTOR::action()
{	
	const unsigned int length = 6;
	unsigned char packet[length];

    packet[0] = MOTOR_START;
    packet[1] = MOTOR_START;
    packet[2] = BROADCAST_ID;
    packet[3] = MOTOR_ACTION_LENGTH;
    packet[4] = MOTOR_ACTION;
    packet[5] = MOTOR_ACTION_CHECKSUM;

    sendMotorPacket(packet, length);
}

int MOTOR::torqueStatus( unsigned char ID, bool Status)
{
	const unsigned int length = 8;
	unsigned char packet[length];

	Checksum = (~(ID + MOTOR_TORQUE_LENGTH + MOTOR_WRITE_DATA + MOTOR_TORQUE_ENABLE + Status)) & 0xFF;

    packet[0] = MOTOR_START;
    packet[1] = MOTOR_START;
    packet[2] = ID;
    packet[3] = MOTOR_TORQUE_LENGTH;
    packet[4] = MOTOR_WRITE_DATA;
    packet[5] = MOTOR_TORQUE_ENABLE;
    packet[6] = Status;
    packet[7] = Checksum;
    
    return (sendMotorPacket(packet, length));
}

int MOTOR::ledStatus(unsigned char ID, bool Status)
{    
	const unsigned int length = 8;
	unsigned char packet[length];

	Checksum = (~(ID + MOTOR_LED_LENGTH + MOTOR_WRITE_DATA + MOTOR_LED + Status)) & 0xFF;

	packet[0] = MOTOR_START;
	packet[1] = MOTOR_START;
	packet[2] = ID;
	packet[3] = MOTOR_LED_LENGTH;
	packet[4] = MOTOR_WRITE_DATA;
	packet[5] = MOTOR_LED;
	packet[6] = Status;
	packet[7] = Checksum;

	return (sendMotorPacket(packet, length)); // return error
}

int MOTOR::readTemperature(unsigned char ID)
{	
	const unsigned int length = 8;
	unsigned char packet[length];

    Checksum = (~(ID + MOTOR_TEM_LENGTH + MOTOR_READ_DATA + MOTOR_PRESENT_TEMPERATURE + MOTOR_BYTE_READ)) & 0xFF;

	packet[0] = MOTOR_START;
	packet[1] = MOTOR_START;
	packet[2] = ID;
	packet[3] = MOTOR_TEM_LENGTH;
	packet[4] = MOTOR_READ_DATA;
	packet[5] = MOTOR_PRESENT_TEMPERATURE;
	packet[6] = MOTOR_BYTE_READ;
	packet[7] = Checksum;

	sendMotorPacketNoError(packet, length);
	
    Temperature_Byte = -1;
    Time_Counter = 0;
    while((availableData() < 6) & (Time_Counter < TIME_OUT))
    {
		Time_Counter++;
		delayus(1000);
    }
	
    while (availableData() > 0)
    {
		Incoming_Byte = readData();
		if ( (Incoming_Byte == 255) & (peekData() == 255) )
		{
			readData();                            // Start Bytes
			readData();                            // MOTOR-12 ID
			readData();                            // Length
			if( (Error_Byte = readData()) != 0 )   // Error
				return (Error_Byte*(-1));
			Temperature_Byte = readData();         // Temperature
		}
    }
	return (Temperature_Byte);               // Returns the read temperature
}

int MOTOR::readPosition(unsigned char ID)
{	
	const unsigned int length = 8;
	unsigned char packet[length];

    Checksum = (~(ID + MOTOR_POS_LENGTH + MOTOR_READ_DATA + MOTOR_PRESENT_POSITION_L + MOTOR_BYTE_READ_POS)) & 0xFF;

	packet[0] = MOTOR_START;
	packet[1] = MOTOR_START;
	packet[2] = ID;
	packet[3] = MOTOR_POS_LENGTH;
	packet[4] = MOTOR_READ_DATA;
	packet[5] = MOTOR_PRESENT_POSITION_L;
	packet[6] = MOTOR_BYTE_READ_POS;
	packet[7] = Checksum;

	sendMotorPacketNoError(packet, length);
	
    Position_Long_Byte = -1;
	Time_Counter = 0;
    while((availableData() < 7) & (Time_Counter < TIME_OUT))
    {
		Time_Counter++;
		delayus(1000);
    }
	
    while (availableData() > 0)
    {
		Incoming_Byte = readData();
		if ( (Incoming_Byte == 255) & (peekData() == 255) )
		{
			readData();                            // Start Bytes
			readData();                            // MOTOR-12 ID
			readData();                            // Length
			if( (Error_Byte = readData()) != 0 )   // Error
				return (Error_Byte*(-1));
    
			Position_Low_Byte = readData();            // Position Bytes
			Position_High_Byte = readData();
			Position_Long_Byte = Position_High_Byte << 8; 
			Position_Long_Byte = Position_Long_Byte + Position_Low_Byte;
		}
    }
    Position_Long_Byte = Position_Long_Byte/11.375;
	return (Position_Long_Byte);     // Returns the read position
}

int MOTOR::readVoltage(unsigned char ID)
{    
	const unsigned int length = 8;
	unsigned char packet[length];

    Checksum = (~(ID + MOTOR_VOLT_LENGTH + MOTOR_READ_DATA + MOTOR_PRESENT_VOLTAGE + MOTOR_BYTE_READ)) & 0xFF;

	packet[0] = MOTOR_START;
	packet[1] = MOTOR_START;
	packet[2] = ID;
	packet[3] = MOTOR_VOLT_LENGTH;
	packet[4] = MOTOR_READ_DATA;
	packet[5] = MOTOR_PRESENT_VOLTAGE;
	packet[6] = MOTOR_BYTE_READ;
	packet[7] = Checksum;

    sendMotorPacketNoError(packet, length);
	
    Voltage_Byte = -1;
	Time_Counter = 0;
    while((availableData() < 6) & (Time_Counter < TIME_OUT))
    {
		Time_Counter++;
		delayus(1000);
    }
	
    while (availableData() > 0)
    {
		Incoming_Byte = readData();
		if ( (Incoming_Byte == 255) & (peekData() == 255) )
		{
			readData();                            // Start Bytes
			readData();                            // MOTOR-12 ID
			readData();                            // Length
			if( (Error_Byte = readData()) != 0 )   // Error
				return (Error_Byte*(-1));
			Voltage_Byte = readData();             // Voltage
		}
    }
	return (Voltage_Byte);               // Returns the read Voltage
}

int MOTOR::setTempLimit(unsigned char ID, unsigned char Temperature)
{
	const unsigned int length = 8;
	unsigned char packet[length];

	Checksum = (~(ID + MOTOR_TL_LENGTH + MOTOR_WRITE_DATA + MOTOR_LIMIT_TEMPERATURE + Temperature)) & 0xFF;
	
	packet[0] = MOTOR_START;
	packet[1] = MOTOR_START;
	packet[2] = ID;
	packet[3] = MOTOR_TL_LENGTH;
	packet[4] = MOTOR_WRITE_DATA;
	packet[5] = MOTOR_LIMIT_TEMPERATURE;
	packet[6] = Temperature;
	packet[7] = Checksum;
	
	return (sendMotorPacket(packet, length));
}

int MOTOR::setVoltageLimit(unsigned char ID, unsigned char DVoltage, unsigned char UVoltage)
{
	const unsigned int length = 9;
	unsigned char packet[length];

	Checksum = (~(ID + MOTOR_VL_LENGTH + MOTOR_WRITE_DATA + MOTOR_DOWN_LIMIT_VOLTAGE + DVoltage + UVoltage)) & 0xFF;
	
	packet[0] = MOTOR_START;
	packet[1] = MOTOR_START;
	packet[2] = ID;
	packet[3] = MOTOR_VL_LENGTH;
	packet[4] = MOTOR_WRITE_DATA;
	packet[5] = MOTOR_DOWN_LIMIT_VOLTAGE;
	packet[6] = DVoltage;
	packet[7] = UVoltage;
	packet[8] = Checksum;
	
	return (sendMotorPacket(packet, length));
}

int MOTOR::setAngleLimit(unsigned char ID, int CWLimit, int CCWLimit)
{
	char CW_H,CW_L,CCW_H,CCW_L;
    CW_H = CWLimit >> 8;    
    CW_L = CWLimit;                // 16 bits - 2 x 8 bits variables
    CCW_H = CCWLimit >> 8;
    CCW_L = CCWLimit;  

	const unsigned int length = 12;
	unsigned char packet[length];
	
	Checksum = (~(ID + MOTOR_VL_LENGTH + MOTOR_WRITE_DATA + MOTOR_CW_ANGLE_LIMIT_L + CW_H + CW_L + MOTOR_CCW_ANGLE_LIMIT_L + CCW_H + CCW_L)) & 0xFF;
	
	packet[0] = MOTOR_START;
	packet[1] = MOTOR_START;
	packet[2] = ID;
	packet[3] = MOTOR_CCW_CW_LENGTH;
	packet[4] = MOTOR_WRITE_DATA;
	packet[5] = MOTOR_CW_ANGLE_LIMIT_L;
	packet[6] = CW_L;
	packet[7] = CW_H;
	packet[8] = MOTOR_CCW_ANGLE_LIMIT_L;
	packet[9] = CCW_L;
	packet[10] = CCW_H;
	packet[11] = Checksum;

	return (sendMotorPacket(packet, length));
}

int MOTOR::setMMOTORTorque(unsigned char ID, int MOTORTorque)
{
    char MOTORTorque_H,MOTORTorque_L;
    MOTORTorque_H = MOTORTorque >> 8;           // 16 bits - 2 x 8 bits variables
    MOTORTorque_L = MOTORTorque;
    
	const unsigned int length = 9;
	unsigned char packet[length];

	Checksum = (~(ID + MOTOR_MT_LENGTH + MOTOR_WRITE_DATA + MOTOR_MOTOR_TORQUE_L + MOTORTorque_L + MOTORTorque_H)) & 0xFF;

	packet[0] = MOTOR_START;
	packet[1] = MOTOR_START;
	packet[2] = ID;
	packet[3] = MOTOR_MT_LENGTH;
	packet[4] = MOTOR_WRITE_DATA;
	packet[5] = MOTOR_MOTOR_TORQUE_L;
	packet[6] = MOTORTorque_L;
	packet[7] = MOTORTorque_H;
	packet[8] = Checksum;
	
	return (sendMotorPacket(packet, length));
}

int MOTOR::setSRL(unsigned char ID, unsigned char SRL)
{    
	const unsigned int length = 8;
	unsigned char packet[length];

	Checksum = (~(ID + MOTOR_SRL_LENGTH + MOTOR_WRITE_DATA + MOTOR_RETURN_LEVEL + SRL)) & 0xFF;

	packet[0] = MOTOR_START;
	packet[1] = MOTOR_START;
	packet[2] = ID;
	packet[3] = MOTOR_SRL_LENGTH;
	packet[4] = MOTOR_WRITE_DATA;
	packet[5] = MOTOR_RETURN_LEVEL;
	packet[6] = SRL;
	packet[7] = Checksum;
	
	return (sendMotorPacket(packet, length));
}

int MOTOR::setRDT(unsigned char ID, unsigned char RDT)
{    
	const unsigned int length = 8;
	unsigned char packet[length];
	
	Checksum = (~(ID + MOTOR_RDT_LENGTH + MOTOR_WRITE_DATA + MOTOR_RETURN_DELAY_TIME + (RDT / 2))) & 0xFF;

	packet[0] = MOTOR_START;
	packet[1] = MOTOR_START;
	packet[2] = ID;
	packet[3] = MOTOR_RDT_LENGTH;
	packet[4] = MOTOR_WRITE_DATA;
	packet[5] = MOTOR_RETURN_DELAY_TIME;
	packet[6] = (RDT/2);
	packet[7] = Checksum;
    
	return (sendMotorPacket(packet, length));
}

int MOTOR::setLEDAlarm(unsigned char ID, unsigned char LEDAlarm)
{    
	const unsigned int length = 8;
	unsigned char packet[length];

	Checksum = (~(ID + MOTOR_LEDALARM_LENGTH + MOTOR_WRITE_DATA + MOTOR_ALARM_LED + LEDAlarm)) & 0xFF;

	packet[0] = MOTOR_START;
	packet[1] = MOTOR_START;
	packet[2] = ID;
	packet[3] = MOTOR_LEDALARM_LENGTH;
	packet[4] = MOTOR_WRITE_DATA;
	packet[5] = MOTOR_ALARM_LED;
	packet[6] = LEDAlarm;
	packet[7] = Checksum;
	
	return (sendMotorPacket(packet, length));
}

int MOTOR::setShutdownAlarm(unsigned char ID, unsigned char SALARM)
{    
	const unsigned int length = 8;
	unsigned char packet[length];
	
	Checksum = (~(ID + MOTOR_SALARM_LENGTH + MOTOR_ALARM_SHUTDOWN + MOTOR_ALARM_LED + SALARM)) & 0xFF;

	packet[0] = MOTOR_START;
	packet[1] = MOTOR_START;
	packet[2] = ID;
	packet[3] = MOTOR_SALARM_LENGTH;
	packet[4] = MOTOR_WRITE_DATA;
	packet[5] = MOTOR_ALARM_SHUTDOWN;
	packet[6] = SALARM;
	packet[7] = Checksum;
    
	return (sendMotorPacket(packet, length));
}

int MOTOR::setCMargin(unsigned char ID, unsigned char CWCMargin, unsigned char CCWCMargin)
{
	const unsigned int length = 10;
	unsigned char packet[length];

	Checksum = (~(ID + MOTOR_CM_LENGTH + MOTOR_WRITE_DATA + MOTOR_CW_COMPLIANCE_MARGIN + CWCMargin + MOTOR_CCW_COMPLIANCE_MARGIN + CCWCMargin)) & 0xFF;
	
	packet[0] = MOTOR_START;
	packet[1] = MOTOR_START;
	packet[2] = ID;
	packet[3] = MOTOR_CM_LENGTH;
	packet[4] = MOTOR_WRITE_DATA;
	packet[5] = MOTOR_CW_COMPLIANCE_MARGIN;
	packet[6] = CWCMargin;
	packet[7] = MOTOR_CCW_COMPLIANCE_MARGIN;
	packet[8] = CCWCMargin;
	packet[9] = Checksum;
	
	return (sendMotorPacket(packet, length));
}

int MOTOR::setCSlope(unsigned char ID, unsigned char CWCSlope, unsigned char CCWCSlope)
{
	const unsigned int length = 10;
	unsigned char packet[length];

	Checksum = (~(ID + MOTOR_CS_LENGTH + MOTOR_WRITE_DATA + MOTOR_CW_COMPLIANCE_SLOPE + CWCSlope + MOTOR_CCW_COMPLIANCE_SLOPE + CCWCSlope)) & 0xFF;
	
	packet[0] = MOTOR_START;
	packet[1] = MOTOR_START;
	packet[2] = ID;
	packet[3] = MOTOR_CS_LENGTH;
	packet[4] = MOTOR_WRITE_DATA;
	packet[5] = MOTOR_CW_COMPLIANCE_SLOPE;
	packet[6] = CWCSlope;
	packet[7] = MOTOR_CCW_COMPLIANCE_SLOPE;
	packet[8] = CCWCSlope;
	packet[9] = Checksum;
	
	return (sendMotorPacket(packet, length));
}

int MOTOR::setPunch(unsigned char ID, int Punch)
{
    char Punch_H,Punch_L;
    Punch_H = Punch >> 8;           // 16 bits - 2 x 8 bits variables
    Punch_L = Punch;

	const unsigned int length = 9;
	unsigned char packet[length];

	Checksum = (~(ID + MOTOR_PUNCH_LENGTH + MOTOR_WRITE_DATA + MOTOR_PUNCH_L + Punch_L + Punch_H)) & 0xFF;
    
	packet[0] = MOTOR_START;
	packet[1] = MOTOR_START;
	packet[2] = ID;
	packet[3] = MOTOR_PUNCH_LENGTH;
	packet[4] = MOTOR_WRITE_DATA;
	packet[5] = MOTOR_PUNCH_L;
	packet[6] = Punch_L;
	packet[7] = Punch_H;
	packet[8] = Checksum;
	
	return (sendMotorPacket(packet, length));
}

int MOTOR::moving(unsigned char ID)
{	
	const unsigned int length = 8;
	unsigned char packet[length];

	Checksum = (~(ID + MOTOR_MOVING_LENGTH + MOTOR_READ_DATA + MOTOR_MOVING + MOTOR_BYTE_READ)) & 0xFF;
    
	packet[0] = MOTOR_START;
	packet[1] = MOTOR_START;
	packet[2] = ID;
	packet[3] = MOTOR_MOVING_LENGTH;
	packet[4] = MOTOR_READ_DATA;
	packet[5] = MOTOR_MOVING;
	packet[6] = MOTOR_BYTE_READ;
	packet[7] = Checksum;
	
	sendMotorPacketNoError(packet, length);

    Moving_Byte = -1;
    Time_Counter = 0;
    while((availableData() < 6) & (Time_Counter < TIME_OUT))
    {
		Time_Counter++;
		delayus(1000);
    }
	
    while (availableData() > 0)
    {
		Incoming_Byte = readData();
		if ( (Incoming_Byte == 255) & (peekData() == 255) )
		{
			readData();                           	// Start Bytes
			readData();                           	// MOTOR-12 ID
			readData();                           	// Length
			if( (Error_Byte = readData()) != 0 )   	// Error
				return (Error_Byte*(-1));
			Moving_Byte = readData();         		// Moving
		}
    }
	return (Moving_Byte);              				// Returns the read Moving
}
int MOTOR::setAcceleration(unsigned char ID, int Acceleration)
{	
	const unsigned int length = 8;
	unsigned char packet[length];

	Checksum = (~(ID + MOTOR_ACC_LENGTH + MOTOR_WRITE_DATA + Goal_Acceleration + Acceleration)) & 0xFF;
    
	packet[0] = MOTOR_START;
	packet[1] = MOTOR_START;
	packet[2] = ID;
	packet[3] = MOTOR_ACC_LENGTH;
	packet[4] = MOTOR_WRITE_DATA;
	packet[5] = Goal_Acceleration;
	packet[6] = Acceleration;
	packet[7] = Checksum;
	
	return (sendMotorPacket(packet, length));
	/*char Acceleration_H,Acceleration_L;
    Acceleration_H = Acceleration >> 8;           // 16 bits - 2 x 8 bits variables
    Acceleration_L = Acceleration;

    const unsigned int length = 9;
    unsigned char packet[length];

	Checksum = (~(ID + MOTOR_ACC_LENGTH + MOTOR_WRITE_DATA + Goal_Acceleration + Acceleration_L + Acceleration_H)) & 0xFF;

    packet[0] = MOTOR_START;
    packet[1] = MOTOR_START;
    packet[2] = ID;
    packet[3] = MOTOR_ACC_LENGTH;
    packet[4] = MOTOR_WRITE_DATA;
    packet[5] = Goal_Acceleration;
    packet[6] = Acceleration_L;
    packet[7] = Acceleration_H;
    packet[8] = Checksum;

    return (sendMotorPacket(packet, length));*/
}
int MOTOR::lockRegister(unsigned char ID)
{    
	const unsigned int length = 8;
	unsigned char packet[length];
	
	Checksum = (~(ID + MOTOR_LR_LENGTH + MOTOR_WRITE_DATA + MOTOR_LOCK + LOCK)) & 0xFF;
    
	packet[0] = MOTOR_START;
	packet[1] = MOTOR_START;
	packet[2] = ID;
	packet[3] = MOTOR_LR_LENGTH;
	packet[4] = MOTOR_WRITE_DATA;
	packet[5] = MOTOR_LOCK;
	packet[6] = LOCK;
	packet[7] = Checksum;

	return (sendMotorPacket(packet, length));
}

int MOTOR::RWStatus(unsigned char ID)
{	
	const unsigned int length = 8;
	unsigned char packet[length];

	Checksum = (~(ID + MOTOR_RWS_LENGTH + MOTOR_READ_DATA + MOTOR_REGISTERED_INSTRUCTION + MOTOR_BYTE_READ)) & 0xFF;
    
	packet[0] = MOTOR_START;
	packet[1] = MOTOR_START;
	packet[2] = ID;
	packet[3] = MOTOR_RWS_LENGTH;
	packet[4] = MOTOR_READ_DATA;
	packet[5] = MOTOR_REGISTERED_INSTRUCTION;
	packet[6] = MOTOR_BYTE_READ;
	packet[7] = Checksum;
	
	sendMotorPacketNoError(packet, length);

    RWS_Byte = -1;
    Time_Counter = 0;
    while((availableData() < 6) & (Time_Counter < TIME_OUT))
    {
		Time_Counter++;
		delayus(1000);
    }
	
    while (availableData() > 0)
    {
		Incoming_Byte = readData();
		if ( (Incoming_Byte == 255) & (peekData() == 255) )
		{
			readData();                            	// Start Bytes
			readData();                            	// MOTOR-12 ID
			readData();                            	// Length
			if( (Error_Byte = readData()) != 0 )   	// Error
				return (Error_Byte*(-1));
			RWS_Byte = readData();         			// RWStatus
		}
    }
	return (RWS_Byte);               				// Returns the read RWStatus
}

int MOTOR::readSpeed(unsigned char ID)
{	
	const unsigned int length = 8;
	unsigned char packet[length];

	Checksum = (~(ID + MOTOR_POS_LENGTH + MOTOR_READ_DATA + MOTOR_PRESENT_SPEED_L + MOTOR_BYTE_READ_POS)) & 0xFF;
	
	packet[0] = MOTOR_START;
	packet[1] = MOTOR_START;
	packet[2] = ID;
	packet[3] = MOTOR_POS_LENGTH;
	packet[4] = MOTOR_READ_DATA;
	packet[5] = MOTOR_PRESENT_SPEED_L;
	packet[6] = MOTOR_BYTE_READ_POS;
	packet[7] = Checksum;
	
	sendMotorPacketNoError(packet, length);

    Speed_Long_Byte = -1;
	Time_Counter = 0;
    while((availableData() < 7) & (Time_Counter < TIME_OUT))
    {
		Time_Counter++;
		delayus(1000);
    }
	
    while (availableData() > 0)
    {
		Incoming_Byte = readData();
		if ( (Incoming_Byte == 255) & (peekData() == 255) )
		{
			readData();                            // Start Bytes
			readData();                            // MOTOR-12 ID
			readData();                            // Length
			if( (Error_Byte = readData()) != 0 )   // Error
				return (Error_Byte*(-1));
			
			Speed_Low_Byte = readData();            // Position Bytes
			Speed_High_Byte = readData();
			Speed_Long_Byte = Speed_High_Byte << 8; 
			Speed_Long_Byte = Speed_Long_Byte + Speed_Low_Byte;
		}
    }
	return (Speed_Long_Byte);     // Returns the read position
}

int MOTOR::readLoad(unsigned char ID)
{	
	const unsigned int length = 8;
	unsigned char packet[length];

	Checksum = (~(ID + MOTOR_POS_LENGTH + MOTOR_READ_DATA + MOTOR_PRESENT_LOAD_L + MOTOR_BYTE_READ_POS)) & 0xFF;

	packet[0] = MOTOR_START;
	packet[1] = MOTOR_START;
	packet[2] = ID;
	packet[3] = MOTOR_POS_LENGTH;
	packet[4] = MOTOR_READ_DATA;
	packet[5] = MOTOR_PRESENT_LOAD_L;
	packet[6] = MOTOR_BYTE_READ_POS;
	packet[7] = Checksum;
	
	sendMotorPacketNoError(packet, length);

    Load_Long_Byte = -1;
	Time_Counter = 0;
    while((availableData() < 7) & (Time_Counter < TIME_OUT))
    {
		Time_Counter++;
		delayus(1000);
    }
	
    while (availableData() > 0)
    {
		Incoming_Byte = readData();
		if ( (Incoming_Byte == 255) & (peekData() == 255) )
		{
			readData();                            // Start Bytes
			readData();                            // MOTOR-12 ID
			readData();                            // Length
			if( (Error_Byte = readData()) != 0 )   // Error
				return (Error_Byte*(-1));
			
			Load_Low_Byte = readData();            // Position Bytes
			Load_High_Byte = readData();
			Load_Long_Byte = Load_High_Byte << 8; 
			Load_Long_Byte = Load_Long_Byte + Load_Low_Byte;
		}
    }
	return (Load_Long_Byte);     // Returns the read position
}

int MOTOR::sendMotorPacket(unsigned char * packet, unsigned int length)
{
	switchCom(Direction_Pin, TX_MODE); 	// Switch to Transmission  Mode

	sendData(packet, length);			// Send data through sending buffer
	flush(); 							// Wait until buffer is empty

	switchCom(Direction_Pin, RX_MODE); 	// Switch back to Reception Mode

	return (read_error());              // Return the read error
}

void MOTOR::sendMotorPacketNoError(unsigned char * packet, unsigned int length)
{
	switchCom(Direction_Pin, TX_MODE); 	// Switch to Transmission  Mode

	sendData(packet, length);			// Send data through sending buffer
	flush(); 							// Wait until buffer is empty

	switchCom(Direction_Pin, RX_MODE); 	// Switch back to Reception Mode
}

int MOTOR::readRegister(unsigned char ID, unsigned char reg, unsigned char reg_len)
{
	const unsigned int length = 8;
	unsigned char packet[length];

	Checksum = (~(ID + 4 + MOTOR_READ_DATA + reg + reg_len)) & 0xFF;

	packet[0] = MOTOR_START;
	packet[1] = MOTOR_START;
	packet[2] = ID;
	packet[3] = 4;
	packet[4] = MOTOR_READ_DATA;
	packet[5] = reg;
	packet[6] = reg_len;
	packet[7] = Checksum;

	sendMotorPacketNoError(packet, length);

	returned_Byte = -1;
	Time_Counter = 0;
	while((availableData() < 7) & (Time_Counter < TIME_OUT))
	{
		Time_Counter++;
		delayus(1000);
	}

	while (availableData() > 0)
	{
		Incoming_Byte = readData();
		if ( (Incoming_Byte == 255) & (peekData() == 255) )
		{
			readData();                            // Start Bytes
			readData();                            // MOTOR-12 ID
			readData();                            // Length
			if( (Error_Byte = readData()) != 0 )   // Error
				return (Error_Byte*(-1));

			switch (reg_len)
			{
				case 1:
					returned_Byte = readData();
					break;
				case 2:
					returned_Byte = readData();
					returned_Byte += readData() << 8;
				break;
			}
		}
	}
	return (returned_Byte);     // Returns the read position
}

MOTOR Motor;
