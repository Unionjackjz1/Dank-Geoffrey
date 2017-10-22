#pragma systemfile

typedef struct {
	float current;
	float kP;
	float kI;
	float kD;
	float target;
	float error;
	float integral;
	float derivative;
	float lastError;
	float threshold;
	int   lastTime;
} pid;

int MC29[ 128 ] = {
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 13, 13, 14,
	14, 14, 14, 15, 15, 15, 15, 16, 16, 16,
	16, 16, 17, 17, 17, 17, 17, 17, 17, 17,
	18, 18, 18, 18, 18, 18, 18, 19, 19, 19,
	19, 19, 20, 20, 20, 20, 21, 21, 21, 22,
	22, 22, 22, 23, 23, 23, 23, 24, 24, 24,
	25, 25, 25, 26, 26, 26, 27, 27, 27, 28,
	28, 28, 29, 29, 30, 30, 30, 31, 31, 32,
	32, 33, 33, 34, 34, 35, 36, 36, 37, 37,
	38, 39, 40, 40, 41, 42, 43, 44, 45, 46,
	46, 48, 49, 50, 51, 53, 55, 56, 58, 60,
	62, 64, 67, 70, 72, 76, 79, 83, 84, 127,
};

int L298[128] = {
	0, 0, 0, 0, 0, 0, 0, 0,
	0, 0, 0, 0, 0, 0, 0, 11, 11, 12,
	12, 12, 12, 13, 13, 13, 14, 14, 14, 14,
	15, 15, 15, 16, 16, 16, 17, 17, 17, 18,
	18, 18, 19, 19, 19, 20, 20, 20, 21, 21,
	22, 22, 23, 23, 23, 24, 25, 25, 26, 26,
	26, 27, 27, 28, 28, 29, 29, 30, 30, 31,
	31, 32, 32, 33, 33, 34, 35, 35, 35, 35,
	35, 35, 38, 38, 39, 41, 41, 41, 42, 43,
	45, 46, 47, 47, 48, 49, 49, 50, 51, 52,
	53, 54, 55, 56, 58, 59, 63, 66, 67, 70,
	73, 74, 75, 78, 80, 84, 87, 88, 92, 95,
	97, 100, 105, 108, 114, 117, 121, 122, 124, 127
};

void
testAllMotors( ) {
	clearLCDLines( );
	for( int i = 0; i < 10; i++ ) {
		displayLCDNumber(0, 0, i + 1);
		motor[ i ] = 127;
		delay( 3000 );
		motor[ i ] = 0;
		clearLCDLines( );
		delay( 1000 );
	}
}

float
logDrive (int iInput, float iMultiply) {
	return (pow(abs(iInput), iMultiply)) * sgn(iInput);
}

void
slewMotor( tMotor enPort, int iSpeedDes ) {
	int iReturn = iSpeedDes-motor[enPort];
	motor[ enPort ] = sgn(iReturn) * MC29[abs(clipNum(iReturn, 127))];
	delay( 2 );
}

void
tank( int iSpeedL, int iSpeedR ) {
	motor[ chassis_l1 ] =
	motor[ chassis_l2 ] =
	motor[ chassis_l3 ] = sgn(iSpeedL) * MC29[abs(clipNum(iSpeedL, 127))];
	motor[ chassis_r3 ] =
	motor[ chassis_r2 ] =
	motor[ chassis_r1 ] = sgn(iSpeedR) * MC29[abs(clipNum(iSpeedR, 127))];
}

void
tankWithoutTrueSpeed( int iSpeedL, int iSpeedR ) {
	motor[ chassis_l1 ] =
	motor[ chassis_l2 ] =
	motor[ chassis_l3 ] = iSpeedL;
	motor[ chassis_r3 ] =
	motor[ chassis_r2 ] =
	motor[ chassis_r1 ] = iSpeedR;
}

void
arm( int iSpeed ) {
	motor[ arm_l1 ] =
	motor[ arm_l2 ] =
	motor[ arm_r2 ] =
	motor[ arm_r1 ] = iSpeed;
}

int
getArm() {
	return SensorValue[arm_sensor];
}

int
getLeftChassis() {
	return SensorValue[left_chassis];
}

int
getRightChassis() {
	return -SensorValue[right_chassis];
}

pid sArmPID;
int
iArmPID( int iDes ) {
	sArmPID.derivative = sArmPID.error - sArmPID.lastError;
	const float kP = 0.75;
	const float kD = 3;
	sArmPID.current    = getArm();
	sArmPID.error      = iDes - sArmPID.current;
	delay( 10 );
	sArmPID.lastError = sArmPID.error;
	return ( (sArmPID.error * kP) + (sArmPID.derivative * kD) );
}
bool
armIsAtPos( int iDes ) {
  return abs(iDes - getArm()) < 30 ? true : false;
}

pid sLCPID;
int
iLeftChassisPID( int iDes ) {
	sLCPID.derivative = sLCPID.error - sLCPID.lastError;
	const float kP = 0.7;
	const float kD = 0.05;
	sLCPID.current    = getLeftChassis();
	sLCPID.error      = iDes - sLCPID.current;
	delay( 10 );
	sLCPID.lastError = sLCPID.error;
	return ( (sLCPID.error * kP) + (sLCPID.derivative * kD) );
}
bool
leftChassisIsAtPos( int iDes ) {
  return abs(iDes - getLeftChassis()) < 50 ? true : false;
}

pid sRCPID;
int
iRightChassisPID( int iDes ) {
	sRCPID.derivative = sRCPID.error - sRCPID.lastError;
	const float kP = 0.7;
	const float kD = 0.05;
	sRCPID.current    = getRightChassis();
	sRCPID.error      = iDes - sRCPID.current;
	delay( 10 );
	sRCPID.lastError = sRCPID.error;
	return ( (sRCPID.error * kP) + (sRCPID.derivative * kD) );
}
bool
rightChassisIsAtPos( int iDes ) {
  return abs(iDes - getLeftChassis()) < 50 ? true : false;
}
