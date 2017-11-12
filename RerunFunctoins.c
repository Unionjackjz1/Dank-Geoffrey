bool bSide;
int iArmCurrentRPM, iChassisLeftRPM, iChassisRightRPM;
task
calculateRPMs( ) {
	while( true ) {
		int iFirstArm             = getArm();
		int iFirstChassisLeftRPM  = getLeftChassis();
		int iFirstChassisRightRPM = getRightChassis();
		delay( 50);
		iArmCurrentRPM   = getArm()          - iFirstArm;
		iChassisLeftRPM  = getLeftChassis()  - iFirstChassisLeftRPM;
		iChassisRightRPM = getRightChassis() - iFirstChassisRightRPM;
	}
}

task
record( ) {
	startTask( calculateRPMs );
	int j = 20, s = 1;
	for( int i = 0; i < 400; i++ ) {
		if( j == 20 ) {
			writeDebugStream( "\n" );
			writeDebugStream( "\n	/* %i Second(s) */", s );
			s++;
			j = 0;
		}
		j++;

		writeDebugStream( "\n	auton( %i, %i, %i, %i, %i, %i );", iArmCurrentRPM, motor[ arm_l1 ], iChassisLeftRPM, motor[ chassis_l1 ], iChassisRightRPM, motor[ chassis_r1 ] );

		delay( 50 );
	}
	stopTask( calculateRPMs );
	writeDebugStream( "\n\n	/* Stop the robot */\n	auton( 0, 0, 0, 0, 0, 0 );");
}

void
auton( int iArmDes, int iArmSpeed, int iChassisLeftDes, int iChassisLeftSpeed, int iChassisRightDes, int iChassisRightSpeed ) {
	int iArmError = 0, iChassisLeftError = 0, iChassisRightError = 0;
	for( int i = 0; i < 50; i++ ) {
		iArmError          -= (iArmDes - iArmCurrentRPM) * 0.015;
		iChassisLeftError  -= (iChassisLeftDes - iChassisLeftRPM) * 0.01;
		iChassisRightError -= (iChassisRightDes - iChassisRightRPM) * 0.01;

		arm( iArmError + iArmSpeed );

		if( !bSide )
			tankWithoutTrueSpeed( iChassisLeftError + iChassisLeftSpeed, iChassisRightError + iChassisRightSpeed );
		else
			tankWithoutTrueSpeed( iChassisRightError + iChassisRightSpeed, iChassisLeftError + iChassisLeftSpeed );

		delay( 1 );
	}
}
