//Mule-2018
#include <iostream>
#include <memory>
#include <string>
//#include "AHRS.h"
#include "WPILib.h"
#include "math.h"

#include <IterativeRobot.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

class Robot: public frc::IterativeRobot {
	DifferentialDrive Adrive;
	frc::LiveWindow* lw = LiveWindow::GetInstance();
	frc::SendableChooser<std::string> chooseDriveEncoder, chooseLowDriveSens,
			chooseLowTurnSens, chooseHighDriveSens, chooseHighTurnSens;
	const std::string RH_Encoder = "RH_Encoder";
	const std::string LH_Encoder = "LH_Encoder";
	const std::string DriveDefault = "Standard";
	const std::string Drive1 = "Sens_x^2";
	const std::string Drive2 = "Sens_x^3";
	const std::string Drive3 = "Sens_x^5";
	const std::string TurnDefault = "Standard";
	const std::string Turn1 = "Sens_x^2";
	const std::string Turn2 = "Sens_x^3";
	const std::string Turn3 = "Sens_x^5";
	std::string encoderSelected, LowDriveChooser, LowTurnChooser,
			HighDriveChooser, HighTurnChooser;
	Joystick Drivestick;
	Joystick OperatorStick;
	VictorSP DriveLeft0;
	VictorSP DriveLeft1;
	VictorSP DriveLeft2;
	VictorSP DriveRight0;
	VictorSP DriveRight1;
	VictorSP DriveRight2;
	VictorSP Dpad1;
	VictorSP Dpad2;
	VictorSP RightStick1;
	VictorSP RightStick2;
	VictorSP Claw1;
	VictorSP Claw2;
	Timer EncoderCheckTimer;
	Encoder EncoderLeft;
	Encoder EncoderRight;

	double OutputX, OutputY;
	double OutputX1, OutputY1;
	DigitalInput DiIn8, DiIn9;

	// create pdp variable
	PowerDistributionPanel *pdp = new PowerDistributionPanel();

	int modeState, DriveState, TurnState;

//Solenoid's declared
	Solenoid *driveSolenoid = new Solenoid(0);
	Solenoid *XYbutton = new Solenoid(4);
	Solenoid *Bbutton = new Solenoid(3);
	Solenoid *Abutton = new Solenoid(2);
	Solenoid *IntakeButton = new Solenoid(1);

	bool useRightEncoder = false;
	bool driveRightTriggerPrev = false;
	bool driveButtonYPrev = false;
	bool operatorRightTriggerPrev = false;
	bool intakeDeployed = false;
	bool XYDeployed = false;
	bool shooterOn = false;

public:
	Robot() :
			Adrive(DriveLeft0, DriveRight0), Drivestick(0), OperatorStick(1), DriveLeft0(
					0), DriveLeft1(1), DriveLeft2(2), DriveRight0(3), DriveRight1(
					4), DriveRight2(5), Dpad1(10), Dpad2(11), RightStick1(6), RightStick2(
					7), Claw1(8), Claw2(9), EncoderLeft(0, 1), EncoderRight(2,
					3), OutputX(0), OutputY(0), OutputX1(0), OutputY1(0), DiIn8(
					8), DiIn9(9), modeState(0), DriveState(0), TurnState(0) {

	}

private:
	void RobotInit() {
		chooseDriveEncoder.AddDefault(LH_Encoder, LH_Encoder);
		chooseDriveEncoder.AddObject(RH_Encoder, RH_Encoder);
		frc::SmartDashboard::PutData("Encoder", &chooseDriveEncoder);

		chooseLowTurnSens.AddObject(TurnDefault, TurnDefault);
		chooseLowTurnSens.AddDefault(Turn1, Turn1);
		chooseLowTurnSens.AddObject(Turn2, Turn2);
		chooseLowTurnSens.AddObject(Turn3, Turn3);
		frc::SmartDashboard::PutData("LowTurnSens", &chooseLowTurnSens);

		chooseLowDriveSens.AddObject(DriveDefault, DriveDefault);
		chooseLowDriveSens.AddDefault(Drive1, Drive1);
		chooseLowDriveSens.AddObject(Drive2, Drive2);
		chooseLowDriveSens.AddObject(Drive3, Drive3);
		frc::SmartDashboard::PutData("LowDriveSens", &chooseLowDriveSens);

		chooseHighTurnSens.AddObject(TurnDefault, TurnDefault);
		chooseHighTurnSens.AddObject(Turn1, Turn1);
		chooseHighTurnSens.AddDefault(Turn2, Turn2);
		chooseHighTurnSens.AddObject(Turn3, Turn3);
		frc::SmartDashboard::PutData("HighTurnSens", &chooseHighTurnSens);

		chooseHighDriveSens.AddObject(DriveDefault, DriveDefault);
		chooseHighDriveSens.AddObject(Drive1, Drive1);
		chooseHighDriveSens.AddDefault(Drive2, Drive2);
		chooseHighDriveSens.AddObject(Drive3, Drive3);
		frc::SmartDashboard::PutData("HighDriveSens", &chooseHighDriveSens);

		//turn off shifter solenoids
		driveSolenoid->Set(false);

		//disable drive watchdogs
		Adrive.SetSafetyEnabled(false);

		//changes these original negative values to positive values
		EncoderLeft.SetReverseDirection(true);
		EncoderRight.SetReverseDirection(false);

		//calibrations for encoders
		EncoderLeft.SetDistancePerPulse(98.0 / 3125.0 * 4.0);
		EncoderRight.SetDistancePerPulse(98.0 / 3125.0 * 4.0);

		//drive command averaging filter
		OutputX = 0, OutputY = 0;

		//variable that chooses which encoder robot is reading for autonomous mode
		useRightEncoder = true;
	}

	void TeleopInit() {
		OutputX = 0, OutputY = 0;

	}

	void RobotPeriodic() {
		//links multiple motors together
		DriveLeft1.Set(DriveLeft0.Get());
		DriveLeft2.Set(DriveLeft0.Get());
		DriveRight1.Set(DriveRight0.Get());
		DriveRight2.Set(DriveRight0.Get());

		// Encoder Selection for autotools
		encoderSelected = chooseDriveEncoder.GetSelected();
		useRightEncoder = (encoderSelected == RH_Encoder);

		LowTurnChooser = chooseLowTurnSens.GetSelected();
		LowDriveChooser = chooseLowDriveSens.GetSelected();
		HighTurnChooser = chooseHighTurnSens.GetSelected();
		HighDriveChooser = chooseHighDriveSens.GetSelected();

	}

	void DisabledPeriodic() {

	}

#define caseDriveDefault 1
#define caseDrive1 2
#define caseDrive2 3
#define caseDrive3 4

	void TeleopPeriodic() {
		double Deadband = 0.11;
		double DPadSpeed = 1.0;
		bool RightStickLimit1 = DiIn8.Get();
		bool RightStickLimit2 = DiIn9.Get();
		double Control_Deadband = 0.10;
		double Drive_Deadband = 0.10;
		double Gain = 1;
		std::string DriveDebug = "";
		std::string TurnDebug = "";

		//high gear & low gear controls
		if (Drivestick.GetRawButton(5))
			driveSolenoid->Set(true);			// High gear press LH bumper
		if (Drivestick.GetRawButton(6))
			driveSolenoid->Set(false);			// Low gear press RH bumper

		//  Rumble code
		//  Read all motor current from PDP and display on drivers station
		double driveCurrent = pdp->GetTotalCurrent();	// Get total current

		// rumble if current to high
		double LHThr = 0.0;		// Define value for rumble
		if (driveCurrent > 125.0)// Rumble if greater than 125 amps motor current
			LHThr = 0.5;
		Joystick::RumbleType Vibrate;				// define Vibrate variable
		Vibrate = Joystick::kLeftRumble;		// set Vibrate to Left
		Drivestick.SetRumble(Vibrate, LHThr);  	// Set Left Rumble to RH Trigger
		Vibrate = Joystick::kRightRumble;		// set vibrate to Right
		Drivestick.SetRumble(Vibrate, LHThr);// Set Right Rumble to RH Trigger

		//drive controls
		double SpeedLinear = Drivestick.GetRawAxis(1) * -1; // get Yaxis value (forward)
		double SpeedRotate = Drivestick.GetRawAxis(4) * 1; // get Xaxis value (turn)

		// Set dead band for X and Y axis
		if (fabs(SpeedLinear) < Deadband)
			SpeedLinear = 0.0;
		if (fabs(SpeedRotate) < Deadband)
			SpeedRotate = 0.0;

		//slow down direction changes from 1 cycle to 5
		OutputY = (0.8 * OutputY) + (0.2 * SpeedLinear);
		OutputX = (0.8 * OutputX) + (0.2 * SpeedRotate);


		if (!driveSolenoid->Get()) {

			if (LowDriveChooser == DriveDefault)
				DriveState = caseDriveDefault;
			else if (LowDriveChooser == Drive1)
				DriveState = caseDrive1;
			else if (LowDriveChooser == Drive2)
				DriveState = caseDrive2;
			else if (LowDriveChooser == Drive3)
				DriveState = caseDrive3;
			else
				DriveState = 0;

			if (LowTurnChooser == TurnDefault)
				TurnState = caseDriveDefault;
			else if (LowTurnChooser == Turn1)
				TurnState = caseDrive1;
			else if (LowTurnChooser == Turn2)
				TurnState = caseDrive2;
			else if (LowTurnChooser == Turn3)
				TurnState = caseDrive3;
			else
				TurnState = 0;

		} else {

			if (HighDriveChooser == DriveDefault)
				DriveState = caseDriveDefault;
			else if (HighDriveChooser == Drive1)
				DriveState = caseDrive1;
			else if (HighDriveChooser == Drive2)
				DriveState = caseDrive2;
			else if (HighDriveChooser == Drive3)
				DriveState = caseDrive3;
			else
				DriveState = 0;

			if (HighTurnChooser == TurnDefault)
				TurnState = caseDriveDefault;
			else if (HighTurnChooser == Turn1)
				TurnState = caseDrive1;
			else if (HighTurnChooser == Turn2)
				TurnState = caseDrive2;
			else if (HighTurnChooser == Turn3)
				TurnState = caseDrive3;
			else
				TurnState = 0;
		}

		switch (DriveState) {
		case caseDriveDefault:
			// Set control to out of box functionality
			OutputY = SpeedLinear;
			DriveDebug = TurnDefault;
			break;
		case caseDrive1:
			// Set  control response curve  to square input
			DriveDebug = Turn1;
			if (SpeedLinear > Control_Deadband)
				OutputY = Drive_Deadband + (Gain * (SpeedLinear * SpeedLinear));
			else if (SpeedLinear < -Control_Deadband)
				OutputY = -Drive_Deadband
						+ (-Gain * (SpeedLinear * SpeedLinear));
			else
				OutputY = 0;

			break;
		case caseDrive2:
			// Set  control response curve  to cubed input
			DriveDebug = Turn2;
			if (SpeedLinear > Control_Deadband)
				OutputY = Drive_Deadband + (Gain * pow(SpeedLinear, 3));
			else if (SpeedLinear < -Control_Deadband)
				OutputY = -Drive_Deadband + (Gain * pow(SpeedLinear, 3));
			else
				OutputY = 0;

			break;
		case caseDrive3:
			// Set control response curve to input^5
			DriveDebug = Turn3;
			if (SpeedLinear > Control_Deadband)
				OutputY = Drive_Deadband + (Gain * pow(SpeedLinear, 5));
			else if (SpeedLinear < -Control_Deadband)
				OutputY = -Drive_Deadband + (Gain * pow(SpeedLinear, 5));
			else
				OutputY = 0;

			break;
		default:
			DriveDebug = "Not Set";
			OutputY = 0;

		}

		switch (TurnState) {
		case caseDriveDefault:
			// Set control to out of box functionality
			OutputX = SpeedRotate;
			TurnDebug = TurnDefault;
			break;
		case caseDrive1:
			// Set  control response curve  to square input
			TurnDebug = Turn1;
			if (SpeedRotate > Control_Deadband)
				OutputX = Drive_Deadband + (Gain * (SpeedRotate * SpeedRotate));
			else if (SpeedRotate < -Control_Deadband)
				OutputX = -Drive_Deadband
						+ (-Gain * (SpeedRotate * SpeedRotate));
			else
				OutputX = 0;
			break;
		case caseDrive2:
			// Set  control response curve  to cubed input
			TurnDebug = Turn2;
			if (SpeedRotate > Control_Deadband)
				OutputX = Drive_Deadband + (Gain * pow(SpeedRotate, 3));
			else if (SpeedRotate < -Control_Deadband)
				OutputX = -Drive_Deadband + (Gain * pow(SpeedRotate, 3));
			else
				OutputX = 0;
			break;
		case caseDrive3:
			// Set control response curve to input^5
			TurnDebug = Turn3;
			if (SpeedRotate > Control_Deadband)
				OutputX = Drive_Deadband + (Gain * pow(SpeedRotate, 5));
			else if (SpeedRotate < -Control_Deadband)
				OutputX = -Drive_Deadband + (Gain * pow(SpeedRotate, 5));
			else
				OutputX = 0;
			break;
		default:
			OutputX = 0;
			TurnDebug = "Not Set";
		}

		SmartDashboard::PutString("Drive response curve", DriveDebug);
		SmartDashboard::PutString("Turn response curve", TurnDebug);
		SmartDashboard::PutNumber("SpeedLinear", SpeedLinear);
		SmartDashboard::PutNumber("SpeedRotate", SpeedRotate);

		SmartDashboard::PutNumber("OutputY", OutputY);
		SmartDashboard::PutNumber("OutputX", OutputX);

		//slow down direction changes from 1 cycle to 5
		OutputY1 = (0.8 * OutputY1) + (0.2 * OutputY);
		OutputX1 = (0.8 * OutputX1) + (0.2 * OutputX);

		Adrive.ArcadeDrive(OutputY1, OutputX1, false);

		/*
		 * MANIP CODE
		 */

		//A Button to extend (Solenoid On)
		Abutton->Set(OperatorStick.GetRawButton(6));

		//B Button to extend (Solenoid On)
		Bbutton->Set(OperatorStick.GetRawButton(2));

		//if Left Bumper button pressed, extend (Solenoid On)
		if (OperatorStick.GetRawButton(5)) {
			intakeDeployed = true;
			IntakeButton->Set(intakeDeployed);
		}

		//else Right Bumper pressed, retract (Solenoid Off)
		else if (OperatorStick.GetRawButton(6)) {
			intakeDeployed = false;
			IntakeButton->Set(intakeDeployed);
		}

		else if (OperatorStick.GetRawAxis(2)) {
			intakeDeployed = false;
			IntakeButton->Set(intakeDeployed);
		}

		else if (OperatorStick.GetRawAxis(3)) {
			intakeDeployed = false;
			IntakeButton->Set(intakeDeployed);
		}

		//if 'X' button pressed, extend (Solenoid On)
		if (OperatorStick.GetRawButton(3)) {
			XYDeployed = true;
			XYbutton->Set(XYDeployed);
		}

		//else 'Y' button pressed, retract (Solenoid Off)
		else if (OperatorStick.GetRawButton(4)) {
			XYDeployed = false;
			XYbutton->Set(XYDeployed);
		}

		//dpad POV stuff
		if (OperatorStick.GetPOV(0) == 0) {
			Dpad1.Set(DPadSpeed);
			Dpad2.Set(DPadSpeed);
		} else if (OperatorStick.GetPOV(0) == 180) {
			Dpad1.Set(-DPadSpeed);
			Dpad2.Set(-DPadSpeed);
		} else {
			Dpad1.Set(0.0);
			Dpad2.Set(0.0);
		}

		double RightSpeed = OperatorStick.GetRawAxis(1) * 1; // get Xaxis value for Left Joystick
		double ClawSpeed = (OperatorStick.GetRawAxis(2)
				+ (OperatorStick.GetRawAxis(3) * -1));

		if (fabs(RightSpeed) < Deadband) {
			RightSpeed = 0.0;
		} else if (RightSpeed < Deadband and !RightStickLimit1)
			RightSpeed = 0.0;
		else if (RightSpeed > Deadband and !RightStickLimit2)
			RightSpeed = 0.0;

		RightStick1.Set(RightSpeed);
		RightStick2.Set(RightSpeed);

		Claw1.Set(ClawSpeed);
		Claw2.Set(ClawSpeed);


	}

// These are the state numbers for each part of autoBlue1
//		These are here so we can easily add states.
// 		State 1 is always the first one to run.
//		Always have an "end" state.

	void motorSpeed(double leftMotor, double rightMotor) {
		DriveLeft0.Set(leftMotor * -1);
		DriveLeft1.Set(leftMotor * -1);
		DriveLeft2.Set(leftMotor * -1);
		DriveRight0.Set(rightMotor);
		DriveRight1.Set(rightMotor);
		DriveRight2.Set(rightMotor);
	}

//need to change signs!!!
	int stopMotors() {
		//sets motor speeds to zero
		motorSpeed(0, 0);
		return 1;
	}

	//------------- Start Code for Running Encoders --------------
	double readEncoder() {
		double usableEncoderData;
		double r = EncoderRight.GetDistance();
		double l = EncoderLeft.GetDistance();
		//If a encoder is disabled switch l or r to each other.
		if (l > 0) {
			usableEncoderData = fmax(r, l);
		} else if (l == 0) {
			usableEncoderData = r;
		} else {
			usableEncoderData = fmin(r, l);
		}
		return usableEncoderData;
	}

	void resetEncoder() {
		EncoderLeft.Reset();
		EncoderRight.Reset();
		EncoderCheckTimer.Reset();
	}
	//------------- End Code for Running Encoders --------------------

private:

}
;

START_ROBOT_CLASS(Robot)
