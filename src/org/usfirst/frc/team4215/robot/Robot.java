/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/********* Version *********/
/* TalonSRX: 3.9
 * VictorSPX: 3.9
 * Pigeon IMU: 0.41
 * Phoenix Framework: 5.6.0 
 */

package org.usfirst.frc.team4215.robot;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;

public class Robot extends IterativeRobot {
	/** Hardware */
	TalonSRX _leftMaster = new TalonSRX(3);
	TalonSRX _rightMaster = new TalonSRX(2);
	TalonSRX lbSlave = new TalonSRX(0);
	TalonSRX rbSlave = new TalonSRX(1);
	PigeonIMU _pidgey = new PigeonIMU(_rightMaster);
	Joystick _gamepad = new Joystick(0);
	
	/** Latched values to detect on-press events for buttons */
	boolean[] _btns = new boolean[Constants.kNumButtonsPlusOne];
	boolean[] btns = new boolean[Constants.kNumButtonsPlusOne];
	
	/** Tracking variables */
	boolean _firstCall = false;
	boolean _state = false;
	double _targetAngle = 0;
	double [] ypr = new double[3];

	@Override
	public void robotInit() {
		/* Not in use */
	}
	
	@Override
	public void teleopInit(){
		/* Disable all motor controllers */
		_rightMaster.set(ControlMode.PercentOutput, 0);
		_leftMaster.set(ControlMode.PercentOutput, 0);
		lbSlave.set(ControlMode.PercentOutput, 0);
		rbSlave.set(ControlMode.PercentOutput, 0);

		
		/* Set Neutral Mode */
		_leftMaster.setNeutralMode(NeutralMode.Brake);
		_rightMaster.setNeutralMode(NeutralMode.Brake);
		lbSlave.setNeutralMode(NeutralMode.Brake);
		rbSlave.setNeutralMode(NeutralMode.Brake);

		
		/** Feedback Sensor Configuration */
		
		/* Configure the Pigeon IMU as a Remote Sensor for the right Talon */
		_rightMaster.configRemoteFeedbackFilter(_pidgey.getDeviceID(),			// Device ID of Source
												RemoteSensorSource.Pigeon_Yaw,	// Remote Feedback Source
												Constants.REMOTE_1,				// Remote number [0, 1]
												Constants.kTimeoutMs);			// Configuration Timeout
		
		/* Configure the Remote Sensor to be the Selected Sensor of the right Talon */
		_rightMaster.configSelectedFeedbackSensor(	FeedbackDevice.RemoteSensor1, 	// Set remote sensor to be used directly
													Constants.PID_TURN, 			// PID Slot for Source [0, 1]
													Constants.kTimeoutMs);			// Configuration Timeout
		
		/* Scale the Selected Sensor using a coefficient (Values explained in Constants.java */
		_rightMaster.configSelectedFeedbackCoefficient(	Constants.kTurnTravelUnitsPerRotation / Constants.kPigeonUnitsPerRotation,	// Coefficient
														Constants.PID_TURN, 														// PID Slot of Source
														Constants.kTimeoutMs);														// Configuration Timeout
		/* Configure output and sensor direction */
		_leftMaster.setInverted(false);
		_leftMaster.setSensorPhase(true);
		_rightMaster.setInverted(true);
		_rightMaster.setSensorPhase(true);
		lbSlave.setInverted(false);
		lbSlave.setSensorPhase(true);
		rbSlave.setInverted(true);
		rbSlave.setSensorPhase(true);
		
		
		/* Set status frame periods */
		_rightMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 50, Constants.kTimeoutMs);
		_rightMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 50, Constants.kTimeoutMs);
		_rightMaster.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 50, Constants.kTimeoutMs);
		_pidgey.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR , 100, 100);
		
		/* Configure neutral deadband */
		_rightMaster.configNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);
		_leftMaster.configNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);	
		lbSlave.configNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);		
		rbSlave.configNeutralDeadband(Constants.kNeutralDeadband, Constants.kTimeoutMs);		


		/* max out the peak output (for all modes).  However you can
		 * limit the output of a given PID object with configClosedLoopPeakOutput().
		 */
		_leftMaster.configPeakOutputForward(+1.0, Constants.kTimeoutMs);
		_leftMaster.configPeakOutputReverse(-1.0, Constants.kTimeoutMs);
		_rightMaster.configPeakOutputForward(+1.0, Constants.kTimeoutMs);
		_rightMaster.configPeakOutputReverse(-1.0, Constants.kTimeoutMs);
		lbSlave.configPeakOutputForward(+1.0, Constants.kTimeoutMs);
		lbSlave.configPeakOutputReverse(-1.0, Constants.kTimeoutMs);
		rbSlave.configPeakOutputForward(+1.0, Constants.kTimeoutMs);
		rbSlave.configPeakOutputReverse(-1.0, Constants.kTimeoutMs);
		
		/* FPID Gains for turn servo */
		/*_rightMaster.config_kP(Constants.kSlot_Turning, Constants.kGains_Turning.kP, Constants.kTimeoutMs);
		_rightMaster.config_kI(Constants.kSlot_Turning, Constants.kGains_Turning.kI, Constants.kTimeoutMs);
		_rightMaster.config_kD(Constants.kSlot_Turning, Constants.kGains_Turning.kD, Constants.kTimeoutMs);
		_rightMaster.config_kF(Constants.kSlot_Turning, Constants.kGains_Turning.kF, Constants.kTimeoutMs);
		_rightMaster.config_IntegralZone(Constants.kSlot_Turning, (int)Constants.kGains_Turning.kIzone, Constants.kTimeoutMs);*/
		_rightMaster.configClosedLoopPeakOutput(Constants.kSlot_Turning, Constants.kGains_Turning.kPeakOutput, Constants.kTimeoutMs);
		_rightMaster.configAllowableClosedloopError(Constants.kSlot_Turning, 0, Constants.kTimeoutMs);	
		
		/* 1ms per loop.  PID loop can be slowed down if need be.
		 * For example,
		 * - if sensor updates are too slow
		 * - sensor deltas are very small per update, so derivative error never gets large enough to be useful.
		 * - sensor movement is very slow causing the derivative error to be near zero.
		 */
		int closedLoopTimeMs = 1;
		_rightMaster.configSetParameter(ParamEnum.ePIDLoopPeriod, closedLoopTimeMs, 0x00, 0, Constants.kTimeoutMs);
		_rightMaster.configSetParameter(ParamEnum.ePIDLoopPeriod, closedLoopTimeMs, 0x00, 1, Constants.kTimeoutMs);

		/* configAuxPIDPolarity(boolean invert, int timeoutMs)
		 * false means talon's local output is PID0 + PID1, and other side Talon is PID0 - PID1
		 * true means talon's local output is PID0 - PID1, and other side Talon is PID0 + PID1
		 */
		_rightMaster.configAuxPIDPolarity(false, Constants.kTimeoutMs);

		/* Initialize */
		_firstCall = true;
		_state = false;
		zeroHeading();
	}
	
	@Override
	public void teleopPeriodic() {
		/* Gamepad processing */
		double forward = -1 * _gamepad.getY();
		double turn = _gamepad.getTwist();
		forward = Deadband(forward);
		turn = Deadband(turn);
	
		/* Button processing for state toggle and sensor zeroing */
		getButtons(btns, _gamepad);
		if(btns[2] && !_btns[2]){
			_state = !_state; 			// Toggle state
			_firstCall = true;			// Mode change, do first call operation
			//_targetAngle = _rightMaster.getSelectedSensorPosition(Constant.PID_TURN);
			_pidgey.getYawPitchRoll(ypr);
			_targetAngle = ypr[0];
		}else if (btns[1] && !_btns[1]) {
			zeroHeading();				//Zero sensors
		}
		/* Store current buttons into previous button array for tracking */
		System.arraycopy(btns, 0, _btns, 0, Constants.kNumButtonsPlusOne);
				
		/* Select drive mode based on current state */
		if(!_state){
			if (_firstCall)
				System.out.println("This is a basic arcade drive.\n");
			
			_leftMaster.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, +turn);
			_rightMaster.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, -turn);
			lbSlave.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, +turn);
			rbSlave.set(ControlMode.PercentOutput, forward, DemandType.ArbitraryFeedForward, -turn);


		}else{
			if (_firstCall) {
				System.out.println("This is Drive Straight using the new Auxillary feature with Pigeon to maintain current angle.\n");
				
				/* Determine which slot affects which PID */
				_rightMaster.selectProfileSlot(Constants.kSlot_Turning, Constants.PID_TURN);
			}

			/* Configured for percentOutput with Auxiliary PID on Pigeon's Yaw */
			_rightMaster.set(ControlMode.PercentOutput, forward, DemandType.AuxPID, _targetAngle);
			_leftMaster.follow(_rightMaster, FollowerType.AuxOutput1);
			lbSlave.follow(_leftMaster, FollowerType.PercentOutput);
			rbSlave.follow(_rightMaster, FollowerType.PercentOutput);

			
			/* Print some Closed Loop information */
			System.out.println(	"TargetAng: " + _targetAngle + 
								" CurrentAng: " + _rightMaster.getSelectedSensorPosition(Constants.PID_TURN) + 
								" Error; " + _rightMaster.getClosedLoopError(Constants.PID_TURN));
		}
		_firstCall = false;
	}
	
	/** Zero all sensors used. */
	void zeroHeading() {
		_pidgey.setYaw(0, 100);
		_pidgey.setAccumZAngle(0, 100);
		System.out.println("[Pigeon] All sensors are zeroed.\n");
	}
	
	/** Deadband 5 percent, used on the gamepad */
	double Deadband(double value) {
		/* Upper deadband */
		if (value >= +0.05) 
			return value;
		
		/* Lower deadband */
		if (value <= -0.05)
			return value;
		
		/* Outside deadband */
		return 0;
	}
	
	/** Gets all buttons from gamepad */
	void getButtons(boolean[] btns, Joystick gamepad) {
		for (int i = 1; i < Constants.kNumButtonsPlusOne; ++i) {
			btns[i] = gamepad.getRawButton(i);
		}
	}
}
