/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.*;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;
import com.kauailabs.navx.frc.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends TimedRobot {

	TalonSRX motorLeft1  = new TalonSRX(12);
	TalonSRX motorLeft2  = new TalonSRX(13);
	TalonSRX motorLeft3  = new TalonSRX(14);
	TalonSRX motorRight1 = new TalonSRX( 1);
	TalonSRX motorRight2 = new TalonSRX( 2);
	TalonSRX motorRight3 = new TalonSRX( 3);

	Joystick leftJoystick = new Joystick(0);

	Encoder  leftEncoder = new Encoder(0,1, false, CounterBase.EncodingType.k1X);
	Encoder rightEncoder = new Encoder(2,3, false, CounterBase.EncodingType.k1X);

	AHRS navx = new AHRS(SPI.Port.kMXP, (byte) 50);

	public enum RobotControlMode {
		MANUAL_CONTROL,
		TURNING,
		MOVING
	}

	RobotControlMode currentControlMode = RobotControlMode.MANUAL_CONTROL;

	double targetYaw = 0;
	double yawCorrection = 0;
	FuntionalPIDSource.Rate navxYawReader = new FuntionalPIDSource.Rate(() -> navx.getYaw());
	PIDOutput correctionWriter = (double correction) -> yawCorrection = correction + (Math.signum(correction) * 0.2);

	boolean manualControl = true;

	PIDController yawCorrector = new PIDController(
		0.06,
		0.001,
		0,
		navxYawReader,
		correctionWriter
	) {{
		setInputRange (-180,180);
		setOutputRange(-0.8,0.8);
		setContinuous (    true);
		setEnabled    (   false);
	}};

	public Robot() {
		motorRight1.setInverted(true);
		motorRight2.setInverted(true);
		motorRight3.setInverted(true);

		motorLeft1 .setNeutralMode(NeutralMode.Brake);
		motorLeft2 .setNeutralMode(NeutralMode.Brake);
		motorLeft3 .setNeutralMode(NeutralMode.Brake);
		motorRight1.setNeutralMode(NeutralMode.Brake);
		motorRight2.setNeutralMode(NeutralMode.Brake);
		motorRight3.setNeutralMode(NeutralMode.Brake);
	}


	@Override
	public void robotInit() {
	}

	@Override
	public void robotPeriodic() {
		SmartDashboard.putNumber( "leftEncoderValue",             leftEncoder.get());
		SmartDashboard.putNumber("rightEncoderValue",            rightEncoder.get());
		SmartDashboard.putNumber(          "heading",                 navx.getYaw());
		SmartDashboard.putNumber(    "targetHeading",                     targetYaw);
		SmartDashboard.putString(      "controlMode", currentControlMode.toString());
	}

	@Override
	public void disabledInit() {
	}

	@Override
	public void disabledPeriodic() {
		if (leftJoystick.getRawButton(4)) {
			leftEncoder.reset();
			rightEncoder.reset();
			navx.reset();
		}
	}

	@Override
	public void autonomousInit() {
	}

	@Override
	public void autonomousPeriodic() {
	}

	@Override
	public void teleopInit() {
	}

	@Override
	public void teleopPeriodic() {

		double currentYaw = navx.getYaw();

		double  leftSpeed = 0;
		double rightSpeed = 0;

		boolean restartLoop;

		do {
			restartLoop = false;
			switch (currentControlMode) {
				default:
				case MANUAL_CONTROL:
					if (leftJoystick.getRawButton( 5)) startTurn(currentYaw-90);
					if (leftJoystick.getRawButton(11)) startTurn(currentYaw+90);
					if (leftJoystick.getRawButton( 9)) startTurn(  0);
					if (leftJoystick.getRawButton(10)) startTurn(180);

					if (currentControlMode != RobotControlMode.MANUAL_CONTROL) {
						restartLoop = true;
						continue;
					}

					double joyY = -leftJoystick.getY();
					double joyX = leftJoystick.getX();

					leftSpeed = joyY + joyX;
					rightSpeed = joyY - joyX;
					break;

				case TURNING:
					if (
						Math.abs(targetYaw-currentYaw) < 2 &&
						Math.abs(yawCorrection) < 0.25
					) assumeDirectControl();
					if (leftJoystick.getRawButton( 1)) assumeDirectControl();

					if (currentControlMode != RobotControlMode.TURNING) {
						restartLoop = true;
						continue;
					}

					double maxSpeed = 0.4;
					double turnPower = maxSpeed * yawCorrection;
					leftSpeed = turnPower;
					rightSpeed = -turnPower;
					break;

			}
		} while (restartLoop);


		SmartDashboard.putNumber(" leftPowerPct",  leftSpeed * 100);
		SmartDashboard.putNumber("rightPowerPct", rightSpeed * 100);
		setMotorPower(leftSpeed,rightSpeed);
	}

	@Override
	public void testInit() {
	}

	@Override
	public void testPeriodic() {
	}

	void assumeDirectControl() {
		yawCorrector.setEnabled(false);
		yawCorrector.reset();
		yawCorrector.setEnabled(false);
		yawCorrection = 0;
		currentControlMode = RobotControlMode.MANUAL_CONTROL;
	}

	void startTurn(double targetAngle) {
		while (targetAngle >  180) targetAngle -= 360;
		while (targetAngle < -180) targetAngle += 360;
		targetYaw = targetAngle;
		currentControlMode = RobotControlMode.TURNING;
		yawCorrector.setSetpoint(targetAngle);
		yawCorrector.setEnabled(true);
	}

	void setMotorPower(double leftMotor, double rightMotor) {
		motorLeft1 .set(ControlMode.PercentOutput,  leftMotor);
		motorLeft2 .set(ControlMode.PercentOutput,  leftMotor);
		motorLeft3 .set(ControlMode.PercentOutput,  leftMotor);
		motorRight1.set(ControlMode.PercentOutput, rightMotor);
		motorRight2.set(ControlMode.PercentOutput, rightMotor);
		motorRight3.set(ControlMode.PercentOutput, rightMotor);
	}

}
