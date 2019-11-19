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
  TalonSRX testTalon;

  TalonSRX motorLeft1 = new TalonSRX(12);
  TalonSRX motorLeft2 = new TalonSRX(13);
  TalonSRX motorLeft3 = new TalonSRX(14);
  TalonSRX motorRight1 = new TalonSRX(1);
  TalonSRX motorRight2 = new TalonSRX(2);
  TalonSRX motorRight3 = new TalonSRX(3);

  Joystick leftJoystick = new Joystick(0);

  Encoder leftEncoder = new Encoder(0, 1, false, CounterBase.EncodingType.k1X);
  Encoder rightEncoder = new Encoder(2, 3, false, CounterBase.EncodingType.k1X);

  AHRS navx = new AHRS(SPI.Port.kMXP, (byte) 50);

  private boolean backwardPressed;


  int firstCheck = 1;

  public Robot() {
    motorRight1.setInverted(true);
    motorRight2.setInverted(true);
    motorRight3.setInverted(true);

    motorLeft1.setNeutralMode(NeutralMode.Brake);
    motorLeft2.setNeutralMode(NeutralMode.Brake);
    motorLeft3.setNeutralMode(NeutralMode.Brake);
    motorRight1.setNeutralMode(NeutralMode.Brake);
    motorRight2.setNeutralMode(NeutralMode.Brake);
    motorRight3.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void robotInit() {
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("leftEncoderValue", leftEncoder.get());
    SmartDashboard.putNumber("rightEncoderValue", rightEncoder.get());
    SmartDashboard.putNumber("heading", navx.getYaw());
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
        leftEncoder.reset();
        rightEncoder.reset();
        navx.reset();

        firstCheck = 1;
    }

    @Override
    public void teleopPeriodic() {
        double leftSpeed = 0;
        double rightSpeed = 0;

        if (leftJoystick.getRawButton(8)) {
            backwardPressed = true;
            firstCheck = 1;
        } else if(leftJoystick.getRawButton(1)){
            backwardPressed = false;
        }

        if (backwardPressed) {

            if (leftEncoder.get() > -2754) {
                leftSpeed = -0.25;
                rightSpeed = -0.25;
            } else {
                leftSpeed = 0;
                rightSpeed = 0;
                leftEncoder.reset();
                backwardPressed = false;
            }

        }

        if (leftJoystick.getRawButton(7)) {
            firstCheck = 0;
            backwardPressed = false;
        }
        if ((firstCheck == 0) && (leftEncoder.get() < 2754)) {
            leftSpeed = .25;
            rightSpeed = .25;
        }
        else if (!backwardPressed){
            leftEncoder.reset();
            firstCheck = 1;
        }
        setDrivePower(leftSpeed,rightSpeed);
    }

    public void setDrivePower(double leftSide, double rightSide) {
        motorLeft1.set(ControlMode.PercentOutput, leftSide);
        motorLeft2.set(ControlMode.PercentOutput, leftSide);
        motorLeft3.set(ControlMode.PercentOutput, leftSide);
        motorRight1.set(ControlMode.PercentOutput, rightSide);
        motorRight2.set(ControlMode.PercentOutput, rightSide);
        motorRight3.set(ControlMode.PercentOutput, rightSide);
    }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}