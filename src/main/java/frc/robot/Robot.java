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

  double goalYaw = 0;
  boolean isTurningLeft = false;
  double error;

  final double TICKS_TO_INCH = 112.08;
  public int autoStep;
  boolean at10Feet = false;



  AHRS navx;

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

    navx = new AHRS(SPI.Port.kMXP, (byte) 50);


    navx.getYaw();

  }


  @Override
  public void robotInit() {
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber( "leftEncoderValue",  leftEncoder.get());
    SmartDashboard.putNumber("rightEncoderValue", rightEncoder.get());
    SmartDashboard.putNumber("heading", navx.getYaw());
    SmartDashboard.putBoolean("Left Turn", isTurningLeft);
    SmartDashboard.putNumber("Goal Yaw", goalYaw);
    SmartDashboard.putNumber("Turn Error", error);
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
    autoStep = 0;
  }

  @Override
  public void autonomousPeriodic() { //Task: 10ft forward, 90deg left

    switch (autoStep){
      case 0:





    }


  }

  @Override
  public void teleopInit() {
    isTurningLeft = false;
    double leftSpeed = 0;
    double rightSpeed = 0;
  }

  @Override
  public void teleopPeriodic() {

    double leftSpeed = 0;
    double rightSpeed = 0;

    double currentYaw = navx.getYaw();



    if (leftJoystick.getRawButtonPressed(5)) {
      goalYaw = currentYaw - 90;

      while (goalYaw < -180) {
        goalYaw = goalYaw + 360;
      }
      while (goalYaw > 180) {
        goalYaw = goalYaw - 360;
      }

      isTurningLeft = true;
    }
    error = (goalYaw - currentYaw);

    if (error > 180) {
      error = error - 360;
    }
    if (error < -180) {
      error = error + 360;
    }

    if(isTurningLeft) {
      if (error > 0) {
        rightSpeed = -.2;
        leftSpeed = .2;
      } else if (error < 0) {
        rightSpeed = .2;
        leftSpeed = -.2;
      }

      if(Math.abs(error) < 3) {
        isTurningLeft = false;
        rightSpeed = 0;
        leftSpeed = 0;
      }

    }









    motorLeft1.set(ControlMode.PercentOutput, leftSpeed);
    motorLeft2.set(ControlMode.PercentOutput, leftSpeed);
    motorLeft3.set(ControlMode.PercentOutput, leftSpeed);
    motorRight1.set(ControlMode.PercentOutput, rightSpeed);
    motorRight2.set(ControlMode.PercentOutput, rightSpeed);
    motorRight3.set(ControlMode.PercentOutput, rightSpeed);



  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }



}

