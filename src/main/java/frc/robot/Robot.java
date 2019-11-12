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

  TalonSRX motorLeft1 = new TalonSRX(12);
  TalonSRX motorLeft2 = new TalonSRX(13);
  TalonSRX motorLeft3 = new TalonSRX(14);
  TalonSRX motorRight1 = new TalonSRX(1);
  TalonSRX motorRight2 = new TalonSRX(2);
  TalonSRX motorRight3 = new TalonSRX(3);

  Joystick leftJoystick = new Joystick(0);

  Encoder leftEncoder = new Encoder(0,1,false, CounterBase.EncodingType.k1X);
  Encoder rightEncoder = new Encoder(2,3,false, CounterBase.EncodingType.k1X);

  AHRS navx = new AHRS(SPI.Port.kMXP, (byte) 50);
  boolean buttonNorth = false;

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

    navx.getYaw();
  }

  @Override
  public void robotInit() {
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("leftEncoderValue", leftEncoder.get());
    SmartDashboard.putNumber("rightEncoderValue", rightEncoder.get());
    SmartDashboard.putNumber("heading", navx.getYaw());
    SmartDashboard.putBoolean("buttonNorth", buttonNorth);
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
      rightEncoder.reset();
      leftEncoder.reset();
  }

  @Override
  public void teleopPeriodic() {

      if (leftJoystick.getRawButton(1)){
          buttonNorth = false;
      }
    //double speedPct = 100;
    //
    //speedPct = - leftJoystick.getY();
    //
    //motorLeft1.set(ControlMode.PercentOutput, speedPct);
    //motorLeft2.set(ControlMode.PercentOutput, speedPct);
    //motorLeft3.set(ControlMode.PercentOutput, speedPct);
    //motorRight1.set(ControlMode.PercentOutput, speedPct);
    //motorRight2.set(ControlMode.PercentOutput, speedPct);
    //motorRight3.set(ControlMode.PercentOutput, speedPct);


      double leftSpeed = 0;
      double rightSpeed = 0;

    double joyY = -leftJoystick.getY();
    double joyX = leftJoystick.getX();

    //leftSpeed = joyY + joyX;
    //rightSpeed = joyY - joyX;
    //make sure to build every time you boot up build
    int buttonHold = 1;

    //if pressing button 1, robot rolls straight ahead slowly

      /**
      //find a way to reset navx when first pressed
      boolean firstHold = false;

      if (!leftJoystick.getRawButton(buttonHold)){
          firstHold = true;
      }
       if (leftJoystick.getRawButton(buttonHold)) {
          if (firstHold == true) {
              navx.reset();
          }
          //firstHold = false;

          double currentYaw = navx.getYaw();
          leftSpeed = 0.25;
          rightSpeed = 0.25;

          if (currentYaw > 1) {
              rightSpeed = 0.30;

          } else if (currentYaw < -1) {
              leftSpeed = 0.30;

          }
      }



       if (leftJoystick.getRawButton(buttonHold)) {
       leftSpeed = 0.25;
       rightSpeed = 0.25;

        if (leftEncoder.get() - rightEncoder.get() > 5) {
            rightSpeed = 0.30;
         } else if (leftEncoder.get() - rightEncoder.get() < -5) {
            leftSpeed = 0.30;
         }
       }
       */

      //never reset navx
      //turn to North
      if (leftJoystick.getRawButton(9)){
          buttonNorth = true;
      }
      int yawTol = 2; //yawTolerance
      if (buttonNorth) {
          double currentYaw = navx.getYaw();
          leftSpeed = 0;
          rightSpeed = 0;
          if (currentYaw > yawTol){
              rightSpeed = 0.20;
              leftSpeed = -0.20;
          } else if (currentYaw < -yawTol){
              leftSpeed = 0.20;
              rightSpeed = -0.20;
          } else {
              buttonNorth = false;
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