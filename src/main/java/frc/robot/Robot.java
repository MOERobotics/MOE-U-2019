/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.*;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;
import com.kauailabs.navx.frc.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.GenericRobots.Camoelot;
import frc.robot.GenericRobots.GenericRobot;
import frc.robot.GenericRobots.KeerthanPractice1;
import frc.robot.PIDModule;

public class Robot extends TimedRobot {

  GenericRobot robbit = new KeerthanPractice1();
  Joystick leftJoystick = new Joystick(0);

  PIDModule MovementPID = new PIDModule(1.0e-3,1.0e-4,1.0e-4);

  NetworkTable table;

  public Robot() {

  }

  @Override
  public void robotInit() {
    try {
      table = NetworkTableInstance.getDefault().getTable("limelight");
    } catch (Exception e) { }

    robbit.shiftUp();
  }

  @Override
  public void robotPeriodic() {

    //limelight
    try {
      NetworkTableEntry tx = table.getEntry("tx");
      NetworkTableEntry ty = table.getEntry("ty");
      NetworkTableEntry ta = table.getEntry("ta");

      double limeX = tx.getDouble(0.0);
      double limeY = ty.getDouble(0.0);
      double limeArea = ta.getDouble(0.0);

      SmartDashboard.putNumber("Limelight X", limeX);
      SmartDashboard.putNumber("Limelight Y", limeY);
      SmartDashboard.putNumber("Limelight Area", limeArea);
    } catch (Exception e) {}

    SmartDashboard.putNumber("leftEncoderValue", robbit.getLeftDistanceTicks());
    SmartDashboard.putNumber("rightEncoderValue", robbit.getRightDistanceTicks());
    SmartDashboard.putNumber("heading", robbit.getHeadingDegrees());
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    if (leftJoystick.getRawButton(4)) {
        robbit.resetLeftEncoders();
        robbit.resetRightEncoders();
        robbit.resetHeading();
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


    double leftSpeed = 0;
    double rightSpeed = 0;

    double joyY = - leftJoystick.getY();
    double joyX = leftJoystick.getX();

    long    startTime = 0;

    long turnTime = 100;
    double turnPower = 0.15;
    boolean turning = false;
    double powerIncrement = 0.05; //tbd

    SmartDashboard.putNumber("turnPower", turnPower); //does this have to be in robotPeriodic?

    if (Math.abs(joyX)<0.1) {
      joyX = 0;
    }
    if (Math.abs(joyY)<0.1) {
      joyY = 0;
    }
    leftSpeed = joyY + joyX;
    rightSpeed = joyY - joyX;


    if (leftJoystick.getRawButtonPressed(16)) {
      robbit.shiftDown();
    }

    if (leftJoystick.getRawButtonPressed(11)) {
      robbit.shiftUp();
    }

    if (leftJoystick.getRawButtonPressed(12)) {
      robbit.resetHeading();
      startTime = System.currentTimeMillis();
      turning = true;
    }

    if (leftJoystick.getRawButtonPressed(13)) { //increase turnPower, button tbd
      turnPower = turnPower + powerIncrement;
    }

    if (leftJoystick.getRawButtonPressed(14)) { //decrease turnPower, button tbd
      turnPower = turnPower - powerIncrement;
    }

    if (turning)
    {
      leftSpeed = turnPower;
      rightSpeed = -turnPower;
      if (System.currentTimeMillis()-startTime > turnTime) {
        turning = false;
      }
    }

    robbit.setMotorPowers(leftSpeed, rightSpeed);

  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}