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
import com.revrobotics.*;



public class Robot extends TimedRobot {
  AHRS navx;

  CANSparkMax driveRightA = new CANSparkMax(20, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax driveRightB = new CANSparkMax(1,  CANSparkMaxLowLevel.MotorType.kBrushless);

  CANSparkMax driveLeftA  = new CANSparkMax(14, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax driveLeftB  = new CANSparkMax(15, CANSparkMaxLowLevel.MotorType.kBrushless);

  CANEncoder encoderRight = new CANEncoder(driveRightA);
  CANEncoder encoderLeft  = new CANEncoder(driveLeftA);

  @Override
  public void robotInit() {

  }

  @Override
  public void robotPeriodic() {
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
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
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }



}
