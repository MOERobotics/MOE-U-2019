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


    TalonSRX motorLeft1 = new TalonSRX(12); //TalonSRX = speed controller
    TalonSRX motorLeft2 = new TalonSRX(13);
    TalonSRX motorLeft3 = new TalonSRX(14);
    TalonSRX motorRight1 = new TalonSRX(1);
    TalonSRX motorRight2 = new TalonSRX(2);
    TalonSRX motorRight3 = new TalonSRX(3);

    Joystick leftJoystick = new Joystick(0);


    Encoder leftEncoder = new Encoder(0, 1, false, CounterBase.EncodingType.k1X); //Encoder = motor diagnostics
    Encoder rightEncoder = new Encoder(2, 3, false, CounterBase.EncodingType.k1X);


    double goalYaw; //for turning left and right 90deg
    boolean isTurningLeft;


    AHRS navx; //navx = directional stuff

    public Robot() {
        motorRight1.setInverted(true); //motors...duh
        motorRight2.setInverted(true);
        motorRight3.setInverted(true);


        motorLeft1.setNeutralMode(NeutralMode.Brake);
        motorLeft2.setNeutralMode(NeutralMode.Brake);
        motorLeft3.setNeutralMode(NeutralMode.Brake);
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

        SmartDashboard.putNumber("leftEncoderValue", leftEncoder.get()); //Screen information
        SmartDashboard.putNumber("rightEncoderValue", rightEncoder.get());
        SmartDashboard.putNumber("heading ", navx.getYaw());


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
        navx.reset(); //resets Yaw (to face current direction)
    }

    @Override
    public void teleopPeriodic() {

        //speedPct = - leftJoystick.getY();


        double leftPower = 0;
        double rightPower = 0;
        double currentYaw = navx.getYaw();


        //double joyY = - leftJoystick.getY(); //inverted Y-axis
        //double joyX = leftJoystick.getX();

        //leftPower = joyY + joyX;
        //rightPower = joyY - joyX;


        if (leftJoystick.getRawButton(1)) { //forward from 0 Yaw (keeps Yaw close to 0)
            if (currentYaw > 1) { //0 is the sweet spot
                rightPower = .25;
                leftPower = .2;
            } else if (currentYaw < -1) {
                rightPower = .2;
                leftPower = .25;
            } else {
                rightPower = .25;
                leftPower = .25;
            }
        }

        double leftEncode = leftEncoder.get();
        double rightEncode = rightEncoder.get();


        //HAS NOT BEEN TESTED!!
            if(leftJoystick.getRawButtonPressed(5)){ //acknowledge turing left on button 5 press
                goalYaw = currentYaw - 90; //-90 is sweet spot
                isTurningLeft = true;
                }

            if(isTurningLeft){ //turn left 90deg
                if (currentYaw > (goalYaw + 3)) {
                    rightPower = .2;
                    leftPower = -.2;
                } else if (currentYaw < (goalYaw - 3)) {
                    rightPower = -.2;
                    leftPower = .2;
                } else{
                    isTurningLeft = false;
                }
            }




        /*
        if(leftJoystick.getRawButton(1)){ //forward from Encoders
            if(leftEncode ...){
                rightPower = .25;
                leftPower = .2;
            }
           if(rightEncode ...){
                    rightPower = .2;
                    leftPower = .25;
                } else {
                    rightPower = .25;
                    leftPower = .25;
                }
        }



         if(true){ //forward from set angle



        }

         */


        motorLeft1.set(ControlMode.PercentOutput, leftPower);
        motorLeft2.set(ControlMode.PercentOutput, leftPower);
        motorLeft3.set(ControlMode.PercentOutput, leftPower);
        motorRight1.set(ControlMode.PercentOutput, rightPower);
        motorRight2.set(ControlMode.PercentOutput, rightPower);
        motorRight3.set(ControlMode.PercentOutput, rightPower);

    }

    @Override
    public void testInit() {
    }

    @Override
    public void testPeriodic() {
    }


}
