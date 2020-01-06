package frc.robot.GenericRobots;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.*;


public class Camoelot extends GenericRobot {

    TalonSRX motorLeft1 = new TalonSRX(12);
    TalonSRX motorLeft2 = new TalonSRX(13);
    TalonSRX motorLeft3 = new TalonSRX(14);
    TalonSRX motorRight1 = new TalonSRX(1);
    TalonSRX motorRight2 = new TalonSRX(2);
    TalonSRX motorRight3 = new TalonSRX(3);

    Encoder leftEncoder = new Encoder(0, 1, false, CounterBase.EncodingType.k1X);
    Encoder rightEncoder = new Encoder(2, 3, false, CounterBase.EncodingType.k1X);

    AHRS navx = new AHRS(SPI.Port.kMXP, (byte) 50);

    public Camoelot() {
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
    public void setMotorPowers(double leftPower, double rightPower) {
        motorLeft1.set(ControlMode.PercentOutput, (leftPower));
        motorLeft2.set(ControlMode.PercentOutput, (leftPower));
        motorLeft3.set(ControlMode.PercentOutput, (leftPower));
        motorRight1.set(ControlMode.PercentOutput, (rightPower));
        motorRight2.set(ControlMode.PercentOutput, (rightPower));
        motorRight3.set(ControlMode.PercentOutput, (rightPower));
    }
    public double getHeadingDegrees(){
        return navx.getYaw();
    }
    public void resetHeading(){
        navx.reset();
    }
    public double getLeftDistanceTicks(){
        return leftEncoder.get();
    }
    public double getRightDistanceTicks(){
        return rightEncoder.get();
    }
    public void resetLeftEncoder() {
        leftEncoder.reset();
    }
    public void resetRightEncoder(){
        rightEncoder.reset();
    }
}

