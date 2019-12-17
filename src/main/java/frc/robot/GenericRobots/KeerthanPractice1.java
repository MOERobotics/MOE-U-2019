package frc.robot.GenericRobots;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;

public class KeerthanPractice1 extends GenericRobot{

    AHRS navx = new AHRS(SPI.Port.kMXP, (byte) 50);

    CANSparkMax driveRightA = new CANSparkMax(20, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax driveRightB = new CANSparkMax(1,  CANSparkMaxLowLevel.MotorType.kBrushless);

    CANSparkMax driveLeftA  = new CANSparkMax(14, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax driveLeftB  = new CANSparkMax(15, CANSparkMaxLowLevel.MotorType.kBrushless);

    CANEncoder encoderRight = new CANEncoder(driveRightA);
    CANEncoder encoderLeft  = new CANEncoder(driveLeftA);

    Solenoid absolutelyUseless = new Solenoid(0);

    public KeerthanPractice1() {
        driveRightB.follow(driveRightA);
        driveLeftB.follow(driveLeftA);

        driveRightA.setIdleMode(CANSparkMax.IdleMode.kBrake);
        driveRightB.setIdleMode(CANSparkMax.IdleMode.kBrake);
        driveLeftA.setIdleMode(CANSparkMax.IdleMode.kBrake);
        driveLeftB.setIdleMode(CANSparkMax.IdleMode.kBrake);

        driveRightA.setInverted(true);
    }

    @Override
    public void setMotorPowers(double leftPower, double rightPower) {
        driveRightA.set(rightPower);
        driveRightB.set(rightPower);
        driveLeftA .set(leftPower) ;
        driveLeftB .set(leftPower) ;
    }

    public double getHeadingDegrees() {
        return navx.getYaw();
    }

    public void resetHeading() {
        navx.reset();
    }

    @Override
    public double getLeftDistanceTicks() {
        return encoderLeft.getPosition();
    }

    @Override
    public double getRightDistanceTicks() {
        return encoderRight.getPosition();
    }

    @Override
    public void resetLeftEncoders() {
        encoderLeft.setPosition(0);
    }

    @Override
    public void resetRightEncoders() {
        encoderRight.setPosition(0);
    }
}
