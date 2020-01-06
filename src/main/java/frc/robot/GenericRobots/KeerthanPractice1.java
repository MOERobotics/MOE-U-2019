package frc.robot.GenericRobots;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;

public class KeerthanPractice1 extends GenericRobot{

    AHRS navx = new AHRS(SPI.Port.kMXP, (byte) 50);

    CANSparkMax driveRightA = new CANSparkMax (20, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax driveRightB = new CANSparkMax (20, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax driveLeftA = new CANSparkMax (20, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax driveLeftB = new CANSparkMax (20, CANSparkMaxLowLevel.MotorType.kBrushless);

    CANEncoder encoderRight = new CANEncoder(driveRightA);
    CANEncoder encoderLeft = new CANEncoder(driveLeftA);

    Solenoid absolutelyUseless= new Solenoid(0);

    public KeerthanPractice1(){
        driveRightB.setIdleMode(CANSparkMax.IdleMode.kBrake);
        driveLeftB.setIdleMode(CANSparkMax.IdleMode.kBrake);

        driveRightA.setIdleMode(CANSparkMax.IdleMode.kBrake);
        driveLeftA.setIdleMode(CANSparkMax.IdleMode.kBrake);

        driveRightA.setInverted(true);
    }

    @Override
    public void setMotorPowers(double leftPower, double rightPower) {
        driveRightA.set(rightPower);
        driveRightB.set(rightPower);
        driveLeftA.set(leftPower);
        driveLeftB.set(leftPower);
    }

    @Override
    public double getHeadingDegrees() {
        return 0;
    }

    @Override
    public void resetHeading() {

    }

    @Override
    public double getLeftDistanceTicks() {
        return 0;
    }

    @Override
    public double getRightDistanceTicks() {
        return 0;
    }

    @Override
    public void resetLeftEncoder() {
        encoderLeft.setPosition(0);
    }

    @Override
    public void resetRightEncoder() {
        encoderRight.setPosition(0);
    }
}
