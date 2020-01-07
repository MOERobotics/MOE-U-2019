package frc.robot.GenericRobots;

public abstract class GenericRobot {
    public void driveForward(double drivePower){
        setMotorPowers(drivePower, drivePower);
    }
    public abstract void setMotorPowers(double leftPower, double rightPower);

    public abstract double getHeadingDegrees();
    public abstract void resetHeading();

    public abstract double getLeftDistanceTicks();
    public abstract double getRightDistanceTicks();
    public abstract void resetLeftEncoders();
    public abstract void resetRightEncoders();

    public abstract void shiftUp();
    public abstract void shiftDown();

}