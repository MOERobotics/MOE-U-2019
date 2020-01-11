package frc.robot;

import frc.robot.GenericRobots.GenericRobot;

public class GoStraightAuto {

    public GenericRobot robot;

    long StartTime = 0;

    /* I am guessing about the PID modules.*/
    PIDModule PIDSteering = new PIDModule(0.06, 1.0e-3, 1.0e-4);
    double correction;

    public int autoStep = 0;

    public void init() {
        autoStep = 0;
        robot.resetHeading();
        StartTime = System.currentTimeMillis();
        PIDSteering.resetError();
    }

    public void run() {
        switch(autoStep) {
            case 0:
                /* Roll forward for 1 second using PID and the NavX to stay straight.
                * Normally, we do not move for a certain time.  Normally, we move for a certain distance.  */

                /*Feed the input into the PID module.  Since we are going straight ahead, the heading is the input.
                If you want to hold a heading of 090 then the correct command would be
                                PIDSteering.setHeading(9robot.getHeadingDegrees()-90);
                */
                PIDSteering.setHeading(robot.getHeadingDegrees());

                /* Read the correction from the PID module.  The PID module is doing all the work.*/
                correction = PIDSteering.getCorrection();

                /* Use the correction to adjust the motor inputs.*/
                robot.setMotorPowers(0.3*(1+correction), 0.3*(1-correction));
                if (System.currentTimeMillis() - StartTime > 1000) {
                    autoStep++;
                }
                break;
            case 1:
                robot.setMotorPowers(0,0);
                break;
        }
    }
}

