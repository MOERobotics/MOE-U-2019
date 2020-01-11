package frc.robot;

import frc.robot.GenericRobots.GenericRobot;

public class DoNothingAuto {

    public GenericRobot robot;

    public int autoStep = 0;

    public void init() {
        autoStep = 0;
    }

    public void run() {
        switch(autoStep) {
            case 0:
                robot.setMotorPowers(0, 0);
                break;

        }
    }
    }

