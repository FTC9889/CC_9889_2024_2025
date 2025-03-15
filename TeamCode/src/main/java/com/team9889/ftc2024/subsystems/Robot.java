package com.team9889.ftc2024.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Pose;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.team9889.lib.pedroPathing.constants.FConstants;
import com.team9889.lib.pedroPathing.constants.LConstants;

public class Robot {
//    AnalogInput ultrasonicSensor;



    public static Pose robotPose = new Pose(0, 0, Math.toRadians(0));

    public Follower mDrive;
    public Intake mIntake = new Intake();
    public Lift mLift = new Lift();
    public Flag mFlag = new Flag();

    /**
     * Converts voltage to distance in inches.
     * URM09 sensor outputs a proportional voltage.
     * Formula: Distance (in) = ((Voltage * 100) / 5) * 0.3937
     */

    private double voltageToDistanceInches(double voltage) {
        return ((voltage * 100) / 5.0) * 0.3937;
    }


//    public double getDistance() {
//        return (41.2 * ultrasonicSensor.getVoltage() - 1.32 + 7.25) * Math.cos(mDrive.getPose().getHeading());
////                * 63.202 + 0.0626 + 3.25)
////                * Math.sin(mDrive.getPose().getHeading());
//    }
//
    public void init(HardwareMap hardwareMap) {
//        ultrasonicSensor = hardwareMap.get(AnalogInput.class, "ultrasonic");

        Constants.setConstants(FConstants.class, LConstants.class);
        mDrive = new Follower(hardwareMap);
        mIntake.init(hardwareMap);
        mLift.init(hardwareMap);
        mFlag.init(hardwareMap);
    }
}


