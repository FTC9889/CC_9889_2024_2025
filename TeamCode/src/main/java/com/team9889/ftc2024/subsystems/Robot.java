package com.team9889.ftc2024.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {

    public PinpointDrive mDrive;
    public Intake mIntake = new Intake();
    public Lift mLift = new Lift();


    public void init(HardwareMap hardwareMap, Pose2d pose2d) {
        mDrive = new PinpointDrive(hardwareMap, pose2d);
        mIntake.init(hardwareMap);
        mLift.init(hardwareMap);
    }

    public void init(HardwareMap hardwareMap) {
        init(hardwareMap, new Pose2d(0,0,0));
    }
}


