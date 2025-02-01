package com.team9889.ftc2024.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.team9889.lib.pedroPathing.constants.FConstants;
import com.team9889.lib.pedroPathing.constants.LConstants;

public class Robot {

    public Follower mDrive;
    public Intake mIntake = new Intake();
    public Lift mLift = new Lift();
    public Flag mFlag = new Flag();


    public void init(HardwareMap hardwareMap) {
        Constants.setConstants(FConstants.class, LConstants.class);
        mDrive = new Follower(hardwareMap);
        mIntake.init(hardwareMap);
        mLift.init(hardwareMap);
        mFlag.init(hardwareMap);
    }
}


