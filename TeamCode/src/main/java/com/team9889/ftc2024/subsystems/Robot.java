package com.team9889.ftc2024.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

public class Robot {

    public Drive mDrive = new Drive();
    public Intake mIntake = new Intake();
    public Lift mLift = new Lift();


    public void init(HardwareMap hardwareMap) {
        mDrive.init(hardwareMap);
        mIntake.init(hardwareMap);
        mLift.init(hardwareMap);
    }




}


