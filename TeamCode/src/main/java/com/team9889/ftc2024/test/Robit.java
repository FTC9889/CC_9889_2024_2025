package com.team9889.ftc2024.test;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.team9889.ftc2024.subsystems.Drive;
import com.team9889.ftc2024.subsystems.ScoringLift;
import com.team9889.ftc2024.subsystems.Intake;
import com.team9889.ftc2024.subsystems.Hanger;

public class Robit {    Drive mDrive=new Drive();
    Intake mIntake=new Intake();
ScoringLift mHopper=new ScoringLift();
Hanger mLift=new Hanger();

public void init (HardwareMap hardwareMap){
    mDrive.init(hardwareMap);
    mIntake.init(hardwareMap);
    mHopper.init(hardwareMap);
    mLift.init(hardwareMap);
}

}


