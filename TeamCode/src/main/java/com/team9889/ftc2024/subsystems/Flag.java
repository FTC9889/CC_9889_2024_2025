package com.team9889.ftc2024.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.logging.Handler;

public class Flag {

    Servo flag;

    public void init(HardwareMap hardwareMap){
        flag = hardwareMap.servo.get("flag");
    }

    public void setFlagPosition(double position){
        flag.setPosition(position);
    }
    //flg deployed 0.65
    //flg initial 0.93
}
