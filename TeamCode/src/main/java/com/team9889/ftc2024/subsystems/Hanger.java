package com.team9889.ftc2024.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Hanger {
    public DcMotor hang;

    public void init(HardwareMap hardwareMap){
        hang = hardwareMap.dcMotor.get("hang");

        hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hang.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setHangPower(double power){
        hang.setPower(power);
    }
}
// max       0.575
// min       0.024