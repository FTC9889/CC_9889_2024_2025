package com.team9889.ftc2024.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Hanger {
    DcMotor hang;

    public void init(HardwareMap hardwaremap) {
        hang = hardwaremap.dcMotor.get("hang");


        hang.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void setHangPower(double h){
        hang.setPower(h);
    }
}
// max       0.575
// min       0.024