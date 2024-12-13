package com.team9889.ftc2024.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.team9889.ftc2024.subsystems.Robot;

@TeleOp
@Disabled
public class TeleOpTest extends LinearOpMode {
    TouchSensor touchSensor;
    Robot mRobot = new Robot();
    ElapsedTime elapseTimer = new ElapsedTime();
    boolean press = false;
    boolean rotationPress = false;
    boolean reset = false;



    double servoPosition = 0.024;
    @Override
    public void runOpMode() throws InterruptedException {
        touchSensor = hardwareMap.get(TouchSensor.class, "armSensor");

        mRobot.init(hardwareMap);

        int newTarget = mRobot.mArm.arm.getCurrentPosition();

        waitForStart();

        ElapsedTime Timer = new ElapsedTime();

        waitForStart();

        while (opModeIsActive()){

//            if (gamepad2.x){
//                mRobot.mArm.setIntake2Power(1);
//            } else if (gamepad2.b) {
//                mRobot.mArm.setIntake2Power(-1);
//            }else {
//                mRobot.mArm.setIntake2Power(0);
//            }
        }
    }
}
