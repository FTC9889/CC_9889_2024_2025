package com.team9889.ftc2024.opmode.NewAutonomus.TeamColor;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.team9889.ftc2024.subsystems.Robot;
@Autonomous(preselectTeleOp = "TeleOperate")
public class HumanPlayer extends LinearOpMode {

    Robot mRobot = new Robot();
    ElapsedTime autoTimer = new ElapsedTime();
    @Override
    public void runOpMode() throws InterruptedException {

        mRobot.init(hardwareMap);


        long tile = 830;
        long side_tile = 2300;
        long turn = 3000;
        double openClaw = 0.75;
        double closedClaw = 0;
        double maxExtension =  0.575;
        double minExtention = 0.024;


        mRobot.mArm.setClawPosition(closedClaw);



        mRobot.mArm.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mRobot.mArm.arm.setTargetPosition(650);
        mRobot.mArm.arm.setPower(0.2);
        mRobot.mArm.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        mRobot.mArm.extend.setTargetPosition(0);
        mRobot.mArm.extend.setPower(1);
        mRobot.mArm.extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(1000);

        waitForStart();

        if (opModeIsActive()) {
            mRobot.mDrive.reset_encoder();
            autoTimer.reset();
            while (opModeIsActive() && mRobot.mDrive.front_encoder() < 1500) {
                double AnglePower = -1.3 * mRobot.mDrive.imu.getNormalHeading() / 180;
                mRobot.mDrive.setPower(0.5, 0, AnglePower);
                telemetry.addData("AnglePower", AnglePower);
                telemetry.addData("Encoder", mRobot.mDrive.front_encoder());
                telemetry.update();
            }
            mRobot.mDrive.brake();
        }

        mRobot.mArm.arm.setTargetPosition(0);
        mRobot.mArm.arm.setPower(1);
        mRobot.mArm.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(500);
    }
}