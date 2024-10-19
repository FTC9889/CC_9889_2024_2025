package com.team9889.ftc2024.opmode.NewAutonomus.TeamColor;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.team9889.ftc2024.subsystems.Robot;
@Autonomous
public class JustParkTC extends LinearOpMode {

    Robot mRobot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {
        mRobot.init(hardwareMap);

        long tile = 830;
        long side_tile = 2300;
        long turn = 3000;
        double openClaw = 0.75;
        double closedClaw = 0;
        double maxExtension = 0.575;
        double minExtention = 0.024;


        mRobot.mArm.setClawPosition(closedClaw);

        mRobot.mArm.setExtetion(minExtention);

        mRobot.mArm.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mRobot.mArm.arm.setTargetPosition(500);
        mRobot.mArm.arm.setPower(0.2);
        mRobot.mArm.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        waitForStart();


        if (opModeIsActive()) {
            mRobot.mArm.setExtetion(maxExtension);
            sleep(1000);

            mRobot.mArm.setClawPosition(openClaw);
            sleep(1000);

            mRobot.mArm.setExtetion(minExtention);
            sleep(1000);

            mRobot.mDrive.setPower(0.5, 0, 0);
            sleep(750);
            mRobot.mDrive.brake();
        }

    }
}