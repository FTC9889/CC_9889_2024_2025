package com.team9889.ftc2024.opmode.NewAutonomus.TeamColor;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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


        waitForStart();


        while (opModeIsActive()) {
            mRobot.mArm.setExtetion(maxExtension);
            sleep(1000);

            mRobot.mArm.setClawPosition(openClaw);
            sleep(1000);

            mRobot.mArm.setExtetion(minExtention);
            sleep(1000);

            mRobot.mDrive.backward(0.5);
            sleep(750);
            mRobot.mDrive.brake();

            mRobot.mDrive.turnRight(0.5);
            sleep(turn);
            mRobot.mDrive.brake();

            mRobot.mDrive.forward(0.5);
            sleep(1200);
            mRobot.mDrive.brake();

            mRobot.mArm.setArmRotation(0.2);
            sleep(1000);
            mRobot.mArm.setArmRotation(0);

            mRobot.mArm.setExtetion(maxExtension);
            sleep(500);

            mRobot.mArm.setArmRotation(-0.2);
            sleep(1000);
            mRobot.mArm.setArmRotation(0);
        }

    }
}