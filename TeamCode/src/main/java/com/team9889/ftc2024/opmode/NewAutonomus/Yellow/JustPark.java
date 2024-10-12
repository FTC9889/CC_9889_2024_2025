package com.team9889.ftc2024.opmode.NewAutonomus.Yellow;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.team9889.ftc2024.subsystems.Robot;

@Autonomous
public class JustPark extends LinearOpMode {

    Robot mRobot = new Robot();
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

        mRobot.mArm.setExtetion(minExtention);

        waitForStart();



        if (opModeIsActive()){
            mRobot.mArm.setArmRotation(0.5);
            sleep(850);
            mRobot.mArm.setArmRotation(0);

            mRobot.mDrive.forward(0.5);
            sleep(1200);
            mRobot.mDrive.brake();

            mRobot.mArm.setExtetion(maxExtension);
            sleep(2000);

            mRobot.mArm.setClawPosition(openClaw);
            sleep(3000);

            mRobot.mArm.setExtetion(minExtention);
            sleep(2000);

            mRobot.mArm.setArmRotation(-0.5);
            sleep(750);
            mRobot.mArm.setArmRotation(0);

            mRobot.mDrive.forward(0.5);
            sleep(500);
            mRobot.mDrive.brake();
        }

    }
}
