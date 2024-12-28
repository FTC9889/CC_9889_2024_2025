package com.team9889.ftc2024.opmode.NewAutonomus.Yellow;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.team9889.ftc2024.subsystems.Robot;

@Autonomous(name="Basket", group="Auto")

public class Cycles extends LinearOpMode {
    Robot mRobot = new Robot();




    public void runOpMode() throws InterruptedException {

        mRobot.init(hardwareMap);

        long tile = 830;
        long side_tile = 2300;
        long turn = 3000;
        double openClaw = 0.75;
        double closedClaw = 0;
        double maxExtension =  0.575;
        double minExtention = 0.024;




        waitForStart();


        while (opModeIsActive()){

//            //First Sample
//            mRobot.mDrive.backward( 0.5);
//            sleep(400);
//            mRobot.mDrive.brake();
//
//            //Score
//            mRobot.mArm.setArmRotation(0.5);
//            sleep(800);
//            mRobot.mArm.setArmRotation(0);
//            sleep(1000);
//
//            mRobot.mArm.setExtetion(maxExtension);
//            sleep(750);
//
//            mRobot.mArm.setClawPosition(openClaw);
//            sleep(400);
//
//            mRobot.mArm.setExtetion(minExtention);
//            sleep(750);
//
//            mRobot.mArm.setArmRotation(-0.5);
//            sleep(800);
//            mRobot.mArm.setArmRotation(0);
//
//            mRobot.mDrive.forward(0.5);
//            sleep(tile);
//            mRobot.mDrive.brake();
//
//            mRobot.mDrive.turnLeft(0.5);
//            sleep(turn);
//            mRobot.mDrive.brake();
//
//
//
//
//
//            // Second Sample // First Cycle
//            //Move
//            mRobot.mDrive.forward(0.5);
//            sleep(1200);
//            mRobot.mDrive.brake();
//
//            mRobot.mDrive.turnLeft(0.5);
//            sleep(turn);
//            mRobot.mDrive.brake();
//
//
//
//            //Intake
//            mRobot.mArm.setExtetion(0.2);
//            sleep(500);
//
//            mRobot.mArm.setClawPosition(closedClaw);
//            sleep(500);
//
//            mRobot.mArm.setExtetion(0);
//            sleep(500);
//
//
//            // Move
//            mRobot.mDrive.turnRight(0.5);
//            sleep(600);
//            mRobot.mDrive.brake();
//
//            mRobot.mDrive.backward(0.5);
//            sleep(1200);
//            mRobot.mDrive.brake();
//
//
//            //Score
//            mRobot.mArm.setArmRotation(0.5);
//            sleep(800);
//            mRobot.mArm.setArmRotation(0);
//
//            mRobot.mArm.setExtetion(maxExtension);
//            sleep(750);
//
//            mRobot.mArm.setClawPosition(openClaw);
//            sleep(400);
//
//            mRobot.mArm.setExtetion(minExtention);
//            sleep(750);
//
//            mRobot.mArm.setArmRotation(-0.5);
//            sleep(800);
//            mRobot.mArm.setArmRotation(0);
//
//
//            //Third Sample // Second Cycle
//            //Move
//            mRobot.mDrive.forward(0.5);
//            sleep(1200);
//            mRobot.mDrive.brake();
//
//            mRobot.mDrive.turnRight(-0.5);
//            sleep(600);
//            mRobot.mDrive.brake();
//
//
//            //Intake
//            mRobot.mArm.setExtetion(0.3);
//            sleep(750);
//
//            mRobot.mArm.setClawPosition(closedClaw);
//            sleep(500);
//
//            mRobot.mArm.setExtetion(minExtention);
//            sleep(750);
//
//
//            // Move
//            mRobot.mDrive.turnRight(0.5);
//            sleep(600);
//            mRobot.mDrive.brake();
//
//            mRobot.mDrive.backward(0.5);
//            sleep(1200);
//            mRobot.mDrive.brake();
//
//
//            //Score
//            mRobot.mArm.setArmRotation(0.5);
//            sleep(800);
//            mRobot.mArm.setArmRotation(0);
//
//            mRobot.mArm.setExtetion(maxExtension);
//            sleep(750);
//
//            mRobot.mArm.setClawPosition(openClaw);
//            sleep(400);
//
//            mRobot.mArm.setExtetion(minExtention);
//            sleep(750);
//
//            mRobot.mArm.setArmRotation(-0.5);
//            sleep(800);
//            mRobot.mArm.setArmRotation(0);
//
//
//
//            //Fourth Sample // Third Cycle
//            //Move
//            mRobot.mDrive.forward(0.5);
//            sleep(1200);
//            mRobot.mDrive.brake();
//
//            mRobot.mDrive.turnRight(-0.5);
//            sleep(600);
//            mRobot.mDrive.brake();
//
//
//            //Intake
//            mRobot.mArm.setExtetion(maxExtension);
//            sleep(750);
//
//            mRobot.mArm.setClawPosition(closedClaw);
//            sleep(500);
//
//            mRobot.mArm.setExtetion(minExtention);
//            sleep(750);
//
//
//            // Move
//            mRobot.mDrive.turnRight(0.5);
//            sleep(600);
//            mRobot.mDrive.brake();
//
//            mRobot.mDrive.backward(0.5);
//            sleep(1200);
//            mRobot.mDrive.brake();
//
//
//            //Score
//            mRobot.mArm.setArmRotation(0.5);
//            sleep(800);
//            mRobot.mArm.setArmRotation(0);
//
//            mRobot.mArm.setExtetion(maxExtension);
//            sleep(750);
//
//            mRobot.mArm.setClawPosition(openClaw);
//            sleep(400);
//
//            mRobot.mArm.setExtetion(minExtention);
//            sleep(750);
//
//            mRobot.mArm.setArmRotation(-0.5);
//            sleep(800);
//            mRobot.mArm.setArmRotation(0);
//






        }


//
    }
}