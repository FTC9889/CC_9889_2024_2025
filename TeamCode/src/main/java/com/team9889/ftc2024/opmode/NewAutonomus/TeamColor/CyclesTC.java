package com.team9889.ftc2024.opmode.NewAutonomus.TeamColor;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.team9889.ftc2024.subsystems.Robot;

@Autonomous
public class CyclesTC extends LinearOpMode {// STOPSHIP: 10/28/2023 }
    Robot mRobot = new Robot();

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


        while (opModeIsActive()){

            //FIrst Sample
            //Drop Off
            mRobot.mArm.setExtetion(maxExtension);
            sleep(1000);

            mRobot.mArm.setClawPosition(openClaw);
            sleep(1000);

            mRobot.mArm.setExtetion(minExtention);
            sleep(1000);

            //Move
            mRobot.mDrive.turnLeft(0.5);
            sleep(turn);
            mRobot.mDrive.brake();

            mRobot.mDrive.forward(0.5);
            sleep(1200);
            mRobot.mDrive.brake();

            mRobot.mDrive.turnRight(0.5);
            sleep(turn);
            mRobot.mDrive.brake();

            //Intake
            mRobot.mArm.setExtetion(0.2);
            sleep(500);

            mRobot.mArm.setClawPosition(closedClaw);
            sleep(500);

            mRobot.mArm.setExtetion(minExtention);
            sleep(500);

            //Move
            mRobot.mDrive.turnLeft(0.5);
            sleep(turn - 300);
            mRobot.mDrive.brake();

            mRobot.mDrive.forward(0.5);
            sleep(500);

            //Drop Off
            mRobot.mArm.setExtetion(maxExtension);
            sleep(500);

            mRobot.mArm.setClawPosition(openClaw);
            sleep(500);







            //Second Sample //First Cycle
            mRobot.mDrive.backward(0.5);
            sleep(500);

            mRobot.mDrive.turnRight(0.5);
            sleep(turn - 300);
            mRobot.mDrive.brake();

            //Intake
            mRobot.mArm.setExtetion(0.35);
            sleep(500);

            mRobot.mArm.setClawPosition(closedClaw);
            sleep(500);

            mRobot.mArm.setExtetion(minExtention);
            sleep(500);

            //Move
            mRobot.mDrive.turnLeft(0.5);
            sleep(turn - 300);
            mRobot.mDrive.brake();

            mRobot.mDrive.forward(0.5);
            sleep(500);

            //Drop Off
            mRobot.mArm.setExtetion(maxExtension);
            sleep(500);

            mRobot.mArm.setClawPosition(openClaw);
            sleep(500);

            // Third Sample //Second Cycle
            mRobot.mDrive.backward(0.5);
            sleep(500);

            mRobot.mDrive.turnRight(0.5);
            sleep(turn - 300);
            mRobot.mDrive.brake();

            //Intake
            mRobot.mArm.setExtetion(0.45);
            sleep(500);

            mRobot.mArm.setClawPosition(closedClaw);
            sleep(500);

            mRobot.mArm.setExtetion(minExtention);
            sleep(500);

            //Move
            mRobot.mDrive.turnLeft(0.5);
            sleep(turn - 300);
            mRobot.mDrive.brake();

            mRobot.mDrive.forward(0.5);
            sleep(500);

            //Drop Off
            mRobot.mArm.setExtetion(maxExtension);
            sleep(500);

            mRobot.mArm.setClawPosition(openClaw);
            sleep(500);







            //Third Sample //Second Cycle
            mRobot.mDrive.backward(0.5);
            sleep(500);

            mRobot.mDrive.turnRight(0.5);
            sleep(turn - 300);
            mRobot.mDrive.brake();

            //Intake
            mRobot.mArm.setExtetion(0.35);
            sleep(500);

            mRobot.mArm.setClawPosition(closedClaw);
            sleep(500);

            mRobot.mArm.setExtetion(minExtention);
            sleep(500);

            //Move
            mRobot.mDrive.turnLeft(0.5);
            sleep(turn - 300);
            mRobot.mDrive.brake();

            mRobot.mDrive.forward(0.5);
            sleep(500);

            //Drop Off
            mRobot.mArm.setExtetion(maxExtension);
            sleep(500);

            mRobot.mArm.setClawPosition(openClaw);
            sleep(500);






            //Fourth Sample //Third Cycle
            mRobot.mDrive.backward(0.5);
            sleep(500);

            mRobot.mDrive.turnRight(0.5);
            sleep(turn - 300);
            mRobot.mDrive.brake();

            //Intake
            mRobot.mArm.setExtetion(maxExtension);
            sleep(1000);

            mRobot.mArm.setClawPosition(closedClaw);
            sleep(500);

            mRobot.mArm.setExtetion(minExtention);
            sleep(500);

            //Move
            mRobot.mDrive.turnLeft(0.5);
            sleep(turn - 300);
            mRobot.mDrive.brake();

            mRobot.mDrive.forward(0.5);
            sleep(500);

            //Drop Off
            mRobot.mArm.setExtetion(maxExtension);
            sleep(500);

            mRobot.mArm.setClawPosition(openClaw);
            sleep(500);





        }

    }
}