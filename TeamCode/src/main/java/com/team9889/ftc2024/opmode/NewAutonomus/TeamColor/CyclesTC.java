package com.team9889.ftc2024.opmode.NewAutonomus.TeamColor;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.team9889.ftc2024.subsystems.Robot;
@Disabled
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

//
        }

    }
}