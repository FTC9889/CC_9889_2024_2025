//package com.team9889.ftc2024.test;
//
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.team9889.ftc2024.camera.TeamPropDetector;
//
//@TeleOp
//@Disabled
//public class testsomeeteampropdetector extends LinearOpMode {
//    @Override
//    public void runOpMode() throws InterruptedException {
//        TeamPropDetector teamPropDetector = new TeamPropDetector(true);
//
//        teamPropDetector.initTfod(hardwareMap);
//        while(opModeInInit()) {
//            teamPropDetector.telemetryTfod(telemetry);
//            telemetry.update();
//        }
//
//        waitForStart();
//
//        while (opModeIsActive()){
//            teamPropDetector.telemetryTfod(telemetry);
//            telemetry.update();
//        }
//
//
//    }
//}
