package com.team9889.ftc2024.opmode;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.team9889.ftc2024.subsystems.Robot;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class Inspection_TeleOp extends LinearOpMode {

    Robot mRobot = new Robot();
    @Override
    public void runOpMode() throws InterruptedException {

        mRobot.init(hardwareMap);

        waitForStart();

        Action intakeAction = null;
        Action liftAction = null;

        TelemetryPacket packet = new TelemetryPacket();

        while (opModeIsActive()){

//            if (gamepad1.a){
//                intakeAction = mRobot.mIntake.InspectionExtend();
//                liftAction = mRobot.mLift.ScorePrepare();
//            }
//
//            if (gamepad1.b) {
//                intakeAction = mRobot.mIntake.Retracted();
//                liftAction = mRobot.mLift.TransferPrepare();
//            }

            if (intakeAction != null) {
                intakeAction.preview(packet.fieldOverlay());
                if(!intakeAction.run(packet)) {
                    intakeAction = null;
                }
            }

            if (liftAction != null) {
                liftAction.preview(packet.fieldOverlay());
                if(!liftAction.run(packet)) {
                    liftAction = null;
                }
            }
        }


    }
}
