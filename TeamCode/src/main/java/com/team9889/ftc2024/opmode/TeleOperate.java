package com.team9889.ftc2024.opmode;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.team9889.ftc2024.subsystems.Intake;
import com.team9889.ftc2024.subsystems.Lift;
import com.team9889.ftc2024.subsystems.Robot;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class TeleOperate extends LinearOpMode{
    Robot mRobot = new Robot();
    private List<Action> requestedActions = new ArrayList<>();

    @Override
    public void runOpMode() throws InterruptedException {
        mRobot.init(hardwareMap);

        waitForStart();

        requestedActions.add(mRobot.mLift.LiftController());

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();

            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            // Send calculated power to wheels

            mRobot.mDrive.setDrivePowers(new PoseVelocity2d(new Vector2d(gamepad1.left_stick_x, -gamepad1.left_stick_y), gamepad1.right_stick_x));

            if (gamepad2.a){
                requestedActions.add(mRobot.mLift.HighRung());
            }

            if (mRobot.mIntake.allowDriverExtension()){
                mRobot.mIntake.setExtensionPower(-gamepad2.left_stick_y);
            }

            if (gamepad1.a) {
                requestedActions.add(mRobot.mIntake.Retracted());
            }

            List<Action> newActions = new ArrayList<>();
            for (Action action : requestedActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            requestedActions = newActions;



//            if (gamepad1.a){
//                //are there encoders on intake extension motors
//                mRobot.mIntake.extension.setTargetPosition();
//                mRobot.mIntake.extension.setPower(0.5);
//                mRobot.mIntake.extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                mRobot.mIntake.setWristPosision();
//            }

            //gamepad1.a = intake slight extend && intake wrist down && power intake          while ( until detect color or if gamepad1.y  is clicked      if color is opponent eject      if color is yellow or alliance color reset robot and transfer) { gamepad1 can change wrist adn intake  slide position with gamepad1.triggrs and wrist with gamepad1.bumpers
            //gamepad1.x = claw
            //gamepad1.triggers = intake slides
            //gamepad1.y = reset robot


            // gamepad2.y = reset robot
            // gamepad2.triggers = lift slides





        }
    }
}