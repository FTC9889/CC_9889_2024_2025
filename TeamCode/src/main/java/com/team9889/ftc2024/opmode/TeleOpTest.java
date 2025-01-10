package com.team9889.ftc2024.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.team9889.ftc2024.subsystems.Robot;

@TeleOp
@Config
public class TeleOpTest extends LinearOpMode {

    public static double intakeLockPosition  = 0.5;
    public static double intkaeWristPosition  = 0.5;
    public static double intakePower  = 1;
    public static double intakeExtensionPower = 1;


    public static double clawPosition = 0.5;
    public static double liftPower = 1;
    public static double liftElbowPosition = 0.5;
    public static double liftWristPosition = 0.5;

    public static double flagPosition = 0.5;

    Robot mRobot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {

        mRobot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()){

            if (gamepad1.a){
                mRobot.mIntake.setIntakeLockPosition(intakeLockPosition);
            }

            if (gamepad1.b){
                mRobot.mIntake.setIntakeWristPosition(intkaeWristPosition);
            }

            if (gamepad1.x){
                mRobot.mIntake.setIntakePower(intakePower);
            }else {
                mRobot.mIntake.setIntakePower(0);
            }

            if (gamepad1.y){
                mRobot.mIntake.setExtensionPower(intakeExtensionPower);
            } else {
                mRobot.mIntake.setExtensionPower(0);
            }



            if (gamepad1.left_bumper){
                mRobot.mLift.setClawPosition(clawPosition);
            }

            if (gamepad1.right_bumper){
                mRobot.mLift.setLiftMotorPower(liftPower);
            }else {
                mRobot.mLift.setLiftMotorPower(0);
            }

            if (gamepad1.dpad_down){
                mRobot.mLift.setElbowPosition(liftElbowPosition);
            }

            if (gamepad1.dpad_right){
                mRobot.mLift.setWristPosition(liftWristPosition);
            }


            telemetry.addData("CurrentLiftPosition", mRobot.mLift.currentLiftPosition());
            telemetry.addData("CurrentIntakePosition", mRobot.mIntake.extension.getCurrentPosition());
            telemetry.addData("IntakeMagnetState", mRobot.mIntake.magnetSensor.getState());
            telemetry.addData("LiftMagnetState", mRobot.mLift.liftMagnetSensor.getState());
            telemetry.update();



            if (gamepad1.dpad_left){
                mRobot.mFlag.setFlagPosition(flagPosition);
            }


        }

    }
}
