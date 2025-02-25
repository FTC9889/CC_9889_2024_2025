package com.team9889.ftc2024.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.team9889.ftc2024.subsystems.Robot;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
@Config
public class TeleOpTest extends LinearOpMode {

    public static double intakeLockPosition  = 0.5;
    public static double intkaeWristPosition  = 0.5;
    public static double intakePower  = 1;
    public static double intakeExtensionPower = 1;

    public static double clutchPositon = 0.5;


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

        while (opModeIsActive()) {

            if (gamepad1.a) {
                mRobot.mIntake.setIntakeLockPosition(intakeLockPosition);
            }

            if (gamepad1.b) {
                mRobot.mIntake.setIntakeWristPosition(intkaeWristPosition);
            }

            if (gamepad1.x) {
                mRobot.mIntake.setIntakePower(intakePower);
            } else {
                mRobot.mIntake.setIntakePower(0);
            }

            if (gamepad1.y) {
                mRobot.mIntake.setExtensionPower(intakeExtensionPower);
            } else {
                mRobot.mIntake.setExtensionPower(0);
            }


            if (gamepad1.left_bumper) {
                mRobot.mLift.setClawPosition(clawPosition);
            }

            if (gamepad1.right_bumper) {
                mRobot.mLift.setLiftMotorPower(liftPower);
            } else {
                mRobot.mLift.setLiftMotorPower(0);
            }

            if (gamepad1.dpad_down) {
                mRobot.mLift.setElbowPosition(liftElbowPosition);
            }

            if (gamepad1.dpad_right) {
                mRobot.mLift.setWristPosition(liftWristPosition);
            }

            if (gamepad2.left_stick_button){
                mRobot.mLift.setClutchPosition(clutchPositon);
            }


            telemetry.addData("CurrentLiftPosition", mRobot.mLift.currentLiftPosition());
            telemetry.addData("CurrentIntakePosition", mRobot.mIntake.extension.getCurrentPosition());
            telemetry.addData("IntakeMagnetState", mRobot.mIntake.magnetSensor.getState());

            telemetry.addData("LiftMagnetState", mRobot.mLift.liftMagnetSensor.getState());
            telemetry.addData("ColorSensorDistance", mRobot.mIntake.colorSensor.getDistance(DistanceUnit.INCH));

            String text = "";
            if (mRobot.mIntake.colorSensor.getDistance(DistanceUnit.INCH) < 1.5){
                double red = mRobot.mIntake.colorSensor.red();
                double green = mRobot.mIntake.colorSensor.green();
                double blue = mRobot.mIntake.colorSensor.blue();

                double total = red + green + blue;

                red /= total;
                green /= total;
                blue /= total;

                red *= 10;
                green *= 10;
                blue *= 10;

                if (red > blue && red > green) {
                    text = "red";
                } else if (blue > red && blue > green) {
                    text = "blue";
                } else {
                    text = "yellow";
                }
            } else {
                text = "nothing";
            }

//            telemetry.addData("ColorSensorColorBlue", blue);
//            telemetry.addData("ColorSensorColorRed", red);
//            telemetry.addData("ColorSensorColorGreen", green);
//            telemetry.addData("Average of Red and Green", (red + green)/2);
            telemetry.addData("Current Color", text);
            telemetry.update();



            if (gamepad1.dpad_left){
                mRobot.mFlag.setFlagPosition(flagPosition);
            }


        }

    }
}
