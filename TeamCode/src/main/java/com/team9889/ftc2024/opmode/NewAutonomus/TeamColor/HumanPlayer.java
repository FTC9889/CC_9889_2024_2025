package com.team9889.ftc2024.opmode.NewAutonomus.TeamColor;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.team9889.ftc2024.subsystems.Robot;
@Autonomous
public class HumanPlayer extends LinearOpMode {

    Robot mRobot = new Robot();
    ElapsedTime autoTimer = new ElapsedTime();

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


        mRobot.mArm.setClawPosition(closedClaw);

        mRobot.mArm.setExtetion(minExtention);

        mRobot.mArm.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mRobot.mArm.arm.setTargetPosition(700);
        mRobot.mArm.arm.setPower(0.2);
        mRobot.mArm.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        waitForStart();

        if (opModeIsActive()) {


            autoTimer.reset();
            while (opModeIsActive() && autoTimer.milliseconds() < 1850) {
                double AnglePower = -1.3 * mRobot.mDrive.imu.getNormalHeading() / 180;
                mRobot.mDrive.setPower(0, 0.5, AnglePower);
                telemetry.addData("AnglePower", AnglePower);
                telemetry.update();
            }
            mRobot.mDrive.brake();

            autoTimer.reset();
            while (opModeIsActive() && autoTimer.milliseconds() < 3300) {
                double AnglePower = -1.3 * mRobot.mDrive.imu.getNormalHeading() / 180;
                mRobot.mDrive.setPower(0.5, 0, AnglePower);
                telemetry.addData("AnglePower", AnglePower);
                telemetry.update();
            }
            mRobot.mDrive.brake();
            sleep(8000);

            autoTimer.reset();
            while (opModeIsActive() && Math.abs(mRobot.mDrive.imu.getNormalHeading() - 52) > 5) {
                double AnglePower = -3 * (mRobot.mDrive.imu.getNormalHeading() - 52) /180;
                mRobot.mDrive.setPower(0, 0, AnglePower);
                telemetry.addData("AnglePower", AnglePower);
                telemetry.update();
            }

            mRobot.mArm.arm.setTargetPosition(850);
            mRobot.mArm.arm.setPower(1);
            mRobot.mArm.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(500);

            mRobot.mArm.setExtetion(maxExtension);
            sleep(1000);

            autoTimer.reset();
            while (opModeIsActive() && autoTimer.milliseconds() < 900) {
                double AnglePower = -1.3 * (mRobot.mDrive.imu.getNormalHeading() - 52) / 180;
                mRobot.mDrive.setPower(0.5, 0, AnglePower);
                telemetry.addData("AnglePower", AnglePower);
                telemetry.update();
            }
            mRobot.mDrive.brake();

            mRobot.mArm.setClawPosition(openClaw);
            sleep(2000);

            mRobot.mArm.setRotation(1);
            sleep(1000);
            mRobot.mArm.setRotation(0.35);

            autoTimer.reset();
            while (opModeIsActive() && autoTimer.milliseconds() < 900) {
                double AnglePower = -1.3 * (mRobot.mDrive.imu.getNormalHeading() - 52) / 180;
                mRobot.mDrive.setPower(-0.5, 0, AnglePower);
                telemetry.addData("AnglePower", AnglePower);
                telemetry.update();
            }
            mRobot.mDrive.brake();

            mRobot.mArm.setExtetion(minExtention);
            sleep(2000);

            mRobot.mArm.arm.setTargetPosition(50);
            mRobot.mArm.arm.setPower(1);
            mRobot.mArm.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            autoTimer.reset();
            while (opModeIsActive() && Math.abs(mRobot.mDrive.imu.getNormalHeading()) > 5) {
                double AnglePower = -3 * (mRobot.mDrive.imu.getNormalHeading()) / 180;
                mRobot.mDrive.setPower(0, 0, AnglePower);
                telemetry.addData("AnglePower", AnglePower);
                telemetry.update();
            }

            autoTimer.reset();
            while (opModeIsActive() && autoTimer.milliseconds() < 3300) {
                double AnglePower = -1.3 * mRobot.mDrive.imu.getNormalHeading() / 180;
                mRobot.mDrive.setPower(-0.5, 0, AnglePower);
                telemetry.addData("AnglePower", AnglePower);
                telemetry.update();
            }
            mRobot.mDrive.brake();

            autoTimer.reset();
            while (opModeIsActive() && autoTimer.milliseconds() < 1450) {
                double AnglePower = -1.3 * mRobot.mDrive.imu.getNormalHeading() / 180;
                mRobot.mDrive.setPower(0, -0.5, AnglePower);
                telemetry.addData("AnglePower", AnglePower);
                telemetry.update();
            }
            mRobot.mDrive.brake();

            autoTimer.reset();
            while (opModeIsActive() && autoTimer.milliseconds() < 1100) {
                double AnglePower = -1.3 * mRobot.mDrive.imu.getNormalHeading() / 180;
                mRobot.mDrive.setPower(-0.5, 0, AnglePower);
                telemetry.addData("AnglePower", AnglePower);
                telemetry.update();
            }
            mRobot.mDrive.brake();
        }

    }
}