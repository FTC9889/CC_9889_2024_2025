package com.team9889.ftc2024.opmode.NewAutonomus.Yellow;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.team9889.ftc2024.subsystems.Robot;

@Autonomous(preselectTeleOp = "TeleOperate")
public class BasketSide extends LinearOpMode {

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
        double maxExtension =  0.575;
        double minExtention = 0.024;


        mRobot.mArm.setClawPosition(closedClaw);

        mRobot.mArm.extend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        mRobot.mArm.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mRobot.mArm.arm.setTargetPosition(650);
        mRobot.mArm.arm.setPower(0.2);
        mRobot.mArm.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        mRobot.mArm.extend.setTargetPosition(0);
        mRobot.mArm.extend.setPower(1);
        mRobot.mArm.extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(1000);

        waitForStart();

//        mRobot.mDrive.setPower(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x);
//

        if (opModeIsActive()) {


            // first sample
            //driving
            mRobot.mArm.arm.setTargetPosition(975);
            mRobot.mArm.arm.setPower(1);
            mRobot.mArm.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            sleep(500);

            mRobot.mArm.extend.setTargetPosition(1250);
            mRobot.mArm.extend.setPower(1);
            mRobot.mArm.extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(500);

            mRobot.mDrive.reset_encoder();
            autoTimer.reset();
            while (opModeIsActive() && mRobot.mDrive.front_encoder() < 330) {
                double AnglePower = -1.3 * mRobot.mDrive.imu.getNormalHeading() / 180;
                mRobot.mDrive.setPower(0.5, 0, AnglePower);
                telemetry.addData("AnglePower", AnglePower);
                telemetry.addData("Encoder", mRobot.mDrive.front_encoder());
                telemetry.update();
            }
            mRobot.mDrive.brake();

            mRobot.mDrive.reset_encoder();

            autoTimer.reset();
            while (opModeIsActive() && Math.abs(mRobot.mDrive.imu.getNormalHeading() - 20) > 5) {
                double AnglePower = -5 * (mRobot.mDrive.imu.getNormalHeading() - 20) / 180;
                mRobot.mDrive.setPower(0, 0, AnglePower);
                telemetry.addData("AnglePower", mRobot.mDrive.imu.getNormalHeading());
                telemetry.update();
            }
            mRobot.mDrive.brake();


            //scoring

            mRobot.mArm.setClawPosition(openClaw);
            sleep(500);

            mRobot.mArm.setRotation(0.33);
            sleep(250);


            // second sample
            //driving
            mRobot.mDrive.reset_encoder();
            autoTimer.reset();
            while (opModeIsActive() && Math.abs(mRobot.mDrive.imu.getNormalHeading()) > 5) {
                double AnglePower = -5 * (mRobot.mDrive.imu.getNormalHeading()) / 180;
                mRobot.mDrive.setPower(0, 0, AnglePower);
                telemetry.addData("AnglePower", AnglePower);
                telemetry.update();
            }
            mRobot.mDrive.brake();
            mRobot.mDrive.reset_encoder();

            autoTimer.reset();
            while (opModeIsActive() && mRobot.mDrive.front_encoder() > -600) {
                double AnglePower = -1.3 * mRobot.mDrive.imu.getNormalHeading() / 180;
                mRobot.mDrive.setPower(-0.5, 0, AnglePower);
                telemetry.addData("AnglePower", AnglePower);
                telemetry.update();
            }
            mRobot.mDrive.brake();

            mRobot.mArm.setRotation(0);


            //reset arm
            mRobot.mArm.extend.setTargetPosition(0);
            mRobot.mArm.extend.setPower(1);
            mRobot.mArm.extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(500);

            mRobot.mArm.arm.setTargetPosition(0);
            mRobot.mArm.arm.setPower(1);
            mRobot.mArm.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(500);


            //driving
            mRobot.mDrive.reset_encoder();
            autoTimer.reset();
            while (opModeIsActive() && mRobot.mDrive.front_encoder() > -1700) {
                double AnglePower = -1.3 * mRobot.mDrive.imu.getNormalHeading() / 180;
                mRobot.mDrive.setPower(0, 0.5, AnglePower);
                telemetry.addData("AnglePower", AnglePower);
                telemetry.addData("Encoder", mRobot.mDrive.front_encoder());
                telemetry.update();
            }
            mRobot.mDrive.brake();

            while (opModeIsActive() && Math.abs(mRobot.mDrive.front_encoder()) > 3)
                mRobot.mDrive.reset_encoder();

            autoTimer.reset();
            while (opModeIsActive() && mRobot.mDrive.front_encoder() < 150) {
                double AnglePower = -1.3 * mRobot.mDrive.imu.getNormalHeading() / 180;
                mRobot.mDrive.setPower(0.5, 0, AnglePower);
                telemetry.addData("AnglePower", AnglePower);
                telemetry.addData("Encoder", mRobot.mDrive.front_encoder());
                telemetry.update();
            }
            mRobot.mDrive.brake();


            //intaking
            mRobot.mArm.setClawPosition(closedClaw);
            sleep(1000);

            mRobot.mArm.arm.setTargetPosition(975);
            mRobot.mArm.arm.setPower(0.5);
            mRobot.mArm.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            //driving

            mRobot.mDrive.reset_encoder();
            autoTimer.reset();
            while (opModeIsActive() && Math.abs(mRobot.mDrive.imu.getNormalHeading() - 52) > 5) {
                double AnglePower = -5 * (mRobot.mDrive.imu.getNormalHeading() - 52) / 180;
                mRobot.mDrive.setPower(0, 0, AnglePower);
                telemetry.addData("AnglePower", mRobot.mDrive.imu.getNormalHeading());
                telemetry.update();
            }
            mRobot.mDrive.brake();
            sleep(500);

            mRobot.mDrive.reset_encoder();
            autoTimer.reset();
            while (opModeIsActive() && mRobot.mDrive.front_encoder() < 1275) {
                double AnglePower = -5 * (mRobot.mDrive.imu.getNormalHeading() - 52) / 180;
                mRobot.mDrive.setPower(0.5, 0, AnglePower);
                telemetry.addData("AnglePower", AnglePower);
                telemetry.addData("Encoder", mRobot.mDrive.front_encoder());
                telemetry.update();
            }
            mRobot.mDrive.brake();


            //scoring
            mRobot.mArm.extend.setTargetPosition(1250);
            mRobot.mArm.extend.setPower(1);
            mRobot.mArm.extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(750);

            mRobot.mArm.setClawPosition(openClaw);
            sleep(500);

            mRobot.mArm.setRotation(0.33);
            sleep(250);









            //thrid sample
            //driving
            mRobot.mDrive.reset_encoder();

            while (opModeIsActive() && mRobot.mDrive.front_encoder() > -1300) {
                double AnglePower = -5 * (mRobot.mDrive.imu.getNormalHeading() - 52) / 180;
                mRobot.mDrive.setPower(-0.5, 0, AnglePower);
                telemetry.addData("AnglePower", AnglePower);
                telemetry.addData("Encoder", mRobot.mDrive.front_encoder());
                telemetry.update();
            }
            mRobot.mDrive.brake();


            //reset arm
            mRobot.mArm.extend.setTargetPosition(0);
            mRobot.mArm.extend.setPower(1);
            mRobot.mArm.extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(500);


            mRobot.mArm.arm.setTargetPosition(110);
            mRobot.mArm.arm.setPower(1);
            mRobot.mArm.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(500);

            mRobot.mArm.setRotation(0);


            mRobot.mDrive.reset_encoder();

            autoTimer.reset();
            while (opModeIsActive() && Math.abs(mRobot.mDrive.imu.getNormalHeading()) > 5) {
                double AnglePower = -3 * (mRobot.mDrive.imu.getNormalHeading()) / 180;
                mRobot.mDrive.setPower(0, 0, AnglePower);
                telemetry.addData("AnglePower", AnglePower);
                telemetry.update();
            }
            mRobot.mDrive.brake();


            //intaking
            mRobot.mArm.extend.setTargetPosition(360);
            mRobot.mArm.extend.setPower(0.5);
            mRobot.mArm.extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(500);


            mRobot.mArm.setClawPosition(closedClaw);
            sleep(500);

            mRobot.mArm.extend.setTargetPosition(0);
            mRobot.mArm.extend.setPower(1);
            mRobot.mArm.extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(500);

            mRobot.mArm.arm.setTargetPosition(975);
            mRobot.mArm.arm.setPower(1);
            mRobot.mArm.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);


            //driving
            mRobot.mDrive.reset_encoder();
            autoTimer.reset();
            while (opModeIsActive() && Math.abs(mRobot.mDrive.imu.getNormalHeading() - 52) > 5) {
                double AnglePower = -5 * (mRobot.mDrive.imu.getNormalHeading() - 52) / 180;
                mRobot.mDrive.setPower(0, 0, AnglePower);
                telemetry.addData("AnglePower", mRobot.mDrive.imu.getNormalHeading());
                telemetry.update();
            }
            mRobot.mDrive.brake();
            sleep(500);

            mRobot.mDrive.reset_encoder();
            autoTimer.reset();
            while (opModeIsActive() && mRobot.mDrive.front_encoder() < 1275) {
                double AnglePower = -5 * (mRobot.mDrive.imu.getNormalHeading() - 52) / 180;
                mRobot.mDrive.setPower(0.5, 0, AnglePower);
                telemetry.addData("AnglePower", AnglePower);
                telemetry.addData("Encoder", mRobot.mDrive.front_encoder());
                telemetry.update();
            }
            mRobot.mDrive.brake();


            //scoring
            mRobot.mArm.extend.setTargetPosition(1250);
            mRobot.mArm.extend.setPower(1);
            mRobot.mArm.extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(750);


            mRobot.mArm.setClawPosition(openClaw);
            sleep(500);

            mRobot.mArm.setRotation(0.33);

            mRobot.mDrive.reset_encoder();


            //4th sample
            //driving
            autoTimer.reset();
            while (opModeIsActive() && mRobot.mDrive.front_encoder() > -375) {
                double AnglePower = -5 * (mRobot.mDrive.imu.getNormalHeading() - 52) / 180;
                mRobot.mDrive.setPower(-0.5, 0, AnglePower);
                telemetry.addData("AnglePower", AnglePower);
                telemetry.addData("Encoder", mRobot.mDrive.front_encoder());
                telemetry.update();
            }
            mRobot.mDrive.brake();

            mRobot.mArm.setRotation(0);

            //reset arm
            mRobot.mArm.extend.setTargetPosition(0);
            mRobot.mArm.extend.setPower(1);
            mRobot.mArm.extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(500);

            mRobot.mArm.arm.setTargetPosition(75);
            mRobot.mArm.arm.setPower(1);
            mRobot.mArm.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(500);


            //turn
            mRobot.mDrive.reset_encoder();
            autoTimer.reset();
            while (opModeIsActive() && Math.abs(mRobot.mDrive.imu.getNormalHeading() + 47.5) > 5) {
                double AnglePower = -2 * (mRobot.mDrive.imu.getNormalHeading() + 47.5) / 180;
                mRobot.mDrive.setPower(0, 0, AnglePower);
                telemetry.addData("AnglePower", mRobot.mDrive.imu.getNormalHeading());
                telemetry.update();
            }
            mRobot.mDrive.brake();

            sleep(250);


            //intake


            mRobot.mArm.extend.setTargetPosition(555);
            mRobot.mArm.extend.setPower(0.5);
            mRobot.mArm.extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(250);

            mRobot.mArm.setClawPosition(closedClaw);
            sleep(1000);

            mRobot.mArm.extend.setTargetPosition(0);
            mRobot.mArm.extend.setPower(0.5);
            mRobot.mArm.extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            sleep(250);


            mRobot.mDrive.reset_encoder();
            autoTimer.reset();
            while (opModeIsActive() && Math.abs(mRobot.mDrive.imu.getNormalHeading() - 52) > 5) {
                double AnglePower = -3 * (mRobot.mDrive.imu.getNormalHeading() - 52) / 180;
                mRobot.mDrive.setPower(0, 0, AnglePower);
                telemetry.addData("AnglePower", mRobot.mDrive.imu.getNormalHeading());
                telemetry.update();
            }
            mRobot.mDrive.brake();

            mRobot.mDrive.reset_encoder();

            autoTimer.reset();
            while (opModeIsActive() && mRobot.mDrive.front_encoder() < 375) {
                double AnglePower = -3 * (mRobot.mDrive.imu.getNormalHeading() - 52) / 180;
                mRobot.mDrive.setPower(0.5, 0, AnglePower);
                telemetry.addData("AnglePower", AnglePower);
                telemetry.addData("Encoder", mRobot.mDrive.front_encoder());
                telemetry.update();
            }
            mRobot.mDrive.brake();


            mRobot.mArm.arm.setTargetPosition(975);
            mRobot.mArm.arm.setPower(1);
            mRobot.mArm.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            sleep(500);

            mRobot.mArm.extend.setTargetPosition(1300);
            mRobot.mArm.extend.setPower(1);
            mRobot.mArm.extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            sleep(500);


            mRobot.mArm.setClawPosition(openClaw);
            sleep(500);

            autoTimer.reset();
            while (opModeIsActive() && mRobot.mDrive.front_encoder() > -200) {
                double AnglePower = -3 * (mRobot.mDrive.imu.getNormalHeading() - 52) / 180;
                mRobot.mDrive.setPower(-0.5, 0, AnglePower);
                telemetry.addData("AnglePower", AnglePower);
                telemetry.addData("Encoder", mRobot.mDrive.front_encoder());
                telemetry.update();
            }
            mRobot.mDrive.brake();

            mRobot.mArm.extend.setTargetPosition(0);
            mRobot.mArm.extend.setPower(1);
            mRobot.mArm.extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            mRobot.mArm.arm.setTargetPosition(0);
            mRobot.mArm.arm.setPower(1);
            mRobot.mArm.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);





        }

    }
}
