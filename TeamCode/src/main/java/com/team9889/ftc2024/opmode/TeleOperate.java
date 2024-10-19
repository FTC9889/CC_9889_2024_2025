package com.team9889.ftc2024.opmode;
//import static com.team9889.ftc2024.opmode.TeleOp.IntakeState.INTAKE;
//import static com.team9889.ftc2024.opmode.TeleOp.IntakeState.OUTTAKE;
//import static com.team9889.ftc2024.opmode.TeleOp.IntakeState.RETRACTED;
//import static com.team9889.ftc2024.opmode.TeleOp.IntakeState.SLIGHT_EXTEND;
//import static com.team9889.ftc2024.opmode.TeleOp.IntakeState.TRANSFER_FIRST_POSITION;
//import static com.team9889.ftc2024.opmode.TeleOp.IntakeState.TRANSFER_SECOND_POSITION;
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

//import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.team9889.ftc2024.subsystems.Robot;


@TeleOp
public class TeleOperate extends LinearOpMode{
    TouchSensor touchSensor;
    Robot mRobot = new Robot();
    ElapsedTime elapseTimer = new ElapsedTime();
    boolean press = false;
    boolean rotationPress = false;


    double servoPosition = 0.024;
    @Override
    public void runOpMode() throws InterruptedException {
        touchSensor = hardwareMap.get(TouchSensor.class, "armSensor");

        mRobot.init(hardwareMap);

        int newTarget = mRobot.mArm.arm.getCurrentPosition();

        waitForStart();

        while (opModeIsActive()) {

            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            // Calculate power for each motor
            double leftPower = drive + turn;
            double rightPower = drive - turn;

            // Send calculated power to wheels
            mRobot.mDrive.setPower(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x / 0.66);


            if (gamepad2.left_bumper){
                mRobot.mArm.setRotation(0);
            }
            if (gamepad2.right_bumper){
                mRobot.mArm.setRotation(33);
            }

//            if (gamepad1.right_trigger > gamepad1.left_trigger) {
//                mRobot.mArm.setArmRotation(0.5*gamepad1.right_trigger);
//            } else if (gamepad1.right_trigger <= gamepad1.left_trigger) {
//                if (touchSensor.isPressed() == true) {
//                    mRobot.mArm.setArmRotation(0);
//                }
//                else {
//                    mRobot.mArm.setArmRotation(-0.5*gamepad1.left_trigger);
//                }
//            }


            if(gamepad1.right_trigger > 0.5) {
                mRobot.mArm.arm.setPower(1);
                newTarget += 5;
            } else if(gamepad1.left_trigger > 0.5) {
                if (touchSensor.isPressed() == true) {
                    mRobot.mArm.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    newTarget = 0;
                } else {
                    mRobot.mArm.arm.setPower(1);
                    newTarget -= 5;
                }
            }

            newTarget = Math.min(newTarget, 938);

            //938



            mRobot.mArm.arm.setTargetPosition(newTarget);
            mRobot.mArm.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // max 0.575
            // min 0.024

            if (gamepad1.dpad_up) {
                servoPosition += 0.006;
            }else if (gamepad1.dpad_down) {
                servoPosition -= 0.006;
            }

            servoPosition = Math.max(0.024, Math.min(servoPosition, 0.575));

            mRobot.mArm.setExtetion(servoPosition);

            if (gamepad1.y){
                servoPosition += 0.01;
            }

            if (gamepad1.x){
                mRobot.mArm.setRotation(0.35);
            }






            if (gamepad1.left_bumper) {
                mRobot.mArm.setClawPosition(0.67);
            }
            if (gamepad1.right_bumper) {
                mRobot.mArm.setClawPosition(0.36);
            }

//            if (gamepad1.left_bumper){
//                mRobot.mArm.setIntake3Power(1);
//            } else if (gamepad1.right_bumper) {
//                mRobot.mArm.setIntake3Power(-1);
//            }else {
//                mRobot.mArm.setIntake3Power(0);
//            }


            if (gamepad2.dpad_up) {
                mRobot.mHanger.setHangPower(1);
            }
            else if (gamepad2.dpad_down){
                mRobot.mHanger.setHangPower(-1);
            }else{
                mRobot.mHanger.setHangPower(0);
            }


//            if (gamepad2.x) {
//                mRobot.mArm.setIntake2Power(1);
//            }else {
//                mRobot.mArm.setIntake2Power(0);
//            }

            if (gamepad1.y){
                newTarget = 870;
                servoPosition = 0.525;
                rotationPress = true;
                mRobot.mArm.arm.setPower(1);
            }

            if (elapseTimer.milliseconds() > 500 && rotationPress){
                servoPosition = 0.525;
            }


            if (gamepad2.y){
                mRobot.mArm.setRotation(0.35);
                press = true;
                servoPosition = (0.024);
                rotationPress = false;
                //newTarget = 0;
            }




            if (elapseTimer.milliseconds() > 500 && press){
                newTarget = 150;
                mRobot.mArm.arm.setPower(0.5);
                press = false;
            } else {
                mRobot.mArm.arm.setPower(1);
            }

            //test

//            if (gamepad2.dpad_up){
//                mRobot.mDrive.forward(1);
//            }
//            if (gamepad2.dpad_down){
//                mRobot.mDrive.backward(1);
//            }
//            if (gamepad2.dpad_right){
//                mRobot.mDrive.strafeRight(1);
//            }
//            if (gamepad2.dpad_left){
//                mRobot.mDrive.strafeLeft(1);
//            }


            if (touchSensor.isPressed() == true) {
                telemetry.addData("Touch Sensor", "Is Pressed");
            }
            telemetry.addData("clawPosistion", mRobot.mArm.claw.getPosition());+
            telemetry.addData("position", mRobot.mArm.arm.getCurrentPosition());
            telemetry.addData("target postion", mRobot.mArm.arm.getTargetPosition());
            telemetry.addData("current extension", mRobot.mArm.extend.getPosition());
            telemetry.update();



            telemetry.addLine("telemetry is working");


        }
    }
}