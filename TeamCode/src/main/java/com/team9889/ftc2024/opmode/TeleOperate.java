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
    boolean reset = false;
    boolean retractPress = false;
    boolean retractPress2 = false;



    double servoPosition = 0.024;
    @Override
    public void runOpMode() throws InterruptedException {
        touchSensor = hardwareMap.get(TouchSensor.class, "armSensor");

        mRobot.init(hardwareMap);

        int newTarget = mRobot.mArm.arm.getCurrentPosition();
        int ExtensionTarget = mRobot.mArm.extend.getCurrentPosition();

        waitForStart();

        ElapsedTime Timer = new ElapsedTime();

        while (opModeIsActive()) {



            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            // Calculate power for each motor
            double leftPower = drive + turn;
            double rightPower = drive - turn;

            // Send calculated power to wheels
            mRobot.mDrive.setPower(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x / 2);


            if (gamepad2.left_bumper){
                mRobot.mArm.setRotation(0);
            }
            if (gamepad2.right_bumper){
                mRobot.mArm.setRotation(0.66);
            }
            if (gamepad1.x){
                mRobot.mArm.setRotation(0.35);
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


            if (Timer.milliseconds() > 2000 || reset){
                if(gamepad1.right_trigger > 0.1) {
                    mRobot.mArm.arm.setPower(1);
                    newTarget += (int) (30 * gamepad1.right_trigger);
                } else if(gamepad1.left_trigger > 0.1) {
                    if (touchSensor.isPressed() == true) {
                        mRobot.mArm.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                        newTarget = 0;
                    } else {
                        mRobot.mArm.arm.setPower(1);
                        newTarget -= (int) (30 * gamepad1.left_trigger);
                    }
                }

                newTarget = Math.min(newTarget, 1300);
                mRobot.mArm.arm.setTargetPosition(newTarget);
                mRobot.mArm.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                if (gamepad2.x){
                   ExtensionTarget = 150;
                    press = true;
                }

                if (elapseTimer.milliseconds() > 700 && press){
                    newTarget = 0;
                    mRobot.mArm.arm.setPower(0.3);
                    press = false;
                }


//            if (gamepad2.x) {
//                mRobot.mArm.setIntake2Power(1);
//            }else {
//                mRobot.mArm.setIntake2Power(0);
//            }

                if (gamepad1.y){
                    newTarget = 950;
                    rotationPress = true;
                    mRobot.mArm.arm.setPower(1);
                    elapseTimer.reset();
                }

                if (elapseTimer.milliseconds() > 500
                        &&
                        rotationPress){
                    ExtensionTarget = 1250;
                    mRobot.mArm.extend.setPower(1);
                    mRobot.mArm.setRotation(0.2);
                }




                if (gamepad2.y && mRobot.mArm.arm.getCurrentPosition() <= 500){
                    mRobot.mArm.setRotation(0);
                    retractPress = true;
                    newTarget = 300;
                    mRobot.mArm.arm.setPower(0.3);
                    rotationPress = false;
                    //newTarget = 0;
                    elapseTimer.reset();
                }

                if (elapseTimer.milliseconds() > 300 && retractPress){
                    ExtensionTarget = 0;
                    mRobot.mArm.extend.setPower(1);
                    retractPress = false;
                }

                if (gamepad2.y && mRobot.mArm.arm.getCurrentPosition() > 500){
                    mRobot.mArm.setRotation(0);
                    retractPress2 = true;
                    ExtensionTarget = 0;
                    rotationPress = false;
                    //newTarget = 0;
                    elapseTimer.reset();
                }

                if (elapseTimer.milliseconds() > 700 && retractPress2){
                    newTarget = 300;
                    mRobot.mArm.arm.setPower(0.3);
                    retractPress2 = false;
                } else {
                    mRobot.mArm.arm.setPower(1);
                }






            } else {
                if (Timer.milliseconds() < 400){
                    mRobot.mArm.arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    mRobot.mArm.arm.setPower(0.5);
                } else if (Timer.milliseconds() < 2000 && touchSensor.isPressed() != true) {
                    mRobot.mArm.arm.setPower(-0.05);
                } else {
                    mRobot.mArm.arm.setPower(0);
                    mRobot.mArm.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    mRobot.mArm.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    mRobot.mArm.arm.setPower(1);
                    mRobot.mArm.arm.setTargetPosition(150);
                    newTarget = 150;
                    reset = true;

                }
            }


            ExtensionTarget = Math.min(Math.max(-20, ExtensionTarget), 1300);
            mRobot.mArm.extend.setPower(1);
            mRobot.mArm.extend.setTargetPosition(ExtensionTarget);
            mRobot.mArm.extend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            if (gamepad1.dpad_up){
                ExtensionTarget += 20;
            }

            if (gamepad1.dpad_down){
                ExtensionTarget -= 20;
            }

//            if (gamepad2.right_trigger > gamepad2.left_trigger){
//                mRobot.mArm.setExtensionPower(gamepad2.right_trigger * 2);
//            } else if (gamepad2.right_trigger < gamepad2.left_trigger) {
//                mRobot.mArm.setExtensionPower(gamepad2.left_trigger * 2);
//            }else {
//                mRobot.mArm.setExtensionPower(0);
//            }


            //938













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
//

            if (gamepad2.dpad_up) {
                mRobot.mHanger.setHangPower(1);
            }
            else if (gamepad2.dpad_down){
                mRobot.mHanger.setHangPower(-1);
            }else{
                mRobot.mHanger.setHangPower(0);
            }





            if (touchSensor.isPressed() == true) {
                telemetry.addData("Touch Sensor", "Is Pressed");
            }
            telemetry.addData("clawPosistion", mRobot.mArm.claw.getPosition());

            telemetry.addData("position", mRobot.mArm.arm.getCurrentPosition());

            telemetry.addData("target extension postion", mRobot.mArm.extend.getTargetPosition());

            telemetry.addData("p", mRobot.mArm.extend.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).p);
            telemetry.addData("i", mRobot.mArm.extend.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).i);
            telemetry.addData("d", mRobot.mArm.extend.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION).d);
            telemetry.update();



            telemetry.addLine("telemetry is working");


        }
    }
}