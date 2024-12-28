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
import com.team9889.ftc2024.subsystems.Intake;
import com.team9889.ftc2024.subsystems.Robot;

import java.util.MissingResourceException;


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



        waitForStart();

        while (opModeIsActive()) {

            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            // Calculate power for each motor
            double leftPower = drive + turn;
            double rightPower = drive - turn;

            // Send calculated power to wheels
            mRobot.mDrive.setPower(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x / 0.66);


//            if (gamepad1.a){
//                //are there encoders on intake extension motors
//                mRobot.mIntake.extension.setTargetPosition();
//                mRobot.mIntake.extension.setPower(0.5);
//                mRobot.mIntake.extension.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                mRobot.mIntake.setWristPosision();
//            }

            //gamepad1.a = intake slight extend && intake wrist down && power intake          while ( until detect color or if gamepad1.y  is clicked      if color is opponent eject      if color is yellow or alliance color reset robot and transfer) { gamepad1 can change wrist adn intake  slide position with gamepad1.triggrs and wrist with gamepad1.bumpers
            //gamepad1.b = specimen intaking position when x(claw) is closed the  automaticly go to netural positon
            //gamepad1.x = claw (open and close)
            //gamepad1.triggers = intake slides
            //gamepad1.y = reset robot


            // gamepad2.y = reset robot
            // gamepad2.triggers = lift slides


        }
    }
}