package com.team9889.ftc2024.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.team9889.ftc2024.subsystems.Robot;

@TeleOp
public class TestUltrasonicSensor extends OpMode {
    Robot mRobot = new Robot();

    @Override
    public void init() {
        mRobot.init(hardwareMap);
    }

    @Override
    public void loop() {
        mRobot.mDrive.update();

        telemetry.addData("ultrasonic Sensor", mRobot.getDistance());

        telemetry.addData("x", mRobot.mDrive.getPose().getX());
        telemetry.addData("y", mRobot.mDrive.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(mRobot.mDrive.getPose().getHeading()));
    }
}
