package com.team9889.ftc2024.subsystems;

import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.team9889.lib.hardware.RevIMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

import java.util.Locale;

@Deprecated
public class Drive {
    DcMotor leftFront,leftBack, rightFront, rightBack;
    public RevIMU imu;

    public void init(HardwareMap hardwareMap){
        leftFront=hardwareMap.dcMotor.get("leftfront");
        leftBack = hardwareMap.dcMotor.get("leftback");
        rightFront=hardwareMap.dcMotor.get("rightfront");
        rightBack=hardwareMap.dcMotor.get("rightback");

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = new RevIMU("imu", hardwareMap);

        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(12.10244, 6.20385); //these are tuned for 3110-0002-0001 Product Insight #1

        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        /*
        Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
        The IMU will automatically calibrate when first powered on, but recalibrating before running
        the robot is a good idea to ensure that the calibration is "good".
        resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
        This is recommended before you run your autonomous, as a bad initial calibration can cause
        an incorrect starting value for x, y, and heading.
         */
        // odo.recalibrateIMU();
        odo.resetPosAndIMU();

    }
    public void setPower(double LFPower,double RFPower,double LBPower, double RBPower){
        rightFront.setPower(RFPower);
        leftFront.setPower(LFPower);
        leftBack.setPower(LBPower);
        rightBack.setPower(RBPower);
    }

    public void setPower(double leftStickX, double leftStickY, double rightStickX){
        double r = Math.hypot(leftStickX, leftStickY);
        double robotAngle = Math.atan2(leftStickY, leftStickX) - PI / 4;
        double rightX = rightStickX;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;
        setPower(v1,v2,v3,v4);

    }

    public double get_angle(){
        return imu.getNormalHeading();
    }

    public void reset_encoder(){
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void brake(){
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE );
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE );
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE );
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE );

        setPower(0,0,  0);
    }

    GoBildaPinpointDriver odo; // Declare OpMode member for the Odometry Computer
    double oldTime = 0;

    public void telemetry (Telemetry telemetry){
        telemetry.addData("Status", "Initialized");
        telemetry.addData("X offset", odo.getXOffset());
        telemetry.addData("Y offset", odo.getYOffset());
        telemetry.addData("Device Version Number:", odo.getDeviceVersion());
        telemetry.addData("Device Scalar", odo.getYawScalar());

        Pose2D pos = odo.getPosition();
        String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getX(DistanceUnit.MM), pos.getY(DistanceUnit.MM), pos.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Position", data);

        Pose2D vel = odo.getVelocity();
        String velocity = String.format(Locale.US,"{XVel: %.3f, YVel: %.3f, HVel: %.3f}", vel.getX(DistanceUnit.MM), vel.getY(DistanceUnit.MM), vel.getHeading(AngleUnit.DEGREES));
        telemetry.addData("Velocity", velocity);
        telemetry.addData("Status", odo.getDeviceStatus());
        telemetry.addData("Pinpoint Frequency", odo.getFrequency());
    }

    public void resetImu(){
        odo.resetPosAndIMU();
    }

    public void UptateODO(){
        odo.update();
    }

    public void recalibrateImu(){
        odo.recalibrateIMU();
    }

    public Pose2d pose() {
        Pose2D pos = odo.getPosition();

        return new Pose2d(
                pos.getX(DistanceUnit.INCH),
                pos.getY(DistanceUnit.INCH),
                angleWrap(pos.getHeading(AngleUnit.RADIANS))
        );
    }

    private double angleWrap(double radians) {

        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }

        // keep in mind that the result is in radians
        return radians;
    }
}

