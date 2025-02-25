package com.team9889.lib.pedroPathing.constants;

import com.pedropathing.localization.*;
import com.pedropathing.localization.constants.*;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class LConstants {
    static {
//        TwoWheelConstants.forwardTicksToInches = 1 / (19.89436789f * 25.4f);
//        TwoWheelConstants.strafeTicksToInches = 1/ (19.89436789f * 25.4f);
//        TwoWheelConstants.forwardY = 0.02166;
//        TwoWheelConstants.strafeX = -1.80918;
//        TwoWheelConstants.forwardEncoder_HardwareMapName = "rightfront";
//        TwoWheelConstants.strafeEncoder_HardwareMapName = "leftfront";
//        TwoWheelConstants.forwardEncoderDirection = Encoder.FORWARD;
//        TwoWheelConstants.strafeEncoderDirection = Encoder.REVERSE;
//        TwoWheelConstants.IMU_HardwareMapName = "imu";
//        TwoWheelConstants.IMU_Orientation = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.DOWN);
        PinpointConstants.forwardY = 0.02166;
        PinpointConstants.strafeX = -1.80918;
        PinpointConstants.distanceUnit = DistanceUnit.INCH;
        PinpointConstants.hardwareMapName = "pinpoint";
        PinpointConstants.useYawScalar = false;
        PinpointConstants.yawScalar = 1.0;
        PinpointConstants.useCustomEncoderResolution = false;
        PinpointConstants.encoderResolution = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
        PinpointConstants.customEncoderResolution = 13.26291192;
        PinpointConstants.forwardEncoderDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        PinpointConstants.strafeEncoderDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
    }
}




