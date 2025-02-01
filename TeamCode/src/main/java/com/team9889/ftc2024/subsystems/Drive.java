package com.team9889.ftc2024.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.pedropathing.follower.Follower;
import com.pedropathing.pathgen.Path;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Drive extends Follower {

    public Drive(HardwareMap hardwareMap) {
        super(hardwareMap);
    }

    public class FollowTrajectoryAction implements Action {

        Path pathToFollow;

        public FollowTrajectoryAction(Path pathToFollow) {
            this.pathToFollow = pathToFollow;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            followPath(pathToFollow);
            return isBusy();
        }
    }

    public Action FollowTrajectoryAction (Path followPath) {
        return new FollowTrajectoryAction(followPath);
    }
}

