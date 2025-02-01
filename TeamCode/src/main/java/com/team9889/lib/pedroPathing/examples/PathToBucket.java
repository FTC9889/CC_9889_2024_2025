package com.team9889.lib.pedroPathing.examples;

import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathBuilder;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;

public class PathToBucket {
    PathBuilder builder = new PathBuilder();
    PathChain pathChain;

    public PathToBucket() {
        builder
                .addPath(
                        // Line 1
                        new BezierLine(
                                new Point(7.500, 79.250, Point.CARTESIAN),
                                new Point(40.000, 79.250, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed(true)
                .addPath(
                        // Line 2
                        new BezierLine(
                                new Point(40.000, 79.250, Point.CARTESIAN),
                                new Point(35.000, 79.250, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Line 3
                        new BezierLine(
                                new Point(35.000, 79.250, Point.CARTESIAN),
                                new Point(35.000, 79.250, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(0))
                .addPath(
                        // Line 4
                        new BezierLine(
                                new Point(35.000, 79.250, Point.CARTESIAN),
                                new Point(40.000, 79.250, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .addPath(
                        // Line 5
                        new BezierCurve(
                                new Point(40.000, 79.250, Point.CARTESIAN),
                                new Point(6.000, 79.250, Point.CARTESIAN),
                                new Point(29.200, 119.638, Point.CARTESIAN),
                                new Point(16.185, 129.149, Point.CARTESIAN)
                        )
                )
                .setTangentHeadingInterpolation()
                .setReversed(true)
                .addPath(
                        // Line 6
                        new BezierLine(
                                new Point(16.185, 129.149, Point.CARTESIAN),
                                new Point(20.000, 126.000, Point.CARTESIAN)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-13));
    }

    public void build() {
        pathChain = builder.build();
    }
}
