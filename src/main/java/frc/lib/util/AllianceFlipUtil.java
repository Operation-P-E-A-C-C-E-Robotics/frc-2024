// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.lib.util;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Utility funcitons for flipping from the blue to red alliance. By default, all translations and
 * poses in {@link FieldConstants} are stored with the origin at the rightmost point on the blue
 * alliance wall.
 */
public class AllianceFlipUtil {
    private static final double fieldLength = Units.inchesToMeters(651.25);
    /** Flips a translation to the correct side of the field based on the current alliance color. */
    public static Translation2d apply(Translation2d translation) {
        if (shouldFlip()) {
            return new Translation2d(fieldLength - translation.getX(), translation.getY());
        } else {
            return translation;
        }
    }

    public static Translation3d apply(Translation3d translation) {
        if (shouldFlip()) {
            return new Translation3d(fieldLength - translation.getX(), translation.getY(), translation.getZ());
        } else {
            return translation;
        }
    }

    /** Flips a rotation based on the current alliance color. */
    public static Rotation2d apply(Rotation2d rotation) {
        if (shouldFlip()) {
            return new Rotation2d(-rotation.getCos(), rotation.getSin());
        } else {
            return rotation;
        }
    }

    /** Flips a pose to the correct side of the field based on the current alliance color. */
    public static Pose2d apply(Pose2d pose) {
        if (shouldFlip()) {
            return new Pose2d(
                    fieldLength - pose.getX(),
                    pose.getY(),
                    new Rotation2d(-pose.getRotation().getCos(), pose.getRotation().getSin()));
        } else {
            return pose;
        }
    }
    /** Flips a pose to the correct side of the field based on the current alliance color. */
    public static Pose2d flip(Pose2d pose) {
            return new Pose2d(
                    fieldLength - pose.getX(),
                    pose.getY(),
                    new Rotation2d(-pose.getRotation().getCos(), pose.getRotation().getSin()));
    }

    /**
     * Flips a trajectory state to the correct side of the field based on the current alliance color.
     */
    public static Trajectory.State apply(Trajectory.State state) {
        if (shouldFlip()) {
            return new Trajectory.State(
                    state.timeSeconds,
                    state.velocityMetersPerSecond,
                    state.accelerationMetersPerSecondSq,
                    new Pose2d(
                            fieldLength - state.poseMeters.getX(),
                            state.poseMeters.getY(),
                            new Rotation2d(
                                    -state.poseMeters.getRotation().getCos(),
                                    state.poseMeters.getRotation().getSin())),
                    -state.curvatureRadPerMeter);
        } else {
            return state;
        }
    }

    private static boolean lastShouldFlip = false;
    private static Timer flipRecheckTimer = new Timer();
    private static boolean hasRecievedGoodData = false;

    public static boolean shouldFlip() {
        flipRecheckTimer.start();
        if (flipRecheckTimer.hasElapsed(10) || !hasRecievedGoodData) {
            var alliance = DriverStation.getAlliance();
            if(alliance.isEmpty()) return false;
            hasRecievedGoodData = true;
            lastShouldFlip = alliance.get() == Alliance.Red;
            flipRecheckTimer.reset();
        }
        return lastShouldFlip;
    }
}