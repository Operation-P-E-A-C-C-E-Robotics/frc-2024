package frc.robot.planners;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.TriggerIntake;

/**
 * In charge of calculating whether the pivot and flywheel can
 * be flattened and retracted, respectively.
 * 
 * Also says whether the diverter can extend
 */
public class MotionPlanner {
    //TODO
    //Angles in rotations
    public static final Rotation2d interferenceLowerPivotAngle = Rotation2d.fromDegrees(30); //threshold for the flywheel intake to avoid the pivot
    public static final Rotation2d interferenceUpperPivotAngle = Rotation2d.fromDegrees(60); //threshold for the trigger intake to avoid the pivot
    public static final Rotation2d flywheelIntakeMinExtensionToFlatten = Rotation2d.fromDegrees(0); //threshold for the pivot to be allowed to flatten
    public static final Rotation2d triggerIntakeMaxExtensionToFlatten = Rotation2d.fromDegrees(0); //threshold for the pivot to be allowed to flatten
    public static final Rotation2d allowIntakeLowerPivotAngle = Rotation2d.fromDegrees(0); //threshold for the flywheel intake to be allowed to intake
    public static final Rotation2d canDiverterExtendMinPivotAngle = Rotation2d.fromDegrees(90); //threshold for the flipper to be allowed to extend
    public static final Rotation2d canPivotFlipMinIntakeExtension = Rotation2d.fromDegrees(40); 


    private boolean canFlattenPivot = false;
    private boolean shouldFlywheelIntakeAvoid = false;
    private boolean shouldTriggerIntakeAvoid = false;
    private boolean readyToIntake = false;
    private boolean canDiverterExtend = false;

    private boolean canFlipPivot = false;

    public MotionPlanner () {
    }

    public void update() {
        var pivotRadians = Pivot.getInstance().getPivotPosition().getRadians();
        // var flywheelIntakeExtension = FlywheelIntake.getInstance().getDeploymentAngle().getRadians();
        var triggerIntakeExtension = TriggerIntake.getInstance().getDeploymentAngle().getRadians();
        canFlattenPivot = true//flywheelIntakeExtension < flywheelIntakeMinExtensionToFlatten.getRadians()
                        && triggerIntakeExtension < triggerIntakeMaxExtensionToFlatten.getRadians();
        shouldFlywheelIntakeAvoid = pivotRadians < interferenceLowerPivotAngle.getRadians();
        shouldTriggerIntakeAvoid = pivotRadians > interferenceUpperPivotAngle.getRadians() ||
                                    pivotRadians < interferenceLowerPivotAngle.getRadians();
        readyToIntake = pivotRadians < allowIntakeLowerPivotAngle.getRadians();
        canDiverterExtend = pivotRadians > canDiverterExtendMinPivotAngle.getRadians();
        canFlipPivot = triggerIntakeExtension > canPivotFlipMinIntakeExtension.getRadians();
    }

    public boolean canFlattenPivot() {
        return canFlattenPivot;
    }

    public boolean canFlipPivot() {
        return canFlipPivot;
    }

    public boolean shouldFlywheelIntakeAvoid() {
        return shouldFlywheelIntakeAvoid;
    }

    public boolean shouldTriggerIntakeAvoid() {
        return shouldTriggerIntakeAvoid;
    }

    public boolean readyToIntake() {
        return readyToIntake;
    }

    public boolean canDiverterExtend() {
        return canDiverterExtend;
    }

    public boolean shouldTransitionToFront() {
        return Swerve.getInstance().getChassisSpeeds().vxMetersPerSecond > 0.5;
    }

    public boolean shouldTransitionToBack() {
        return Swerve.getInstance().getChassisSpeeds().vxMetersPerSecond < -0.5;
    }
}
