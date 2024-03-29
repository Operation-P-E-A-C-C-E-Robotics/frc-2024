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
    //Angles in rotations
    public static final Rotation2d interferenceLowerPivotAngle = Rotation2d.fromDegrees(30); //threshold for the flywheel intake to avoid the pivot
    public static final Rotation2d interferenceUpperPivotAngle = Rotation2d.fromDegrees(60); //threshold for the trigger intake to avoid the pivot
    public static final Rotation2d triggerIntakeMaxExtensionToFlatten = Rotation2d.fromDegrees(0); //threshold for the pivot to be allowed to flatten
    public static final Rotation2d canDiverterExtendMinPivotAngle = Rotation2d.fromDegrees(90); //threshold for the flipper to be allowed to extend
    public static final Rotation2d canPivotFlipMinIntakeExtension = Rotation2d.fromDegrees(20); 


    private boolean canFlattenPivot = false;
    private boolean shouldTriggerIntakeAvoid = false;
    private boolean canDiverterExtend = false;

    private boolean canFlipPivot = false;

    public MotionPlanner () {
    }

    public void update() {
        var pivotRadians = Pivot.getInstance().getPivotPosition().getRadians();
        var triggerIntakeExtension = TriggerIntake.getInstance().getDeploymentAngle().getRadians();
        canFlattenPivot = triggerIntakeExtension < triggerIntakeMaxExtensionToFlatten.getRadians();
        shouldTriggerIntakeAvoid = pivotRadians > interferenceUpperPivotAngle.getRadians() ||
                                    pivotRadians < interferenceLowerPivotAngle.getRadians();
        canDiverterExtend = pivotRadians > canDiverterExtendMinPivotAngle.getRadians();
        canFlipPivot = triggerIntakeExtension > canPivotFlipMinIntakeExtension.getRadians();
    }

    public boolean canFlattenPivot() {
        return canFlattenPivot;
    }

    public boolean canFlipPivot() {
        return canFlipPivot;
    }

    public boolean shouldTriggerIntakeAvoid() {
        return shouldTriggerIntakeAvoid;
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
