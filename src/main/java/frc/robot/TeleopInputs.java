package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.AllianceFlipUtil;
import frc.robot.RobotStatemachine.SuperstructureState;
import frc.robot.planners.NoteTracker;
import frc.robot.planners.NoteTracker.NoteLocation;
import frc.robot.statemachines.SwerveStatemachine.SwerveState;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.TriggerIntake;

/**
 * This class is used to define the inputs for the teleop state machine.
 * Joystick inputs get mapped here, and also automated inputs (for instance, triggering
 * a state when the robot is in a certain position or when a certain sensor is triggered).
 * Note that the actual inputs are defined in the OI class to make it easier to modify,
 * but the actual logic for the state machine is defined here.
 */
public class TeleopInputs {
    private TeleopMode mode = TeleopMode.PANIC;
    private IntakingMode intakingMode = IntakingMode.NONE;
    private ClimbMode climbMode = ClimbMode.ALIGN;
    private AimMode aimMode = AimMode.AUTO;
    private boolean aiming = false;

    private final double AUTO_AIM_X = 7; // distance from left wall to start aiming.
    private final double LAYUP_X = 2; // distance from left wall to start aiming.
    private final double PROTECTED_X = 3.5;
    private final double UNDER_STAGE_X = 7;
    private final double WINGLINE_X = FieldConstants.wingX + 3;
    private final double CENTERLINE_X = WINGLINE_X + 10;

    private final double AMP_ALIGN_X = 4.5;
    private final double AMP_ALIGN_Y = 6.5;

    //whether the joystick is overriding the pivot
    private boolean jogPivotMode = false;
    private boolean jogClimberMode = false;
    private boolean jogTriggerMode = false;

    private boolean doTheAmpAlign = false;

    private Timer ampResetTimer = new Timer();

    private TeleopInputs() {
    }


    /**
     * Get what state the swerve should be in, 
     * based on the joystick inputs and any automated inputs.
     * @return
     */
    public SwerveState getWantedSwerveState() {
        if(OI.Overrides.forceAim.getAsBoolean() || (aiming && mode == TeleopMode.SPEAKER && intakingMode == IntakingMode.NONE)) {
            return aimMode == AimMode.SHUTTLE ? SwerveState.AIM_SHUTTLE : SwerveState.AIM;
        }

        if(OI.Swerve.isLockIn.getAsBoolean()) {
            return SwerveState.LOCK_IN;
        }

        if(OI.Swerve.isRobotCentric.getAsBoolean()) {
            return SwerveState.ROBOT_CENTRIC;
        }

        if(OI.Swerve.wantsDriveToNote.getAsBoolean()){
            return SwerveState.DRIVE_TO_NOTE;
        }

        return OI.Swerve.isOpenLoop.getAsBoolean() ? SwerveState.OPEN_LOOP_TELEOP : SwerveState.CLOSED_LOOP_TELEOP;
    }

    /**
     * Get what state the robot should be in,
     * based on the joystick inputs and automated inputs.
     * @return
     */
    public SuperstructureState getWantedTeleopState() {
        var blueAlliancePose = AllianceFlipUtil.apply(Swerve.getInstance().getPose()); //robot pose for automation

        if(mode == TeleopMode.AMP) {
            // if(ampResetTimer.get() > 0.7) {
            //     mode = TeleopMode.PANIC;
            // }
            if(intakingMode != IntakingMode.NONE) {
                ampResetTimer.stop();
                ampResetTimer.reset();
            }
            if(OI.Inputs.wantsPlace.getAsBoolean()) ampResetTimer.start();
        } else {
            ampResetTimer.stop();
            ampResetTimer.reset();
        }
        // change the mode based on the operator inputs
        if(OI.Modes.wantsAmpMode.getAsBoolean()) mode = TeleopMode.AMP;
        if(OI.Modes.wantsClimbMode.getAsBoolean()) mode = TeleopMode.CLIMB;
        if(OI.Modes.wantsSpeakerMode.getAsBoolean()) mode = TeleopMode.SPEAKER;
        if(OI.Modes.wantsPanicMode.getAsBoolean()) mode = TeleopMode.PANIC;

        SmartDashboard.putString("Teleop Mode", mode.name());

        // reset the climb mode if we're not climbing
        if(mode != TeleopMode.CLIMB) climbMode = ClimbMode.ALIGN;

        SmartDashboard.putString("Climb Mode", climbMode.name());

        // operator overrides - these take precedence over everything else
        if(OI.Inputs.wantsStow.getAsBoolean())  return SuperstructureState.STOW;
        if(OI.Overrides.forceAim.getAsBoolean()) return SuperstructureState.AUTO_AIM;

        //handle the drivers' intaking requests, these take precedence over modes & automation
        intakingMode = wantedIntakeMode();
        SmartDashboard.putString("Intaking Mode", intakingMode.name());
        if(intakingMode != IntakingMode.NONE) {
            return SuperstructureState.INTAKE_BACK;
        }

        if(OI.Inputs.wantsAimLayup.getAsBoolean()) aimMode = AimMode.LAYUP;
        if(OI.Inputs.wantsAimProtected.getAsBoolean()) aimMode = AimMode.PROTECTED;
        if(OI.Inputs.wantsAimUnderStage.getAsBoolean()) aimMode = AimMode.UNDER_STAGE;
        if(OI.Inputs.wantsAimWingline.getAsBoolean()) aimMode = AimMode.WINGLINE;
        if(OI.Inputs.wantsAimCenterline.getAsBoolean()) aimMode = AimMode.CENTERLINE;
        if(OI.Inputs.wantsAutoAim.getAsBoolean()) aimMode = AimMode.AUTO;
        if(OI.Inputs.wantsAimShuttle.getAsBoolean()) aimMode = AimMode.SHUTTLE;

        if(mode == TeleopMode.PANIC) return SuperstructureState.REST;


        //handle mode-specific automation
        switch (mode) {
            case AMP:
                aiming = false;
                return wantsAmp(new Pose2d()) ? SuperstructureState.ALIGN_AMP : SuperstructureState.REST;
            case CLIMB:
                aiming = false;
                climbMode = wantedClimbMode();
                if(climbMode == ClimbMode.ALIGN) return SuperstructureState.ALIGN_CLIMB;
                if(climbMode == ClimbMode.EXTEND) return SuperstructureState.CLIMB_EXTEND;
                if(climbMode == ClimbMode.RETRACT) return SuperstructureState.CLIMB_RETRACT;
                return SuperstructureState.ALIGN_CLIMB;
            case SPEAKER:
                aiming = wantsAim(blueAlliancePose); // stored for use in swerve state
                if(aiming) {
                    return aimState();
                }
                return SuperstructureState.REST;
            default:
                aiming = false;
                break;
        }

        return SuperstructureState.REST;
    }

    /**
     * Handle operator overrides that directly set subsystems,
     * These should take precedence over the state machines and automation.
     * Call this method after the state machines have been updated.
     */
    public void handleOverrides() {
        if(OI.Overrides.eject.getAsBoolean()) {
            TriggerIntake.getInstance().setRollerSpeed(-1);
            Shooter.getInstance().setTriggerPercent(1);
            Shooter.getInstance().setFlywheelVelocity(20);
        }

        var manualPivot = OI.ManualInputs.jogPivot.getAsDouble() * 0.35;
        var manualTrigger = OI.ManualInputs.jogTrigger.getAsDouble();
        var manualClimber = OI.ManualInputs.jogClimber.getAsDouble();

        if(mode != TeleopMode.CLIMB) {
            manualClimber = 0;
        }

        if(OI.ManualInputs.resetManualInputs.getAsBoolean()) {
            jogPivotMode = false;
            jogClimberMode = false;
            jogTriggerMode = false;
        }

        if(mode != TeleopMode.CLIMB) {
            jogClimberMode = false;
        }

        if(jogPivotMode || Math.abs(manualPivot) > 0.2 && mode != TeleopMode.CLIMB) {
            jogPivotMode = true;
            Pivot.getInstance().setPivotPercent(manualPivot);
        }

        if(jogClimberMode || Math.abs(manualClimber) > 0.2 && mode == TeleopMode.CLIMB) {
            jogClimberMode = true;
            Pivot.getInstance().climbMode();
            Climber.getInstance().setClimberPercent(manualClimber);
        }

        if(jogTriggerMode || Math.abs(manualTrigger) > 0.1 && mode != TeleopMode.CLIMB) {
            jogTriggerMode = true;
            Shooter.getInstance().setTriggerPercent(manualTrigger/2);
            if(manualTrigger < 0) {
                //don't let notes stay stuck in the flywheel.
                Shooter.getInstance().setFlywheelVelocity(manualTrigger * 10);
            }
        }
    }
    
    public TeleopMode getMode() {
        return mode;
    }

    private boolean wantsAmp(Pose2d blueAlliancePose){
        var pose = AllianceFlipUtil.apply(Swerve.getInstance().getPose());
        if(!OI.ManualInputs.resetManualInputs.getAsBoolean()) return true;
        if(pose.getX() < AMP_ALIGN_X && pose.getY() > AMP_ALIGN_Y && ampResetTimer.get() < 0.7) return true;
        return false;
    }

    private boolean wantsAim(Pose2d blueAlliancePose) {
        if(NoteTracker.getLocation() != NoteLocation.SHOOTER) return false;
        var x = AllianceFlipUtil.apply(Swerve.getInstance().getPose()).getX();
        switch (aimMode) {
            case AUTO:
                if (x > AUTO_AIM_X) return false;
                break;
            case CENTERLINE:
                if (x > CENTERLINE_X) return false;
                break;
            case LAYUP:
                if (x > LAYUP_X) return false;
                break;
            case PROTECTED:
                if (x > PROTECTED_X) return false;
                break;
            case UNDER_STAGE:
                if (x > UNDER_STAGE_X) return false;
                break;
            case WINGLINE:
                if (x > WINGLINE_X) return false;
                break;
            case SHUTTLE:
                return true;
            default:
                return false;
            
        }
        return true;
    }

    private SuperstructureState aimState() {
        switch (aimMode) {
            case LAYUP:
                return SuperstructureState.AIM_LAYUP;
            case PROTECTED:
                return SuperstructureState.AIM_PROTECTED;
            case UNDER_STAGE:
                return SuperstructureState.AIM_UNDER_STAGE;
            case WINGLINE:
                return SuperstructureState.AIM_WINGLINE;
            case CENTERLINE:
                return SuperstructureState.AIM_CENTERLINE;
            case SHUTTLE:
                return SuperstructureState.AIM_SHUTTLE;
            case AUTO:
                return SuperstructureState.AUTO_AIM;
            default:
                return SuperstructureState.REST;
        }
    }

    private IntakingMode wantedIntakeMode() {
        // operator overrides
        if(OI.Overrides.forceIntakeBack.getAsBoolean()) return IntakingMode.BACK;

        return IntakingMode.NONE;
    }

    private ClimbMode wantedClimbMode() {
        if(!OI.Inputs.wantsAlign.getAsBoolean()) return ClimbMode.ALIGN;
        if(OI.Inputs.wantsClimbExtend.getAsBoolean()) return ClimbMode.EXTEND;
        if(OI.Inputs.wantsClimbRetract.getAsBoolean()) return ClimbMode.RETRACT;
        return climbMode;
    }

    public enum TeleopMode {
        SPEAKER, AMP, CLIMB, PANIC
    }

    public enum IntakingMode {
        BACK, NONE
    }

    public enum ClimbMode {
        EXTEND, RETRACT, ALIGN
    }

    public enum AimMode {
        LAYUP, PROTECTED, UNDER_STAGE, WINGLINE, CENTERLINE, AUTO, SHUTTLE
    }

    private static TeleopInputs instance = new TeleopInputs();
    public static TeleopInputs getInstance() {
        return instance;
    }
}
