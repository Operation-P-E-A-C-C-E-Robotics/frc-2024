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
import frc.robot.subsystems.Diverter;
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
    private boolean aiming = false;

    private final double AUTO_AIM_X = 7; // distance from left wall to start aiming.
    // private final double AMP_HANDOFF_X = 5; // distance from left wall to start handoff.
    // private final double AMP_ALIGN_X = 3; // distance from left wall to start aligning.
    // private final double AMP_ALIGN_Y = 3; // distance from bottom wall to start aligning.

    //whether the joystick is overriding the pivot
    private boolean jogPivotMode = false;
    private boolean jogClimberMode = false;
    private boolean jogFlipperMode = false;

    private Timer ampResetTimer = new Timer();

    private TeleopInputs() {
    }


    /**
     * Get what state the swerve should be in, 
     * based on the joystick inputs and any automated inputs.
     * @return
     */
    public SwerveState getWantedSwerveState() {
        if(OI.Overrides.forceAim.getAsBoolean() || (aiming && mode == TeleopMode.SPEAKER)) {
            return SwerveState.AIM;
        }

        if(intakingMode == IntakingMode.BACK || RobotContainer.getInstance().getTeleopStatemachine().getState() == SuperstructureState.INTAKE_BACK) {
            return SwerveState.ALIGN_INTAKING;
        }

        if(OI.Swerve.isRobotCentric.getAsBoolean()) {
            return SwerveState.ROBOT_CENTRIC;
        }

        if(OI.Swerve.isLockIn.getAsBoolean()) {
            return SwerveState.LOCK_IN;
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
            if(ampResetTimer.get() > 0.5) {
                mode = TeleopMode.PANIC;
            }
            if(Shooter.getInstance().flywheelSwitchTripped()) {
                ampResetTimer.start();
            }
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
        if(OI.Overrides.forceHandoff.getAsBoolean()) return SuperstructureState.ALIGN_AMP;
        if(OI.Overrides.forceAim.getAsBoolean()) return SuperstructureState.AUTO_AIM;

        //handle the drivers' intaking requests, these take precedence over modes & automation
        intakingMode = wantedIntakeMode();
        SmartDashboard.putString("Intaking Mode", intakingMode.name());
        if(intakingMode != IntakingMode.NONE) {
            return intakingMode == IntakingMode.FRONT ? SuperstructureState.INTAKE_FRONT : SuperstructureState.INTAKE_BACK;
        }

        if(OI.Inputs.wantsAimLayup.getAsBoolean()) return SuperstructureState.AIM_LAYUP;
        if(OI.Inputs.wantsAimProtected.getAsBoolean()) return SuperstructureState.AIM_PROTECTED;
        if(OI.Inputs.wantsIntakeSource.getAsBoolean()) return SuperstructureState.INTAKE_SOURCE;

        //handle the driver's request to "place" (a button that does different things based on the mode)
        if(OI.Inputs.wantsPlace.getAsBoolean()) {
            switch (mode) {
                case AMP:
                    // return SuperstructureState.PLACE_AMP;
                case CLIMB:
                    if(climbMode == ClimbMode.RETRACT) return SuperstructureState.PLACE_TRAP;
                case SPEAKER:
                    // return TeleopState.SHOOT;
                default:
                    break;
            }
        }

        if(mode == TeleopMode.PANIC) return SuperstructureState.REST;


        //handle mode-specific automation
        switch (mode) {
            case AMP:
                aiming = false;
                // if(wantsHandoff(blueAlliancePose)) {
                //     return SuperstructureState.HANDOFF;
                // }
                // if(wantsAlignAmp(blueAlliancePose)) {
                //     return SuperstructureState.ALIGN_AMP;
                // }
                return SuperstructureState.ALIGN_AMP;
            case CLIMB:
                aiming = false;
                climbMode = wantedClimbMode();
                if(climbMode == ClimbMode.ALIGN) return SuperstructureState.ALIGN_CLIMB;
                if(climbMode == ClimbMode.EXTEND) return SuperstructureState.CLIMB_EXTEND;
                if(climbMode == ClimbMode.RETRACT) return SuperstructureState.CLIMB_RETRACT;
                if(climbMode == ClimbMode.BALANCE) return SuperstructureState.CLIMB_BALANCE;
                return SuperstructureState.ALIGN_CLIMB;
            case SPEAKER:
                aiming = wantsAim(blueAlliancePose); // stored for use in swerve state
                // if(OI.Inputs.wantsShoot.getAsBoolean()) return TeleopState.SHOOT;
                if(aiming) {
                    return SuperstructureState.AUTO_AIM;
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
        if(OI.Overrides.forceTrigger.getAsBoolean()) {
            Shooter.getInstance().setTrigerPercent(1);
        }

        if(OI.Overrides.eject.getAsBoolean()) {
            TriggerIntake.getInstance().setRollerSpeed(-1);
            Shooter.getInstance().setTrigerPercent(-1);
            Shooter.getInstance().setFlywheelVelocity(10);
            Diverter.getInstance().setDiverterRoller(1);
        }

        var manualPivot = OI.ManualInputs.jogPivot.getAsDouble() * 0.35;
        var manualTrigger = OI.ManualInputs.jogTrigger.getAsDouble();
        var manualClimberLeft = OI.ManualInputs.jogClimberLeft.getAsDouble();
        var manualClimberRight = OI.ManualInputs.jogClimberRight.getAsDouble();
        var manualFlipper = OI.ManualInputs.jogFlipper.getAsDouble();

        if(mode != TeleopMode.CLIMB) {
            manualClimberRight = 0;
            manualClimberLeft = 0;
        }

        if(OI.ManualInputs.resetManualInputs.getAsBoolean()) {
            jogPivotMode = false;
            jogClimberMode = false;
            jogFlipperMode = false;
        }

        if(jogPivotMode || Math.abs(manualPivot) > 0.2 && mode != TeleopMode.CLIMB) {
            jogPivotMode = true;
            Pivot.getInstance().setPivotPercent(manualPivot);
        }

        if(jogClimberMode || Math.abs(manualClimberLeft) > 0.2 || Math.abs(manualClimberRight) > 0.2 && mode == TeleopMode.CLIMB) {
            jogClimberMode = true;
            Climber.getInstance().setClimberPercent(manualClimberLeft, manualClimberRight);
        }

        if(jogFlipperMode || Math.abs(manualFlipper) > 0.2 && mode != TeleopMode.CLIMB) {
            jogFlipperMode = true;
            Diverter.getInstance().setDiverterExtensionPercent(manualFlipper);
        }

        if(Math.abs(manualTrigger) > 0.1 && mode != TeleopMode.CLIMB) {
            // jogTriggerMode = true;
            Shooter.getInstance().setTrigerPercent(manualTrigger/2);
            if(manualTrigger < 0) {
                //don't let notes stay stuck in the flywheel.
                Shooter.getInstance().setFlywheelVelocity(manualTrigger * 10);
            }
        }

        // if((OI.Inputs.wantsAimLayup.getAsBoolean() || OI.Inputs.wantsAimProtected.getAsBoolean()) && OI.Inputs.wantsPlace.getAsBoolean()) {
        //     Shooter.getInstance().setTrigerPercent(1);
        // }
    }
    
    public TeleopMode getMode() {
        return mode;
    }

    // private boolean wantsAlignAmp(Pose2d blueAlliancePose) {
    //     if (blueAlliancePose.getX() < AMP_ALIGN_X && blueAlliancePose.getY() < AMP_ALIGN_Y) return true;
    //     return false;
    // }

    // private boolean wantsHandoff(Pose2d blueAlliancePose) {
    //     if (blueAlliancePose.getX() < AMP_HANDOFF_X) return true;
    //     return false;
    // }

    private boolean wantsAim(Pose2d blueAlliancePose) {
        if(NoteTracker.getLocation() != NoteLocation.SHOOTER) return false;
        if (blueAlliancePose.getX() < AUTO_AIM_X) return true;
        return false;
    }

    private IntakingMode wantedIntakeMode() {
        // operator overrides
        if(OI.Overrides.forceIntakeFront.getAsBoolean()) return IntakingMode.FRONT;
        if(OI.Overrides.forceIntakeBack.getAsBoolean()) return IntakingMode.BACK;

        // if the driver doesn't want to intake, return NONE
        // otherwise, default to the back intake
        if(!OI.Inputs.wantsIntake.getAsBoolean()) return IntakingMode.NONE;
        else if(NoteTracker.getLocation() == NoteLocation.NONE)intakingMode = IntakingMode.BACK;

        return intakingMode;
    }

    private ClimbMode wantedClimbMode() {
        if(!OI.Inputs.wantsAlign.getAsBoolean()) return ClimbMode.ALIGN;
        if(OI.Inputs.wantsBalance.getAsBoolean()) return ClimbMode.BALANCE;
        if(OI.Inputs.wantsClimbExtend.getAsBoolean()) return ClimbMode.EXTEND;
        if(OI.Inputs.wantsClimbRetract.getAsBoolean()) return ClimbMode.RETRACT;
        return climbMode;
    }

    public enum TeleopMode {
        SPEAKER, AMP, CLIMB, PANIC
    }

    public enum IntakingMode {
        FRONT, BACK, NONE
    }

    public enum ClimbMode {
        EXTEND, RETRACT, BALANCE, ALIGN
    }

    private static TeleopInputs instance = new TeleopInputs();
    public static TeleopInputs getInstance() {
        return instance;
    }
}
