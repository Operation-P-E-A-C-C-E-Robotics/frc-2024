package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.state.StateMachine;
import frc.lib.telemetry.MultiTracers;
import frc.robot.planners.AimPlanner;
import frc.robot.planners.MotionPlanner;
import frc.robot.statemachines.ClimberStatemachine;
import frc.robot.statemachines.PivotStatemachine;
import frc.robot.statemachines.ShooterStatemachine;
import frc.robot.statemachines.SwerveStatemachine;
import frc.robot.statemachines.TriggerIntakeStatemachine;
import frc.robot.statemachines.ClimberStatemachine.ClimberState;
import frc.robot.statemachines.SwerveStatemachine.SwerveState;
import frc.robot.statemachines.PivotStatemachine.PivotState;
import frc.robot.statemachines.ShooterStatemachine.ShooterState;
import frc.robot.statemachines.TriggerIntakeStatemachine.TriggerIntakeState;
import frc.robot.subsystems.Shooter;

/**
 * TeleopStatemachine controls the state of the whole robot by setting the states
 * of all the subsystems' state machines.
 */
public class RobotStatemachine extends StateMachine<RobotStatemachine.SuperstructureState>{
    private SuperstructureState state = SuperstructureState.REST;
    
    private final SwerveStatemachine swerveStatemachine;
    private final TriggerIntakeStatemachine triggerIntakeStatemachine;
    private final ShooterStatemachine shooterStatemachine;
    private final PivotStatemachine pivotStatemachine;
    private final ClimberStatemachine climberStatemachine;

    // private final MotionPlanner intakeMotionPlanner;
    // private final AimPlanner aimPlanner;
    //private final StageAvoidancePlanner stageAvoidancePlanner;

    public RobotStatemachine (SwerveStatemachine swerveStatemachine, 
                            TriggerIntakeStatemachine triggerIntakeStatemachine, 
                            ShooterStatemachine shooterStatemachine, 
                            PivotStatemachine pivotStatemachine, 
                            ClimberStatemachine climberStatemachine,
                            MotionPlanner intakeMotionPlanner,
                            AimPlanner aimPlanner) {
        this.swerveStatemachine = swerveStatemachine;
        this.triggerIntakeStatemachine = triggerIntakeStatemachine;
        this.shooterStatemachine = shooterStatemachine;
        this.pivotStatemachine = pivotStatemachine;
        this.climberStatemachine = climberStatemachine;
        // this.intakeMotionPlanner = intakeMotionPlanner;
        // this.aimPlanner = aimPlanner;
        // this.stageAvoidancePlanner = stageAvoidancePlanner;
    }

    /**
     * Request a state change.
     * Won't change state if the requested state results in a dangerous situation,
     * e.g. intaking while climbing.
     * @param state
     */
    @Override
    public void requestState(SuperstructureState state){
        //no intaking while climbing
        if(state == SuperstructureState.INTAKE_FRONT && this.state == SuperstructureState.CLIMB_RETRACT) return;
        if(state == SuperstructureState.INTAKE_BACK && this.state == SuperstructureState.CLIMB_RETRACT) return;
        
        this.state = state;
    }

    /**
     * The drivetrain is it's own thing lol
     * @param state
     */
    public void requestSwerveState(SwerveState state){
        swerveStatemachine.requestState(state);
    }

    public void requestSwervePath(Command pathCommand) {
        swerveStatemachine.setPathCommand(pathCommand);
        requestSwerveState(SwerveState.FOLLOW_PATH);
    }

    /**
     * Make the robot attain the desired state
     */
    @Override
    public void update(){
        SmartDashboard.putString("Robot State", state.name());
        MultiTracers.trace("TeleopStatemachine", "start update");
        triggerIntakeStatemachine.requestState(state.getTriggerIntakeState());
        MultiTracers.trace("TeleopStatemachine", "triggerIntakeStatemachine.requestState");
        shooterStatemachine.requestState(state.getShooterState());
        MultiTracers.trace("TeleopStatemachine", "shooterStatemachine.requestState");
        pivotStatemachine.requestState(state.getPivotState());
        MultiTracers.trace("TeleopStatemachine", "pivotStatemachine.requestState");
        climberStatemachine.requestState(state.getClimberState());
        MultiTracers.trace("TeleopStatemachine", "climberStatemachine.requestState");

        if(state == SuperstructureState.INTAKE_N_AIM && Shooter.getInstance().triggerSwitchTripped()) {
            pivotStatemachine.requestState(PivotState.AUTO_AIM);
        }
        MultiTracers.print("TeleopStatemachine");
    }

    /**
     * Get the state that is currently being
     * executed by the state machine
     */
    @Override
    public SuperstructureState getState(){
        return state;
    }

    /**
     * Check if the robot has finished attaining the desired state
     */
    @Override
    public boolean transitioning(){
        return triggerIntakeStatemachine.transitioning() ||
                shooterStatemachine.transitioning() ||
                pivotStatemachine.transitioning() ||
                climberStatemachine.transitioning();
    }
    

    public enum SuperstructureState {
        REST,
        STOW(
            TriggerIntakeState.RETRACT,
            ShooterState.RAMP_DOWN,
            PivotState.STOW
        ),
        INTAKE_FRONT (
            TriggerIntakeState.RETRACT,
            ShooterState.INTAKE,
            PivotState.INTAKE
        ),
        INTAKE_BACK (
            TriggerIntakeState.INTAKE,
            ShooterState.INTAKE,
            PivotState.INTAKE
        ),
        AIM_LAYUP (
            ShooterState.AIM_LAYUP,
            PivotState.AIM_LAYUP
        ),
        AIM_PROTECTED (
            ShooterState.AIM_PROTECTED,
            PivotState.AIM_PROTECTED
        ),
        AIM_UNDER_STAGE (
            ShooterState.AIM_UNDER_STAGE,
            PivotState.AIM_UNDER_STAGE
        ),
        AIM_WINGLINE (
            ShooterState.AIM_WINGLINE,
            PivotState.AIM_WINGLINE
        ),
        AIM_CENTERLINE (
            ShooterState.AIM_CENTERLINE,
            PivotState.AIM_CENTERLINE
        ),
        AUTO_AIM(
            ShooterState.AUTO_AIM,
            PivotState.AUTO_AIM
        ),
        SHOOT(
            ShooterState.SHOOT,
            PivotState.AUTO_AIM
        ),
        PLACE_AMP(
            ShooterState.RAMP_DOWN,
            PivotState.AMP
        ),
        ALIGN_CLIMB(
            ShooterState.RAMP_DOWN,
            PivotState.PRE_CLIMB
        ),
        CLIMB_EXTEND(
            TriggerIntakeState.AVOID,
            PivotState.CLIMB,
            ClimberState.EXTEND
        ),
        CLIMB_RETRACT(
            TriggerIntakeState.AVOID,
            PivotState.CLIMB,
            ClimberState.RETRACT
        ),
        ALIGN_AMP(
            ShooterState.AMP, 
            PivotState.AMP
        ),
        INTAKE_SOURCE(
            ShooterState.INTAKE,
            PivotState.INTAKE_SOURCE
        ),
        INTAKE_N_AIM(
            TriggerIntakeState.INTAKE,
            ShooterState.INTAKE_N_AIM,
            PivotState.INTAKE
        ),
        INTAKE_N_PIVOT_AIM(
            TriggerIntakeState.INTAKE,
            ShooterState.INTAKE_N_AIM,
            PivotState.AUTO_AIM
        ),
        INTAKE_N_SHOOT(
            TriggerIntakeState.INTAKE,
            ShooterState.SHOOT,
            PivotState.AUTO_AIM
        );

        private TriggerIntakeState triggerIntakeState;
        private ShooterState shooterState;
        private PivotState pivotState;
        private ClimberState climberState;

        public TriggerIntakeState getTriggerIntakeState(){
            return triggerIntakeState;
        }

        public ShooterState getShooterState(){
            return shooterState;
        }

        public PivotState getPivotState(){
            return pivotState;
        }

        public ClimberState getClimberState(){
            return climberState;
        }

        private SuperstructureState (TriggerIntakeState triggerIntakeState,
                                ShooterState shooterState,
                                PivotState pivotState,
                                ClimberState climberState){
            this.triggerIntakeState = triggerIntakeState;
            this.shooterState = shooterState;
            this.pivotState = pivotState;
            this.climberState = climberState;
        }

        private SuperstructureState(TriggerIntakeState triggerIntakeState,
                                ShooterState shooterState) {
            this(triggerIntakeState, shooterState, PivotState.REST, ClimberState.RETRACT);
        }

        private SuperstructureState(TriggerIntakeState triggerIntakeState,
                                ShooterState shooterState,
                                PivotState pivotState) {
            this(triggerIntakeState, shooterState, pivotState, ClimberState.RETRACT);
        }

        private SuperstructureState(ShooterState shooterState, PivotState pivotState){
            this(TriggerIntakeState.RETRACT, shooterState, pivotState, ClimberState.RETRACT);
        }

        private SuperstructureState(TriggerIntakeState triggerIntakeState, PivotState pivotState, ClimberState climberState){
            this(triggerIntakeState, ShooterState.RAMP_DOWN, pivotState, climberState);
        }

        private SuperstructureState(){
            this(TriggerIntakeState.RETRACT, ShooterState.RAMP_DOWN, PivotState.REST, ClimberState.RETRACT);
        }
    }
}
