package frc.robot.statemachines;

import java.util.function.BooleanSupplier;

import frc.lib.state.StateMachine;
import frc.robot.planners.IntakeMotionPlanner;
import frc.robot.subsystems.TriggerIntake;

public class TriggerIntakeStatemachine extends StateMachine<TriggerIntakeStatemachine.TriggerIntakeState>{
    private final TriggerIntake triggerIntake;
    private final IntakeMotionPlanner intakeMotionPlanner;
    
    private TriggerIntakeState state = TriggerIntakeState.RETRACT;
    private BooleanSupplier hasNote;

    public TriggerIntakeStatemachine(TriggerIntake triggerIntake, IntakeMotionPlanner intakeMotionPlanner, BooleanSupplier hasNote){
        this.triggerIntake = triggerIntake;
        this.intakeMotionPlanner = intakeMotionPlanner;
        this.hasNote = hasNote;
    }

    /**
     * Update the desired state of the mechanism
     * This is where the logic for the state machine goes.
     * (e.g. transitioning from intaking to resting when the game piece is detected)
     */
    private void updateState(){
        //don't intake until the shooter is ready (otherwise we'd spit it onto the bellypan which would be incredibly bad)
        if(state == TriggerIntakeState.INTAKE && !intakeMotionPlanner.readyToIntake()) state = TriggerIntakeState.EXTEND;

        //don't allow the shooter to hit the intake
        if(state == TriggerIntakeState.RETRACT && intakeMotionPlanner.shouldFlywheelIntakeAvoid()) state = TriggerIntakeState.AVOID;
        if(state == TriggerIntakeState.AVOID && !intakeMotionPlanner.shouldFlywheelIntakeAvoid()) state = TriggerIntakeState.RETRACT;

        //automatically transition to retracting the intake once we have detected a note
        if(state == TriggerIntakeState.INTAKE && hasNote.getAsBoolean()) state = TriggerIntakeState.RETRACT; //TODO make sure this isn't prone to sensor failure / noise
    }

    /**
     * Request a state for the mechanism to attain
     * Won't allow for a state that would result in a collision or other dangerous situation
     * @param state the desired state
     */
    @Override
    public void requestState(TriggerIntakeState state){
        this.state = state;
    }

    /**
     * Make the mechanism attain the desired state
     */
    @Override
    public void update(){
        updateState();
        triggerIntake.setDeploymentAngle(state.deployAngle);
        triggerIntake.setRollerSpeed(state.speed);
    }

    @Override
    public TriggerIntakeState getState(){
        return state;
    }

    @Override
    public boolean isDone(){
        return triggerIntake.deployedToSetpoint();
    }

    @Override
    public boolean isDynamic() {
        return true;
    }

    public enum TriggerIntakeState{
        //TODO
        RETRACT(0.0,0.0),
        EXTEND(0.0,0.0),
        INTAKE(0.0,0.0),
        AVOID(0.0,0.0),
        EJECT(0.0,0.0);
        
        private Double deployAngle;
        private Double speed;

        public Double getDeployAngle(){
            return deployAngle;
        }

        public Double getSpeed(){
            return speed;
        }

        private TriggerIntakeState (Double deployAngle, Double speed){
            this.deployAngle = deployAngle;
            this.speed = speed;
        }
    }
}
