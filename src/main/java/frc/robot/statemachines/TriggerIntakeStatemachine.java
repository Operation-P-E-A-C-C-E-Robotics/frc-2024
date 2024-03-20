package frc.robot.statemachines;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.state.StateMachine;
import frc.robot.planners.MotionPlanner;
import frc.robot.subsystems.TriggerIntake;

public class TriggerIntakeStatemachine extends StateMachine<TriggerIntakeStatemachine.TriggerIntakeState>{
    private final TriggerIntake triggerIntake;
    private final MotionPlanner intakeMotionPlanner;
    
    private TriggerIntakeState state = TriggerIntakeState.RETRACT;

    public TriggerIntakeStatemachine(TriggerIntake triggerIntake, MotionPlanner intakeMotionPlanner){
        this.triggerIntake = triggerIntake;
        this.intakeMotionPlanner = intakeMotionPlanner;
    }

    /**
     * Update the desired state of the mechanism
     * This is where the logic for the state machine goes.
     * (e.g. transitioning from intaking to resting when the game piece is detected)
     */
    private void updateState(){
        //don't allow the shooter to hit the intake
        if(state == TriggerIntakeState.RETRACT && intakeMotionPlanner.shouldTriggerIntakeAvoid()) state = TriggerIntakeState.AVOID;
        if(state == TriggerIntakeState.AVOID && !intakeMotionPlanner.shouldTriggerIntakeAvoid()) state = TriggerIntakeState.RETRACT;
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

        SmartDashboard.putString("Trigger Intake State", state.name());

        triggerIntake.setDeploymentAngle(state.deployAngle);
        triggerIntake.setRollerSpeed(state.speed);
        
        if(state == TriggerIntakeState.INTAKE || state == TriggerIntakeState.EXTEND) {
            if(triggerIntake.getDeploymentAngle().getDegrees() > 145) {
                triggerIntake.setDeploymentSpeed(0.1);
            }
        }
    }

    @Override
    public TriggerIntakeState getState(){
        return state;
    }

    @Override
    public boolean transitioning(){
        return !triggerIntake.deployedToSetpoint();
    }

    @Override
    public boolean isDynamic() {
        return state == TriggerIntakeState.RETRACT || state == TriggerIntakeState.AVOID;
    }

    public enum TriggerIntakeState{
        //TODO
        RETRACT(Rotation2d.fromDegrees(0),0.0),
        EXTEND(Rotation2d.fromDegrees(150),0.0),
        INTAKE(Rotation2d.fromDegrees(150),1.0),
        AVOID(Rotation2d.fromDegrees(50),0.0),
        EJECT(Rotation2d.fromDegrees(140),-1.0);
        
        private Rotation2d deployAngle;
        private Double speed;

        public Rotation2d getDeployAngle(){
            return deployAngle;
        }

        public Double getSpeed(){
            return speed;
        }

        private TriggerIntakeState (Rotation2d deployAngle, Double speed){
            this.deployAngle = deployAngle;
            this.speed = speed;
        }
    }
}
