package frc.robot.statemachines;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.state.StateMachine;
import frc.robot.planners.MotionPlanner;
import frc.robot.subsystems.FlywheelIntake;

public class FlywheelIntakeStatemachine extends StateMachine<FlywheelIntakeStatemachine.FlywheelIntakeState>{
    private FlywheelIntakeState state = FlywheelIntakeState.RETRACT;

    private final FlywheelIntake flywheelIntake;
    private final MotionPlanner intakeMotionPlanner;

    public FlywheelIntakeStatemachine(FlywheelIntake flywheelIntake, MotionPlanner intakeMotionPlanner){
        this.flywheelIntake = flywheelIntake;
        this.intakeMotionPlanner = intakeMotionPlanner;
    }

    private void updateState(){
        //don't intake until the shooter is ready (otherwise we'd spit it onto the bellypan which would be incredibly bad)
        if(state == FlywheelIntakeState.INTAKE && !intakeMotionPlanner.readyToIntake()) state = FlywheelIntakeState.EXTEND;

        //don't allow the shooter to hit the intake
        if(state == FlywheelIntakeState.RETRACT && intakeMotionPlanner.shouldFlywheelIntakeAvoid()) state = FlywheelIntakeState.AVOID;
        if(state == FlywheelIntakeState.AVOID && !intakeMotionPlanner.shouldFlywheelIntakeAvoid()) state = FlywheelIntakeState.RETRACT;
    }

    @Override
    public void requestState(FlywheelIntakeState state){
        this.state = state;
    }

    @Override
    public void update(){
        updateState();
        flywheelIntake.setDeploymentAngle(state.deployAngle);
        flywheelIntake.setRollerSpeed(state.speed);

        SmartDashboard.putString("Flywheel Intake State", state.name());
    }

    @Override
    public FlywheelIntakeState getState(){
        return state;
    }

    @Override
    public boolean transitioning(){
        return !flywheelIntake.deployedToSetpoint();
    }

    @Override
    public boolean isDynamic() {
        return true;
    }

    public enum FlywheelIntakeState{
        //TODO
        RETRACT(Rotation2d.fromDegrees(0),0.0),
        EXTEND(Rotation2d.fromDegrees(90),0.0),
        INTAKE(Rotation2d.fromDegrees(90),1.0),
        AVOID(Rotation2d.fromDegrees(90),0.0),
        EJECT(Rotation2d.fromDegrees(90),-1.0);
        
        private Rotation2d deployAngle;
        private Double speed;

        public Rotation2d getDeployAngle(){
            return deployAngle;
        }

        public Double getSpeed(){
            return speed;
        }

        private FlywheelIntakeState (Rotation2d deployAngle, Double speed){
            this.deployAngle = deployAngle;
            this.speed = speed;
        }
    }
}
