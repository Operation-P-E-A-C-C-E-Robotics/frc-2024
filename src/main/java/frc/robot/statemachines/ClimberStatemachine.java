package frc.robot.statemachines;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.state.StateMachine;
import frc.robot.subsystems.Climber;

/**
 * In charge of controlling the climber.
 */
public class ClimberStatemachine extends StateMachine<ClimberStatemachine.ClimberState>{
    private ClimberState state = ClimberState.RETRACT;

    private final Climber climber;
    private final double extensionTolerance = 0.0;

    public ClimberStatemachine(Climber climber, DoubleSupplier robotRollSupplier){
        this.climber = climber;
    }

    @Override
    public void requestState(ClimberState state){
        this.state = state;
    }

    @Override
    public void update(){
        SmartDashboard.putString("Climber State", state.name());
        climber.setClimberPosition(state.getPosition());
        return;
    }

    @Override
    public ClimberState getState(){
        return state;
    }

    @Override
    public boolean transitioning(){
        //done when we are at the desired position and the robot is balanced or we are not balancing
        var atPosition = Math.abs(climber.getClimberPosition() - state.getPosition()) < extensionTolerance;
        return !atPosition;
    }

    @Override
    public boolean isDynamic() {
        return false;
    }

    public enum ClimberState{
        //todo
        RETRACT(0.0),
        EXTEND(1.0);

        private Double position;

        public Double getPosition(){
            return position;
        }

        private ClimberState(Double position){
            this.position = position;
        }
    }
}
