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

    private double balanceOffset = 0.0;
    private final double balanceOffsestGain = 0.0;
    private DoubleSupplier robotRollSupplier;

    private final double balanceTolerance = 0.0;
    private final double extensionTolerance = 0.0;

    public ClimberStatemachine(Climber climber, DoubleSupplier robotRollSupplier){
        this.climber = climber;
        this.robotRollSupplier = robotRollSupplier;
    }

    @Override
    public void requestState(ClimberState state){
        this.state = state;
    }

    @Override
    public void update(){
        SmartDashboard.putString("Climber State", state.name());
        if(!state.isBalance()){
            climber.setClimberPosition(state.getPosition());
            balanceOffset = 0.0;
            return;
        }
        
        balanceOffset += robotRollSupplier.getAsDouble() * balanceOffsestGain;
        climber.setClimberPosition(state.getPosition() + balanceOffset, state.getPosition() - balanceOffset);

    }

    @Override
    public ClimberState getState(){
        return state;
    }

    @Override
    public boolean transitioning(){
        //done when we are at the desired position and the robot is balanced or we are not balancing
        var atPosition = Math.abs(climber.getClimberPosition() - state.getPosition()) < extensionTolerance;
        var balanced = Math.abs(robotRollSupplier.getAsDouble()) < balanceTolerance;
        return !atPosition || !(balanced || !state.isBalance());
    }

    @Override
    public boolean isDynamic() {
        return false;
    }

    public enum ClimberState{
        //todo
        RETRACT(0.0, false),
        EXTEND(0.0, false),
        CENTER(0.0, false),
        BALANCE(0.0, true);

        private Double position;
        private Boolean balance;

        public Double getPosition(){
            return position;
        }

        public Boolean isBalance(){
            return balance;
        }

        private ClimberState(Double position, Boolean balance){
            this.position = position;
            this.balance = balance;
        }
    }
}
