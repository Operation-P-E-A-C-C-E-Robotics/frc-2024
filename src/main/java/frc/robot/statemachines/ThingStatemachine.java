package frc.robot.statemachines;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.state.StateMachine;
import frc.robot.planners.MotionPlanner;
import frc.robot.subsystems.Thing;

public class ThingStatemachine extends StateMachine<ThingStatemachine.FlipperState> {
    private FlipperState state = FlipperState.RETRACT;

    private final Thing diverter;

    public ThingStatemachine(Thing diverter, MotionPlanner planner){
        this.diverter = diverter;
    }

    @Override
    public void requestState(FlipperState state){
        this.state = state;
    }

    @Override
    public FlipperState getState(){
        return state;
    }

    @Override
    public void update(){
        SmartDashboard.putString("Diverter State", state.name());
        diverter.setThingExtension(state.getExtension());
    }

    @Override
    public boolean transitioning(){
        return !diverter.atSetpoint();
    }

    @Override
    public boolean isDynamic() {
        return false;
    }

    public enum FlipperState {
        RETRACT(0.0),
        EXTEND(0.0);

        private Double extension;

        public Double getExtension(){
            return extension;
        }

        private FlipperState(Double extension){
            this.extension = extension;
        }
    }
}
