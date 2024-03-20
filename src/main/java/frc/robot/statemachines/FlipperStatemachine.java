package frc.robot.statemachines;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.state.StateMachine;
import frc.robot.planners.MotionPlanner;
import frc.robot.subsystems.Diverter;

public class FlipperStatemachine extends StateMachine<FlipperStatemachine.FlipperState> {
    private FlipperState state = FlipperState.RETRACT;

    private final Diverter diverter;
    private final MotionPlanner planner;

    public FlipperStatemachine(Diverter diverter, MotionPlanner planner){
        this.diverter = diverter;
        this.planner = planner;
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
        if(!planner.canDiverterExtend() && state != FlipperState.HANDOFF) state = FlipperState.RETRACT;

        SmartDashboard.putString("Diverter State", state.name());
        diverter.setDiverterExtension(state.getExtension());
        
        if(state != FlipperState.HANDOFF && state.rollerSpeed == 0.0) diverter.lockRollerPosition();
        else diverter.setDiverterRoller(state.getRollerSpeed());
    }

    @Override
    public boolean transitioning(){
        return !diverter.atSetpoint();
    }

    @Override
    public boolean isDynamic() {
        return state == FlipperState.HANDOFF;
    }

    public enum FlipperState {
        RETRACT(1.0,0.0),
        HANDOFF(0.0,0.0), //from shooter
        ALIGN_AMP(5.0,0.0),
        ALIGN_TRAP(5.0,0.0),
        PLACE_AMP(5.0,1.0),
        CLIMB(5.0,0.0),
        PLACE_TRAP(5.0,1.0);

        private Double extension, rollerSpeed;

        public Double getExtension(){
            return extension;
        }

        public Double getRollerSpeed(){
            return rollerSpeed;
        }

        private FlipperState(Double extension, Double rollerSpeed){
            this.extension = extension;
            this.rollerSpeed = rollerSpeed;
        }
    }
}
