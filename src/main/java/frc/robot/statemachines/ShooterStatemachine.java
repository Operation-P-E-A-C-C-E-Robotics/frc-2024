package frc.robot.statemachines;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.state.StateMachine;
import frc.robot.OI;
import frc.robot.planners.AimPlanner;
import frc.robot.subsystems.Shooter;

public class ShooterStatemachine extends StateMachine<ShooterStatemachine.ShooterState> {
    private ShooterState state = ShooterState.RAMP_DOWN;

    private ShooterState lastAimingState = ShooterState.AUTO_AIM;

    private final Shooter shooter;
    private final AimPlanner aimPlanner;
    private final BooleanSupplier FIRE;

    private final Timer sketchyTimer = new Timer();

    public ShooterStatemachine(Shooter shooter, AimPlanner aimPlanner, BooleanSupplier FIRE){
        this.shooter = shooter;
        this.aimPlanner = aimPlanner;
        this.FIRE = FIRE;
    }

    /**
     * Handle the logic for changing states
     * e.g. intaking to indexing when the gamepiece is detected
     */
    private void updateState(){
        SmartDashboard.putBoolean("flywheel switch", shooter.flywheelSwitchTripped());
        SmartDashboard.putBoolean("trigger switch", shooter.triggerSwitchTripped());

        if(state == ShooterState.RAMP_DOWN) if(shooter.flywheelSwitchTripped()) state = ShooterState.INDEX;
        else if(state == ShooterState.INDEX) if(!(shooter.triggerSwitchTripped() || shooter.flywheelSwitchTripped())) state = ShooterState.RAMP_DOWN;
        if (
              (state == ShooterState.AUTO_AIM
            ||state == ShooterState.AIM_LAYUP
            ||state == ShooterState.AIM_PROTECTED
            ||state == ShooterState.AIM_UNDER_STAGE
            ||state == ShooterState.AIM_WINGLINE
            ||state == ShooterState.AIM_CENTERLINE)
            && FIRE.getAsBoolean()
            && DriverStation.isTeleop()
        ) {
            lastAimingState = state;
            state = ShooterState.SHOOT;
            printShotData();
        }
        if(state == ShooterState.INTAKE && shooter.flywheelSwitchTripped()) {
            state = ShooterState.INDEX;
        }
    }

    /**
     * Request a state for the mechanism to attain
     * Won't allow for a state that would result in a collision or other dangerous situation
     * e.g. changing state before we have finished INDEXing
     * @param state
     */
    @Override
    public void requestState(ShooterState state){
        if(state == ShooterState.AUTO_AIM && (shooter.flywheelSwitchTripped() || shooter.triggerSwitchTripped())) {
            this.state = ShooterState.INDEX;
        }
        if(state == ShooterState.AUTO_AIM && this.state == ShooterState.INTAKE) {
            this.state = ShooterState.INDEX;
            return;
        }
        if(state == ShooterState.AUTO_AIM && this.state == ShooterState.SHOOT) return;
        if(state == ShooterState.SHOOT && this.state != ShooterState.SHOOT) printShotData();
        if(state == ShooterState.INTAKE && this.state == ShooterState.INDEX) return;
        this.state = state;
    }

    /**
     * make the mechanism attain the desired state
     */
    @Override
    public void update(){
        updateState();
        SmartDashboard.putString("Shooter State", state.name());
        
        if(state == ShooterState.AUTO_AIM) {
            shooter.setFlywheelVelocity(aimPlanner.getTargetFlywheelVelocityRPS());
            shooter.setTriggerPercent(0);
            return;
        }

        if(state == ShooterState.SHOOT) {
            if(lastAimingState == ShooterState.AUTO_AIM) shooter.setFlywheelVelocity(aimPlanner.getTargetFlywheelVelocityRPS());
            else shooter.setFlywheelVelocity(lastAimingState.getFlywheelVelocity());
            shooter.setTriggerPercent(state.getTriggerPercent());
            return;
        }

        if (state == ShooterState.INTAKE_N_AIM){
            shooter.setFlywheelVelocity(aimPlanner.getTargetFlywheelVelocityRPS());
            if(shooter.triggerSwitchTripped()) sketchyTimer.restart();
            if(!shooter.triggerSwitchTripped() || sketchyTimer.get() < 0.3) {
                shooter.setTriggerPercent(1);
            }
            else shooter.setTriggerPercent(0);
            return;
        }

        if(state == ShooterState.INDEX){
            if(shooter.flywheelSwitchTripped()) shooter.setTriggerPercent(-state.getTriggerPercent());
            else if (shooter.triggerSwitchTripped() && !shooter.flywheelSwitchTripped()) shooter.setTriggerPercent(state.getTriggerPercent());
            else shooter.setTriggerPercent(0);

            shooter.setFlywheelVelocity(state.flywheelVelocity);
            return;
        }

        if (state == ShooterState.COAST) {
            shooter.coastFlywheel();
            shooter.setTriggerPercent(0.0);
            return;
        }

        if (state == ShooterState.RAMP_DOWN) {
            shooter.brakeFlywheel();
            shooter.setTriggerPercent(0.0);
            return;
        }

        if(state == ShooterState.AMP) {
            shooter.setFlywheelVelocity(24, 0);
            if(OI.Inputs.wantsPlace.getAsBoolean()) shooter.setTriggerPercent(0.4);
            else shooter.setTriggerPercent(0);
            return;
        }

        shooter.setFlywheelVelocity(state.getFlywheelVelocity());
        shooter.setTriggerPercent(state.getTriggerPercent());
    }

    @Override
    public ShooterState getState(){
        return state;
    }

    @Override
    public boolean transitioning(){
        return !shooter.flywheelAtTargetVelocity();
    }

    @Override
    public boolean isDynamic() {
        return true;
    }

    private int shotsFired = 0;
    private void printShotData() {
        shotsFired++;
        System.out.println("Firing Shot!");
        System.out.println("Shots Fired: " + shotsFired);
        System.out.println("Wanted Shot Angle: " + aimPlanner.getWantedShotAngle());
        System.out.println("Measured Shot Angle: " + aimPlanner.getMeasuredShotAngle());
        System.out.println("Using Shoot-on-the-move: " + aimPlanner.isSotm());
        System.out.println("Localization Strategy: " + (aimPlanner.isSimpleLocalizer() ? "limelight tx/ty" : "pose estimation"));
    }

    public enum ShooterState{
        RAMP_DOWN(0.0,0.0),
        COAST (0.0, 0.0),
        INTAKE(-10.0,1.0), //NOTE: this should fold flat if the flywheel-side intake is out
        INDEX(-5.0,0.15),
        AMP(10.0,1.0), //to diverter
        AIM_LAYUP(40.0,0.0),
        AIM_PROTECTED(45.0,0.0),
        AIM_UNDER_STAGE(50.0,0.0),
        AIM_WINGLINE(60.0,0.0),
        AIM_CENTERLINE(60.0,0.0),
        AIM_SHUTTLE(50.0, 0.0),
        AUTO_AIM(0.0,0.0),
        SHOOT(0.0,1.0),
        INTAKE_N_AIM(0.0,0.0);

        private Double flywheelVelocity, triggerPercent;

        public Double getFlywheelVelocity(){
            return flywheelVelocity;
        }

        public Double getTriggerPercent(){
            return triggerPercent;
        }

        private ShooterState (Double flywheelVelocity, Double triggerPercentage){
            this.flywheelVelocity = flywheelVelocity;
            this.triggerPercent = triggerPercentage;
        }

        private ShooterState() {
            flywheelVelocity = Double.NaN;
            triggerPercent = Double.NaN;
        }
    }
}
