package frc.robot;

import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.robot.RobotStatemachine.SuperstructureState;
import frc.robot.planners.NoteTracker;
import frc.robot.planners.NoteTracker.NoteLocation;
import frc.robot.subsystems.Shooter;

/**
 * This class is used to map all the inputs to joystick buttons and axes.
 * all the inputs are defined as lambda functios to maximize flexibility and
 * make it easier to modify the inputs.
 */
public class OI {
    private static final Joystick driverJoystick = new Joystick(0);
    private static final Joystick operatorJoystick = new Joystick(1);
    public static class Swerve{
        public static final DoubleSupplier translation = () -> -driverJoystick.getRawAxis(5); //how fast the robot should be going forward
        public static final DoubleSupplier strafe = () -> -driverJoystick.getRawAxis(4); //how fast the robot should be going sideways
        public static final DoubleSupplier rotation = () -> -driverJoystick.getRawAxis(0); //how fast the robot should be rotating
        public static final DoubleSupplier heading = () -> (double) -driverJoystick.getPOV(); //the angle the robot should be facing
        public static final BooleanSupplier useHeading = () -> driverJoystick.getPOV() != -1; //whether the robot should use the heading above
        public static final BooleanSupplier isRobotCentric = () -> driverJoystick.getRawButton(7); //is forward always forward?
        public static final BooleanSupplier isLockIn = () -> driverJoystick.getRawButton(1); //make the wheels point in
        public static final BooleanSupplier isZeroOdometry = () -> driverJoystick.getRawButton(8); //zero the odometry
        public static final BooleanSupplier isFastVisionReset = () -> driverJoystick.getRawButton(9); //reset pose from vision quickly
        public static final BooleanSupplier isAttemptProperZero = () -> driverJoystick.getRawButton(10); //zero field centric properly
        public static final BooleanSupplier isOpenLoop = () -> false; //how hard should we try to actually follow the inputs (false = use the PID, which feels unnatural to me)
    }
    
    public static class Modes {
        //speaker mode automatically aims once the robot is past the center line (set wantsShoot to true and it should shoot automatically too)
        public static final BooleanSupplier wantsSpeakerMode = () -> operatorJoystick.getRawButton(1);

        //amp mode automatically hands off past a centain x position and then aligns once nearer to the goal
        public static final BooleanSupplier wantsAmpMode = () -> operatorJoystick.getRawButton(2);

        //climb mode prepares the robot to climb, and changes the joystick inputs to control the climber
        public static final BooleanSupplier wantsClimbMode = () -> operatorJoystick.getRawButton(3);

        //panic mode does nothing, litterally. (allows manual overrides without automations screwing things up)
        public static final BooleanSupplier wantsPanicMode = () -> operatorJoystick.getRawButton(4);
    }

    public static class Inputs {
        public static final BooleanSupplier wantsStow = () -> driverJoystick.getRawButton(5); //prevent decapitation
        public static final BooleanSupplier wantsPlace = () -> driverJoystick.getRawAxis(3) > 0.2; //general place button, varies by mode

        //climber states. these are all mutually exclusive, and will override each other if multiple are true.
        //they are also sticky, so the climber will stay in the state until another state is requested.
        public static final BooleanSupplier wantsAlign = () -> false; //aligns the robot to drive under the chain
        public static final BooleanSupplier wantsClimbExtend = () -> false; //extends the climber
        public static final BooleanSupplier wantsClimbRetract = () -> false; //retracts the climber fully to climb

        //shooter setpoints. these are all mutually exclusive, and will override each other if multiple are true.
        //they are not sticky, so the shooter only aims while the button is held down.
        public static final BooleanSupplier wantsAimLayup = () -> operatorJoystick.getPOV() == 0;
        public static final BooleanSupplier wantsAimProtected = () -> operatorJoystick.getPOV() == 180;
        public static final BooleanSupplier wantsAimUnderStage = () -> false;
        public static final BooleanSupplier wantsAimWingline = () -> false;
        public static final BooleanSupplier wantsAimCenterline = () -> false;
        public static final BooleanSupplier wantsAutoAim = () -> false;

        public static final BooleanSupplier wantsIntakeSource = () -> operatorJoystick.getPOV() == 90;

        //let the shooter get steezy.
        public static final BooleanSupplier enableShootWhileMoving = () -> driverJoystick.getRawButton(6);
    }
    
    public static class Overrides {
        /* MODE OVERRIDES */ //overrides the state requested by the mode
        public static final BooleanSupplier forceAim = () -> operatorJoystick.getRawButton(6);//force the robot into auto aim state
        public static final BooleanSupplier forceIntakeFront = () -> false; //force the robot to intake from the front
        public static final BooleanSupplier forceIntakeBack = () -> driverJoystick.getRawAxis(2) > 0.2 || operatorJoystick.getRawButton(7);
        public static final BooleanSupplier forceAmp = () -> false; //force the robot to go into the place amp state
        
        /* DIRECT OVERRIDES */ //directly sets the state of the subsystem
        public static final BooleanSupplier disableAutoHeading = () -> false; //disables the auto heading of the swerve
        public static final BooleanSupplier forceTrigger = () -> false; //force the trigger to run
        public static final BooleanSupplier eject = () -> operatorJoystick.getRawButton(5); //oopsie (very overridy) spins everything backwards
    }

    public static class ManualInputs {
        //Direct joystick inputs for critical systems. These are somewhat dangerous since they override most safety features.
        //They are persistent so the robot won't return to automated control until the reset button is pressed.
        //(well the trigger might but the pivot and climber won't)
        public static final DoubleSupplier jogTrigger = () -> -operatorJoystick.getRawAxis(1);
        public static final DoubleSupplier jogPivot = () -> -operatorJoystick.getRawAxis(3);
        public static final DoubleSupplier jogClimber = () -> -operatorJoystick.getRawAxis(3);
        public static final DoubleSupplier jogThing = () -> -operatorJoystick.getRawAxis(1);
  

        public static final BooleanSupplier resetManualInputs = () -> !operatorJoystick.getRawButton(8);
    }

    public static void updateRumble () {
        if(Shooter.getInstance().shotDetected()) {
            driverJoystick.setRumble(RumbleType.kBothRumble, 0.5);
            operatorJoystick.setRumble(RumbleType.kBothRumble, 0.5);
        } else if(RobotContainer.getInstance().getTeleopStatemachine().getState() == SuperstructureState.INTAKE_BACK && Shooter.getInstance().flywheelSwitchTripped()) {
            driverJoystick.setRumble(RumbleType.kBothRumble, 0.5);
            operatorJoystick.setRumble(RumbleType.kBothRumble, 0.5);
        }  else {
            driverJoystick.setRumble(RumbleType.kBothRumble, 0);
            operatorJoystick.setRumble(RumbleType.kBothRumble, 0);
        }
    }
}
