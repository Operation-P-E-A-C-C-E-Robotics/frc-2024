package frc.robot;

import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.telemetry.MultiTracers;
import frc.lib.telemetry.StrategyTelemetry;
import frc.robot.auto.Autonomous;
import frc.robot.auto.Autonomous.TimedAuto;
import frc.robot.planners.AimPlanner;
import frc.robot.planners.MotionPlanner;
import frc.robot.planners.NoteTracker;
import frc.robot.statemachines.ClimberStatemachine;
import frc.robot.statemachines.ThingStatemachine;
import frc.robot.statemachines.FlywheelIntakeStatemachine;
import frc.robot.statemachines.PivotStatemachine;
import frc.robot.statemachines.ShooterStatemachine;
import frc.robot.statemachines.SwerveStatemachine;
import frc.robot.statemachines.TriggerIntakeStatemachine;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Thing;
import frc.robot.subsystems.FlywheelIntake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.TriggerIntake;

/**
 * The container for all the subsystems and state machines of the robot.
 * This is where the bulk of the robot logic is stored and updated.
 */
public class RobotContainer {
    private static RobotContainer instance = null;

    /* SUBSYSTEMS */
    private final Swerve swerve = Swerve.getInstance();
    private final FlywheelIntake flywheelIntake = FlywheelIntake.getInstance();
    private final TriggerIntake triggerIntake = TriggerIntake.getInstance();
    private final Pivot pivot = Pivot.getInstance();
    private final Shooter shooter = Shooter.getInstance();
    private final Thing thing = Thing.getInstance();
    private final Climber climber = Climber.getInstance();

    /* PLANNERS */
    private final MotionPlanner motionPlanner = new MotionPlanner();

    private final AimPlanner aimPlanner = new AimPlanner(
        () -> swerve.getPose(),
        () -> swerve.getChassisSpeeds(),
        OI.Inputs.enableShootWhileMoving
    );

    /* STATE MACHINES */
    private final SwerveStatemachine swerveStatemachine = new SwerveStatemachine(swerve, aimPlanner);
    private final FlywheelIntakeStatemachine flywheelIntakeStatemachine = new FlywheelIntakeStatemachine(flywheelIntake, motionPlanner);
    private final TriggerIntakeStatemachine triggerIntakeStatemachine = new TriggerIntakeStatemachine(triggerIntake, motionPlanner);
    private final PivotStatemachine pivotStatemachine = new PivotStatemachine(pivot, aimPlanner, motionPlanner);
    private final ShooterStatemachine shooterStatemachine = new ShooterStatemachine(shooter, aimPlanner, this::readyToShoot);
    private final ThingStatemachine thingStatemachine = new ThingStatemachine(thing, motionPlanner);
    private final ClimberStatemachine climberStatemachine = new ClimberStatemachine(climber, () -> swerve.getGyroAngle().getX());

    private final RobotStatemachine teleopStatemachine = new RobotStatemachine(
        swerveStatemachine,
        flywheelIntakeStatemachine,
        triggerIntakeStatemachine,
        shooterStatemachine,
        pivotStatemachine,
        thingStatemachine,
        climberStatemachine,
        motionPlanner,
        aimPlanner
    );

    private SendableChooser<TimedAuto> autoChooser = new SendableChooser<>();

    private RobotContainer() {
        autoChooser.setDefaultOption("do nothing", Autonomous.doNothing);
        autoChooser.addOption("LAYUP", Autonomous.layupOnly);
        autoChooser.addOption("SHOOT", Autonomous.shootOnly);
        autoChooser.addOption("START 1 + WING 1", Autonomous.twoNoteStageSide);
        autoChooser.addOption("START 2 + WING 2", Autonomous.twoNoteCenter);
        autoChooser.addOption("START 3 + WING 3", Autonomous.twoNoteAmpSide);
        autoChooser.addOption("START 3 + WING 3 + WING 2 + WING 1", Autonomous.fourNote);
        autoChooser.addOption("START 3 + WING 3 + CENTER 5", Autonomous.start3ThreeNoteCenter5);
        autoChooser.addOption("START 1 + WING 1 + CENTER 2", Autonomous.start1ThreeNoteCenter2);
        autoChooser.addOption("START 1 + WING 1 + CENTER 3", Autonomous.start1ThreeNoteCenter3);
        autoChooser.addOption("(new) START 2 + WING 2 + CENTER 4", Autonomous.start2ThreeNoteCenter4);
        autoChooser.addOption("(new) START 2 + WING 2 + CENTER 4 + CENTER 3", Autonomous.start2FourNote);
        autoChooser.addOption("(new) START 3 + WING 3 + CENTER 4)", Autonomous.start3ThreeNoteCenter4);
        autoChooser.addOption("NEW CENTER 3 + CENTER 4", Autonomous.newAutoThing);
        autoChooser.addOption("DEFENCE 1", Autonomous.defence1);
        autoChooser.addOption("DEFENCE 2", Autonomous.defence2);
        autoChooser.addOption("DEFENCE 3", Autonomous.defence3);
        autoChooser.addOption("DEFENCE 4", Autonomous.defence4L);
        autoChooser.addOption("DEFENCE 4 (start amp)", Autonomous.defence4R);
        autoChooser.addOption("DEFENCE 5 (start amp)", Autonomous.defence5R);

        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    public static RobotContainer getInstance() {
        if(instance == null) instance = new RobotContainer();
        return instance;
    }

    Timer readyTimer = new Timer();
    public boolean kindaReadyToShoot(){
        if(!shooter.flywheelAtTargetVelocity()) return false;
        if(!pivot.atSetpoint()) return false;
        if(!swerveStatemachine.transitioning()) return false;
        if((swerve.getChassisSpeeds().vxMetersPerSecond > 0.005 && swerve.getChassisSpeeds().vyMetersPerSecond > 0.005) && !OI.Inputs.enableShootWhileMoving.getAsBoolean()) return false;
        // if(OI.Inputs.wantsPlace.getAsBoolean()) return false;
        if(swerve.getEyes().getOdometryError() > 25) return false;
        return true;
        // return OI.Inputs.wantsPlace.getAsBoolean();
    }

    public boolean readyToShoot(){
        // if(OI.Inputs.wantsPlace.getAsBoolean()) {
        //     // if(OI.Inputs.enableShootWhileMoving.getAsBoolean()) return true;
        //     return false;
        // } else if (OI.Inputs.enableShootWhileMoving.getAsBoolean()) {
        //     return true;
        // }
        // if(kindaReadyToShoot()) {
        //     readyTimer.start();
        // }
        // if(readyTimer.get() > aimPlanner.getDistanceToTarget()/6){
        //     readyTimer.stop();
        //     readyTimer.reset();
        //     return true;
        // }
        // return false;
        return OI.Inputs.wantsPlace.getAsBoolean();
    }

    public double getOdometryError() {
        return swerve.getEyes().getOdometryError();
    }

    public RobotStatemachine getTeleopStatemachine() {
        return teleopStatemachine;
    }

    public void zeroAutoHeading() {
        swerveStatemachine.zeroAutoHeading();
    }

    public double getDistanceToTarget(){
        return aimPlanner.getDistanceToTarget();
    }

    /**
     * The main update loop of the robot.
     * This is called periodically by the main robot class.
     * It updates the supersystem state, planners, and state machines.
     */
    public void run() {
        MultiTracers.trace("RobotContainer::run", "RobotContainer::run");
        /* UPDATE PLANNERS */
        motionPlanner.update();
        MultiTracers.trace("RobotContainer::run", "motionPlanner.update");
        aimPlanner.update();
        MultiTracers.trace("RobotContainer::run", "aimPlanner.update");
        StrategyTelemetry.update();
        MultiTracers.trace("RobotContainer::run", "StrategyTelemetry.update");

        /* TEST DASHBOARD */
        if(RobotState.isTest()) {
            Swerve.getInstance().characterizeSteer();
            return;
        }

        /* LET THE DRIVERS COOK */
        if(RobotState.isTeleop()) {
            // update with the state the driver wants
            teleopStatemachine.requestState(TeleopInputs.getInstance().getWantedTeleopState());
            MultiTracers.trace("RobotContainer::run", "teleopStatemachine.requestState");
            swerveStatemachine.requestState(TeleopInputs.getInstance().getWantedSwerveState());
            MultiTracers.trace("RobotContainer::run", "swerveStatemachine.requestState");

            //run all the state machines
            teleopStatemachine.update();
            MultiTracers.trace("RobotContainer::run", "teleopStatemachine.update");
            swerveStatemachine.update();
            MultiTracers.trace("RobotContainer::run", "swerveStatemachine.update");
            triggerIntakeStatemachine.update();
            MultiTracers.trace("RobotContainer::run", "triggerIntakeStatemachine.update");
            pivotStatemachine.update();
            MultiTracers.trace("RobotContainer::run", "pivotStatemachine.update");
            shooterStatemachine.update();
            MultiTracers.trace("RobotContainer::run", "shooterStatemachine.update");
            thingStatemachine.update();
            MultiTracers.trace("RobotContainer::run", "diverterStatemachine.update");
            climberStatemachine.update();
            MultiTracers.trace("RobotContainer::run", "climberStatemachine.update");
            
            // handle driver overrides
            TeleopInputs.getInstance().handleOverrides();

            OI.updateRumble();

            MultiTracers.trace("RobotContainer::run", "TeleopInputs.getInstance().handleOverrides");
            SmartDashboard.putString("Note Location", NoteTracker.getLocation().name());
        }

        /* AUTONOMOUS */
        if(RobotState.isAutonomous()) {
            autoChooser.getSelected().run(teleopStatemachine);
            teleopStatemachine.update();
            swerveStatemachine.update();
            // flywheelIntakeStatemachine.update();
            triggerIntakeStatemachine.update();
            pivotStatemachine.update();
            shooterStatemachine.update();
        } else {
            autoChooser.getSelected().reset();
        }
        NoteTracker.update(teleopStatemachine.getState());

        MultiTracers.print("RobotContainer::run (end)");
    }

    public void resetAuto(){
        autoChooser.getSelected().reset();
    }
}
