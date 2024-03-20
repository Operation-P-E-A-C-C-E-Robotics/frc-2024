package frc.lib.telemetry;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PneumaticsBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;

public class ControlSystemTelemetry {
    private static final NetworkTable controlSystemTable = NetworkTableInstance.getDefault().getTable("Control System");
    private static final NetworkTable roboRIOTable = controlSystemTable.getSubTable("RoboRIO");
    private static final NetworkTable pneumaticsTable = controlSystemTable.getSubTable("Pneumatics");
    private static final NetworkTable canTable = controlSystemTable.getSubTable("CAN");

    private static final DataLog log = DataLogManager.getLog();
    
    //rio
    private static final DoublePublisher loopTimePublisher = roboRIOTable.getDoubleTopic("RIO Loop Time").publish();
    private static final DoublePublisher commandsOverheadPublisher = roboRIOTable.getDoubleTopic("Software Run Time").publish();
    private static final BooleanPublisher brownoutPublisher = roboRIOTable.getBooleanTopic("Brownout").publish();
    private static final DoublePublisher batteryVoltagePublisher = roboRIOTable.getDoubleTopic("Battery Voltage").publish();
    private static final DoublePublisher brownoutVoltagePublisher = roboRIOTable.getDoubleTopic("Brownout Voltage").publish();  
    private static final DoublePublisher inputVoltagePublisher = roboRIOTable.getDoubleTopic("Input Voltage").publish();
    private static final DoublePublisher inputCurrentPublisher = roboRIOTable.getDoubleTopic("Input Current").publish();
    private static final DoubleLogEntry v3_3FaultsLog = new DoubleLogEntry(log, "Control System/RoboRIO/3.3v Rail Faults");
    private static final DoubleLogEntry v3_3CurrentLog = new DoubleLogEntry(log, "Control System/RoboRIO/3.3v Rail Current");
    private static final DoubleLogEntry v5FaultsLog = new DoubleLogEntry(log, "Control System/RoboRIO/5v Rail Faults");
    private static final DoubleLogEntry v5CurrentLog = new DoubleLogEntry(log, "Control System/RoboRIO/5v Rail Current");
    private static final DoubleLogEntry v6FaultsLog = new DoubleLogEntry(log, "Control System/RoboRIO/6v Rail Faults");
    private static final DoubleLogEntry v6CurrentLog = new DoubleLogEntry(log, "Control System/RoboRIO/6v Rail Current");
    private static final DoubleLogEntry cpuTempLog = new DoubleLogEntry(log, "Control System/RoboRIO/CPU Temperature");
   
    private static final BooleanLogEntry outputsEnabledLog = new BooleanLogEntry(log, "Control System/RoboRIO/Outputs Enabled");

    //can
    private static final DoublePublisher canBusUtilization = canTable.getDoubleTopic("Bus Utilization").publish();
    private static final DoublePublisher canBusOffCount = canTable.getDoubleTopic("Bus Off Count").publish();
    private static final DoublePublisher canRxErrorCount = canTable.getDoubleTopic("Rx Error Count").publish();
    private static final DoublePublisher canTxErrorCount = canTable.getDoubleTopic("Tx Error Count").publish();
    private static final DoublePublisher canTxFullCount = canTable.getDoubleTopic("Tx Full Count").publish();

    //pneumatics
    private static final BooleanPublisher compressorEnabled = pneumaticsTable.getBooleanTopic("Compressor Enabled").publish();
    private static final DoublePublisher pneumaticPressure = pneumaticsTable.getDoubleTopic("Pneumatic Pressure").publish();
    private static final StringPublisher pneumaticSolenoidStates = pneumaticsTable.getStringTopic("Solenoid States").publish();

    private static Timer time = new Timer();

    public static void update(PneumaticsBase pcm, double schedulerTime) {
        loopTimePublisher.accept(time.get());
        commandsOverheadPublisher.accept(schedulerTime);
        time.reset();
        time.start();

        brownoutPublisher.accept(RobotController.isBrownedOut());
        batteryVoltagePublisher.accept(RobotController.getBatteryVoltage());
        brownoutVoltagePublisher.accept(RobotController.getBrownoutVoltage());
        inputVoltagePublisher.accept(RobotController.getInputVoltage());
        inputCurrentPublisher.accept(RobotController.getInputCurrent());
        v3_3FaultsLog.append(RobotController.getFaultCount3V3());
        v3_3CurrentLog.append(RobotController.getVoltage3V3());
        v5FaultsLog.append(RobotController.getFaultCount5V());
        v5CurrentLog.append(RobotController.getVoltage5V());
        v6FaultsLog.append(RobotController.getFaultCount6V());
        v6CurrentLog.append(RobotController.getVoltage6V());
        cpuTempLog.append(RobotController.getCPUTemp());

        outputsEnabledLog.append(RobotController.isSysActive());

        var canStatus = RobotController.getCANStatus();

        canBusUtilization.accept(canStatus.percentBusUtilization);
        canBusOffCount.accept(canStatus.busOffCount);
        canRxErrorCount.accept(canStatus.receiveErrorCount);
        canTxErrorCount.accept(canStatus.transmitErrorCount);
        canTxFullCount.accept(canStatus.txFullCount);


        if(pcm != null){
            compressorEnabled.accept(pcm.getCompressor());
            pneumaticPressure.accept(pcm.getPressure(0));
            pneumaticSolenoidStates.accept(Integer.toBinaryString(pcm.getSolenoids()));
        } else {
            compressorEnabled.accept(false);
            pneumaticPressure.accept(0.0);
            pneumaticSolenoidStates.accept("Pneumatics-free robot :)");
        }

        RobotController.getBatteryVoltage();
    }
}
