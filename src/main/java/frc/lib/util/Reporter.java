package frc.lib.util;

import java.util.HashMap;

import com.ctre.phoenix6.StatusCode;
import com.revrobotics.REVLibError;

import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.LinkedList;

public class Reporter {
    private static LinkedList<String> meesageBuffer = new LinkedList<>();
    private static int messageLimit = 10;

    /**
     * Reports a message to the dashboard
     * @param message the message to report
     * @param error prints the message to the driver station as an error if true
     */
    public static void report(String message, boolean error) {
        meesageBuffer.add(message);
        if(meesageBuffer.size() > messageLimit) {
            meesageBuffer.removeFirst();
        }
        if (error) DriverStation.reportError(message, false);
    }

    /**
     * Reports a message to the dashboard
     * @param message the message to report
     */
    public static void report(String message) {
        report(message, false);
    }

    /**
     * Reports a CTRE status code to the driver station
     * if it is an error code.
     * @param status the status code to report
     * @param message the message to report with the status code
     */
    public static void report (StatusCode status, String message) {
        if (!status.isOK()) {
            report(message + ": " + status.getDescription(), true);
        }
    }

    /**
     * Reports a REVLib status code to the driver station
     * if it is an error code.
     * @param status the status code to report
     * @param message the message to report with the status code
     */
    public static void report (REVLibError status, String message) {
        if (status != REVLibError.kOk) {
            report(message + ": " + status.toString(), true);
        }
    }

    private static HashMap<String, StringLogEntry> logEntries = new HashMap<>();

    /**
     * Logs a CTRE status code to the data logger
     * if it is an error code.
     * @param status the status code to log
     * @param message the message to log with the status code
     */
    public static void log (StatusCode status, String message) {
        if (!status.isOK()) {
            if(logEntries.containsKey(message)){
                logEntries.get(message).append(status.getDescription());
            } else {
                logEntries.put(message, new StringLogEntry(DataLogManager.getLog(), "Reporter/CTRE errors/" + message));
            }
        }
    }

    /**
     * Logs a REVLib status code to the data logger
     * if it is an error code.
     * @param status the status code to log
     * @param message the message to log with the status code
     */
    public static void log (REVLibError status, String message) {
        if (status != REVLibError.kOk) {
            if(logEntries.containsKey(message)){
                logEntries.get(message).append(status.toString());
            } else {
                logEntries.put(message, new StringLogEntry(DataLogManager.getLog(), "Reporter/REV errors/" + message));
            }
        }
    }

    public static void putDashboard() {
        String message = "";
        for(String s : meesageBuffer) {
            message += s + "\n";
        }
        SmartDashboard.putString("Reporter Messages", message);
    }
}
