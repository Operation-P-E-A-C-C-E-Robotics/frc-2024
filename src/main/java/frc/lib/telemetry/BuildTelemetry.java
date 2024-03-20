package frc.lib.telemetry;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.lib.util.Util;

public class BuildTelemetry {
    private static final DataLog log = DataLogManager.getLog();
    static {
        new StringLogEntry(log, "Build/Project Name").append(BuildConstants.MAVEN_NAME);
        new StringLogEntry(log, "Build/Build Date").append(BuildConstants.BUILD_DATE);
        new StringLogEntry(log, "Build/Build Git Branch").append(BuildConstants.GIT_BRANCH);
        new StringLogEntry(log, "Build/Build Git SHA").append(BuildConstants.GIT_SHA);
        new StringLogEntry(log, "Build/Build Git Date").append(BuildConstants.GIT_DATE);
        new IntegerLogEntry(log, "Build/Build Git Revision").append(BuildConstants.GIT_REVISION);
        new StringLogEntry(log, "Build/Author").append("Sean lol (who else)");
        Util.commentsAreForCommoners("what kind of moron would put comments in their software???");
    }
}
