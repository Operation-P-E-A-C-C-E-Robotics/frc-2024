package frc.lib.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public interface ApriltagCamera {
    public VisionResults getLatestResults(Pose2d referencePose);
    public double getTrust();

    public static class ApriltagLimelight implements ApriltagCamera {
        private final String name;
        private final double trust;
        private VisionResults last = new VisionResults(new Pose3d(), 0, 0, false);

        public ApriltagLimelight (String name, double trust) {
            this.name = name;
            this.trust = trust;
        }

        public VisionResults getLatestResults(Pose2d referencePose){
            var result = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
            last = new VisionResults(
                result.pose,
                result.tagCount,
                result.timestampSeconds,
                result.timestampSeconds > last.getTimestamp()
            );
            return last;
        }

        public double getTrust(){
            return trust;
        }
    }

    public static class ApriltagPhotonvision implements ApriltagCamera {
        private final PhotonCamera camera;
        private final PhotonPoseEstimator poseEstimator;
        private VisionResults last = new VisionResults(new Pose3d(), 0, 0, false);
        private final double trust;

        public ApriltagPhotonvision(String name, Transform3d robotToCamera, AprilTagFieldLayout layout, double trust) {
            camera = new PhotonCamera(name);
            poseEstimator = new PhotonPoseEstimator(layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, robotToCamera);
            poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
            this.trust = trust;
        }

        @Override
        public VisionResults getLatestResults(Pose2d referencePose) {
            poseEstimator.setReferencePose(referencePose);
            var result = poseEstimator.update();
            if(result.isEmpty()) return last.stale();
            last = new VisionResults(
                result.get().estimatedPose,
                result.get().targetsUsed.size(),
                result.get().timestampSeconds,
                result.get().timestampSeconds > last.getTimestamp()
            );

            return last;
        }

        public PhotonCamera getCamera(){
            return camera;
        }

        @Override
        public double getTrust() {
            return trust;
        }
    }

    public static class VisionResults {
        private final Pose3d pose;
        private final int numTags;
        private final double timestamp;
        private final boolean hasUpdated;
        private final boolean hasTags;
        private final boolean hasAverageDistance;
        private final double averageDistance;

        public VisionResults(Pose3d pose, int numTags, double timestamp, boolean hasUpdated){
            this.pose = pose;
            this.numTags = numTags;
            this.timestamp = timestamp;
            this.hasUpdated = hasUpdated;
            this.hasTags = numTags > 0;
            this.hasAverageDistance = false;
            this.averageDistance = 0;
        }

        public VisionResults(Pose3d pose, int numTags, double timestamp, boolean hasUpdated, double averageDistance){
            this.pose = pose;
            this.numTags = numTags;
            this.timestamp = timestamp;
            this.hasUpdated = hasUpdated;
            this.hasTags = numTags > 0;
            this.hasAverageDistance = true;
            this.averageDistance = averageDistance;
        }

        public VisionResults(Pose2d pose, int numTags, double timestamp, boolean hasUpdated){
            this.pose = new Pose3d(
                pose.getTranslation().getX(), 
                pose.getTranslation().getY(), 
                0, 
                new Rotation3d(
                    0,
                    0,
                    pose.getRotation().getRadians()
                )
            );
            this.numTags = numTags;
            this.timestamp = timestamp;
            this.hasUpdated = hasUpdated;
            this.hasTags = numTags > 0;
            this.hasAverageDistance = false;
            this.averageDistance = 0;
        }

        public VisionResults(Pose2d pose, int numTags, double timestamp, boolean hasUpdated, double averageDistance){
            this.pose = new Pose3d(
                pose.getTranslation().getX(), 
                pose.getTranslation().getY(), 
                0, 
                new Rotation3d(
                    0,
                    0,
                    pose.getRotation().getRadians()
                )
            );
            this.numTags = numTags;
            this.timestamp = timestamp;
            this.hasUpdated = hasUpdated;
            this.hasTags = numTags > 0;
            this.hasAverageDistance = true;
            this.averageDistance = averageDistance;
        }

        public Pose3d getPose(){
            return pose;
        }

        public Pose2d getPose2d(){
            return new Pose2d(pose.getTranslation().getX(), pose.getTranslation().getY(), new Rotation2d(pose.getRotation().getZ()));
        }

        public int getNumTags(){
            return numTags;
        }

        public double getTimestamp(){
            return timestamp;
        }

        public boolean hasUpdated(){
            return hasUpdated;
        }

        public boolean hasTags(){
            return hasTags;
        }

        public boolean hasAverageDistance(){
            return hasAverageDistance;
        }

        public double getAverageDistance(){
            return averageDistance;
        }

        public VisionResults stale() {
            return new VisionResults(pose, numTags, timestamp, false);
        }

        public String toString(){
            return "VisionResults: pose: " + pose + ", numTags: " + numTags + ", timestamp: " + timestamp + ", hasUpdated: " + hasUpdated + ", hasTags: " + hasTags + ", hasAverageDistance: " + hasAverageDistance + ", averageDistance: " + averageDistance;
        }
    }
}
