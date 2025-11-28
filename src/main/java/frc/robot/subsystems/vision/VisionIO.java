package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Pose3d;


public interface VisionIO {

    @AutoLog
    class VisionInputs {
        // Back camera
        public boolean cameraSeesTarget_B;
        public Pose3d targetPose_B = new Pose3d();
        public int primaryTagId_B;
        public MegatagPoseEstimate cameraMegatagPoseEstimate_B = new MegatagPoseEstimate();
        public MegatagPoseEstimate cameraMegatag2PoseEstimate_B = new MegatagPoseEstimate();
        public FiducialObservation[] cameraFiducialObservationsMt_B = new FiducialObservation[0];
        public FiducialObservation[] cameraFiducialObservationsMt2_B = new FiducialObservation[0];
        // Front left camera
        public boolean cameraSeesTarget_FL;
        public Pose3d targetPose_FL = new Pose3d();
        public int primaryTagId_FL;
        public MegatagPoseEstimate cameraMegatagPoseEstimate_FL = new MegatagPoseEstimate();
        public MegatagPoseEstimate cameraMegatag2PoseEstimate_FL = new MegatagPoseEstimate();
        public FiducialObservation[] cameraFiducialObservationsMt_FL = new FiducialObservation[0];
        public FiducialObservation[] cameraFiducialObservationsMt2_FL = new FiducialObservation[0];
        // Front right camera
        public boolean cameraSeesTarget_FR;
        public Pose3d targetPose_FR = new Pose3d();
        public int primaryTagId_FR;
        public MegatagPoseEstimate cameraMegatagPoseEstimate_FR = new MegatagPoseEstimate();
        public MegatagPoseEstimate cameraMegatag2PoseEstimate_FR = new MegatagPoseEstimate();
        public FiducialObservation[] cameraFiducialObservationsMt_FR = new FiducialObservation[0];
        public FiducialObservation[] cameraFiducialObservationsMt2_FR = new FiducialObservation[0];
    }

    // Funcion para leer los inputs reales del robot
    default void readInputs(VisionInputs inputs, Pose3d robotPose){}

    // Funcion usada para actualizar los inputs en la simulacion
    default void update(final VisionInputs inputs) { }

}