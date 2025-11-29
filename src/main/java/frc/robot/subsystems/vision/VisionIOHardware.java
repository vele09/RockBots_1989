package frc.robot.subsystems.vision;

///////////////////////////////////////////////////////////////////////////////
// Description: Clase con los metodos para administrar las salidas y entradas del subsistema usando el robot real.
// Authors: Paola, Pablo
// Notes:
//  - 
///////////////////////////////////////////////////////////////////////////////

import frc.robot.Constants;
import lib.limelight.LimelightHelpers;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionIOHardware implements VisionIO {

    // Variables para guardar las referencias a las networktables de las limelights
    NetworkTable tableB = NetworkTableInstance.getDefault().getTable(Constants.Vision.Back);
    NetworkTable tableFL = NetworkTableInstance.getDefault().getTable(Constants.Vision.FrontL);
    NetworkTable tableFR = NetworkTableInstance.getDefault().getTable(Constants.Vision.FrontR);

    // Funcion para leer los inputs reales del robot
    @Override
    public void readInputs(VisionInputs inputs, Pose3d robotPose) {
        // Validamos la camara tenga una lectura valida
        inputs.cameraSeesTarget_B = tableB.getEntry("tv").getDouble(0) == 1.0;
        if (inputs.cameraSeesTarget_B) {
            // Save the primary tag id (el principal usado para sacar las mediciones)
            inputs.primaryTagId_B = (int) LimelightHelpers.getFiducialID(Constants.Vision.Back);
            // Get robot position using megatag
            var megatag_B = LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.Vision.Back);
            inputs.cameraMegatagPoseEstimate_B = MegatagPoseEstimate.fromLimelight(megatag_B);
            // Get robot position using megatag2
            var megatag2_B = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.Vision.Back);
            inputs.cameraMegatag2PoseEstimate_B = MegatagPoseEstimate.fromLimelight(megatag2_B);
            // Get filudials value used for megatag read
            inputs.cameraFiducialObservationsMt_B = FiducialObservation.fromLimelight(megatag_B.rawFiducials);
            // Get filudials value used for megatag2 read
            inputs.cameraFiducialObservationsMt2_B = FiducialObservation.fromLimelight(megatag2_B.rawFiducials);
            // Get taget pose in camera space
            inputs.targetPose_B = toPose3D(LimelightHelpers.getTargetPose_CameraSpace(Constants.Vision.Back));
        }
        // Update robot angle in limelight tables
        LimelightHelpers.SetRobotOrientation(Constants.Vision.Back, Math.toDegrees(robotPose.getRotation().getZ()), 0, 0, 0, 0, 0);
        
         // Validamos la camara tenga una lectura valida
        inputs.cameraSeesTarget_FL = tableFL.getEntry("tv").getDouble(0) == 1.0;
        if (inputs.cameraSeesTarget_FL) {
            // Save the primary tag id (el principal usado para sacar las mediciones)
            inputs.primaryTagId_FL = (int) LimelightHelpers.getFiducialID(Constants.Vision.FrontL);
            // Get robot position using megatag
            var megatag_FL = LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.Vision.FrontL);
            inputs.cameraMegatagPoseEstimate_FL = MegatagPoseEstimate.fromLimelight(megatag_FL);
            // Get robot position using megatag2
            var megatag2_FL = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.Vision.FrontL);
            inputs.cameraMegatag2PoseEstimate_FL = MegatagPoseEstimate.fromLimelight(megatag2_FL);
            // Get filudials value used for megatag read
            inputs.cameraFiducialObservationsMt_FL = FiducialObservation.fromLimelight(megatag_FL.rawFiducials);
            // Get filudials value used for megatag2 read
            inputs.cameraFiducialObservationsMt2_FL = FiducialObservation.fromLimelight(megatag2_FL.rawFiducials);
            // Get taget pose in camera space
            inputs.targetPose_FL = toPose3D(LimelightHelpers.getTargetPose_CameraSpace(Constants.Vision.FrontL));
        }
        // Update robot angle in limelight tables
        LimelightHelpers.SetRobotOrientation(Constants.Vision.FrontL, Math.toDegrees(robotPose.getRotation().getZ()), 0, 0, 0, 0, 0);
        
        // Validamos la camara tenga una lectura valida
        inputs.cameraSeesTarget_FR = tableFR.getEntry("tv").getDouble(0) == 1.0;
        if (inputs.cameraSeesTarget_FR) {
            // Save the primary tag id (el principal usado para sacar las mediciones)
            inputs.primaryTagId_FR = (int) LimelightHelpers.getFiducialID(Constants.Vision.FrontR);
            // Get robot position using megatag
            var megatag_FR = LimelightHelpers.getBotPoseEstimate_wpiBlue(Constants.Vision.FrontR);
            inputs.cameraMegatagPoseEstimate_FR = MegatagPoseEstimate.fromLimelight(megatag_FR);
            // Get robot position using megatag2
            var megatag2_FR = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(Constants.Vision.FrontR);
            inputs.cameraMegatag2PoseEstimate_FR = MegatagPoseEstimate.fromLimelight(megatag2_FR);
            // Get filudials value used for megatag read
            inputs.cameraFiducialObservationsMt_FR = FiducialObservation.fromLimelight(megatag_FR.rawFiducials);
            // Get filudials value used for megatag2 read
            inputs.cameraFiducialObservationsMt2_FR = FiducialObservation.fromLimelight(megatag2_FR.rawFiducials);
            // Get taget pose in camera space
            inputs.targetPose_FR = toPose3D(LimelightHelpers.getTargetPose_CameraSpace(Constants.Vision.FrontR));
        }
        // Update robot angle in limelight tables
        LimelightHelpers.SetRobotOrientation(Constants.Vision.FrontR,Math.toDegrees(robotPose.getRotation().getZ()), 0, 0, 0, 0, 0);
    }

    // Function to convert the Limelight's double array to a Pose3d in robot space
    private Pose3d toPose3D(double[] inData) {
        if (inData.length < 6) {
            return new Pose3d();
        }
        // Convert translation from camera space to robot space
        Translation3d robotTranslation = new Translation3d(
            inData[2],             // Z (camera forward) becomes X (robot forward)
            -inData[0],            // X (camera right) becomes -Y (robot left)
            -inData[1]             // Y (camera down) becomes -Z (robot up)
        );
        // Convert rotation from camera space to robot space
        Rotation3d robotRotation = new Rotation3d(
            Units.degreesToRadians(inData[5]),  // Roll (camera) becomes Pitch (robot)
            Units.degreesToRadians(inData[3]),  // Pitch (camera) becomes Roll (robot)
            -Units.degreesToRadians(inData[4])  // Yaw (camera) becomes -Yaw (robot)
        );
        return new Pose3d(robotTranslation, robotRotation);
    }
}