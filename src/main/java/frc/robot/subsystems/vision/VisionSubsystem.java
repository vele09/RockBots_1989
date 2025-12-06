package frc.robot.subsystems.vision;

import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import frc.robot.FieldConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.DrivePidDrive;
import lib.time.RobotTime;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    
    // Clase para juntar toda la informacion obetnida de una medicion de la camara
    private class CameraMeasure {
        // Identificador de la camara
        public int cameraIndex = 0;
        // Variable para saber si uso megatag (1) o megatag2 (2)
        public int model=0;
        // Pose returned by the camara
        public Pose2d pose = null;
        // Confidence level (para ajustar que tanto caso le hace la camara contra los encoders para ajustar la odometria)
        public double confidence = 0;
        // Measure timestamp (para evitar procesar medidas repetidas)
        public double timestamp = 0;
        // Average distance to targets (used to estimate pose)
        public double avgDistance = 0;
        // Primary tag id
        public int primaryTagId = 0;

        public CameraMeasure(int cameraIndex,int model,Pose2d pose, double confidence, double avgDistance, double timestamp, int primaryTagId){
            this.cameraIndex=cameraIndex;
            this.model=model;
            this.pose = pose;
            this.confidence = confidence;
            this.avgDistance = avgDistance;
            this.timestamp = timestamp;
            this.primaryTagId = primaryTagId;
        }
    }
    
    // Advantage kit inputs object
    private final VisionInputsAutoLogged inputs = new VisionInputsAutoLogged();
    // Inputs/Outputs object
    private final VisionIO io;
    // Variable para guardar la referencia al robot container
    private final RobotContainer container;

    // Subsystem constructor
    public VisionSubsystem(VisionIO io, final RobotContainer container) {
        // Save inputs and outputs object
        this.io = io;
        // Save robotcontainer reference
        this.container = container;
    }

    @Override
    public void periodic() {
        double timestamp = RobotTime.getTimestampSeconds();
          // Get subsystem sensor inputs (from real robot)
        io.readInputs(inputs,getRobotPose());
        // Send inputs log to advantage kit (processInputs records in real robot, and in replay inject values)
        Logger.processInputs("Vision", inputs);
        // Update inputs (for simulation)
        io.update(inputs);
        // Actualizamos la odometria usando los valores de las camaras
        updateDriveOdometry();
        Logger.recordOutput("Vision/latencyPeriodicSec", RobotTime.getTimestampSeconds() - timestamp);
    }

    // Funcion para actualizar la odometria del robot utilizando el mejor valor de todas las camaras
    private void updateDriveOdometry() {
        // If robot is rotating fast ignore camera measures
        if (Math.abs(container.getDriveSubsystem().getInputs().yawAngleRate) > 250) {
            outputNullTagsPoses(container.getDriveSubsystem().getPose());
            return;
        }
        // Get camera measures
        CameraMeasure camFL = getCameraMeasurement(1, inputs.cameraSeesTarget_FL, inputs.cameraMegatagPoseEstimate_FL, inputs.cameraFiducialObservationsMt_FL, 
                                                inputs.cameraMegatag2PoseEstimate_FL, inputs.cameraFiducialObservationsMt2_FL,inputs.primaryTagId_FL);
        CameraMeasure camFR = getCameraMeasurement(2, inputs.cameraSeesTarget_FR, inputs.cameraMegatagPoseEstimate_FR, inputs.cameraFiducialObservationsMt_FR, 
                                                inputs.cameraMegatag2PoseEstimate_FR, inputs.cameraFiducialObservationsMt2_FR,inputs.primaryTagId_FR);
        CameraMeasure camB = getCameraMeasurement(3, inputs.cameraSeesTarget_B, inputs.cameraMegatagPoseEstimate_B, inputs.cameraFiducialObservationsMt_B, 
                                                inputs.cameraMegatag2PoseEstimate_B, inputs.cameraFiducialObservationsMt2_B,inputs.primaryTagId_B);
        // Select the best value from all cameras
        CameraMeasure bestCam = selectBestCamera(camFL, camFR, camB);
        if (bestCam != null) {
            handleCameraMeasurement(bestCam, inputs);
        } else {
            outputNullTagsPoses(container.getDriveSubsystem().getPose());
        }
    }

    // Funcion para obtener la medicion de la camara usando el algoritmo correcto
    private CameraMeasure getCameraMeasurement(int cameraIndex, boolean seesTarget, MegatagPoseEstimate poseEstimateMT, FiducialObservation[] fiducialsMT, 
                                            MegatagPoseEstimate poseEstimateMT2, FiducialObservation[] fiducialsMT2, int primaryTagId) {
        // Validate camera sees something
        if (!seesTarget) return null;
        // Validate if should use megatag measure
        double confidence = shouldUseMegatag(poseEstimateMT, fiducialsMT);
        if (confidence > 0.01) {
            return new CameraMeasure(cameraIndex,1, poseEstimateMT.fieldToRobot, confidence, tagsAverageDistance(fiducialsMT), poseEstimateMT.timestampSeconds,primaryTagId);
        }
        // Validate if should use megatag2 measure
        confidence = shouldUseMegatag2(poseEstimateMT2, fiducialsMT2);
        if (confidence > 0.01) {
            return new CameraMeasure(cameraIndex,2, poseEstimateMT2.fieldToRobot, confidence, tagsAverageDistance(fiducialsMT2), poseEstimateMT2.timestampSeconds,primaryTagId);
        }
        return null;
    }

    // Funcion para seleccionar el valor de la mejor camara en base a cual es la mas cercana a los tags
    private CameraMeasure selectBestCamera(CameraMeasure... cameras) {
        CameraMeasure bestCamera = null;
        for (CameraMeasure cam : cameras) {
            if (cam != null && (bestCamera == null || cam.avgDistance < bestCamera.avgDistance)) {
                bestCamera = cam;
            }
        }
        return bestCamera;
    }

    // Make needed updates using camera measure
    private void handleCameraMeasurement(CameraMeasure cam, VisionInputsAutoLogged inputs) {
        // Pid camera measures are only updated using front cameras
        if(cam.cameraIndex<3){
            updatePidVisionMeasure(cam, inputs);
        }
        // Update odometry using camera measure
        container.getDriveSubsystem().AddVisionMeasurement(cam.pose, cam.confidence, cam.timestamp);
        // Update logs outputs
        outputTagsPoses(cam, inputs);
    }

    // Funcion para saber si debemos usar el algoritmo de megatag para sacar la medicion
    private double shouldUseMegatag(MegatagPoseEstimate poseEstimate, FiducialObservation[] fiducials) {
        final int kExpectedTagCount = 2;
        double minArea = DriverStation.isDisabled() ? 0.05 : 0.4;
        // Filter average tag area, Validate we see two tags and Validate is not retuning a pose very near point to origin (ucladian distance)
        if (poseEstimate.avgTagArea < minArea || poseEstimate.fiducialIds.length != kExpectedTagCount || poseEstimate.fieldToRobot.getTranslation().getNorm() < 1.0) {
            return 0;
        }
        // Validate tags are not too far or have high ambiguity
        double distance = tagsAverageDistance(fiducials);
        if (distance >= 6 || fiducialsHaveHighAmbiguity(fiducials)) {
            return 0;
        }
        // Return confidence level
        return distance > 3.6576 ? 0.5 : 0.7;
    }

    // Funcion para saber si debemos usar el algoritmo de megatag 2 para sacar la medicion
    private double shouldUseMegatag2(MegatagPoseEstimate poseEstimate, FiducialObservation[] fiducials) {
        double distance = tagsAverageDistance(fiducials);
        return (poseEstimate.fiducialIds.length >= 1 && distance < 6) ? (distance > 4.5 ? 0.5 : 0.7) : 0;
    }

    // Funcion para calcular la distancia promedio de los tags vistos
    private double tagsAverageDistance(FiducialObservation[] fiducials) {
        double distance = 0;
        for (FiducialObservation fiducial : fiducials) {
            distance += Math.abs(fiducial.distToRobot);
        }
        return distance / fiducials.length;
    }

    // Funcion para saber si utilizo algun tag con mucha ambiguedad
    private boolean fiducialsHaveHighAmbiguity(FiducialObservation[] fiducials) {
        for (FiducialObservation fiducial : fiducials) {
            if (fiducial.ambiguity > 0.9) {
                return true;
            }
        }
        return false;
    }

    // Funcion para actualizar la medicion de la camara utilizada para ajustar el pid drive
    private void updatePidVisionMeasure(CameraMeasure cam, VisionInputsAutoLogged inputs) {
        // Utilizamos los robot pose correctos segun el algoritmo usado
        MegatagPoseEstimate poseEstimate = getPoseEstimate(cam, inputs);
        // Nos aseguramos solo se utilize si este activo el modo pid en el drive
        if (container.getDriveSubsystem().getPidSwerveDriveState() != 0) {
            if (container.getDriveSubsystem().getPidIsValidTag(cam.primaryTagId) && cam.primaryTagId >= 1 && cam.primaryTagId <= FieldConstants.tagsmatrix.length) {
                // Actualizamos la medicion en el drive
                Pose3d targetPose = getTargetPose(cam, inputs);
                container.getDriveSubsystem().updatePidVisionMeasure(new DrivePidDrive.VisionMeasure(cam.primaryTagId, cam.cameraIndex, poseEstimate.timestampSeconds, targetPose, FieldConstants.tagsmatrix[cam.primaryTagId - 1]));
            }
        }
    }

    // Funcion para actualizar los valores de los tags vistos (para la simulacion) con los tags vistos utilizados para la medida
    private void outputTagsPoses(CameraMeasure cam, VisionInputsAutoLogged inputs) {
        // Utilizamos los filudials correctos segun el algoritmo usado
        FiducialObservation[] fiducials = getFiducials(cam, inputs);
        // Obtenemos los poses de los tags vistos
        List<Pose3d> tagsPoses = new ArrayList<>();
        for (FiducialObservation fiducial : fiducials) {
            if (fiducial.id >= 1 && fiducial.id <= FieldConstants.tagsmatrix.length) {
                tagsPoses.add(FieldConstants.tagsmatrix[fiducial.id - 1]);
            }
        }
        // Creamos los logs con al informacion de los tags, si no existe lo seteamos al pose del robot para evitar que la simulacion cree una linea
        Pose2d robotPose = container.getDriveSubsystem().getPose();
        for (int i = 0; i < 4; i++) {
            Logger.recordOutput(String.format("Vision/TagPose_%d", i), 
                i < tagsPoses.size() ? tagsPoses.get(i) : new Pose3d(new Translation3d(robotPose.getX(), robotPose.getY(), 0),
                new Rotation3d(0, 0, robotPose.getRotation().getRadians())));
        }
    }

    // Funcion para actualizar los valores de los tags vistos (para la simulacion) cuando no vemos nada
    // Los configuramos para que tengan el mismo valor que el robot pose para evitar la simulacion cree lineas de tag visto.
    private void outputNullTagsPoses(Pose2d robotPose) {
        for (int i = 0; i < 4; i++) {
            Logger.recordOutput(String.format("Vision/TagPose_%d", i), 
                new Pose3d(new Translation3d(robotPose.getX(), robotPose.getY(), 0), new Rotation3d(0, 0, robotPose.getRotation().getRadians())));
        }
    }

    // Funcion para obtener los filudials segun la camara seleccionada y algorithmo utilizado
    private FiducialObservation[] getFiducials(CameraMeasure cam, VisionInputsAutoLogged inputs) {
        switch (cam.cameraIndex) {
            case 1: return cam.model == 1 ? inputs.cameraFiducialObservationsMt_FL : inputs.cameraFiducialObservationsMt2_FL;
            case 2: return cam.model == 1 ? inputs.cameraFiducialObservationsMt_FR : inputs.cameraFiducialObservationsMt2_FR;
            case 3: return cam.model == 1 ? inputs.cameraFiducialObservationsMt_B : inputs.cameraFiducialObservationsMt2_B;
            default: return null;
        }
    }

    // Funcion para obtener el robot pose en el field space segun la camara seleccionada y algorithmo utilizado
    private MegatagPoseEstimate getPoseEstimate(CameraMeasure cam, VisionInputsAutoLogged inputs) {
        switch (cam.cameraIndex) {
            case 1: return cam.model == 1 ? inputs.cameraMegatagPoseEstimate_FL : inputs.cameraMegatag2PoseEstimate_FL;
            case 2: return cam.model == 1 ? inputs.cameraMegatagPoseEstimate_FR : inputs.cameraMegatag2PoseEstimate_FR;
            case 3: return cam.model == 1 ? inputs.cameraMegatagPoseEstimate_B : inputs.cameraMegatag2PoseEstimate_B;
            default: return null;
        }
    }

    // Funcion para obtener el target pose en camera space segun la camara seleccionada
    private Pose3d getTargetPose(CameraMeasure cam, VisionInputsAutoLogged inputs) {
        switch (cam.cameraIndex) {
            case 1: return inputs.targetPose_FL;
            case 2: return inputs.targetPose_FR;
            case 3: return inputs.targetPose_B;
            default: return null;
        }
    }

    // Function to get the robot pose
    private Pose3d getRobotPose() {
        Pose2d robotPose2D = container.getDriveSubsystem().getPose();
        double z = 0.0;
        Rotation3d rotation = new Rotation3d(0, 0, Math.toRadians(container.getDriveSubsystem().getInputs().yawAngle));
        return new Pose3d(new Translation3d(robotPose2D.getX(), robotPose2D.getY(), z), rotation);
    }

}