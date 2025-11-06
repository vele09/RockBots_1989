package frc.robot.subsystems.drive;

///////////////////////////////////////////////////////////////////////////////
// Description: Clase con los metodos para administrar las salidas y entradas del subsistema usando una simulacion.
// Notes:
//  - 
///////////////////////////////////////////////////////////////////////////////

import com.team3478.frc2025.Constants;
import com.team3478.lib.util.Util;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class DriveIOSim implements DriveIO {

    // Variables to simulate motors movements
    private final FlywheelSim frDriveSim;
    private final FlywheelSim flDriveSim;
    private final FlywheelSim brDriveSim;
    private final FlywheelSim blDriveSim;
    private final FlywheelSim frSteeringSim;
    private final FlywheelSim flSteeringSim;
    private final FlywheelSim brSteeringSim;
    private final FlywheelSim blSteeringSim;
    // Variables to get the kinematics of the chassis
    private SwerveDriveKinematics kinematics;
    // Variables to save swerve states
    private SwerveModulePosition[] positions = null;
    private SwerveModuleState[] states = null;
    // Variable para saber cuando es el primer ciclo
    private boolean isFirstCycle=true;

    // Class constructor
    public DriveIOSim() {
        // Corrected kV and kA for drive motors with 6.75:1 gear reduction
        // Max wheel speed = 4.5 m/s, Wheel radius = 0.05 m
        // Max angular velocity (wheel) = 90 rad/s
        // Corresponding motor-side angular velocity at 6.75:1 gear ratio = 607.5 rad/s
        // Corrected kV_drive ≈ 0.01975 V/rad/s, kA_drive ≈ 0.00287 V/rad/s²
        frDriveSim = new FlywheelSim(LinearSystemId.identifyVelocitySystem(0.01975, 0.00287), DCMotor.getKrakenX60Foc(1), 0.0);
        flDriveSim = new FlywheelSim(LinearSystemId.identifyVelocitySystem(0.01975, 0.00287), DCMotor.getKrakenX60Foc(1), 0.0);
        brDriveSim = new FlywheelSim(LinearSystemId.identifyVelocitySystem(0.01975, 0.00287), DCMotor.getKrakenX60Foc(1), 0.0);
        blDriveSim = new FlywheelSim(LinearSystemId.identifyVelocitySystem(0.01975, 0.00287), DCMotor.getKrakenX60Foc(1), 0.0);
        // Corrected kV and kA for steering motors with 21.42:1 gear reduction
        // These values are typically smaller due to lower speed requirements for steering
        frSteeringSim = new FlywheelSim(LinearSystemId.identifyVelocitySystem(0.01, 0.005), DCMotor.getKrakenX60Foc(1), 0.0);
        flSteeringSim = new FlywheelSim(LinearSystemId.identifyVelocitySystem(0.01, 0.005), DCMotor.getKrakenX60Foc(1), 0.0);
        brSteeringSim = new FlywheelSim(LinearSystemId.identifyVelocitySystem(0.01, 0.005), DCMotor.getKrakenX60Foc(1), 0.0);
        blSteeringSim = new FlywheelSim(LinearSystemId.identifyVelocitySystem(0.01, 0.005), DCMotor.getKrakenX60Foc(1), 0.0);
        // Init Swerve Kinematics object
        Translation2d[] moduleLocations = new Translation2d[Constants.Drive.kNumberOfModules];
        moduleLocations[0] =
            new Translation2d(
                Constants.Drive.kXFrontRightLocation, Constants.Drive.kYFrontRightLocation);
        moduleLocations[1] =
            new Translation2d(Constants.Drive.kXFrontLeftLocation, Constants.Drive.kYFrontLeftLocation);
        moduleLocations[2] =
            new Translation2d(Constants.Drive.kXBackRightLocation, Constants.Drive.kYBackRightLocation);
        moduleLocations[3] =
            new Translation2d(Constants.Drive.kXBackLeftLocation, Constants.Drive.kYBackLeftLocation);
        kinematics = new SwerveDriveKinematics(moduleLocations);
        // Init states and positions
        positions = new SwerveModulePosition[Constants.Drive.kNumberOfModules];
        states = new SwerveModuleState[Constants.Drive.kNumberOfModules];
        for (int i = 0; i < Constants.Drive.kNumberOfModules; i++) {
            positions[i] = new SwerveModulePosition(0, Rotation2d.fromDegrees(0));
            states[i] = new SwerveModuleState(0, Rotation2d.fromDegrees(0));
        }
    }

    // Funcion para leer los inputs reales del robot
    @Override
    public void readInputs(DriveInputs inputs) {
        // In simulation only run this if not in replay mode
        if(!Constants.kIsReplay){
            // Get delta time
            double tempTime = Timer.getFPGATimestamp();
            if(inputs.timestamp>0){
                inputs.deltaTime = tempTime - inputs.timestamp;
            }
            inputs.timestamp = tempTime;
            // Advance the simulation by one timestep (20ms)
            frDriveSim.update(inputs.deltaTime);
            flDriveSim.update(inputs.deltaTime);
            brDriveSim.update(inputs.deltaTime);
            blDriveSim.update(inputs.deltaTime);
            frSteeringSim.update(inputs.deltaTime);
            flSteeringSim.update(inputs.deltaTime);
            brSteeringSim.update(inputs.deltaTime);
            blSteeringSim.update(inputs.deltaTime);
            // Get chassis speeds (using kinematics)
            SwerveModuleState[] moduleStates = new SwerveModuleState[] {
                getFrModuleState(),
                getFlModuleState(),
                getBrModuleState(),
                getBlModuleState(),
            };
            ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(moduleStates);
            // Simulate pigeon angles by integrating the angular velocity
            inputs.yawAngleRate = Math.toDegrees(chassisSpeeds.omegaRadiansPerSecond);
            // Ajuste de angulo porque iniciamos volteados en el lado azul (contra el 0,0 en el lado azul)
            if(Util.isBlueAllience() && isFirstCycle){
                inputs.yawAngle = 180;
                isFirstCycle=false;
            }
            inputs.yawAngle += inputs.yawAngleRate * inputs.deltaTime;
            // Update steering positions and drive positions
            double frSteeringAngularVelocity = frSteeringSim.getAngularVelocityRPM() * 360.0 / 60.0;  // Convert RPM to degrees/sec
            double flSteeringAngularVelocity = flSteeringSim.getAngularVelocityRPM() * 360.0 / 60.0;
            double brSteeringAngularVelocity = brSteeringSim.getAngularVelocityRPM() * 360.0 / 60.0;
            double blSteeringAngularVelocity = blSteeringSim.getAngularVelocityRPM() * 360.0 / 60.0;
            // Integrate angular velocity over time to update the steering positions
            inputs.frSteeringPosition = Util.Limit360Angle(inputs.frSteeringPosition + (frSteeringAngularVelocity * inputs.deltaTime));
            inputs.flSteeringPosition = Util.Limit360Angle(inputs.flSteeringPosition + (flSteeringAngularVelocity * inputs.deltaTime));
            inputs.brSteeringPosition = Util.Limit360Angle(inputs.brSteeringPosition + (brSteeringAngularVelocity * inputs.deltaTime));
            inputs.blSteeringPosition = Util.Limit360Angle(inputs.blSteeringPosition + (blSteeringAngularVelocity * inputs.deltaTime));
            // Simulate motor velocities (in rotations per second)
            inputs.frDriveVelocity = frDriveSim.getAngularVelocityRPM() / 60.0;  // Convert RPM to RPS
            inputs.flDriveVelocity = flDriveSim.getAngularVelocityRPM() / 60.0;
            inputs.brDriveVelocity = brDriveSim.getAngularVelocityRPM() / 60.0;
            inputs.blDriveVelocity = blDriveSim.getAngularVelocityRPM() / 60.0;
            // Integrate the drive velocity to calculate the drive motor positions (in rotations)
            inputs.frDrivePosition += inputs.frDriveVelocity * inputs.deltaTime;
            inputs.flDrivePosition += inputs.flDriveVelocity * inputs.deltaTime;
            inputs.brDrivePosition += inputs.brDriveVelocity * inputs.deltaTime;
            inputs.blDrivePosition += inputs.blDriveVelocity * inputs.deltaTime;
        }
    }

    // Funcion usada para actualizar los inputs en la simulacion
    @Override
    public void update(final DriveInputs inputs) {
        positions[0].distanceMeters = ConvertToMeters(inputs.frDrivePosition);
        positions[0].angle = Rotation2d.fromDegrees(inputs.frSteeringPosition);
        positions[1].distanceMeters = ConvertToMeters(inputs.flDrivePosition);
        positions[1].angle = Rotation2d.fromDegrees(inputs.flSteeringPosition);
        positions[2].distanceMeters = ConvertToMeters(inputs.brDrivePosition);
        positions[2].angle = Rotation2d.fromDegrees(inputs.brSteeringPosition);
        positions[3].distanceMeters = ConvertToMeters(inputs.blDrivePosition);
        positions[3].angle = Rotation2d.fromDegrees(inputs.blSteeringPosition);

        states[0].speedMetersPerSecond = ConvertToMeters(inputs.frDriveVelocity);
        states[0].angle = Rotation2d.fromDegrees(inputs.frSteeringPosition);
        states[1].speedMetersPerSecond = ConvertToMeters(inputs.flDriveVelocity);
        states[1].angle = Rotation2d.fromDegrees(inputs.flSteeringPosition);
        states[2].speedMetersPerSecond = ConvertToMeters(inputs.brDriveVelocity);
        states[2].angle = Rotation2d.fromDegrees(inputs.brSteeringPosition);
        states[3].speedMetersPerSecond = ConvertToMeters(inputs.blDriveVelocity);
        states[3].angle = Rotation2d.fromDegrees(inputs.blSteeringPosition);
    }

    // Funcion utilizada para actualizar la salida de los motores
    @Override
    public void writeOutputs(double frSteeringMotorValue, double frDriveMotorValue,
                            double flSteeringMotorValue, double flDriveMotorValue,
                            double brSteeringMotorValue, double brDriveMotorValue,
                            double blSteeringMotorValue, double blDriveMotorValue) {
        if(!Constants.kIsReplay){
            // Simulate the effect of the motor values on the wheel speed
            frDriveSim.setInputVoltage(frDriveMotorValue * 12);
            flDriveSim.setInputVoltage(flDriveMotorValue * 12);
            brDriveSim.setInputVoltage(brDriveMotorValue * 12);
            blDriveSim.setInputVoltage(blDriveMotorValue * 12);
            frSteeringSim.setInputVoltage(frSteeringMotorValue * 12);
            flSteeringSim.setInputVoltage(flSteeringMotorValue * 12);
            brSteeringSim.setInputVoltage(brSteeringMotorValue * 12);
            blSteeringSim.setInputVoltage(blSteeringMotorValue * 12);
        }
    }

     // Funciones para leer las posiciones de los modulos de swerve
    @Override
    public SwerveModulePosition getFrModulePosition() { return positions[0]; }
    @Override
    public SwerveModulePosition getFlModulePosition() { return positions[1]; }
    @Override
    public SwerveModulePosition getBrModulePosition() { return positions[2]; }
    @Override
    public SwerveModulePosition getBlModulePosition() { return positions[3]; }

    // Funciones para leer los estados de los modulos de swerve
    @Override
    public SwerveModuleState getFrModuleState() { return states[0]; }
    @Override
    public SwerveModuleState getFlModuleState() { return states[1]; }
    @Override
    public SwerveModuleState getBrModuleState() { return states[2]; }
    @Override
    public SwerveModuleState getBlModuleState() { return states[3]; }

    // Function to convert rotations to meters
    public double ConvertToMeters(double value){
        // Update module state
        return value * Math.PI * Constants.Drive.kWheelRadius * 2 * Constants.Drive.kGearDriveReduction;
    }
}
