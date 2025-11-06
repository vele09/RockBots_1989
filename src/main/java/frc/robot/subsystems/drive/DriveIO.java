package frc.robot.subsystems.drive;

///////////////////////////////////////////////////////////////////////////////
// Description: Interface con los metodos para administrar las salidas y entradas del subsistema.
// Notes:
//  - 
///////////////////////////////////////////////////////////////////////////////

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public interface DriveIO {
    
    // Class to store sensors data from real robot
    @AutoLog
    class DriveInputs {
        // Time
        public double timestamp = 0;
        public double deltaTime = 0;
        // Navx data
        public double yawAngle = 0;
        public double yawAngleRate = 0;
        // Steering motor angles
        public double frSteeringPosition = 0;
        public double flSteeringPosition = 0;
        public double brSteeringPosition = 0;
        public double blSteeringPosition = 0;
        // Drive Motors velocity
        public double frDriveVelocity = 0;
        public double flDriveVelocity = 0;
        public double brDriveVelocity = 0;
        public double blDriveVelocity = 0;
        // Speed Motors positions
        public double frDrivePosition = 0;
        public double flDrivePosition = 0;
        public double brDrivePosition = 0;
        public double blDrivePosition = 0;
    }

    // Funcion para leer los inputs reales del robot
    default void readInputs(DriveInputs inputs) {}

    // Funcion usada para actualizar los inputs en la simulacion
    default void update(final DriveInputs inputs) { }

    // Funcion utilizada para actualizar la salida de los motores
    default void writeOutputs(double frSteeringMotorValue, double frDriveMotorValue,
                            double flSteeringMotorValue, double flDriveMotorValue,
                            double brSteeringMotorValue, double brDriveMotorValue,
                            double blSteeringMotorValue, double blDriveMotorValue) {}

                           
    // Funciones para leer las posiciones de los modulos de swerve
    default SwerveModulePosition getFrModulePosition() { return null; }
    default SwerveModulePosition getFlModulePosition() { return null; }
    default SwerveModulePosition getBrModulePosition() { return null; }
    default SwerveModulePosition getBlModulePosition() { return null; }

    // Funciones para leer los estados de los modulos de swerve
    default SwerveModuleState getFrModuleState() { return null; }
    default SwerveModuleState getFlModuleState() { return null; }
    default SwerveModuleState getBrModuleState() { return null; }
    default SwerveModuleState getBlModuleState() { return null; }

}
