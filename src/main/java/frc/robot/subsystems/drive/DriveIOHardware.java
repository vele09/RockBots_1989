package frc.robot.subsystems.drive;

///////////////////////////////////////////////////////////////////////////////
// Description: Clase con los metodos para administrar las salidas y entradas del subsistema usando el robot real.
// Notes:
//  - 
///////////////////////////////////////////////////////////////////////////////

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;
import frc.robot.Constants;
import lib.swerve.SwerveModule;
import lib.util.Util;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;

public class DriveIOHardware implements DriveIO { 

    //Pigeon sensor
    Pigeon2 pigeon = null;
    BaseStatusSignal yawAngle;
    BaseStatusSignal yawAngleRate;
    // Aqui se declaran los modulos de swerve
    private SwerveModule frModule, flModule, brModule, blModule;
 
    // Class constructor
    public DriveIOHardware(){
        // Init Pigeon sensor and reset
        pigeon = new Pigeon2(Constants.Drive.kPigeonID,"canivore");
        pigeon.reset();
        yawAngle = pigeon.getYaw();
        yawAngleRate = pigeon.getAngularVelocityZWorld();
        // Init swerve modules
        frModule = new SwerveModule(Constants.Drive.kFrontRightDriveMotorID, 
                                    Constants.Drive.kFrontRightSteeringMotorID, 
                                    Constants.Drive.kFrontRightCANCoderID,  
                                    "canivore");
        flModule = new SwerveModule(Constants.Drive.kFrontLeftDriveMotorID, 
                                    Constants.Drive.kFrontLeftSteeringMotorID, 
                                    Constants.Drive.kFrontLeftCANCoderID,  
                                    "canivore");
        brModule = new SwerveModule(Constants.Drive.kBackRightDriveMotorID, 
                                    Constants.Drive.kBackRightSteeringMotorID, 
                                    Constants.Drive.kBackRightCANCoderID,  
                                    "canivore");
        blModule = new SwerveModule(Constants.Drive.kBackLeftDriveMotorID, 
                                    Constants.Drive.kBackLeftSteeringMotorID, 
                                    Constants.Drive.kBackLeftCANCoderID,  
                                    "canivore");
    }

    // Funcion para leer los inputs reales del robot
    @Override
    public void readInputs(DriveInputs inputs) {
        // Refresh sensors values
        frModule.refreshSignals();
        flModule.refreshSignals();
        brModule.refreshSignals();
        blModule.refreshSignals();
        BaseStatusSignal.refreshAll(yawAngle, yawAngleRate);
        // Get delta time
        double tempTime = Timer.getFPGATimestamp();
        if(inputs.timestamp>0){
            inputs.deltaTime = tempTime - inputs.timestamp;
        }
        inputs.timestamp = tempTime;
        // Get pigeon angles (este es acumulativo)
        inputs.yawAngle = yawAngle.getValueAsDouble();
        // Ajuste de angulo porque iniciamos volteados en el lado azul (contra el 0,0 en el lado azul)
        if(Util.isBlueAllience()){
            inputs.yawAngle = inputs.yawAngle + 180;
        }
        inputs.yawAngleRate = yawAngleRate.getValueAsDouble();
        // Se leen los angulos de steering de las ruedas convirtiendo de 0 a 360 grados y aplicando el offset inicial de las ruedas.
        inputs.frSteeringPosition = (frModule.getEncoderAngle() * 360 + Constants.Drive.kFrontRightEncoderInitPos);
        inputs.flSteeringPosition = (flModule.getEncoderAngle() * 360 + Constants.Drive.kFrontLeftEncoderInitPos);
        inputs.brSteeringPosition = (brModule.getEncoderAngle() * 360 + Constants.Drive.kBackRightEncoderInitPos);
        inputs.blSteeringPosition = (blModule.getEncoderAngle() * 360 + Constants.Drive.kBackLeftEncoderInitPos);
        // Se limitan los angulos de 0 a 360
        inputs.frSteeringPosition = Util.Limit360Angle(inputs.frSteeringPosition);
        inputs.flSteeringPosition = Util.Limit360Angle(inputs.flSteeringPosition);
        inputs.brSteeringPosition = Util.Limit360Angle(inputs.brSteeringPosition);
        inputs.blSteeringPosition = Util.Limit360Angle(inputs.blSteeringPosition);
        // Se calcula la velocidad lineal (rotaciones/segundo) de las ruedas del drive
        inputs.frDriveVelocity = frModule.getDriveMotorVelocity();
        inputs.flDriveVelocity = flModule.getDriveMotorVelocity();
        inputs.brDriveVelocity = brModule.getDriveMotorVelocity();
        inputs.blDriveVelocity = blModule.getDriveMotorVelocity();
        // Se calcula la velocidad lineal (rotaciones) de las ruedas del drive
        inputs.frDrivePosition = frModule.getDriveMotorPosition();
        inputs.flDrivePosition = flModule.getDriveMotorPosition();
        inputs.brDrivePosition = brModule.getDriveMotorPosition();
        inputs.blDrivePosition = blModule.getDriveMotorPosition();
        // Update swerve module states
        frModule.update(inputs.frDriveVelocity, inputs.frDrivePosition, inputs.frSteeringPosition);
        flModule.update(inputs.flDriveVelocity, inputs.flDrivePosition, inputs.flSteeringPosition);
        brModule.update(inputs.brDriveVelocity, inputs.brDrivePosition, inputs.brSteeringPosition);
        blModule.update(inputs.blDriveVelocity, inputs.blDrivePosition, inputs.blSteeringPosition);
    }

    // Funcion utilizada para actualizar la salida de los motores
    @Override
    public void writeOutputs(double frSteeringMotorValue, double frDriveMotorValue,
                    double flSteeringMotorValue, double flDriveMotorValue,
                    double brSteeringMotorValue, double brDriveMotorValue,
                    double blSteeringMotorValue, double blDriveMotorValue) {
        frModule.getDriveMotor().set(frDriveMotorValue);
        frModule.getSteeringMotor().set(frSteeringMotorValue);
        flModule.getDriveMotor().set(flDriveMotorValue);
        flModule.getSteeringMotor().set(flSteeringMotorValue);
        brModule.getDriveMotor().set(brDriveMotorValue);
        brModule.getSteeringMotor().set(brSteeringMotorValue);
        blModule.getDriveMotor().set(blDriveMotorValue);
        blModule.getSteeringMotor().set(blSteeringMotorValue);
    }

    // Funciones para leer las posiciones de los modulos de swerve
    @Override
    public SwerveModulePosition getFrModulePosition() { return frModule.getPositionState(); }
    @Override
    public SwerveModulePosition getFlModulePosition() { return flModule.getPositionState(); }
    @Override
    public SwerveModulePosition getBrModulePosition() { return brModule.getPositionState(); }
    @Override
    public SwerveModulePosition getBlModulePosition() { return blModule.getPositionState(); }

    // Funciones para leer los estados de los modulos de swerve
    @Override
    public SwerveModuleState getFrModuleState() { return frModule.getModuleState(); }
    @Override
    public SwerveModuleState getFlModuleState() { return flModule.getModuleState(); }
    @Override
    public SwerveModuleState getBrModuleState() { return brModule.getModuleState(); }
    @Override
    public SwerveModuleState getBlModuleState() { return blModule.getModuleState(); }
   
}
