package frc.robot.subsystems.drive;

///////////////////////////////////////////////////////////////////////////////
// Description: Clase con los metodos para administrar las salidas y entradas del subsistema usando el robot real.
// Notes:
//  -
///////////////////////////////////////////////////////////////////////////////

import com.studica.frc.AHRS;
import com.ctre.phoenix6.signals.InvertedValue;
import frc.robot.Constants;
import lib.swerve.SwerveModule;
import lib.util.Util;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;

public class DriveIOHardware implements DriveIO { 

    //Navx sensor
    private AHRS navx;

    // Aqui se declaran los modulos de swerve
    private SwerveModule frModule, flModule, brModule, blModule;
 
    // Class constructor
    public DriveIOHardware(){
        // Init NavX sensor and reset
        try {
            navx = new AHRS(AHRS.NavXComType.kMXP_SPI, (byte) 50); // cambiando a 100 hertz el update time
            Timer.delay(0.5);
            navx.reset();
            Timer.delay(0.5);
          } catch (Exception e) {
            System.out.print("navx not working");
      }
        // Init swerve modules
        // FR: Drive motor ID=1 (abajo), Steering motor ID=2 (arriba)
        frModule = new SwerveModule(Constants.Drive.kFrontRightDriveMotorID,
                                    Constants.Drive.kFrontRightSteeringMotorID,
                                    Constants.Drive.kFrontRightCANCoderID,
                                    "rio",
                                    InvertedValue.CounterClockwise_Positive,  // Drive motor (ID 1) - abajo
                                    InvertedValue.Clockwise_Positive);         // Steering motor (ID 2) - arriba
        // FL: Drive motor ID=4 (arriba), Steering motor ID=3 (abajo)
        flModule = new SwerveModule(Constants.Drive.kFrontLeftDriveMotorID,
                                    Constants.Drive.kFrontLeftSteeringMotorID,
                                    Constants.Drive.kFrontLeftCANCoderID,
                                    "rio",
                                    InvertedValue.Clockwise_Positive,          // Drive motor (ID 4) - arriba
                                    InvertedValue.CounterClockwise_Positive);  // Steering motor (ID 3) - abajo
        // BR: Drive motor ID=7 (arriba), Steering motor ID=8 (abajo)
        brModule = new SwerveModule(Constants.Drive.kBackRightDriveMotorID,
                                    Constants.Drive.kBackRightSteeringMotorID,
                                    Constants.Drive.kBackRightCANCoderID,
                                    "rio",
                                    InvertedValue.Clockwise_Positive,          // Drive motor (ID 7) - arriba
                                    InvertedValue.CounterClockwise_Positive);  // Steering motor (ID 8) - abajo
        // BL: Drive motor ID=5 (arriba), Steering motor ID=6 (arriba)
        blModule = new SwerveModule(Constants.Drive.kBackLeftDriveMotorID,
                                    Constants.Drive.kBackLeftSteeringMotorID,
                                    Constants.Drive.kBackLeftCANCoderID,
                                    "rio",
                                    InvertedValue.Clockwise_Positive,          // Drive motor (ID 5) - arriba
                                    InvertedValue.Clockwise_Positive);         // Steering motor (ID 6) - arriba
    }

    // Funcion para leer los inputs reales del robot
    @Override
    public void readInputs(DriveInputs inputs) {
        // Refresh sensors values
        frModule.refreshSignals();
        flModule.refreshSignals();
        brModule.refreshSignals();
        blModule.refreshSignals();
        // Get delta time
        double tempTime = Timer.getFPGATimestamp();
        if(inputs.timestamp>0){
            inputs.deltaTime = tempTime - inputs.timestamp;
        }
        inputs.timestamp = tempTime;
        // Get navx angles (este es acumulativo)
        inputs.yawAngle = navx.getAngle() * -1;
        inputs.yawAngleRate = navx.getRate();

        // Ajuste de angulo porque iniciamos volteados en el lado azul (contra el 0,0 en el lado azul)
        if(Util.isBlueAllience()){
            inputs.yawAngle = inputs.yawAngle + 180;
        }
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
