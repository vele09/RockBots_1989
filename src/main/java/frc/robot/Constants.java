package frc.robot;

import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

public final class Constants {

  // Bandera para saber si haremos replay de inputs en la simulacion
  public static final boolean kIsReplay = false;

  // Constants for the controlboard
  public static class OperatorConstants {
    // Driver controller port
    public static final int kDriverControlPort = 0;
    // Operator controller port
    public static final int kOperatorControlPort = 1;
  }
    
  // Constants for the drive subsystem
  public static class Drive {
    // Motors current limits
    public static final int kDriveMotorCurrentLimit = 60;
    public static final int kSteeringMotorCurrentLimit = 30;
    // Variables para offsetear el 0 de las llantas del swerve(steering angle encoders)
    public static final double kFrontRightEncoderInitPos = -78.14; // -199.8633
    public static final double kFrontLeftEncoderInitPos = -239.23;  //-214.0137
    public static final double kBackRightEncoderInitPos = -148.4;  //-209.0039
    public static final double kBackLeftEncoderInitPos = -20.3;  //-89.2968
    // Variables con constantes de las ruedas
    public static final double kWheelRadius = 0.0508;
    public static final double kWheelTrack = 0.58;
    // Variables con constantes de los motores
    public static final int kMotorRPM = 6000;
    // Variables con el id del pigeon
    public static final int kPigeonID = 0;
    // Variables con los ids de los cancoders
    public static final int kFrontRightCANCoderID = 2;
    public static final int kFrontLeftCANCoderID = 1;
    public static final int kBackRightCANCoderID = 3;
    public static final int kBackLeftCANCoderID = 4;
    // Variables con los ids de los motores
    public static final int kFrontRightDriveMotorID = 4;
    public static final int kFrontRightSteeringMotorID = 3;
    public static final int kFrontLeftDriveMotorID = 2;
    public static final int kFrontLeftSteeringMotorID = 1;
    public static final int kBackRightDriveMotorID = 6;
    public static final int kBackRightSteeringMotorID = 5;
    public static final int kBackLeftDriveMotorID = 8;
    public static final int kBackLeftSteeringMotorID = 7;
    // Variable para indicar el numero de modulos de swerve en el robot
    public static int kNumberOfModules = 4;
    // Variables con las posiciones de los moduloes en metros
    // Distancias contra el centro del chassis


    public static final double kXFrontLeftLocation = 0.3492; //Cambio de valores
    public static final double kYFrontLeftLocation = 0.478;
    public static final double kXFrontRightLocation = 0.3492;
    public static final double kYFrontRightLocation = -0.478;
    public static final double kXBackLeftLocation = -0.3492;
    public static final double kYBackLeftLocation = 0.333;
    public static final double kXBackRightLocation = -0.3492;
    public static final double kYBackRightLocation = -0.333;

    // Reduccion de los motores de velocidad
    public static final double kGearDriveReduction = 1 / 6.75;
    public static final double kGearSteeringReduction = 1 / 21.42;
    
    // Variables para el controlador PD de los motores de steering
    public static final double kPSteeringValue = 0.006;//0.006;
    public static final double kDSteeringValue = 0.0002;
    // Variable limitar la deteccion del stick de giro (y evitar giros muy despacios)
    public static final double kDriveTurnDeadband = 0.1;
    // Variable para el acumulador de inercia
    public static final double kAcumulatorChange = 0.08;
    public static final double kAcumulatorAlpha = 0.1;
    // Variables para el controlador PD para ajustar el giro del robot
    public static final double kPTurnValueBigError = 0.005;//0.0016;
    public static final double kDTurnValueBigError = 0.0;
    public static final double kPTurnValueSmallError = 0.012;//0.0018;
    public static final double kDTurnValueSmallError = 0.0;
    public static final double kPTurnValueSmallerError = 0.006;//0.0026;
    public static final double kDTurnValueSmallerError = 0;
    // PPHolonomicDriveController constants for the pathplanner
    public static final PIDConstants kTranslationConstants = new PIDConstants(5.0, 0.0, 0.0);
    public static final PIDConstants kRotationConstants = new PIDConstants(5.0, 0.0, 0.0);
    // Camera position relative to robot center
    public static final Pose3d kFrontLeftCameraPose = new Pose3d(new Translation3d(-0.195, -0.0935, 0), new Rotation3d(0, Math.toRadians(0), Math.toRadians(0)));
    public static final Pose3d kFrontRightCameraPose = new Pose3d(new Translation3d(0.175, -0.0935, 0), new Rotation3d(0, Math.toRadians(0), Math.toRadians(0)));
  }

  // Constants for the vision subsystem
  public static class Vision {
    public static final String Back   = "limelight-back";
    public static final String FrontR = "limelight-frontr";
    public static final String FrontL = "limelight-frontl";
  }


  // Constants for the elevator subsystem
  public static class Elevator {
    // Elevator motor Ids
    public static final int kElevatorMotor1ID = 10;  //Right
    public static final int kElevatorMotor2ID = 11;  // Left
    // Limit switches (io) channels
    public static final int kForwardLimit1Channel = 3;
    public static final int kForwardLimit2Channel = 1 ;
    public static final int kReverseLimit1Channel = 0;
    public static final int kReverseLimit2Channel = 2;
    // Motor current limit
    public static final int kMotorCurrentLimit = 60;
    // Constantes para el PID del Elevator
    public static final double kMotionProfileP= 0.06;
    public static final double kMotionProfileI = 0;
    public static final double kMotionProfileD = 0;
    public static final double kMotionProfileMaxAccelerationUp= 1000;
    public static final double kMotionProfileMaxVelocityUp= 400;
    public static final double kMotionProfileMaxAccelerationDown= 250;
    public static final double kMotionProfileMaxVelocityDown= 75;
    public static final double kMotionProfilePeriod = 0.020;
    // Constantes para establecer los límites del Elevator
    public static final double kElevatorUpperLimit = 1.17;
    public static final double kElevatorLowerLimit = 0;
    // Error tolerance elevator motor rotations
    public static final double kElevatorTolerance = 0.03;
    // Variables para calcular la distancia del Elevator
    public static final double kElevatorReduction = 5;
    public static final double kElevatorDiameter = 0.0466;
  }

  public static final class Intake {

    public static final int kIntakeLiftID = 14;
    public static final int kIntakeWheelsID = 17;
    
    // Sensores
    //public static final int KLimitSwitchChannel = 0; 
    

    // TOF1 configuración (sensor de pieza en la entrada)
    public static final double kTofThreshold = 500;  // Distancia en mm para detectar pieza  

    // Posiciones angulares del intake (grados)
    public static final double kIntakeStartPositionAngle = 0.0;   
    public static final double kIntake1PositionAngle = -17.0;
    public static final double kIntakeLowPositionAngle = -148.0;    
    public static final double kIntakeHangerPosition = -135;
    //public static final double kLimitSwitchAngleDegrees = 55.0;   
    
    // Limites de posicion del intake
    public static final double kHardLimitUp = 0;
    public static final double kHardLimitDown = -150;   

    // Rango seguro para ruedas 
    public static final double kSafeLiftMinDegree = 100.0;
    public static final double kSafeLiftMaxDegrees = 80.0;

    // Motion profile (para control del lift con PID trapezoidal)
    public static final double kMotionProfileMaxVelocity = 110;    // rot/s
    public static final double kMotionProfileMaxAcceleration = 300; // rot/s^2
    public static final double kMotionProfileP = 0.1;              
    public static final double kMotionProfileI = 0;             
    public static final double kMotionProfileD = 0;              

    public static final double kMotionProfilePeriod = 0.02;       
    public static final double kIntakeWheelsSpeed = 0.6;        
    public static final double kReduction = 56.12;
    public static final double kEscupirDurationSec = 0.8;
    public static final double kMotorCurrentLimit = 40.0;    
}
  

public static class Shooter{
  // ids de los motores
  public static final int kShooterLowMotorID = 15;
  public static final int kShooterUpperMotorID = 16;
  //TOF
  public static final int kTofSensorID = 5;
  public static final int kShooterSensor1ID = 0; 

  public static final double kTofThreshold = 50;

  public static final int kTofSensorID = 5;
  public static final int kShooterSensor1ID = 0; 

}


  

  public static class Hanger {
    // Hanger motors Ids
    public static final int kHangerMotor1ID = 18;
    // Variables para calcular la distancia del escalador
    public static final double kHangerReduction = 20;
    // Hanger motion profile configuration
    public static final double kP = 0.01;
    public static final double kI = 0;
    public static final double kD = 0;
    // Limites de seguridad del hanger
    public static final double kHangerForwardLimit = 11 * kHangerReduction;
    public static final double kHangerReverseLimit = 0;
    // Se declara el limite de corriente para los motores del escalador
    public static final int kMotorCurrentLimit = 80;
    // Hanger positions
    public static final double kHangerStartPosition = 0;
    public static final double kHangerDownPosition = 2.45 * kHangerReduction;
    public static final double kHangerUpPosition = 9.5 * kHangerReduction;
    // Error tolerance hanger mechanism in motor rotations
    public static final double kHangerTolerance = 1;

  }


  public static class Simulation {

      //En general estos valores se basan del CONFIG del CAD de Advantage Scope, 
      //donde esta para la pieza el 0,0,0, y aqui se setean esos valores para mandarlos a 
      //llamar a los publishers de la simulacion en sus respectivos subsistemas,
      //se in vierten los signos porque si le quitaste tal cantidad para que este en 0 pues se la sumas para que este en la correcta posicion
      //y el 0 de la pieza este bien seteado y gire en su respectivo eje.

      //Recordar setear los valores en el archivo config con publishers que den 0.

      //Tambien las el -xTotalRobotX es para que agarre el 0,0,0 que le seteamos a cada pieza y que no use el 0,0,0 robot.
      //Ya que esto haria que se muevan raro las piezas y haria que las piezas no esten en su lugar.

    // Constantes para x, y y z de la posición del robot en el simulador
    public static final double kTotalRobotX = -0.073;
      
    // Constantes para x, y y z de la posición del Gripper en el simulador
    public static final double kGripperX = 0.137 - kTotalRobotX;
    public static final double kGripperY = 0;
    public static final double kGripperZ = 0.6341;

    // Constantes para x, y y z de la posición del Intake/pivote en el simulador
    public static final double kIntakeX = -0.37155 - kTotalRobotX;
    public static final double kIntakeY = 0;
    public static final double kIntakeZ = 0.306;

    // Constantes para x, y y z de la posición del Hanger en el simulador
    public static final double kHangerX = 0.023 - kTotalRobotX;
    public static final double kHangerY = 0;
    public static final double kHangerZ = 0.32;


  }

  

  public static class Leds {
    // Leds pwm port
    public static final int kPwmPort = 2;
    // Numero de leds en la tira unica
    public static final int kLeds = 38;
    // Numero de leds en cada borde (Left/Right edges)
    public static final int kLedsEdge = 10;
    // Numero de leds en la seccion central
    public static final int kLedsCenter = kLeds - (2 * kLedsEdge); // 24 LEDs
  }

}
