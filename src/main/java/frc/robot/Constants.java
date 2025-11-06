package frc.robot;


public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;



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
  
  }

  
}
