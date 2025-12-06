package frc.robot.subsystems.drive;
///////////////////////////////////////////////////////////////////////////////
// Description: Clase con los metodos del subsistema del drive.
// Notes:
//  - Codigo para un swerver drive considerando para la odometria:
//    x positivo hacia adelante, y positivo hacia la izquierda, z positivo hacia arriba
//    con base en el lado azul como 0,0
///////////////////////////////////////////////////////////////////////////////

import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.LazyCommandWrapper;
import lib.time.RobotTime;
import lib.util.MotorUtil;
import lib.util.Util;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

    // Enum con la spsociones de alineacion en el reef
    public static enum Position {
        left,
        center,
        right
    }

    // Advantage kit inputs object
    private final DriveInputsAutoLogged inputs = new DriveInputsAutoLogged();
    // Inputs/Outputs object
    private final DriveIO io;

    // Esta clase se usa para hacer calculos cinematicos con las velocidades de los motores.
    private MotorUtil mMotorUtilDrive;
    // Objetos de controladores PID para alinear la rotacion del robot
    private PIDController robotYawPIDSmaller, robotYawPID, robotYawPIDdinamic;
    // Esta es la clase para la cinematica del Swerve
    private SwerveDriveKinematics mSwerveDriveKinematics = null;
    // Variable para guardar la odometria del robot
    private SwerveDrivePoseEstimator odometry = null;
    private SwerveDrivePoseEstimator odometry_encoderonly = null;
    // Variable para guardar el ultimo error de las steering wheels
    private double[] lastError = {0, 0, 0, 0};
    // Variable para guardar el setpoint del angulo en yaw
    private double yawAngleSetPoint = 0;
    // Variable para guardar el estado del acumulador de inercia
    private double quickStopAccumulator = 0;
    // Variable para administar el pid drive (apara alinear contra un april tag)
    private DrivePidDrive pidDrive = null;
    // Variable con el objeto para el identificador de zonas
    private DriveZone driveZone = null;
    // Bandera para deshabilitar el pathplanner on the fly
    private boolean pathPlannerOnTheFlyEnabled=true;
    // Variable para desactivar la escritura al smarthdashboard de ser necesario
    private boolean logToSmarthDashBoard = true;
    // Variable para guardar la referencia al robot container
    private final RobotContainer container;

    // Constructor del subsistema
    public DriveSubsystem(final DriveIO io,final RobotContainer container) {
        // Save inputs and outputs object
        this.io = io;
        this.container = container;
        // Init motor converter object
        mMotorUtilDrive = new MotorUtil(Constants.Drive.kGearDriveReduction);
        // Init Modules Positions
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
        // Init Modules States
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[Constants.Drive.kNumberOfModules];
        for (int i = 0; i < Constants.Drive.kNumberOfModules; i++) {
            modulePositions[i] = new SwerveModulePosition(0, Rotation2d.fromDegrees(0));
        }
        // Init Swerve Kinematics object
        mSwerveDriveKinematics = new SwerveDriveKinematics(moduleLocations);
        // Init odometry objects
        odometry = new SwerveDrivePoseEstimator(mSwerveDriveKinematics, Rotation2d.fromDegrees(0), modulePositions, new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(0)));
        odometry_encoderonly = new SwerveDrivePoseEstimator(mSwerveDriveKinematics, Rotation2d.fromDegrees(0), modulePositions, new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(0)));
        // Inicializamos el pid para mantener el robot en el angulo deseado(yaw)
        robotYawPIDSmaller =
            new PIDController(
                Constants.Drive.kPTurnValueSmallerError, 0, Constants.Drive.kDTurnValueSmallerError);
        robotYawPID =
            new PIDController(
                Constants.Drive.kPTurnValueSmallError, 0, Constants.Drive.kDTurnValueSmallError);
        robotYawPIDdinamic =
            new PIDController(
                Constants.Drive.kPTurnValueBigError, 0, Constants.Drive.kDTurnValueBigError);
        // Configure path planner
        try{
            // Read settings from the pathplanner file
            RobotConfig config = RobotConfig.fromGUISettings();
            // Configure AutoBuilder
            AutoBuilder.configure(
                this.odometry::getEstimatedPosition, 
                this::resetPose, 
                this::getChassisSpeeds, 
                (speeds, feedforwards) -> swervePathPlannerDrive(speeds), 
                new PPHolonomicDriveController(
                    Constants.Drive.kTranslationConstants,
                    Constants.Drive.kRotationConstants
                ),
                config,
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this
            );
        }catch(Exception e){
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
        }
        // Create pid drive object
        pidDrive = new DrivePidDrive(this);
        // Create drive zone object
        driveZone = new DriveZone();
    }

    // Periodic cycle of the subsystem
    @Override
    public void periodic() {
        double timestamp = RobotTime.getTimestampSeconds(); // Time the advantage kit
        // Get subsystem sensor inputs (from real robot)
        io.readInputs(inputs);
        // Send inputs log to advantage kit (processInputs records in real robot, and in replay inject values)
        Logger.processInputs("Drive", inputs);
        // Update inputs (for simulation)
        io.update(inputs);
        // Update odometry
        updateOdometry();
        // Update SmartDashboard values
        UpdateSmartDashboard();
        // Log drive periodic loop latency
        Logger.recordOutput("Drive/latencyPeriodicSec", RobotTime.getTimestampSeconds() - timestamp);
    }

    // Funcion para actualizar el calculo de la odometria en base a los inputs
    private void updateOdometry(){
        // Get modules positions
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[Constants.Drive.kNumberOfModules];
        modulePositions[0] = io.getFrModulePosition();
        modulePositions[1] = io.getFlModulePosition();
        modulePositions[2] = io.getBrModulePosition();
        modulePositions[3] = io.getBlModulePosition();
        // Update odometry (encoders+vision)
        odometry.updateWithTime(inputs.timestamp, Rotation2d.fromDegrees(inputs.yawAngle),modulePositions);
        // Update odometry (encoders only)
        odometry_encoderonly.updateWithTime(inputs.timestamp, Rotation2d.fromDegrees(inputs.yawAngle),modulePositions);
        // Log the odometry positions
        Logger.recordOutput("Drive/odometry",odometry.getEstimatedPosition());
        Logger.recordOutput("Drive/odometry-encoder",odometry_encoderonly.getEstimatedPosition());
    }

    // Comando para correr el main swerve drive al entrar al swerve drive
    public Command startMainDrive(DoubleSupplier rawXSupplier, DoubleSupplier rawYSupplier, DoubleSupplier turnSupplier){
        return new SequentialCommandGroup(
            runOnce(() -> {
                // Init main drive states
                resetYawSetpoint();
            }),
            run(() -> {
                // Read Controller inputs
                double rawX = Util.handleDeadband(rawXSupplier.getAsDouble(), 0.1);
                double rawY = Util.handleDeadband(rawYSupplier.getAsDouble(), 0.1);
                // Ajustamos el field oriented drive, considernado que nuestro 0,0 no cambia entre azul y rojo
                if(Util.isBlueAllience()){
                    rawX = rawX * -1;
                    rawY = rawY * -1;
                } 
            
                double turn = Util.handleDeadband(turnSupplier.getAsDouble(), 0.1) * -1;
                // Run main drive logic
                swerveMainDrive(rawX,rawY,turn);
                // Check odometry to set leds states
                if(Util.isBlueAllience()){
                    if(odometry.getEstimatedPosition().getTranslation().getX()>7.7 && odometry.getEstimatedPosition().getTranslation().getX()<8){
                        // container.getLedsSubsystem().setColor(Section.Left,Colors.Purple);
                    }
                }else{
                    if(odometry.getEstimatedPosition().getTranslation().getX()>9.55 && odometry.getEstimatedPosition().getTranslation().getX()<9.85){
                        // container.getLedsSubsystem().setColor(Section.Left,Colors.Purple);
                    }
                }
            })
        ).finallyDo((interrupted)-> { stopDrive(); } ).withName("Drive MainDriveCommand");
    }

    // Funcion que corre la logica del drive (modo normal)
    public void swerveMainDrive(double _rawX, double _rawY, double _turn) {
        // Se calcula el error de la rotacion del robot
        double deltaAngle = getYawAngleError();
        // Agregamos un acumulador de inercia para permitir el frenado rapido al girar el el propio eje
        // del robot
        boolean yesquickturn = Math.abs(_turn) > Constants.Drive.kDriveTurnDeadband;
        double acumulatorChange = Constants.Drive.kAcumulatorChange;
        double alpha = Constants.Drive.kAcumulatorAlpha;
        if (!yesquickturn) {
            _turn = _turn - quickStopAccumulator;
            if (quickStopAccumulator > acumulatorChange)
                quickStopAccumulator -= acumulatorChange;
            else if (quickStopAccumulator < -acumulatorChange)
                quickStopAccumulator += acumulatorChange;
            else quickStopAccumulator = 0.0;
        } else { // fast turns
            quickStopAccumulator = (1 - alpha) * quickStopAccumulator + alpha * _turn;
        }
        // Se calcula la salida del PID en base a la rotacion del robot y si el robot esta girando
        // actualiza el setpoint.
        if (Math.abs(_turn) < 1e-2) {
            if (Math.abs(deltaAngle) > 10) {
                _turn = robotYawPIDdinamic.calculate(inputs.yawAngle, yawAngleSetPoint);
            } else if(Math.abs(deltaAngle) > 3.5) {
                _turn = robotYawPID.calculate(inputs.yawAngle, yawAngleSetPoint);
            } else {
                _turn = robotYawPIDSmaller.calculate(inputs.yawAngle, yawAngleSetPoint);
            }
        } else {
            resetYawSetpoint();
        }
        // Se convierten los valores de los sticks a velocidad en m/s (van invertidos porque nuestro eje
        // de referencia apunta al frente del robot x positivo)
        double speedX = StickToVelocity(_rawY);
        double speedY = StickToVelocity(_rawX);
        // Convertir el valor de stick a velocidad angular
        double turnSpeed = StickToAngularVelocity(_turn);
        // Cambia los ejes de referencia de las velocidades entre la cancha al robot.
        ChassisSpeeds speeds =
            ChassisSpeeds.fromFieldRelativeSpeeds(
                speedX, speedY, turnSpeed, Rotation2d.fromDegrees((inputs.yawAngle)));

        // Con Inverse Kinematics se convierten las velocidades a los estados de los modulos
        SwerveModuleState[] moduleStates = mSwerveDriveKinematics.toSwerveModuleStates(speeds);
        // Normalizar las velocidades para que no superen el máximo de las velocidades
        SwerveDriveKinematics.desaturateWheelSpeeds(
            moduleStates,
            mMotorUtilDrive.GetMaxVelocity(Constants.Drive.kWheelRadius, Constants.Drive.kMotorRPM));
        // Optimizar los valores de cada angulo en los modulos considerando el cambio de direccion en
        // velocidad parar reducir el giro
        moduleStates[0].optimize(Rotation2d.fromDegrees(inputs.frSteeringPosition));
        moduleStates[1].optimize(Rotation2d.fromDegrees(inputs.flSteeringPosition));
        moduleStates[2].optimize(Rotation2d.fromDegrees(inputs.brSteeringPosition));
        moduleStates[3].optimize(Rotation2d.fromDegrees(inputs.blSteeringPosition));
        // Actualizar el estado de las salidas
        UpdateDriveSpeed(moduleStates);
    }

    // Funcion que corre la logica del drive en modo de pathplanner
    private void swervePathPlannerDrive(ChassisSpeeds speeds) {
        // Con Inverse Kinematics se convierten las velocidades a los estados de los modulos
        SwerveModuleState[] moduleStates = mSwerveDriveKinematics.toSwerveModuleStates(speeds);
        // Normalizar las velocidades para que no superen el máximo de las velocidades
        SwerveDriveKinematics.desaturateWheelSpeeds(
            moduleStates,
            mMotorUtilDrive.GetMaxVelocity(Constants.Drive.kWheelRadius, Constants.Drive.kMotorRPM));
        // Optimizar los valores de cada angulo en los modulos considerando el cambio de direccion en
        // velocidad parar reducir el giro
        moduleStates[0].optimize(Rotation2d.fromDegrees(inputs.frSteeringPosition));
        moduleStates[1].optimize(Rotation2d.fromDegrees(inputs.flSteeringPosition));
        moduleStates[2].optimize(Rotation2d.fromDegrees(inputs.brSteeringPosition));
        moduleStates[3].optimize(Rotation2d.fromDegrees(inputs.blSteeringPosition));
        // Actualizar el estado de las salidas
        UpdateDriveSpeed(moduleStates);
    }

    // Funcion para actualizar el estado de salidas de los modulos
    private void UpdateDriveSpeed(SwerveModuleState[] moduleStates) {
        double deltadegrees = 0;
        double frSteeringOutput =0, frDriveOutput =0,
               flSteeringOutput =0, flDriveOutput =0,
               brSteeringOutput =0, brDriveOutput =0,
               blSteeringOutput =0, blDriveOutput =0;
        // Revisamos si la velocidad es muy baja y seteamos a 0 si no calculamos los valores deseados.
        if (Math.abs(moduleStates[0].speedMetersPerSecond) < 0.1) {
            frSteeringOutput = 0;
            frDriveOutput = 0;
        } else {
            // Calculamos la diferencia entre el angulo deseado y el angulo actual del modulo
            deltadegrees =
                Util.DeltaAngle(
                    (double) Util.Limit360Angle(moduleStates[0].angle.getDegrees()),
                    inputs.frSteeringPosition);
            // Aplicamos el controlador PD para calcular la salida del motor steering
            frSteeringOutput =
                deltadegrees * Constants.Drive.kPSteeringValue
                    + (deltadegrees - lastError[0])
                        / inputs.deltaTime
                        * Constants.Drive.kDSteeringValue;
            // Actualizamos el ultimo error del motor de steering
            lastError[0] = deltadegrees;
            // Convertimos la velocidad lineal a porcentaje para el motor de velocidad.
            frDriveOutput =
                mMotorUtilDrive.GetPercentageFromVelocity(
                    moduleStates[0].speedMetersPerSecond,
                    Constants.Drive.kWheelRadius,
                    Constants.Drive.kMotorRPM);
        }
        // Revisamos si la velocidad es muy baja y seteamos a 0 si no calculamos los valores deseados.
        if (Math.abs(moduleStates[1].speedMetersPerSecond) < 0.1) {
            flSteeringOutput = 0;
            flDriveOutput = 0;
        } else {
            // Calculamos la diferencia entre el angulo deseado y el angulo actual del modulo
            deltadegrees =
                Util.DeltaAngle(
                    (double) Util.Limit360Angle(moduleStates[1].angle.getDegrees()),
                    inputs.flSteeringPosition);
            // Aplicamos el controlador PD para calcular la salida del motor steering
            flSteeringOutput =
                deltadegrees * Constants.Drive.kPSteeringValue
                    + (deltadegrees - lastError[1])
                        / inputs.deltaTime
                        * Constants.Drive.kDSteeringValue;
            // Actualizamos el ultimo error del motor de steering
            lastError[1] = deltadegrees;
            // Convertimos la velocidad lineal a porcentaje para el motor de velocidad.
            flDriveOutput =
                mMotorUtilDrive.GetPercentageFromVelocity(
                    moduleStates[1].speedMetersPerSecond,
                    Constants.Drive.kWheelRadius,
                    Constants.Drive.kMotorRPM);
        }
        // Revisamos si la velocidad es muy baja y seteamos a 0 si no calculamos los valores deseados.
        if (Math.abs(moduleStates[2].speedMetersPerSecond) < 0.1) {
            brSteeringOutput = 0;
            brDriveOutput = 0;
        } else {
            // Calculamos la diferencia entre el angulo deseado y el angulo actual del modulo
            deltadegrees =
                Util.DeltaAngle(
                    (double) Util.Limit360Angle(moduleStates[2].angle.getDegrees()),
                    inputs.brSteeringPosition);
            // Aplicamos el controlador PD para calcular la salida del motor steering
            brSteeringOutput =
                deltadegrees * Constants.Drive.kPSteeringValue
                    + (deltadegrees - lastError[2])
                        / inputs.deltaTime
                        * Constants.Drive.kDSteeringValue;
            // Actualizamos el ultimo error del motor de steering
            lastError[2] = deltadegrees;
            // Convertimos la velocidad lineal a porcentaje para el motor de velocidad.
            brDriveOutput =
                mMotorUtilDrive.GetPercentageFromVelocity(
                    moduleStates[2].speedMetersPerSecond,
                    Constants.Drive.kWheelRadius,
                    Constants.Drive.kMotorRPM);
        }
        // Revisamos si la velocidad es muy baja y seteamos a 0 si no calculamos los valores deseados.
        if (Math.abs(moduleStates[3].speedMetersPerSecond) < 0.1) {
            blSteeringOutput = 0;
            blDriveOutput = 0;
        } else {
            // Calculamos la diferencia entre el angulo deseado y el angulo actual del modulo
            deltadegrees =
                Util.DeltaAngle(
                    (double) Util.Limit360Angle(moduleStates[3].angle.getDegrees()),
                    inputs.blSteeringPosition);
            // Aplicamos el controlador PD para calcular la salida del motor steering
            blSteeringOutput =
                deltadegrees * Constants.Drive.kPSteeringValue
                    + (deltadegrees - lastError[3])
                        / inputs.deltaTime
                        * Constants.Drive.kDSteeringValue;
            // Actualizamos el ultimo error del motor de steering
            lastError[3] = deltadegrees;
            // Convertimos la velocidad lineal a porcentaje para el motor de velocidad.
            blDriveOutput =
                mMotorUtilDrive.GetPercentageFromVelocity(
                    moduleStates[3].speedMetersPerSecond,
                    Constants.Drive.kWheelRadius,
                    Constants.Drive.kMotorRPM);
        }
        // Write outputs
        io.writeOutputs(frSteeringOutput, frDriveOutput,
                        flSteeringOutput, flDriveOutput,
                        brSteeringOutput, brDriveOutput,
                        blSteeringOutput, blDriveOutput);
    }

    // Funcion para convertir el valor del stick a velocidad lineal
    // @param {double} stickInput: valor del stick a convertir
    private double StickToVelocity(double stickInput) {
        return stickInput
            * mMotorUtilDrive.GetMaxVelocity(Constants.Drive.kWheelRadius, Constants.Drive.kMotorRPM);
    }

    // Funcion para convertir el valor del stick a velocidad angular
    // @param {double} stickInput: valor del stick a convertir
    private double StickToAngularVelocity(double stickInput) {
        return stickInput
            * (mMotorUtilDrive.GetMaxVelocity(Constants.Drive.kWheelRadius, Constants.Drive.kMotorRPM)
                / (double) Constants.Drive.kWheelTrack);
    }

    // Funcion para leer las velocidades reales del chassis
    private ChassisSpeeds getChassisSpeeds(){
        SwerveModuleState[] modulePositions = new SwerveModuleState[Constants.Drive.kNumberOfModules];
        modulePositions[0] = io.getFrModuleState();
        modulePositions[1] = io.getFlModuleState();
        modulePositions[2] = io.getBrModuleState();
        modulePositions[3] = io.getBlModuleState();
        return mSwerveDriveKinematics.toChassisSpeeds(modulePositions);
    }

    // Funcion para obtener los inputs del subsistema
    public DriveInputsAutoLogged getInputs(){
        return inputs;
    }

    // Funcion para resetar el pose de las odometrias
    private void resetPose(Pose2d pose){
        if(odometry!=null){
            odometry.resetPose(pose);
        }
        if(odometry_encoderonly!=null){
            odometry_encoderonly.resetPose(pose);
        }
    }

    // Funcion para leer el pose de la odometria del chasis
    public Pose2d getPose(){
        return odometry.getEstimatedPosition();
    }

    // Funcion para agregar los valores de la camara para actualizar la odometria
    public void AddVisionMeasurement(Pose2d measure, double confidence, double timestamp){
        if(measure!=null){
            odometry.setVisionMeasurementStdDevs(VecBuilder.fill(confidence,confidence,9999999));
            odometry.addVisionMeasurement(measure,timestamp);
            Logger.recordOutput("Drive/odometry-vision",measure);
        }
    }

    // Funcion para hacer el auto align al reef (usando pathplanner y pid drive)
    // Cada que se llama la funcion creamos un comando con un contexto nuevo para que tenga sus valores independientes
    public Command alignToReef(Position position, int zone, BooleanSupplier interrupt){
        AlignToReefContext context = new AlignToReefContext();
        return runOnce(() -> {
            // container.getLedsSubsystem().setColor(Section.Edges,Colors.Red);
            // Get y offset
            context.yOffset = 0.55; 
            // Get x offset
            context.xOffset = 0;
            if(position==Position.left){
                context.xOffset = -0.12;
            }else if(position==Position.right){
                context.xOffset = 0.2;
            }
            // Get odometry of the robot
            context.robotPose = getPose();
            // Determine in which zone is the robot (if needed)
            context.zone = (zone < 0) ? driveZone.getRobotZone(context.robotPose) : zone;
            // Calculate distance to target point
            if (context.zone < 0) {
                context.skipCommand=true;
                context.distanceToTarget = -1;
            } else {
                context.skipCommand=false;
                // Calculate distance to the target point
                context.distanceToTarget = context.robotPose.getTranslation()
                        .getDistance(driveZone.getReefZonesPathPoints(context.zone).getTranslation());
            }
        }).andThen(
            new ConditionalCommand(
                Commands.none(),
                new ConditionalCommand(
                    new SequentialCommandGroup(
                        // Usamos el proxy command, para que no genere el comando en la creacion, sino hasta que lo corra (para tener la zone correcta)
                        new LazyCommandWrapper(() -> getOnTheFlyPathPlannerCommand(driveZone.getReefZonesPathPoints(context.zone))),
                        new LazyCommandWrapper(() -> pidDrive.runPidDrive(context.xOffset, context.yOffset, context.zone))
                    ),
                    new LazyCommandWrapper(() -> pidDrive.runPidDrive(context.xOffset, context.yOffset, context.zone)),
                    () -> context.distanceToTarget > 1.0 && pathPlannerOnTheFlyEnabled
                ),
                () -> context.skipCommand
            )
        ).until(interrupt)
        .finallyDo((interrupted) -> {
            if(!pidDrive.finishedCorrectly()){
                // container.getLedsSubsystem().setColor(Section.Edges,Colors.Yellow);
            }else{
                // container.getLedsSubsystem().setColor(Section.Edges,Colors.Green);
            }
            stopDrive();
        });
    }

    // Funcion para correr el pathplanner mode creando el path on the fly considerando la posicion actual del robot
    public Command getOnTheFlyPathPlannerCommand(Pose2d endPos) {
        Pose2d currentPose = odometry.getEstimatedPosition();
        Pose2d startPos = new Pose2d(currentPose.getTranslation(), new Rotation2d());
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPos,new Pose2d(endPos.getTranslation(), new Rotation2d()));
        PathConstraints constraints = new PathConstraints(2.0, 2.0,Units.degreesToRadians(360), Units.degreesToRadians(540));
        // Create the path using the waypoints created above
        PathPlannerPath path = new PathPlannerPath(
            waypoints,
            constraints,
            null,
            new GoalEndState(0.0, Rotation2d.fromDegrees(endPos.getRotation().getDegrees()))
        );
        path.preventFlipping = true;
        return AutoBuilder.followPath(path).finallyDo((interrupted)->{ resetYawSetpoint(); stopDrive(); });
    }

    // Funcion para detener el drive
    public void stopDrive() {
        // Reseteamos el acumulador de iniercia
        quickStopAccumulator = 0;
        // Detenemos los motores
        io.writeOutputs(0,0,0,0,0,0,0,0);
    }

    // Funcion para reseter el yaw setpoint (poniendo el valor actual de la posicion del robot)
    public void resetYawSetpoint() {
        yawAngleSetPoint = inputs.yawAngle;
    }

    // Funcion para actualizar el setpoint de yaw angle del robot (considerando es un valor acumulativo)
    public void updateYawSetpoint(double angle){
        double temp = (yawAngleSetPoint%360);
        temp = Util.DeltaAngle(Util.Limit360Angle(angle), temp);
        yawAngleSetPoint = yawAngleSetPoint + temp;
    }

    // Funcion para leer el error de yaw del robot
    public double getYawAngleError(){
        return yawAngleSetPoint - inputs.yawAngle;
    }

    // Funcion para actualizar lo medido por la camara para el pid drive
    public void updatePidVisionMeasure(DrivePidDrive.VisionMeasure measure){
        pidDrive.updateMeasure(measure);
    }

    // Funcion para saber si ya acabo el pid drive
    public boolean finishedPidSwerveDrive(){
        return pidDrive.finished();
    }

    // Funcion para saber el estado del pid drive
    public int getPidSwerveDriveState(){
        return pidDrive.getState();
    }

    // Funcion para saber si es un tag valido dependiendo la zona que estamos ajustando
    public boolean getPidIsValidTag(int tagID){
        return pidDrive.isTagValid(tagID);
    }


    // Funcion para escribir informacion a la smath dashboard
    private void UpdateSmartDashboard(){
        if(logToSmarthDashBoard){
            SmartDashboard.putNumber("Yaw", inputs.yawAngle);
            SmartDashboard.putNumber("Yawsetpoint", yawAngleSetPoint);
            SmartDashboard.putNumber("Yaw Angle rate", inputs.yawAngleRate);
            SmartDashboard.putString("Odometry", odometry.getEstimatedPosition().toString());
            SmartDashboard.putNumber("Steering Angle FR", inputs.frSteeringPosition);
            SmartDashboard.putNumber("Steering Angle FL", inputs.flSteeringPosition);
            SmartDashboard.putNumber("Steering Angle BR", inputs.brSteeringPosition);
            SmartDashboard.putNumber("Steering Angle BL", inputs.blSteeringPosition);
            SmartDashboard.putString("Module State Fr", io.getFrModuleState().toString());
            SmartDashboard.putString("Module State Fl", io.getFlModuleState().toString());
            SmartDashboard.putString("Module State Br", io.getBrModuleState().toString());
            SmartDashboard.putString("Module State Bl", io.getBlModuleState().toString());
            SmartDashboard.putString("Chassis Speeds", getChassisSpeeds().toString());
        }
    }

    // Clase para guardar la infomacion que se pasa entre comandos del pid drive
    public class AlignToReefContext {
        public int zone = -1;
        public double xOffset = 0;
        public double yOffset = 0;
        public Pose2d robotPose = null;
        public double distanceToTarget = -1;
        public boolean skipCommand = false;
    }

}