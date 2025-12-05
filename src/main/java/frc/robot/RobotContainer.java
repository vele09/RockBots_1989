package frc.robot;

import frc.robot.controlboard.ControlBoard;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.intake.IntakeIOHardware;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterIOHardware;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.DriveIO;
import frc.robot.subsystems.drive.DriveIOHardware;
import frc.robot.subsystems.drive.DriveIOSim;


public class RobotContainer {

  //Llaman los subsistemas
  private final DriveSubsystem driveSubsystem = buildDriveSubsystem();
  private final IntakeSubsystem intakeSubsystem = buildIntakeSubsystem();

  private final ShooterSubsystem shooterSubsystem = buildShooterSubsystem();

  private final ControlBoard controlBoard;

  
  public RobotContainer() { //Constructor
    // Configure the trigger bindings
    controlBoard = ControlBoard.getInstance();

    configureBindings();
  }
 //Funciones para crear los subsistemas
  private DriveSubsystem buildDriveSubsystem() {
    if (RobotBase.isSimulation()){
      return new DriveSubsystem(new DriveIOSim(), this);
    } else{
      return new DriveSubsystem(new DriveIOHardware(), this);
    }
  }

  private IntakeSubsystem buildIntakeSubsystem() {
    if (RobotBase.isSimulation()){
      return new IntakeSubsystem(new IntakeIOSim(), this);
    } else{
      return new IntakeSubsystem(new IntakeIOHardware(), this);
    }
  }

  private ShooterSubsystem buildShooterSubsystem() {
    if (RobotBase.isSimulation()){
      return new ShooterSubsystem(new ShooterIOSim(), this);
    } else{
      return new ShooterSubsystem(new ShooterIOHardware(), this);
    }
  }

  public DriveSubsystem getDriveSubsystem() {
    return driveSubsystem;
  }

  public IntakeSubsystem getIntakeSubsystem() {
    return intakeSubsystem;
  }

  public ShooterSubsystem gShooterSubsystem (){
    return shooterSubsystem;
  }
  
  private void configureBindings() {

    // Configurar comando por defecto del drive
    driveSubsystem.setDefaultCommand(
      driveSubsystem.startMainDrive(
        controlBoard::driverLeftStickY,
        controlBoard::driverLeftStickX,
        controlBoard::driverRightStickX
      )
    );

    intakeSubsystem.setDefaultCommand(
      intakeSubsystem.controlLoopCommand() //No estoy segura si esto va asi :(
    );

    controlBoard.driverLeftBumper().whileTrue(
      intakeSubsystem.runWheelsUnsafeCommand().withName("driver_EatPiece")
    );
    
    controlBoard.driverAButton().whileTrue(
      shooterSubsystem.dispararRockola().withName("driver_ShootRockola")
    );

    controlBoard.driverBButton().whileTrue(
      shooterSubsystem.dispararTocadiscos().withName("driver_ShootTOcaDiscos")
    );

    controlBoard.driverLeftTrigger().whileTrue(
      intakeSubsystem.comerCommand(controlBoard.driverRightTriggerValue())
    );
    
  }

  


  
}
