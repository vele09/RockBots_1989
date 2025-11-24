package frc.robot;

import frc.robot.controlboard.ControlBoard;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.autocommands.DoNothingAuto;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.intake.IntakeIOHardware;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeSubsystem;


public class RobotContainer {

  //Llaman los subsistemas
  private final IntakeSubsystem intakeSubsystem = buildIntakeSubsystem();
  
  private final ControlBoard controlBoard;

  
  public RobotContainer() { //Constructor
    // Configure the trigger bindings
    controlBoard = ControlBoard.getInstance();

    configureBindings();
  }
 //Funciones para crear los subsistemas
  private IntakeSubsystem buildIntakeSubsystem() {
    if (RobotBase.isSimulation()){
      return new IntakeSubsystem(new IntakeIOSim(), this);
    } else{
      return new IntakeSubsystem(new IntakeIOHardware(), this);
    }
  }

    public IntakeSubsystem getIntakeSubsystem() {
      return intakeSubsystem;
  }
  
  private void configureBindings() {
    
    intakeSubsystem.setDefaultCommand(
      intakeSubsystem.controlLoopCommand() //No estoy segura si esto va asi :(
    );

      controlBoard.driverLeftBumper().whileTrue(
        intakeSubsystem.runWheelsUnsafeCommand().withName("driver_EatPiece")

  );



  }

}

