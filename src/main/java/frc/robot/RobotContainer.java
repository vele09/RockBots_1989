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


public class RobotContainer {
  private final IntakeSubsystem intakeSubsystem = buildIntakeSubsystem();
  
  private final ControlBoard controlBoard;

  
  public RobotContainer() {
    // Configure the trigger bindings
    controlBoard = ControlBoard.getInstance();

    configureBindings();
  }

  
  private void configureBindings() {
    

      controlBoard.driverLeftBumper().whileTrue(
        intakeSubsystem.runWheelsUnsafeCommand().withName("driver_EatPiece")



  );
    
  }

  private IntakeSubsystem buildIntakeSubsystem() {
      return new IntakeSubsystem();
  }


  
}
