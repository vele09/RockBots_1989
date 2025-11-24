package frc.robot.commands.autocommands;

import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class DoNothingAuto extends SequentialCommandGroup {

    public DoNothingAuto(RobotContainer container) {
        // Agregamos siempre un log para validar que cargo correctamente
        System.out.println("Do Nothing Auto Loaded");
        addCommands(Commands.none());
    }
}
