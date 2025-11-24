package frc.robot.commands.autocommands;

import frc.robot.RobotContainer;

import edu.wpi.first.wpilibj2.command.*;

public class TemplateAuto extends SequentialCommandGroup {

    public TemplateAuto(RobotContainer container) {
        // Agregamos siempre un log para validar que cargo correctamente
        System.out.println("TemplateAuto Loaded");

        // Step 1: Print a message and wait for 1 second
        addCommands(
            new PrintCommand("Starting Auto"),
            new WaitCommand(1.0)
        );

        // Step 2: Run two commands in parallel for 5 seconds
        addCommands(
            new ParallelCommandGroup(
                new RunCommand(() -> System.out.println("Driving...")).withTimeout(1.0),
                new RunCommand(() -> System.out.println("Lowering Intake...")).withTimeout(1.0)
            )
        );

        // Step 3: Wait for 1 second
        addCommands(new WaitCommand(1.0));

        // Step 4: Run another command until a random condition or timeout occurs
        addCommands(
            new RunCommand(() -> System.out.println("Driving to Target..."))
                .withTimeout(5.0)
                .until(() -> Math.random() > 0.95)  // Interrupt if random value > 0.95
        );

        // Step 5: Print "Auto Complete"
        addCommands(new PrintCommand("Auto Complete"));
    }
}