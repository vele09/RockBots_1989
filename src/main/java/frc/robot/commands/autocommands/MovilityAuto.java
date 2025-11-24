package frc.robot.commands.autocommands;

import com.pathplanner.lib.commands.PathPlannerAuto;
import frc.robot.RobotContainer;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj2.command.*;

public class MovilityAuto extends SequentialCommandGroup {

    public MovilityAuto(RobotContainer container) {
        // Agregamos siempre un log para validar que cargo correctamente
        System.out.println("Movility Auto Loaded");
        // Logic to run after a pathplanner command
        BooleanConsumer endPathPlanner = (interrupted)->{container.getDriveSubsystem().resetYawSetpoint(); container.getDriveSubsystem().stopDrive();};
        // Test a pathplanner
        addCommands(
            new PathPlannerAuto("MobilityAuto").finallyDo(endPathPlanner));
    }
}