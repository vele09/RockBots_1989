package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeIOHardware;

public class ShooterSubsystem extends SubsystemBase {

    ShooterIOHardware io = new ShooterIOHardware();

    public Command ShooterRun() {
        return run(() -> {
            // Ruedas directas sin verificar zona segura
            io.writeOutputs(0.1);
        })
        .finallyDo(interrupted -> {
            io.writeOutputs(0);
        })
        .withName("Shooter RunWheelsUnsafeCommand");
    }
    
}
