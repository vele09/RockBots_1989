package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
    IntakeIOHardware io = new IntakeIOHardware();

    public Command runWheelsUnsafeCommand() {
        return run(() -> {
            // Ruedas directas sin verificar zona segura
            io.writeOutputs(0.1);
        })
        .finallyDo(interrupted -> {
            io.writeOutputs(0);
        })
        .withName("Intake RunWheelsUnsafeCommand");
    }

}
