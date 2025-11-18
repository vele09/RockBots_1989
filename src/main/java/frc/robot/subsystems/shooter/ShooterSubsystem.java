package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOHardware;

public class ShooterSubsystem extends SubsystemBase {

     private ShooterIO io;
    private RobotContainer container;

    public ShooterSubsystem (final ShooterIO io,final RobotContainer container) {
        this.io = io;
        this.container = container;
    }


    public Command ShooterRun() {
        return run(() -> {
            // Ruedas directas sin verificar zona segura
            io.writeOutputs(0.1);
        })
        .finallyDo(interrupted -> {
            io.writeOutputs(0);
        })
        .withName("Shooter RunWheelsUnsafeCommand");
        
    }//agregar una funcón que mueva las ruedas ¿Como) No se :(

    
}
