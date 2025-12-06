package frc.robot.subsystems.transfere;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;


public class TransfereSubsystem extends SubsystemBase{
    private TransfereIO io;
    private RobotContainer container;

    public TransfereSubsystem (final TransfereIO io,final RobotContainer container) {
        this.io = io;
        this.container = container;
    }

    public Command comer_TransfereCommand() {
        return run(() -> {
            // Ruedas directas sin verificar zona segura
            io.writeOutputs(-0.5);

        })
        .finallyDo(interrupted -> {
            io.writeOutputs(0);
   
        })
        .withName("Transfere Command");
    }

    
}
