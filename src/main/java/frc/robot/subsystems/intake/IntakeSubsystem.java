package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.DriveIO;

public class IntakeSubsystem extends SubsystemBase {
    private IntakeIO io;
    private RobotContainer container;

    public IntakeSubsystem (final IntakeIO io,final RobotContainer container) {
        this.io = io;
        this.container = container;
    }

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

    public void intakeControlLoop(){
        //Va lo de checar sensores y asi
        //Seguridades
    }

    public Command controlLoopCommand (){   //Esto se repite en cada código
        return run(() -> intakeControlLoop())
        .finallyDo (interrupted -> {
            io.writeOutputs(0); 
        })
        .withName("Intake ControlLoopCommand");
    }

    //agregar una funcón que mueva las ruedas ¿Como) No se :(

}
