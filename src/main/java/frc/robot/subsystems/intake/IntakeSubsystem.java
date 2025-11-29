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
        //NO se necesita nada
    }

    public Command controlLoopCommand (){   //Esto se repite en cada código
        return run(() -> intakeControlLoop())
        .finallyDo (interrupted -> {
            io.writeOutputs(0); 
        })
        .withName("Intake ControlLoopCommand");
    }

    public Command comerCommand(double Trigger){
        return runOnce(() -> Comer(Trigger))
        .finallyDo(interrupted -> {
            io.writeOutputs(0); } );
        }

        public void Comer(double Trigger) {
            io.writeOutputs(Trigger); //Con que boton o que sea fija
        }

        

}
