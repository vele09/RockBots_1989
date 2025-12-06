package frc.robot.subsystems.hanger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.Hanger;

public class HangerSubsystem extends SubsystemBase{

     private HangerIO io;
    private RobotContainer container;
    
    public HangerSubsystem (final HangerIO io,final RobotContainer container) {
        this.io = io;
        this.container = container;
    }

    public Command runHangerCommand() {
        return run(() -> {
            // Ruedas directas sin verificar zona segura
            io.writeOutputs(0.5);

        })
        .finallyDo(interrupted -> {
            io.writeOutputs(0);
   
        })
        .withName("Hanger RunWheelsUnsafeCommand");
    }

    public Command downHangerCommand() {
        return run(() -> {
            // Ruedas directas sin verificar zona segura
            io.writeOutputs(-0.5);

        })
        .finallyDo(interrupted -> {
            io.writeOutputs(0);
   
        })
        .withName("Hanger RunWheelsUnsafeCommand");
    }

    public void HangerControlLoop(){
        //NO se necesita nada
    }

    public Command controlLoopCommand (){   //Esto se repite en cada código
        return run(() -> HangerControlLoop())
        .finallyDo (interrupted -> {
            io.writeOutputs(0); 
        })
        .withName("Hanger ControlLoopCommand");
    }

}
