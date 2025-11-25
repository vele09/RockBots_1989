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

    public void setShooterSpeed(double speed) {
        io.writeOutputs(speed); // aquí dentro ShooterIO pone los dos motores a esa velocidad
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


        private boolean hasPieceInShooter() {
            var intake = container.getIntakeSubsystem();
            if (intake == null) return false;
        
            
            return intake.hasGamePiece();
        }
        



        public Command shootWhenPieceDetectedCommand() {
            return run(() -> {
                if (hasPieceInShooter()) {
                    setShooterSpeed(0.6);
                } else {
                    setShooterSpeed(0.0);
                }
            }).finallyDo(interrupted -> {
                setShooterSpeed(0.0);
            }).withName("Shooter ShootWhenPieceDetected");
        }
        

    
}
