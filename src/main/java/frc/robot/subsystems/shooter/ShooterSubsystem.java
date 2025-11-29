package frc.robot.subsystems.shooter;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOHardware;
import lib.time.RobotTime;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShooterSubsystem extends SubsystemBase {

    private ShooterIO io;
    private RobotContainer container;

    public ShooterSubsystem (final ShooterIO io,final RobotContainer container) {
        this.io = io;
        this.container = container;
    }

      @Override
    public void periodic() {
        UpdateSmartDashboard();

    }

    public void shooterControlLoop(){
        //Va lo de checar sensores y asi
        //Seguridades

        
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
        
        
    public void Disparar(boolean shoot, boolean leave) {
        boolean pieceReady = io.pieceReady();
        if(pieceReady && shoot){
            io.writeOutputs(0.6); //Con que boton o que sea fija
        }
        else if(pieceReady && leave ){
            io.writeOutputs(0.3);
            
        }
        else{
            io.writeOutputs(0); 
            
        }
    
    }

    // Dashboard - similar structure to Straightnator
    private void UpdateSmartDashboard() {
        SmartDashboard.putBoolean("ShooterTOF", io.pieceReady());
}
    


    
}
