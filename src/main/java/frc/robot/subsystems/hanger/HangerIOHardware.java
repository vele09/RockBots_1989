package frc.robot.subsystems.hanger;

import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.Constants;


public class HangerIOHardware implements HangerIO {
    private TalonFX h_motor ;

    public HangerIOHardware() { 
        h_motor = new TalonFX(Constants.Hanger.kHangerMotor1ID); //No tenego idea de como se hace :(
        
    }

    public void writeOutputs(double output) {
        h_motor.set(output);
    }

}   
