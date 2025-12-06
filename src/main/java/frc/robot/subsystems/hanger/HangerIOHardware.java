package frc.robot.subsystems.hanger;

import frc.robot.Constants;
import frc.robot.Constants.Hanger;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.motorcontrol.Talon;


public class HangerIOHardware implements HangerIO {
    private TalonSRX h_motor ;

    public HangerIOHardware() { 
        h_motor = new TalonSRX(Constants.Hanger.kHangerMotor1ID); //No tenego idea de como se hace :(
        
    }

    public void writeOutputs(double output) {
        h_motor.set(TalonSRXControlMode.PercentOutput, output);
    }

}   
