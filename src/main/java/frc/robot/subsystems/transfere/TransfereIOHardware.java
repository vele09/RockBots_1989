package frc.robot.subsystems.transfere;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;

public class TransfereIOHardware implements TransfereIO{

    private SparkMax mini_NEO;

    public TransfereIOHardware() {

        mini_NEO = new SparkMax(Constants.Intake.kmini_NEO_ID, MotorType.kBrushless);
    }

    public void writeOutputs(double output) {
        mini_NEO.set(output);
    }

    
}
