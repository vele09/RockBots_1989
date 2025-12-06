package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;

public class IntakeIOHardware implements IntakeIO {
    private SparkMax m_motor;
    private SparkMax mini_NEO;
    private double count;
    private double speed;

    public IntakeIOHardware() {
        speed = 0;
        m_motor = new SparkMax(Constants.Intake.kIntakeLiftID, MotorType.kBrushless);
        mini_NEO = new SparkMax(Constants.Intake.kmini_NEO_ID, MotorType.kBrushless);
    }

    public void writeOutputs(double output) {
        m_motor.set(output);
        mini_NEO.set(output);
    }
}
