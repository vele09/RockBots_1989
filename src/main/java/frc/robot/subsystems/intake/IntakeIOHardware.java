package frc.robot.subsystems.intake;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.robot.Constants;

import com.revrobotics.spark.SparkMax;

public class IntakeIOHardware implements IntakeIO {
    private SparkMax m_motor;
    private double count;
    private double speed;

    public IntakeIOHardware() {
        speed = 0;
        m_motor = new SparkMax(Constants.Intake.kIntakeLiftID, MotorType.kBrushless);
    }

    public void writeOutputs(double output) {
        m_motor.set(output);
    }
}
