package frc.robot.subsystems.shooter;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.SparkMax;

import frc.robot.Constants;

//Agregar un TOF ¿Cómo? 

public class ShooterIOHardware implements ShooterIO {
    private SparkMax low_motor;
    private SparkMax up_motor;
    private double count;
    private double speed;

    public ShooterIOHardware(){
        speed = 0;
        low_motor = new SparkMax(Constants.Shooter.kShooterLowMotorID, MotorType.kBrushless);
        up_motor = new SparkMax(Constants.Shooter.kShooterUpperMotorID, MotorType.kBrushless);
    }
}
