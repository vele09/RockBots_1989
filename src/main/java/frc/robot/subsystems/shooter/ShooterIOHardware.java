package frc.robot.subsystems.shooter;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import java.util.ArrayList;
import java.util.List;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import frc.robot.Constants;

//Agregar un TOF ¿Cómo? 

public class ShooterIOHardware implements ShooterIO {
    private SparkMax low_motor;
    private SparkMax up_motor;
    private double count;
    private double speed;
    private TimeOfFlight sensor1;
    private List<Double> filterSensor1;
    private int filterSize = 2;
    private double lastSensor1Measure;

    public ShooterIOHardware(){
        speed = 0;
        //Motores
        low_motor = new SparkMax(Constants.Shooter.kShooterLowMotorID, MotorType.kBrushless);
        up_motor = new SparkMax(Constants.Shooter.kShooterUpperMotorID, MotorType.kBrushless);

        //TOF
        sensor1 = new TimeOfFlight(Constants.Shooter.kShooterSensor1ID);
        sensor1.setRangingMode(RangingMode.Short, 24);
        //Filtros
        filterSensor1 = new ArrayList<>();
        for (int x = 0; x < filterSize; x++) {
            filterSensor1.add(0.0);
        }
        lastSensor1Measure = 0;
    }

    public void writeOutputs(double output) {
        low_motor.set(output);
        up_motor.set(output);
    }

    @Override
    public double getTofDistance() {
        // Usar promedio filtrado 
        double average = 0;
        for (int x = 0; x < filterSize; x++) {
            average += filterSensor1.get(x);
        }
        return average / filterSize;
    }

    @Override
    public boolean pieceReady() {
        return getTofDistance() < Constants.Shooter.kTofThreshold;
    }
}
