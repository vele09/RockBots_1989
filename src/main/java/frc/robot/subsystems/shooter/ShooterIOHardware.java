package frc.robot.subsystems.shooter;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import frc.robot.Constants;

//Agregar un TOF ¿Cómo? 

public class ShooterIOHardware implements ShooterIO {
    private SparkMax low_motor;
    private SparkMax up_motor;
    private double count;
    private double speed;

    public ShooterIOHardware(){
        speed = 0;
        //Motores
        low_motor = new SparkMax(Constants.Shooter.kShooterLowMotorID, MotorType.kBrushless);
        up_motor = new SparkMax(Constants.Shooter.kShooterUpperMotorID, MotorType.kBrushless);

        //TOF
        tofSensor = new TimeOfFlight(Constants.Shooter.kTofSensorID);
        tofSensor.setRangingMode(RangingMode.Short, 24);

    }

    public void writeOutputs(double output) {
        low_motor.set(output);
        up_motor.set(output);
    }
}
