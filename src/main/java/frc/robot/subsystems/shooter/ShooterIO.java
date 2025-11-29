package frc.robot.subsystems.shooter;

import frc.robot.Constants;
import frc.robot.subsystems.intake.IntakeIO.IntakeInputs;


public interface ShooterIO {

    class ShooterInputs {
        public double speed;

        // Leer entradas del robot real
    default void readInputs(ShooterInputs inputs) {}

    // Actualizar inputs en simulación
    default void update(final ShooterInputs inputs) {}

    // Escribir outputs a los motores
    default void writeOutputs(double output) {}

<<<<<<< Updated upstream

=======
    double getTofDistance();

    boolean pieceReady();
>>>>>>> Stashed changes
    
}

    }

    
    
    

