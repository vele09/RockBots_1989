package frc.robot.subsystems.intake;

import frc.robot.Constants;

public interface IntakeIO {
    class IntakeInputs {
        public double position;
    }

    // Leer entradas del robot real
    default void readInputs(IntakeInputs inputs) {}

    // Actualizar inputs en simulación
    default void update(final IntakeInputs inputs) {}

    // Escribir outputs a los motores
    default void writeOutputs(double output) {}
}
