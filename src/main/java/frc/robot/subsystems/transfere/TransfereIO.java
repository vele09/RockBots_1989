package frc.robot.subsystems.transfere;

public interface TransfereIO {
    class TransereInputs {
    }

    // Leer entradas del robot real
    default void readInputs(TransereInputs inputs) {}

    // Actualizar inputs en simulación
    default void update(final TransereInputs inputs) {}

    // Escribir outputs a los motores
    default void writeOutputs(double output) {}
}
