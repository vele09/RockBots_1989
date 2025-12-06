package frc.robot.subsystems.hanger;

    public interface HangerIO {
        class HangerInputs {
        }
    
        // Leer entradas del robot real
        default void readInputs(HangerInputs inputs) {}
    
        // Actualizar inputs en simulación
        default void update(final HangerInputs inputs) {}
    
        // Escribir outputs a los motores
        default void writeOutputs(double output) {}
    }

