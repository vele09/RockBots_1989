package frc.robot.subsystems.intake;

import frc.robot.Constants;
import edu.wpi.first.wpilibj.Timer;

///////////////////////////////////////////////////////////////////////////////
// Description: Simulación de los métodos de entrada/salida del subsistema Intake
// People:
///////////////////////////////////////////////////////////////////////////////
public class IntakeIOSim implements IntakeIO {

    // Posiciones de los motores (rotaciones)
    private double motorLiftPosition = 0.0;   
    private double motorWheelPosition = 0.0;  

    private double motorLiftOutput = 0.0;    
    private double motorWheelOutput = 0.0;    

    // TOF 1 simulado 
    private double tofDistance = 1000.0; 

    @Override
    public void readInputs(IntakeInputsAutoLogged inputs) {
        double now = Timer.getFPGATimestamp();

        // Calcular deltaTime
        if (inputs.timestamp > 0) {
            inputs.deltaTime = now - inputs.timestamp;
        }
        inputs.timestamp = now;

        // Actualizar posición del motor lift usando output simulado
        motorLiftPosition += motorLiftOutput * inputs.deltaTime;

        // Actualizar posición del motor wheels (simulación básica, opcional)
        motorWheelPosition += motorWheelOutput * inputs.deltaTime;

        //inputs.hasGamePiece = hasGamePiece();
        //inputs.limitSwitch = getLimitSwitch();

        // TOF simulado
        inputs.tofDistance = tofDistance;
        inputs.hasGamePiece = tofDistance > 0 && tofDistance < Constants.Intake.kTofThreshold;
    }

    @Override
    public void writeOutputs(double liftOutput) {
        // Solo se controla lift con writeOutputs (igual que en IntakeSubsystem)
        motorLiftOutput = liftOutput;
    }

    // Para las ruedas, agregar método separado para simular el writeWheelOutputs
    public void writeWheelOutputs(double wheelOutput) {
        motorWheelOutput = wheelOutput;
    }

    /*@Override
    public double getMotorPosition() {
        return motorLiftPosition;
    }

    @Override
    public double getTofDistance() {
        return tofDistance;
    }

    @Override
    public boolean hasGamePiece() {
        return tofDistance > 0 && tofDistance < Constants.Intake.kTofThreshold;
    }

    

    // Método auxiliar para tests: simular que el TOF ve una pieza
    public void setHasGamePiece(boolean pieceDetected) {
        tofDistance = pieceDetected ? 50.0 : 1000.0;
    }

    // Método auxiliar para tests: simular mover el lift manualmente
    public void setLiftPosition(double rotations) {
        motorLiftPosition = rotations;
    }

    @Override
    public void writeLiftOutputs(double output) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'writeLiftOutputs'");
    }

    @Override
    public double getMotorLiftPosition() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getMotorLiftPosition'");
    }

    @Override
    public boolean getLimitSwitch() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'getLimitSwitch'");
    }
        */
}
