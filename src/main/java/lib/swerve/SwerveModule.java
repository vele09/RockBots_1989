package com.team3478.lib.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.team3478.frc2025.Constants;
import com.team3478.lib.drivers.TalonUtil;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Swerve Module class that encapsulates a swerve module.
 */
public class SwerveModule {
   
    private final TalonFX driveMotor;
    private final TalonFX steeringMotor;
    private final CANcoder encoder;

    private final BaseStatusSignal velocitySignal;
    private final BaseStatusSignal positionSignal;
    private final BaseStatusSignal encoderSignal;
    private final BaseStatusSignal[] m_signals;

    private final SwerveModulePosition positionState = new SwerveModulePosition();
    private SwerveModuleState moduleState = new SwerveModuleState();

    // Constructor del swerve module
    public SwerveModule(int driveMotorId, int steeringMotorId, int encoderId, String canbusName) {
        // Create hardware objects
        driveMotor = new TalonFX(driveMotorId, canbusName);
        steeringMotor = new TalonFX(steeringMotorId, canbusName);
        encoder = new CANcoder(encoderId, canbusName);
        // Create speed motors configuration and apply
        TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();
        CurrentLimitsConfigs driveMotorConfigCurrent = new CurrentLimitsConfigs();
        driveMotorConfigCurrent.SupplyCurrentLimitEnable = true;
        driveMotorConfigCurrent.SupplyCurrentLimit = Constants.Drive.kDriveMotorCurrentLimit;
        driveMotorConfigCurrent.SupplyCurrentLowerLimit = 0.2;
        driveMotorConfigCurrent.SupplyCurrentLowerLimit = 0;
        driveMotorConfig.CurrentLimits = driveMotorConfigCurrent;
        driveMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        driveMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        TalonUtil.checkError(
            driveMotor.getConfigurator().apply(driveMotorConfig),
            "Drive: Could not set configuration for swerve drive motor");
        // Create steering motors configuration and apply
        TalonFXConfiguration steeringMotorConfig = new TalonFXConfiguration();
        CurrentLimitsConfigs steeringMotorConfigCurrent = new CurrentLimitsConfigs();
        steeringMotorConfigCurrent.SupplyCurrentLimitEnable = true;
        steeringMotorConfigCurrent.SupplyCurrentLimit = Constants.Drive.kSteeringMotorCurrentLimit;
        steeringMotorConfigCurrent.SupplyCurrentLowerLimit = 0.2;
        steeringMotorConfigCurrent.SupplyCurrentLowerLimit = 0;
        steeringMotorConfig.CurrentLimits = steeringMotorConfigCurrent;
        steeringMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        steeringMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        TalonUtil.checkError(
            steeringMotor.getConfigurator().apply(steeringMotorConfig),
            "Drive: Could not set configuration for swerve steering motor");
        // Create cancoder configuration and apply
        CANcoderConfiguration configs = new CANcoderConfiguration();
        configs.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        configs.MagnetSensor.MagnetOffset = 0;
        configs.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        encoder.getConfigurator().apply(configs);
        // Store status signals
        m_signals = new BaseStatusSignal[3];
        encoderSignal = encoder.getAbsolutePosition();
        m_signals[0] = encoderSignal;
        velocitySignal = driveMotor.getVelocity();
        m_signals[1] = velocitySignal;
        positionSignal = driveMotor.getPosition();
        m_signals[2] = positionSignal;
        // reseteamos el encoder de drive
        resetPosition();
    }

    // Funcion para hacer refresh de los valores de los sensores
    public void refreshSignals(){
        BaseStatusSignal.refreshAll(m_signals);
    }

    // Funcion para hacer refresh de los valores de los sensores
    // Velocidad en rotaciones por segundo del motor, posicion en rotaciones del motor
    public void update(double driveMotorVelocity, double driveMotorPosition, double steeringMotorAngle){
        // Update module state
        moduleState.speedMetersPerSecond = driveMotorVelocity * Math.PI * Constants.Drive.kWheelRadius * 2 * Constants.Drive.kGearDriveReduction;
        moduleState.angle = Rotation2d.fromDegrees(steeringMotorAngle);
        // Update module position
        positionState.distanceMeters = driveMotorPosition * Math.PI * Constants.Drive.kWheelRadius * 2 * Constants.Drive.kGearDriveReduction;
        positionState.angle = Rotation2d.fromDegrees(steeringMotorAngle);
    }

    // Get encoder value
    public double getEncoderAngle() {
        return encoderSignal.getValueAsDouble();
    }

    // Get velocity from drive motor
    public double getDriveMotorPosition() {
        return positionSignal.getValueAsDouble();
    }

    // Get module state
    public double getDriveMotorVelocity() {
        return velocitySignal.getValueAsDouble();
    }

    // Get module state
    public SwerveModuleState getModuleState() {
        return moduleState;
    }

    // Get module state
    public SwerveModulePosition getPositionState() {
        return positionState;
    }

    // Get stering motor
    public TalonFX getSteeringMotor() {
        return steeringMotor;
    }

    // Get stering motor
    public TalonFX getDriveMotor() {
        return driveMotor;
    }

    // Get status signals
    BaseStatusSignal[] getSignals() {
        return m_signals;
    }

    /**
     * Resets this module's drive motor position to 0 rotations.
     */
    public void resetPosition() {
        /* Only touch drive pos, not steer */
        driveMotor.setPosition(0);
    }
}
