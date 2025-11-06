package com.team3478.lib.drivers;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

/**
 * This class is a thin wrapper around the CANTalon that reduces CAN bus / CPU overhead by skipping
 * duplicate set commands.
 */
public class LazySparkMax extends SparkMax {
  protected double mLastSet = Double.NaN;
  protected ControlType mLastControlType = null;

  // Set if is a follower
  protected SparkMax mLeader = null;
  protected SparkMaxConfig config;

  public LazySparkMax(int deviceNumber) {
    super(deviceNumber, MotorType.kBrushless);
    config = new SparkMaxConfig();
    ChangeCanPeriod();
  }

  public SparkMax getLeader() {
    return mLeader;
  }

  public REVLibError follow(final SparkMax leader) {
    mLeader = leader;
    config.follow(mLeader);
    return super.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /** wrapper method to mimic TalonSRX set method */
  public void set(ControlType type, double setpoint) {
    if (setpoint != mLastSet || type != mLastControlType) {
      mLastSet = setpoint;
      mLastControlType = type;
      super.getClosedLoopController().setReference(setpoint, type);
    }
  }

  public void ChangeCanPeriod() {
    config
        .signals
        .appliedOutputPeriodMs(25)
        .primaryEncoderVelocityPeriodMs(30)
        .primaryEncoderPositionPeriodMs(30)
        .analogVoltagePeriodMs(30);
    super.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setInverted(boolean isInverted) {
    config.inverted(isInverted);
    super.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setIdleMode(IdleMode mode) {
    config.idleMode(mode);
    super.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setSmartCurrentLimit(int limit) {
    config.smartCurrentLimit(limit);
    super.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
}