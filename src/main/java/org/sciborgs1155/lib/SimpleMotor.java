package org.sciborgs1155.lib;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkBase;
import java.util.function.DoubleConsumer;

/**
 * Simple Motor that utilizes a {@link DoubleConsumer} for setting power and provides no feedback.
 * Can work with {@link TalonFX}'s and {@link SparkBase}'s. Can be closed with the specified {@link
 * Runnable}.
 */
public class SimpleMotor {
  /** Function for setting motor power. */
  private final DoubleConsumer set;

  /** Function for closing the motor. */
  private final Runnable close;

  /**
   * Constructor.
   *
   * @param set : {@link DoubleConsumer} for setting motor power.
   * @param close : {@link Runnable} interface for the motor.
   */
  public SimpleMotor(DoubleConsumer set, Runnable close) {
    this.set = set;
    this.close = close;
  }

  /**
   * @return Returns a new {@link SimpleMotor} that controls a {@link TalonFX} motor. The motor is
   *     registered with {@link TalonUtils} and {@link FaultLogger}.
   * @param motor : {@link TalonFX} controller instance with device ID.
   * @param config : {@link TalonFXConfiguration} to apply to the motor.
   */
  public static SimpleMotor talon(TalonFX motor, TalonFXConfiguration config) {
    FaultLogger.register(motor);
    TalonUtils.addMotor(motor);
    motor.getConfigurator().apply(config);
    return new SimpleMotor(motor::set, motor::close);
  }

  /** Returns a new {@link SimpleMotor} that does absolutely nothing. */
  public static SimpleMotor none() {
    return new SimpleMotor(v -> {}, () -> {});
  }

  /** Passes power into the '{@link #set}' consumer specified in the constructor. */
  public void set(double power) {
    set.accept(power);
  }

  public void close() {
    close.run();
  }
}
