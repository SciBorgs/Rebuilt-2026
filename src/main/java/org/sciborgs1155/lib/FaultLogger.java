package org.sciborgs1155.lib;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.reduxrobotics.sensors.canandgyro.Canandgyro;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import com.studica.frc.AHRS;
import edu.wpi.first.hal.PowerDistributionFaults;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PowerDistribution;
import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.BiConsumer;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.photonvision.PhotonCamera;
import org.sciborgs1155.robot.Ports;

/**
 * FaultLogger allows for faults to be logged and displayed.
 *
 * <pre>
 * FaultLogger.register(spark); // registers a spark, periodically checking for hardware faults
 * spark.set(0.5);
 * FaultLogger.check(spark); // checks that the previous set call did not encounter an error.
 * </pre>
 */
public final class FaultLogger {
  /** An individual fault, containing necessary information. */
  public static record Fault(String name, String description, FaultType type) {
    @Override
    public String toString() {
      return name + ": " + description;
    }
  }

  /**
   * The type of fault, used for detecting whether the fallible is in a failure state and displaying
   * to NetworkTables.
   */
  public static enum FaultType {
    INFO,
    WARNING,
    ERROR,
  }

  /** A class to represent an alerts widget on NetworkTables */
  public static class Alerts {
    private final NetworkTable table;
    private StringArrayPublisher errors;
    private StringArrayPublisher warnings;
    private StringArrayPublisher infos;

    public Alerts(NetworkTable base, String name) {
      table = base.getSubTable(name);
      table.getStringTopic(".type").publish().set("Alerts");
      errors = table.getStringArrayTopic("errors").publish();
      warnings = table.getStringArrayTopic("warnings").publish();
      infos = table.getStringArrayTopic("infos").publish();
    }

    public void set(Set<Fault> faults) {
      errors.set(filteredStrings(faults, FaultType.ERROR));
      warnings.set(filteredStrings(faults, FaultType.WARNING));
      infos.set(filteredStrings(faults, FaultType.INFO));
    }

    public void reset() {
      errors.close();
      warnings.close();
      infos.close();

      errors = table.getStringArrayTopic("errors").publish();
      warnings = table.getStringArrayTopic("warnings").publish();
      infos = table.getStringArrayTopic("infos").publish();
    }
  }

  // DATA
  private static final List<Supplier<Optional<Fault>>> faultReporters = new ArrayList<>();
  private static final Set<Fault> activeFaults = new HashSet<>();
  private static final Set<Fault> totalFaults = new HashSet<>();

  // NETWORK TABLES
  private static final NetworkTable base = NetworkTableInstance.getDefault().getTable("Faults");
  private static final Alerts activeAlerts = new Alerts(base, "Active Faults");
  private static final Alerts totalAlerts = new Alerts(base, "Total Faults");

  /** Polls registered fallibles. This method should be called periodically. */
  public static void update() {
    faultReporters.forEach(r -> r.get().ifPresent(fault -> report(fault)));

    totalFaults.addAll(activeFaults);

    activeAlerts.set(activeFaults);
    totalAlerts.set(totalFaults);

    activeFaults.clear();
  }

  /** Clears total faults. */
  public static void clear() {
    totalFaults.clear();
    activeFaults.clear();

    totalAlerts.reset();
    activeAlerts.reset();
  }

  /** Clears fault suppliers. */
  public static void unregisterAll() {
    faultReporters.clear();
  }

  /**
   * Returns the set of all current faults.
   *
   * @return The set of all current faults.
   */
  public static Set<Fault> activeFaults() {
    return activeFaults;
  }

  /**
   * Returns the set of all total faults.
   *
   * @return The set of all total faults.
   */
  public static Set<Fault> totalFaults() {
    return totalFaults;
  }

  /**
   * Reports a fault.
   *
   * @param fault The fault to report.
   */
  public static void report(Fault fault) {
    activeFaults.add(fault);
    switch (fault.type) {
      case ERROR -> DriverStation.reportError(fault.toString(), false);
      case WARNING -> DriverStation.reportWarning(fault.toString(), false);
      case INFO -> System.out.println(fault.toString());
    }
  }

  /**
   * Reports a fault.
   *
   * @param name The name of the fault.
   * @param description The description of the fault.
   * @param type The type of the fault.
   */
  public static void report(String name, String description, FaultType type) {
    report(new Fault(name, description, type));
  }

  /** Conditionally reports a fault and returns the condition */
  public static boolean report(boolean condition, Fault fault) {
    if (condition) {
      report(fault);
    }
    return condition;
  }

  /**
   * Registers a new fault supplier.
   *
   * @param supplier A supplier of an optional fault.
   */
  public static void register(Supplier<Optional<Fault>> supplier) {
    faultReporters.add(supplier);
  }

  /**
   * Registers a new fault supplier.
   *
   * @param condition Whether a failure is occuring.
   * @param description The failure's description.
   * @param type The type of failure.
   */
  public static void register(
      BooleanSupplier condition, String name, String description, FaultType type) {
    register(
        () ->
            condition.getAsBoolean()
                ? Optional.of(new Fault(name, description, type))
                : Optional.empty());
  }

  /**
   * Registers fault suppliers for a CAN-based Spark motor controller.
   *
   * @param spark The Spark Max or Spark Flex to manage.
   */
  public static void register(SparkBase spark) {
    register(
        () -> spark.getFaults().other,
        SparkUtils.name(spark),
        "other strange error",
        FaultType.ERROR);
    register(
        () -> spark.getFaults().motorType,
        SparkUtils.name(spark),
        "motor type error",
        FaultType.ERROR);
    register(
        () -> spark.getFaults().sensor, SparkUtils.name(spark), "sensor error", FaultType.ERROR);
    register(() -> spark.getFaults().can, SparkUtils.name(spark), "CAN error", FaultType.ERROR);
    register(
        () -> spark.getFaults().temperature,
        SparkUtils.name(spark),
        "temperature error",
        FaultType.ERROR);
    register(
        () -> spark.getFaults().gateDriver,
        SparkUtils.name(spark),
        "gate driver error",
        FaultType.ERROR);
    register(
        () -> spark.getFaults().escEeprom,
        SparkUtils.name(spark),
        "escEeprom? error",
        FaultType.ERROR);
    register(
        () -> spark.getFaults().firmware,
        SparkUtils.name(spark),
        "firmware error",
        FaultType.ERROR);
    register(
        () -> spark.getMotorTemperature() > 100,
        SparkUtils.name(spark),
        "motor above 100°C",
        FaultType.WARNING);
  }

  /**
   * Registers fault suppliers for a duty cycle encoder.
   *
   * @param encoder The duty cycle encoder to manage.
   */
  public static void register(DutyCycleEncoder encoder) {
    register(
        () -> !encoder.isConnected(),
        "Duty Cycle Encoder [" + encoder.getSourceChannel() + "]",
        "disconnected",
        FaultType.ERROR);
  }

  /**
   * Registers fault suppliers for a NavX.
   *
   * @param ahrs The NavX to manage.
   */
  public static void register(AHRS ahrs) {
    register(() -> !ahrs.isConnected(), "NavX", "disconnected", FaultType.ERROR);
  }

  /**
   * Registers Alerts for faults of a Redux Boron CANandGyro.
   *
   * @param canandgyro The Redux Boron CANandGyro to manage.
   */
  public static void register(Canandgyro canandgyro) {
    register(() -> !canandgyro.isConnected(), "CANandGyro", "disconnected", FaultType.ERROR);
    register(
        () -> canandgyro.getActiveFaults().accelerationSaturation(),
        "CANandGyro",
        "acceleration saturated",
        FaultType.WARNING);
    register(
        () -> canandgyro.getActiveFaults().angularVelocitySaturation(),
        "CANandGyro",
        "angular velocity saturated",
        FaultType.WARNING);
    register(
        () -> canandgyro.getActiveFaults().calibrating(),
        "CANandGyro",
        "calibrating",
        FaultType.WARNING);
    register(
        () -> canandgyro.getActiveFaults().canGeneralError(),
        "CANandGyro",
        "general CAN error",
        FaultType.ERROR);
    register(
        () -> canandgyro.getActiveFaults().canIDConflict(),
        "CANandGyro",
        "CAN ID conflict",
        FaultType.ERROR);
    register(
        () -> canandgyro.getActiveFaults().outOfTemperatureRange(),
        "CANandGyro",
        "temperature error",
        FaultType.ERROR);
    register(
        () -> canandgyro.getActiveFaults().powerCycle(),
        "CANandGyro",
        "power cycling",
        FaultType.WARNING);
  }

  /**
   * Registers fault suppliers for a power distribution hub/panel.
   *
   * @param powerDistribution The power distribution to manage.
   */
  public static void register(PowerDistribution powerDistribution) {
    var fields = PowerDistributionFaults.class.getFields();
    for (Field fault : fields) {
      register(
          () -> {
            try {
              if (fault.getBoolean(powerDistribution.getFaults())) {
                return Optional.of(
                    new Fault("Power Distribution", fault.getName(), FaultType.ERROR));
              }
            } catch (Exception e) {
            }
            return Optional.empty();
          });
    }
    ;
  }

  /**
   * Registers fault suppliers for a camera.
   *
   * @param camera The camera to manage.
   */
  public static void register(PhotonCamera camera) {
    register(
        () -> !camera.isConnected(),
        "Photon Camera [" + camera.getName() + "]",
        "disconnected",
        FaultType.ERROR);
  }

  /**
   * Registers fault suppliers for a CANcoder.
   *
   * @param cancoder The CANcoder to manage.
   */
  public static void register(CANcoder cancoder) {
    String nickname = Ports.idToName.get(cancoder.getDeviceID());
    register(
        () -> cancoder.getFault_BadMagnet().getValue(),
        "CANcoder " + nickname,
        "The magnet distance is not correct or magnet is missing.",
        FaultType.ERROR);
    register(
        () -> cancoder.getFault_BootDuringEnable().getValue(),
        "CANcoder " + nickname,
        "Device boot while detecting the enable signal.",
        FaultType.WARNING);
    register(
        () -> cancoder.getFault_Hardware().getValue(),
        "CANcoder " + nickname,
        "Hardware fault occurred.",
        FaultType.WARNING);
    register(
        () -> cancoder.getFault_Undervoltage().getValue(),
        "CANcoder " + nickname,
        "Device supply voltage dropped to near brownout levels.",
        FaultType.WARNING);
  }

  /**
   * Registers fault suppliers for a talon.
   *
   * @param talon The talon to manage.
   */
  public static void register(TalonFX talon) {
    register(
        () -> !talon.isConnected(),
        "Talon " + Ports.idToName.get(talon.getDeviceID()),
        "disconnected",
        FaultType.ERROR);

    BiConsumer<StatusSignal<Boolean>, String> regFault =
        (f, d) ->
            register(
                () -> f.getValue(),
                "Talon " + Ports.idToName.get(talon.getDeviceID()),
                d,
                FaultType.ERROR);

    // TODO: Remove all the unnecessary faults.
    regFault.accept(talon.getFault_Hardware(), "Hardware fault occurred");
    regFault.accept(talon.getFault_ProcTemp(), "Processor temperature exceeded limit");
    regFault.accept(talon.getFault_Hardware(), "Hardware fault occurred");
    regFault.accept(talon.getFault_ProcTemp(), "Processor temperature exceeded limit");
    regFault.accept(talon.getFault_DeviceTemp(), "Device temperature exceeded limit");
    regFault.accept(
        talon.getFault_Undervoltage(), "Device supply voltage dropped to near brownout levels");
    regFault.accept(
        talon.getFault_BootDuringEnable(), "Device boot while detecting the enable signal");
    regFault.accept(
        talon.getFault_UnlicensedFeatureInUse(),
        "An unlicensed feature is in use, device may not behave as expected.");
    regFault.accept(
        talon.getFault_BridgeBrownout(),
        "Bridge was disabled most likely due to supply voltage dropping too low.");
    regFault.accept(talon.getFault_RemoteSensorReset(), "The remote sensor has reset.");
    regFault.accept(
        talon.getFault_MissingDifferentialFX(),
        "The remote Talon FX used for differential control is not present on CAN Bus.");
    regFault.accept(
        talon.getFault_RemoteSensorPosOverflow(), "The remote sensor position has overflowed.");
    regFault.accept(
        talon.getFault_OverSupplyV(),
        "Supply Voltage has exceeded the maximum voltage rating of device.");
    regFault.accept(talon.getFault_UnstableSupplyV(), "Supply Voltage is unstable.");
    regFault.accept(
        talon.getFault_ReverseHardLimit(),
        "Reverse limit switch has been asserted.  Output is set to neutral.");
    regFault.accept(
        talon.getFault_ForwardHardLimit(),
        "Forward limit switch has been asserted.  Output is set to neutral.");
    regFault.accept(
        talon.getFault_ReverseSoftLimit(),
        "Reverse soft limit has been asserted.  Output is set to neutral.");
    regFault.accept(
        talon.getFault_ForwardSoftLimit(),
        "Forward soft limit has been asserted.  Output is set to neutral.");
    regFault.accept(
        talon.getFault_RemoteSensorDataInvalid(), "The remote sensor's data is no longer trusted.");
    regFault.accept(
        talon.getFault_FusedSensorOutOfSync(),
        "The remote sensor used for fusion has fallen out of sync to the local sensor.");
    regFault.accept(talon.getFault_StatorCurrLimit(), "Stator current limit occured.");
    regFault.accept(talon.getFault_SupplyCurrLimit(), "Supply current limit occured.");
    regFault.accept(
        talon.getFault_UsingFusedCANcoderWhileUnlicensed(),
        "Using Fused CANcoder feature while unlicensed. Device has fallen back to remote CANcoder.");
  }

  /**
   * Reports REVLibErrors from a spark.
   *
   * <p>This should be called immediately after any call to the spark.
   *
   * @param spark The spark to report REVLibErrors from.
   * @return If the spark is working without errors.
   */
  public static boolean check(SparkBase spark) {
    REVLibError error = spark.getLastError();
    return check(spark, error);
  }

  /**
   * Reports REVLibErrors from a spark.
   *
   * <p>This should be called immediately after any call to the spark.
   *
   * @param spark The spark to report REVLibErrors from.
   * @param error Any REVLibErrors that may be returned from a method for a spark.
   * @return If the spark is working without errors.
   */
  public static boolean check(SparkBase spark, REVLibError error) {
    if (error != REVLibError.kOk) {
      report(SparkUtils.name(spark), error.name(), FaultType.ERROR);
      return false;
    }
    return true;
  }

  /**
   * Returns an array of descriptions of all faults that match the specified type.
   *
   * @param type The type to filter for.
   * @return An array of description strings.
   */
  private static String[] filteredStrings(Set<Fault> faults, FaultType type) {
    return faults.stream()
        .filter(a -> a.type() == type)
        .map(Fault::toString)
        .toArray(String[]::new);
  }
}
