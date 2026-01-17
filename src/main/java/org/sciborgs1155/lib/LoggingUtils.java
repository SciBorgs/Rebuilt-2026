package org.sciborgs1155.lib;

import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.util.struct.Struct;
import java.util.Collection;

/**
 * A class that allows all epilogue logging methods to be simplified to
 *
 * <pre>
 * log(identifier, value);
 * </pre>
 *
 * which reduces the redundant
 *
 * <pre>
 * Epilogue.getConfig().backend.log(identifier, value);
 * </pre>
 */
public final class LoggingUtils {

  // Prevents instantiation
  private LoggingUtils() {}

  /**
   * Logs an integer value to the Epilogue backend.
   *
   * @param identifier The identifier for the logged value.
   * @param value The integer value to log.
   */
  public static void log(String identifier, int value) {
    Epilogue.getConfig().backend.log(identifier, value);
  }

  /**
   * Logs a long value to the Epilogue backend.
   *
   * @param identifier The identifier for the logged value.
   * @param value The long value to log.
   */
  public static void log(String identifier, long value) {
    Epilogue.getConfig().backend.log(identifier, value);
  }

  /**
   * Logs a float value to the Epilogue backend.
   *
   * @param identifier The identifier for the logged value.
   * @param value The float value to log.
   */
  public static void log(String identifier, float value) {
    Epilogue.getConfig().backend.log(identifier, value);
  }

  /**
   * Logs a double value to the Epilogue backend.
   *
   * @param identifier The identifier for the logged value.
   * @param value The double value to log.
   */
  public static void log(String identifier, double value) {
    Epilogue.getConfig().backend.log(identifier, value);
  }

  /**
   * Logs a boolean value to the Epilogue backend.
   *
   * @param identifier The identifier for the logged value.
   * @param value The boolean value to log.
   */
  public static void log(String identifier, boolean value) {
    Epilogue.getConfig().backend.log(identifier, value);
  }

  /**
   * Logs a byte array to the Epilogue backend.
   *
   * @param identifier The identifier for the logged value.
   * @param value The byte array to log.
   */
  public static void log(String identifier, byte[] value) {
    Epilogue.getConfig().backend.log(identifier, value);
  }

  /**
   * Logs an int array to the Epilogue backend.
   *
   * @param identifier The identifier for the logged value.
   * @param value The int array to log.
   */
  public static void log(String identifier, int[] value) {
    Epilogue.getConfig().backend.log(identifier, value);
  }

  /**
   * Logs a long array to the Epilogue backend.
   *
   * @param identifier The identifier for the logged value.
   * @param value The long array to log.
   */
  public static void log(String identifier, long[] value) {
    Epilogue.getConfig().backend.log(identifier, value);
  }

  /**
   * Logs a float array to the Epilogue backend.
   *
   * @param identifier The identifier for the logged value.
   * @param value The float array to log.
   */
  public static void log(String identifier, float[] value) {
    Epilogue.getConfig().backend.log(identifier, value);
  }

  /**
   * Logs a double array to the Epilogue backend.
   *
   * @param identifier The identifier for the logged value.
   * @param value The double array to log.
   */
  public static void log(String identifier, double[] value) {
    Epilogue.getConfig().backend.log(identifier, value);
  }

  /**
   * Logs a boolean array to the Epilogue backend.
   *
   * @param identifier The identifier for the logged value.
   * @param value The boolean array to log.
   */
  public static void log(String identifier, boolean[] value) {
    Epilogue.getConfig().backend.log(identifier, value);
  }

  /**
   * Logs a String value to the Epilogue backend.
   *
   * @param identifier The identifier for the logged value.
   * @param value The String value to log.
   */
  public static void log(String identifier, String value) {
    Epilogue.getConfig().backend.log(identifier, value);
  }

  /**
   * Logs a String array to the Epilogue backend.
   *
   * @param identifier The identifier for the logged value.
   * @param value The String array to log.
   */
  public static void log(String identifier, String[] value) {
    Epilogue.getConfig().backend.log(identifier, value);
  }

  /**
   * Logs a collection of Strings to the Epilogue backend.
   *
   * @param identifier The identifier for the logged value.
   * @param value The collection of Strings to log.
   */
  public static void log(String identifier, Collection<String> value) {
    Epilogue.getConfig().backend.log(identifier, value);
  }

  /**
   * Logs a struct value to the Epilogue backend.
   *
   * @param <S> The type of the struct value.
   * @param identifier The identifier for the logged value.
   * @param value The struct value to log.
   * @param struct The struct descriptor.
   */
  @SuppressWarnings("unchecked")
  public static <S> void log(String identifier, S value, Struct<S> struct) {
    Epilogue.getConfig().backend.log(identifier, value, struct);
  }

  /**
   * Logs a struct array to the Epilogue backend.
   *
   * @param <S> The type of the struct values.
   * @param identifier The identifier for the logged value.
   * @param value The struct array to log.
   * @param struct The struct descriptor.
   */
  @SuppressWarnings("unchecked")
  public static <S> void log(String identifier, S[] value, Struct<S> struct) {
    Epilogue.getConfig().backend.log(identifier, value, struct);
  }

  /**
   * Logs a Measure value to the Epilogue backend.
   *
   * @param identifier The identifier for the logged value.
   * @param value The Measure value to log.
   */
  public static void log(String identifier, Measure<?> value) {
    Epilogue.getConfig().backend.log(identifier, value);
  }

  /**
   * Logs a Measure value with a specific unit to the Epilogue backend.
   *
   * @param identifier The identifier for the logged value.
   * @param value The Measure value to log.
   * @param unit The unit of the measure.
   */
  public static void log(String identifier, Measure<Unit> value, Unit unit) {
    Epilogue.getConfig().backend.log(identifier, value);
  }

  /**
   * Logs an Enum value to the Epilogue backend.
   *
   * @param identifier The identifier for the logged value.
   * @param value The Enum value to log.
   */
  public static void log(String identifier, Enum<?> value) {
    Epilogue.getConfig().backend.log(identifier, value);
  }
}
