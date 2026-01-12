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
public class LoggingUtils {
  public static void log(String identifier, int value) {
    Epilogue.getConfig().backend.log(identifier, value);
  }

  public static void log(String identifier, long value) {
    Epilogue.getConfig().backend.log(identifier, value);
  }

  public static void log(String identifier, float value) {
    Epilogue.getConfig().backend.log(identifier, value);
  }

  public static void log(String identifier, double value) {
    Epilogue.getConfig().backend.log(identifier, value);
  }

  public static void log(String identifier, boolean value) {
    Epilogue.getConfig().backend.log(identifier, value);
  }

  public static void log(String identifier, byte[] value) {
    Epilogue.getConfig().backend.log(identifier, value);
  }

  public static void log(String identifier, int[] value) {
    Epilogue.getConfig().backend.log(identifier, value);
  }

  public static void log(String identifier, long[] value) {
    Epilogue.getConfig().backend.log(identifier, value);
  }

  public static void log(String identifier, float[] value) {
    Epilogue.getConfig().backend.log(identifier, value);
  }

  public static void log(String identifier, double[] value) {
    Epilogue.getConfig().backend.log(identifier, value);
  }

  public static void log(String identifier, boolean[] value) {
    Epilogue.getConfig().backend.log(identifier, value);
  }

  public static void log(String identifier, String value) {
    Epilogue.getConfig().backend.log(identifier, value);
  }

  public static void log(String identifier, String[] value) {
    Epilogue.getConfig().backend.log(identifier, value);
  }

  public static void log(String identifier, Collection<String> value) {
    Epilogue.getConfig().backend.log(identifier, value);
  }

  @SuppressWarnings("unchecked")
  public static <S> void log(String identifier, S value, Struct<S> struct) {
    Epilogue.getConfig().backend.log(identifier, value, struct);
  }

  @SuppressWarnings("unchecked")
  public static <S> void log(String identifier, S[] value, Struct<S> struct) {
    Epilogue.getConfig().backend.log(identifier, value, struct);
  }

  public static void log(String identifier, Measure<?> value) {
    Epilogue.getConfig().backend.log(identifier, value);
  }

  public static void log(String identifier, Measure<Unit> value, Unit unit) {
    Epilogue.getConfig().backend.log(identifier, value);
  }

  public static void log(String identifier, Enum<?> value) {
    Epilogue.getConfig().backend.log(identifier, value);
  }
}
