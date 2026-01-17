package org.sciborgs1155.lib;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.ArrayList;
import java.util.List;

public final class TalonUtils {
  private static final Orchestra ORCHESTRA = new Orchestra();
  private static final List<TalonFX> TALONS = new ArrayList<>(4);
  private static BaseStatusSignal[] talonSignals = new BaseStatusSignal[0];

  private static boolean fileLoaded;

  // Prevents instantiation
  private TalonUtils() {}

  /**
   * Adds motor to the processing list.
   *
   * @param talon The motor to add.
   */
  public static void addMotor(TalonFX talon) {
    TALONS.add(talon);
  }

  public static void addSignal(StatusSignal signal) {
    BaseStatusSignal[] newSignals = new BaseStatusSignal[talonSignals.length + 1];
    System.arraycopy(talonSignals, 0, newSignals, 0, talonSignals.length);
    newSignals[talonSignals.length] = signal;
    talonSignals = newSignals;
  }

  public static void refreshAll() {
    BaseStatusSignal.refreshAll(talonSignals);
  }

  /**
   * Configure all motors to play a selected Chirp (CHRP) file in the deploy directory. Should be
   * called once after addition of all Talons to TalonUtils.
   *
   * <p>Use {@code loadOrchestraFile()} after configuration to change the played file.
   *
   * @param fileName The path of the file to play.
   * @return Whether loading the file was successful.
   */
  public static boolean configureOrchestra(String fileName) {
    AudioConfigs audioCfg = new AudioConfigs().withAllowMusicDurDisable(true);
    for (TalonFX talon : TALONS) {
      talon.getConfigurator().apply(audioCfg);
      ORCHESTRA.addInstrument(talon);
    }
    return loadOrchestraFile(fileName);
  }

  /**
   * Load the selected CHRP file located in the deploy directory.
   *
   * @param fileName The name of the file to play.
   * @return Whether loading the file was successful.
   */
  public static boolean loadOrchestraFile(String fileName) {
    fileLoaded = ORCHESTRA.loadMusic(fileName).isOK();
    if (!fileLoaded) {
      fileNotFound();
    }

    return fileLoaded;
  }

  /**
   * Begin playback of the loaded file.
   *
   * @return Whether the operation was successful.
   */
  public static boolean play() {
    if (fileLoaded) {
      return ORCHESTRA.play().isOK();
    }
    fileNotFound();
    return false;
  }

  /**
   * Stop and restart playback of the loaded file.
   *
   * @return Whether the operation was successful.
   */
  public static boolean stop() {
    if (fileLoaded) {
      return ORCHESTRA.stop().isOK();
    }
    fileNotFound();
    return false;
  }

  /**
   * Pause playback of the loaded file.
   *
   * @return Whether the operation was successful.
   */
  public static boolean pause() {
    if (fileLoaded) {
      return ORCHESTRA.pause().isOK();
    }
    fileNotFound();
    return false;
  }

  private static void fileNotFound() {
    fileLoaded = false;
    DriverStation.reportError(
        "CHRP file not loaded. Check that it is in the deploy directory & includes file extension.",
        true);
  }
}
