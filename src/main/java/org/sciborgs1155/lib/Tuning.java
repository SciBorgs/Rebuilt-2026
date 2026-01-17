package org.sciborgs1155.lib;

import static edu.wpi.first.units.Units.Milliseconds;
import static org.sciborgs1155.robot.Constants.PERIOD;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerTopic;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StringTopic;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.sciborgs1155.robot.Constants;

/**
 * Tuning creates an entry with a specified topic and configurable value in Network Tables.
 *
 * <pre>
 * [Type]Entry entry = Tuning.entry("/[FOLDER_NAME]/[TOPIC_NAME]", [CONFIGURABLE_VALUE]);
 *   // This creates a new configurable datatype value corresponding to the path given and sets the value.
 *
 * Tuning.put([TOPIC], [CONFIGURABLE]);
 *   // This inserts a value under the given topic.
 *
 * ArrayList<[DataType]> arrayList = Tuning.recentChanges([TOPIC]);
 *   // This is returns a list of all of the changes made from a specific topic
 *
 * Tuning.update[DataType]([DataType]Entry);
 *   // This is called periodically in order to check for changes made on Network Tables.
 * </pre>
 */
@SuppressWarnings({
  "PMD.GodClass",
  "PMD.UseConcurrentHashMap"
}) // Not sure if there are concurrent accesses
public final class Tuning {
  /* HashMap of values of each topic path */
  private static final Map<String, List<Double>> DOUBLE_HASH = new HashMap<>();
  private static final Map<String, List<Long>> INT_HASH = new HashMap<>();
  private static final Map<String, List<String>> STRING_HASH = new HashMap<>();
  private static final Map<String, List<Boolean>> BOOLEAN_HASH = new HashMap<>();

  /* Previous values of each topic path */
  private static final Map<String, Double> PREV_DOUBLE = new HashMap<>();
  private static final Map<String, Long> PREV_INT = new HashMap<>();
  private static final Map<String, String> PREV_STRING = new HashMap<>();
  private static final Map<String, Boolean> PREV_BOOLEAN = new HashMap<>();

  // Prevents instantiation
  private Tuning() {}

  /**
   * Creates a trigger that activates when a DoubleEntry has been changed recently.
   *
   * @param entry The DoubleEntry to monitor for changes.
   * @return A Trigger that activates when the entry changes.
   */
  public static Trigger changes(DoubleEntry entry) {
    return new Trigger(
        Constants.tuning
            ? () -> System.currentTimeMillis() - entry.getLastChange() <= PERIOD.in(Milliseconds)
            : () -> false);
  }

  /**
   * Logs a DoubleEntry on Network Tables.
   *
   * @param path The file path, "/[FOLDER_NAME]/[TOPIC_NAME]", ie: "/Robot/exampleTopicName".
   * @param value The configurable double that you would like to correspond to the topic.
   * @return The DoubleEntry - contains all methods of DoublePublisher, DoubleSubscriber.
   */
  public static DoubleEntry entry(String path, double value) {

    DoubleEntry entry = NetworkTableInstance.getDefault().getDoubleTopic(path).getEntry(value);
    entry.set(value);

    List<Double> doubleList = new ArrayList<>();
    doubleList.add(value);

    List<Double> previousDble = DOUBLE_HASH.put(path, doubleList);
    if (previousDble == null) {
      previousDble = new ArrayList<>();
      previousDble.add(0.0);
    }
    PREV_DOUBLE.put(path, previousDble.get(previousDble.size() - 1));

    return entry;
  }

  /**
   * Logs a IntegerEntry on Network Tables.
   *
   * @param path The file path, "/[FOLDER_NAME]/[TOPIC_NAME]", ie: "/Robot/exampleTopicName".
   * @param value The configurable (int)long that you would like to correspond to the topic.
   * @return The IntegerEntry - contains all methods of IntegerPublisher, IntegerSubscriber.
   */
  public static IntegerEntry entry(String path, long value) {

    IntegerEntry entry = NetworkTableInstance.getDefault().getIntegerTopic(path).getEntry(value);
    entry.set(value);

    List<Long> intList = new ArrayList<>();
    intList.add(value);

    List<Long> previousInt = INT_HASH.put(path, intList);
    if (previousInt == null) {
      previousInt = new ArrayList<>();
      previousInt.add((long) 0);
    }
    PREV_INT.put(path, previousInt.get(previousInt.size() - 1));

    return entry;
  }

  /**
   * Logs a StringEntry on Network Tables.
   *
   * @param path The file path, "/[FOLDER_NAME]/[TOPIC_NAME]", ie: "/Robot/exampleTopicName".
   * @param value The configurable String that you would like to correspond to the topic.
   * @return The StringEntry - contains all methods of StringPublisher, StringSubscriber.
   */
  public static StringEntry entry(String path, String value) {
    StringEntry entry = NetworkTableInstance.getDefault().getStringTopic(path).getEntry(value);
    entry.set(value);

    List<String> strList = new ArrayList<>();

    strList.add(value);

    List<String> previousStr = STRING_HASH.put(path, strList);
    if (previousStr == null) {
      previousStr = new ArrayList<>();
      previousStr.add("");
    }
    PREV_STRING.put(path, previousStr.get(previousStr.size() - 1));

    return entry;
  }

  /**
   * Logs a BooleanEntry on Network Tables.
   *
   * @param path The file path, "/[FOLDER_NAME]/[TOPIC_NAME]", ie: "/Robot/exampleTopicName".
   * @param value The configurable boolean that you would like to correspond to the topic.
   * @return The BooleanEntry - contains all methods of BooleanPublisher, BooleanSubscriber.
   */
  public static BooleanEntry entry(String path, boolean value) {
    BooleanEntry entry = NetworkTableInstance.getDefault().getBooleanTopic(path).getEntry(value);
    entry.set(value);

    List<Boolean> boolList = new ArrayList<>();

    boolList.add(value);

    List<Boolean> previousBool = BOOLEAN_HASH.put(path, boolList);
    if (previousBool == null) {
      previousBool = new ArrayList<>();
      previousBool.add(true);
    }
    PREV_BOOLEAN.put(path, previousBool.get(previousBool.size() - 1));

    return entry;
  }

  /**
   * Puts a new double value under an already-defined topic on Network Tables
   *
   * @param subtopic The topic under which you want to insert a new value.
   * @param value The value that you want to put under the topic.
   */
  public static void put(DoubleTopic subtopic, double value) {
    DoublePublisher dblePub = subtopic.publish();
    dblePub.accept(value);

    List<Double> arrayList = new ArrayList<>();
    if (DOUBLE_HASH.get(subtopic.getName()) != null) {
      arrayList = DOUBLE_HASH.get(subtopic.getName());
    }
    arrayList.add(value);
    List<Double> previousDble = DOUBLE_HASH.put(subtopic.getName(), arrayList);
    if (previousDble == null) {
      previousDble = new ArrayList<>();
      previousDble.add(0.0);
    }
    PREV_DOUBLE.put(subtopic.getName(), previousDble.get(previousDble.size() - 1));
  }

  /**
   * Puts a int value under an already-defined topic on Network Tables
   *
   * @param subtopic The topic under which you want to insert a new value.
   * @param value The value that you want to put under the topic.
   */
  public static void put(IntegerTopic subtopic, long value) {
    IntegerPublisher intPub = subtopic.publish();
    intPub.accept(value);

    List<Long> arrayList = new ArrayList<>();
    if (INT_HASH.get(subtopic.getName()) != null) {
      arrayList = INT_HASH.get(subtopic.getName());
    }
    arrayList.add(value);
    List<Long> previousInt = INT_HASH.put(subtopic.getName(), arrayList);
    if (previousInt == null) {
      previousInt = new ArrayList<>();
      previousInt.add((long) 0);
    }
    PREV_INT.put(subtopic.getName(), previousInt.get(previousInt.size() - 1));
  }

  /**
   * Puts a String value under an already-defined topic on Network Tables
   *
   * @param subtopic The topic under which you want to insert a new value.
   * @param value The value that you want to put under the topic.
   */
  public static void put(StringTopic subtopic, String value) {
    StringPublisher strPub = subtopic.publish();
    strPub.accept(value);

    List<String> arrayList = new ArrayList<>();
    if (STRING_HASH.get(subtopic.getName()) != null) {
      arrayList = STRING_HASH.get(subtopic.getName());
    }
    arrayList.add(value);
    List<String> previousStr = STRING_HASH.put(subtopic.getName(), arrayList);
    if (previousStr == null) {
      previousStr = new ArrayList<>();
      previousStr.add("");
    }
    PREV_STRING.put(subtopic.getName(), previousStr.get(previousStr.size() - 1));
  }

  /**
   * Puts a boolean value under an already-defined topic on Network Tables
   *
   * @param subtopic The topic under which you want to insert a new value.
   * @param value The value that you want to put under the topic.
   */
  public static void put(BooleanTopic subtopic, Boolean value) {
    BooleanPublisher boolPub = subtopic.publish();
    boolPub.accept(value);

    List<Boolean> arrayList = new ArrayList<>();
    if (BOOLEAN_HASH.get(subtopic.getName()) != null) {
      arrayList = BOOLEAN_HASH.get(subtopic.getName());
    }
    arrayList.add(value);
    List<Boolean> previousBool = BOOLEAN_HASH.put(subtopic.getName(), arrayList);
    if (previousBool == null) {
      previousBool = new ArrayList<>();
      previousBool.add(true);
    }
    PREV_BOOLEAN.put(subtopic.getName(), previousBool.get(previousBool.size() - 1));
  }

  /**
   * @param topic The topic that you want to get changes from.
   * @return An ArrayList containing all of the changes of the given topic
   */
  public static List<Double> recentChanges(DoubleTopic topic) {
    if (DOUBLE_HASH.containsKey(topic.getName())) {
      return DOUBLE_HASH.get(topic.getName());
    }
    return new ArrayList<>();
  }

  /**
   * Returns a list of the most recent changes of the given topic.
   *
   * @param topic The topic that you want to get changes from.
   * @param pastNIndexes The number of recent changes to retrieve.
   * @return An ArrayList containing the most recent changes of the given topic.
   */
  public static List<Double> recentChanges(DoubleTopic topic, int pastNIndexes) {
    List<Double> arrayList = recentChanges(topic);

    if (pastNIndexes >= arrayList.size()) {
      return arrayList;
    }
    if (pastNIndexes < 0) {
      List<Double> list = new ArrayList<>();
      list.add(arrayList.get(arrayList.size() - 1));
      return list;
    }

    List<Double> croppedList = new ArrayList<>();
    for (int i = arrayList.size() - 1; i > arrayList.size() - 1 - pastNIndexes; i--) {
      croppedList.add(arrayList.get(i));
    }
    return croppedList;
  }

  /**
   * @param topic The topic that you want to get changes from.
   * @return An ArrayList containing all of the changes of the given topic
   */
  public static List<Long> recentChanges(IntegerTopic topic) {
    if (INT_HASH.containsKey(topic.getName())) {
      return INT_HASH.get(topic.getName());
    }
    return new ArrayList<>();
  }

  /**
   * Returns a list of the most recent changes of the given topic.
   *
   * @param topic The topic that you want to get changes from.
   * @param pastNIndexes The number of recent changes to retrieve.
   * @return An ArrayList containing the most recent changes of the given topic.
   */
  @SuppressWarnings("PMD.AvoidReassigningParameters")
  public static List<Long> recentChanges(IntegerTopic topic, int pastNIndexes) {
    List<Long> arrayList = recentChanges(topic);
    List<Long> croppedList = new ArrayList<>();
    if (pastNIndexes >= arrayList.size() || pastNIndexes < 0) {
      pastNIndexes = arrayList.size() - 1;
    }
    for (int i = arrayList.size() - 1; i > arrayList.size() - 1 - pastNIndexes; i--) {
      croppedList.add(arrayList.get(i));
    }
    return croppedList;
  }

  /**
   * @param topic The topic that you want to get changes from.
   * @return An ArrayList containing all of the changes of the given topic
   */
  public static List<String> recentChanges(StringTopic topic) {
    if (STRING_HASH.containsKey(topic.getName())) {
      return STRING_HASH.get(topic.getName());
    }
    return new ArrayList<>();
  }

  /**
   * Returns a list of the most recent changes of the given topic.
   *
   * @param topic The topic that you want to get changes from.
   * @param pastNIndexes The number of recent changes to retrieve.
   * @return An ArrayList containing the most recent changes of the given topic.
   */
  @SuppressWarnings("PMD.AvoidReassigningParameters")
  public static List<String> recentChanges(StringTopic topic, int pastNIndexes) {
    List<String> arrayList = recentChanges(topic);
    List<String> croppedList = new ArrayList<>();
    if (pastNIndexes >= arrayList.size() || pastNIndexes < 0) {
      pastNIndexes = arrayList.size() - 1;
    }
    for (int i = arrayList.size() - 1; i > arrayList.size() - 1 - pastNIndexes; i--) {
      croppedList.add(arrayList.get(i));
    }
    return croppedList;
  }

  /**
   * @param topic The topic that you want to get changes from.
   * @return An ArrayList containing all of the changes of the given topic
   */
  public static List<Boolean> recentChanges(BooleanTopic topic) {
    if (BOOLEAN_HASH.containsKey(topic.getName())) {
      return BOOLEAN_HASH.get(topic.getName());
    }
    return new ArrayList<>();
  }

  /**
   * Returns a list of the most recent changes of the given topic.
   *
   * @param topic The topic that you want to get changes from.
   * @param pastNIndexes The number of recent changes to retrieve.
   * @return An ArrayList containing the most recent changes of the given topic.
   */
  @SuppressWarnings("PMD.AvoidReassigningParameters")
  public static List<Boolean> recentChanges(BooleanTopic topic, int pastNIndexes) {
    List<Boolean> arrayList = recentChanges(topic);
    List<Boolean> croppedList = new ArrayList<>();
    if (pastNIndexes >= arrayList.size() || pastNIndexes < 0) {
      pastNIndexes = arrayList.size() - 1;
    }
    for (int i = arrayList.size() - 1; i > arrayList.size() - 1 - pastNIndexes; i--) {
      croppedList.add(arrayList.get(i));
    }
    return croppedList;
  }

  /**
   * Updates static records the values of all doubles from a specific entry (use periodically when
   * calling recentChanges()).
   *
   * @param entryList A list of DoubleEntries
   */
  public static void updateDoubles(List<DoubleEntry> entryList) {
    for (DoubleEntry doubleEntry : entryList) {
      String topicName = doubleEntry.getTopic().getName();

      /* For the circumstance that the given key doesn't exist */
      if (!DOUBLE_HASH.containsKey(topicName)) {

        List<Double> arrayList = new ArrayList<>();
        // arrayList.add(entryList.get(i).get());

        DOUBLE_HASH.put(topicName, arrayList);
        // prevDouble.put(topicName, arrayList.get(arrayList.size() - 1));
      }

      List<Double> arrayList = DOUBLE_HASH.get(topicName);

      if (PREV_DOUBLE.get(topicName) != doubleEntry.get()) {

        arrayList.add(doubleEntry.get());

        /* Updating the previous double value */
        PREV_DOUBLE.put(topicName, doubleEntry.get());
      }
    }
  }

  /**
   * Updates static records the values of all ints from a specific entry (use periodically when
   * calling recentChanges()).
   *
   * @param entryList A list of IntegerEntries
   */
  public static void updateInts(List<IntegerEntry> entryList) {
    for (IntegerEntry integerEntry : entryList) {
      String topicName = integerEntry.getTopic().getName();

      /* For the circumstance that the given key doesn't exist */
      if (!INT_HASH.containsKey(topicName)) {

        List<Long> arrayList = new ArrayList<>();
        arrayList.add(integerEntry.get());

        INT_HASH.put(topicName, arrayList);

        PREV_INT.put(topicName, arrayList.get(arrayList.size() - 1));
      }

      List<Long> arrayList = INT_HASH.get(topicName);

      if (PREV_INT.get(topicName) != integerEntry.get()) {

        arrayList.add(integerEntry.get());

        /* Updating the previous int value */
        PREV_INT.put(topicName, integerEntry.get());
      }
    }
  }

  /**
   * Updates static records the values of all Strings from a specific entry (use periodically when
   * calling recentChanges()).
   *
   * @param entryList A list of StringEntries
   */
  public static void updateStrings(List<StringEntry> entryList) {
    for (StringEntry stringEntry : entryList) {
      String topicName = stringEntry.getTopic().getName();

      /* For the circumstance that the given key doesn't exist */
      if (!STRING_HASH.containsKey(topicName)) {

        List<String> arrayList = new ArrayList<>();
        arrayList.add(stringEntry.get());

        STRING_HASH.put(topicName, arrayList);

        PREV_STRING.put(topicName, arrayList.get(arrayList.size() - 1));
      }

      List<String> arrayList = STRING_HASH.get(topicName);

      if (!PREV_STRING.get(topicName).equals(stringEntry.get())) {

        arrayList.add(stringEntry.get());

        /* Updating the previous String value */
        PREV_STRING.put(topicName, stringEntry.get());
      }
    }
  }

  /**
   * Updates static records the values of all booleans from a specific entry (use periodically when
   * calling recentChanges()).
   *
   * @param entryList A list of BooleanEntries
   */
  public static void updateBooleans(List<BooleanEntry> entryList) {
    for (BooleanEntry booleanEntry : entryList) {
      String topicName = booleanEntry.getTopic().getName();

      /* For the circumstance that the given key doesn't exist */
      if (!BOOLEAN_HASH.containsKey(topicName)) {

        List<Boolean> arrayList = new ArrayList<>();
        arrayList.add(booleanEntry.get());

        BOOLEAN_HASH.put(topicName, arrayList);

        PREV_BOOLEAN.put(topicName, arrayList.get(arrayList.size() - 1));
      }

      List<Boolean> arrayList = BOOLEAN_HASH.get(topicName);

      if (PREV_BOOLEAN.get(topicName) != booleanEntry.get()) {

        arrayList.add(booleanEntry.get());

        /* Updating the previous boolean value */
        PREV_BOOLEAN.put(topicName, booleanEntry.get());
      }
    }
  }
}
