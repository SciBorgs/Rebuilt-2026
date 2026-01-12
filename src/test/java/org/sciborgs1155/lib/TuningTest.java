package org.sciborgs1155.lib;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.sciborgs1155.lib.UnitTestingUtil.setupTests;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.IntegerEntry;
import edu.wpi.first.networktables.StringEntry;
import java.util.ArrayList;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class TuningTest {

  @BeforeEach
  public void setup() {
    setupTests();
  }

  @Test
  void fullEntryTest() {
    DoubleEntry dbleEnt;
    IntegerEntry intEnt;
    StringEntry strEnt;
    BooleanEntry boolEnt;

    double dbleVal = 2.0;
    long intVal = 7823; // IntgerTopic.getEntry() accepts longs for default values
    String strVal = "Hello, World! <3";
    boolean boolVal = true;

    dbleEnt = Tuning.entry("/Robot/a", dbleVal);
    intEnt = Tuning.entry("/Robot/b", intVal);
    strEnt = Tuning.entry("/Robot/c", strVal);
    boolEnt = Tuning.entry("/Robot/d", boolVal);

    assertEquals(dbleVal, dbleEnt.get());
    assertEquals(intVal, intEnt.get());
    assertEquals(strVal, strEnt.get());
    assertEquals(boolVal, boolEnt.get());

    Tuning.put(dbleEnt.getTopic(), 1155.2265);
    Tuning.put(intEnt.getTopic(), 2612668);
    Tuning.put(strEnt.getTopic(), "como estas");
    Tuning.put(boolEnt.getTopic(), false);

    assertEquals(1155.2265, dbleEnt.get());
    assertEquals(2612668, intEnt.get());
    assertEquals("como estas", strEnt.get());
    assertEquals(false, boolEnt.get());

    ArrayList<Double> doubleList = new ArrayList<>();
    doubleList.add(dbleVal);
    doubleList.add(1155.2265);

    ArrayList<Long> intList = new ArrayList<>();
    intList.add(intVal);
    intList.add((long) 2612668);

    ArrayList<String> strList = new ArrayList<>();
    strList.add(strVal);
    strList.add("como estas");

    ArrayList<Boolean> boolList = new ArrayList<>();
    boolList.add(boolVal);
    boolList.add(false);

    assertEquals(doubleList, Tuning.recentChanges(dbleEnt.getTopic()));
    assertEquals(intList, Tuning.recentChanges(intEnt.getTopic()));
    assertEquals(strList, Tuning.recentChanges(strEnt.getTopic()));
    assertEquals(boolList, Tuning.recentChanges(boolEnt.getTopic()));

    assertEquals(doubleList.get(1), Tuning.recentChanges(dbleEnt.getTopic(), 1).get(0));
    assertEquals(intList.get(1), Tuning.recentChanges(intEnt.getTopic(), 1).get(0));
    assertEquals(strList.get(1), Tuning.recentChanges(strEnt.getTopic(), 1).get(0));
    assertEquals(boolList.get(1), Tuning.recentChanges(boolEnt.getTopic(), 1).get(0));
  }
}
