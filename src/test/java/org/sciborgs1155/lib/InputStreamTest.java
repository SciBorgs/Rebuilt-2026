package org.sciborgs1155.lib;

import static edu.wpi.first.units.Units.Seconds;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.sciborgs1155.lib.InputStream.*;

import edu.wpi.first.math.MathSharedStore;
import org.junit.jupiter.api.Test;

public class InputStreamTest {

  private InputStream stream(double n) {
    return of(() -> n);
  }

  @Test
  void hypotenuse() {
    assertEquals(5, hypot(stream(3), stream(4)).get());
  }

  @Test
  void arctan() {
    assertEquals(-Math.PI / 4, atan(stream(1), stream(-1)).get());
  }

  @Test
  void map() {
    assertEquals(8, stream(3).map(n -> 40 * n % 16).get());
  }

  @Test
  void scale() {
    assertEquals(-12.44, stream(4).scale(() -> -3.11).get());
    assertEquals(-12.44, stream(4).scale(-3.11).get());
  }

  @Test
  void negate() {
    assertEquals(-13.123, stream(13.123).negate().get());
  }

  @Test
  void add() {
    assertEquals(9, stream(7).add(2).get());
    assertEquals(5.232, stream(1.232).add(() -> 4).get());
  }

  @Test
  void pow() {
    assertEquals(8, stream(2).pow(3).get());
  }

  @Test
  void signedPow() {
    assertEquals(0.25, stream(0.5).signedPow(2).get());
    assertEquals(-0.25, stream(-0.5).signedPow(2).get());
  }

  @Test
  void deadband() {
    assertEquals(0, stream(-0.5).deadband(0.6, 1).get());
    assertEquals(2.25, stream(2).deadband(0.2, 1).get());
  }

  @Test
  void clamp() {
    assertEquals(5.32, stream(5.32).clamp(6.5).get());
    assertEquals(-6.5, stream(-7).clamp(6.5).get());
  }

  @Test
  void rateLimit() {
    UnitTestingUtil.setupTests();
    InputStream stream = of(() -> MathSharedStore.getTimestamp() * 2);
    InputStream limited = stream.rateLimit(1);
    double initial = limited.get();
    UnitTestingUtil.fastForward(Seconds.of(15));
    assertEquals(30, stream.get() - initial, 0.2);
    assertEquals(15, limited.get() - initial, 0.2);
  }
}
