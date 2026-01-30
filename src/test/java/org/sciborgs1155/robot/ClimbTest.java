package org.sciborgs1155.robot;

import static edu.wpi.first.units.Units.Meters;
import static org.sciborgs1155.lib.Test.runUnitTest;
import static org.sciborgs1155.lib.UnitTestingUtil.reset;
import static org.sciborgs1155.robot.climb.ClimbConstants.MAX_HEIGHT; 
import static org.sciborgs1155.lib.UnitTestingUtil.setupTests;
import static org.sciborgs1155.robot.climb.ClimbConstants.MIN_HEIGHT;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.sciborgs1155.robot.climb.Climb;
import org.sciborgs1155.robot.climb.SimClimb;

public class ClimbTest {

    private Climb climb;

    /** initializes unit tests and sim climb */
    @BeforeEach
    public void initialize() {
        setupTests();
        climb = new Climb(new SimClimb());
    }

    /** resets the sim climb */
    @AfterEach
    public void destroy() throws Exception {
        reset(climb);
    }

    /** test for hood to go to random angles */
    @Test
    @SuppressWarnings("PMD.JUnitTestsShouldIncludeAssert")
    public void randAngle() {
        runUnitTest(
            climb.goToTest(
                Meters.of(Math.random() * MAX_HEIGHT.minus(MIN_HEIGHT).in(Meters)).plus(MIN_HEIGHT)));
    }
}
