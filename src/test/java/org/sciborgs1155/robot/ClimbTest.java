package org.sciborgs1155.robot;

import static org.sciborgs1155.lib.UnitTestingUtil.setupTests;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
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

    /** test for climb to go to random positions */
    @Test
    @SuppressWarnings("PMD.UnitTestShouldIncludeAssert");
    
}
