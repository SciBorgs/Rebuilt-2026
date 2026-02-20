package org.sciborgs1155.lib;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Set;

/** A test, consisting of a command and assertions. Can be used for unit tests or ran on robot. */
public record Test(Command testCommand, Set<Assertion> assertions) {
  /**
   * @param command
   * @return a Test with no assertions
   */
  public static Test fromCommand(Command command) {
    return new Test(command, Set.of());
  }

  private static Command toCommand(Test test) {
    return test.testCommand.finallyDo(() -> test.assertions.forEach(a -> a.apply()));
  }

  /**
   * Creates a sequential command from Tests.
   *
   * @return a command that runs the testCommand and assertions from each test in turn
   */
  public static Command toCommand(Test... tests) {
    Command c = Commands.none();
    for (Test test : tests) {
      c = c.andThen(toCommand(test));
    }
    return c;
  }
}
