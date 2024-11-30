/*
 * Example program that shows a variety of command based and programming "best practices."
 * 
 * Includes six different techniques useful in Command-Based programming. In addition all of the
 * examples are written in a similar suggested style of handling commands and triggers with
 * suggested variable naming style and minimal scope.
 *  1. Goal-Oriented subsystem to feed setpoints to a command-scheduled control calculation. (PID
 *     example)
 *  2. Use of historical data in addition to current state and events as input to a Finite State
 *     Machine. (Random, non-repeating colors)
 *  3. Example of splitting an apparent single resource into pieces for independent use.
 *     (Addressable LED strip)
 *  4. Minimal example of a robot subsystem. (Command triggered by an event)
 *  5. Use of "proxies" to disjoint composed groups such that a subsystem default command may run
 *     within a group instead of the normal behavior of being blocked until the group completes.
 *     (Simple subsystems running commands and default commands concurrently in groups)
 *  6. Example of using a Moore-like FSM structure using an input state and a triggering event to
 *     transition to a new state.
 *
 * Because all but one demonstration uses an addressable LED strip as output (one has console
 * output) there is significant overlap and depth in demonstrating style of using the 2025 advanced
 * addressable LED classes and methods.
 * 
 * Demonstration output is on seven sets of eight identical LEDs to show the program is operating;
 * operator input is Xbox controller. The eighth demonstration output is the terminal console
 * "prints."
 *
 * 1. LED set 1 usage Top LEDView subsystem default blue.
 *  Autonomous mode command brown fast blink.
 *  Non-autonomous displays colors slowly around the color wheel initiated by pressing "X" button.
 *
 * 2. LED set 2 usage Main LEDView subsystem default cyan.
 *  Game Piece Intake Acquired subsystem signal intake game piece acquired magenta fast blink
 *  (simulate game piece intake acquired by pressing "B" button).
 *  Autonomous mode command light green (no requirement for Game Piece Intake Acquired).
 *
 * 3. LED set 3 usage EnableDisable LEDView subsystem.
 *  Enabled mode green slow blink; disabled mode red slow blink.
 *
 * 4. LED set 4 usage HistoryDemo LEDView subsystem.
 *  HistoryFSM subsystem displays random colors that don't repeat for awhile (time history).
 *  Periodic color changing initiated by pressing "Y" button then self perpetuating. Colors also
 *  change if the "Y" button is pressed. Runs in enabled mode.
 *
 * 5. LED set 5 usage AchieveHueGoal LEDView subsystem.
 *  AchieveHueGoal subsystem controller command to achieve the goal set by the goal supplier.
 *  Colors on color wheel position show PID controller converging on a color selected by Xbox right
 *  trigger axis. Press trigger axis a little to start and modulate to select hue goal. Press "A"
 *  button to interrupt controller before the goal has been achieved. The selected color blinks
 *  shortly at the end to indicate the controller is off and then gray. Fast blink white default
 *  command indicates the LED display command has malfunctioned and is not running even if the
 *  controller runs.
 * 
 * 6. LED set 6 usage MooreLikeFSM LEDView subsystem.
 *  Moore Like FSM structured subsystem runs continuously (except for a brief pause to show the FSM
 *  can be deactivated and reactivated) to display a KnightRider Kitt red LED Scanner. State is
 *  implemented as a single command.
 * 
 *  In addition to the LED output the SmartDashboard/ShuffleBoard display the actions of the FSM
 *  last entry, steadystate, and exit methods.
 *
 * 7. LED set 7 usage MooreLikeFSMMultiCommand LEDView subsystem.
 *  Alternate version of the Moore Like FSM showing use of multiple commands to implement a state.
 *  Uses a faster speed and orange color for this Kitt imposter and has the same deactivated pause
 *  and reactivation as the other Moore Like FSM.
 *
 *  In addition to the LED output the SmartDashboard/ShuffleBoard display the actions of the FSM
 *  last entry, steadystate, and exit methods.
 *
 * 8. Console Terminal usage GroupDisjoint subsystem.
 *  Disjoint Sequential Group Demo console output initiated by entering teleop enable mode.
 *  Show that subsystem default command doesn't run within a group command unless the command with
 *  the subsystem requirement is disjointed from the group by using a Proxy structure or separated
 *  commands structure.
 *
 * There are user-selectable options to run the various examples. The defualt is to run all of them.
 * 
 * There are user-selectable options to run various logging protocols. The default is to create the
 * DataLog which also has its own SmartdashBoard/ShuffleBoard entries. Other options are to log to
 * the Console and to create ShuffleBoardLog event markers.
 * 
 * All commands are interruptible. Some button presses are debounced.
 */

/*
 * Example program demonstrating:
 *
 * Splitting a common resource (string of LEDs into multiple separately used resources).
 * Configure button trigger.
 * Triggers.
 * Use of command parameters set at command creation time.
 * Use of command parameters set at dynamically at runtime (Suppliers).
 * Use of method reference.
 * Some commentary on composite commands and mode changes.
 * Command logging.
 * Configuring an autonomous command.
 * Use of Xbox controller to produce fake events.
 * Use of Xbox controller to trigger an event.
 * Use of public command factories in subsystems.
 * Overloading method parameter types.
 * No commands with the word Command in the name.
 * No triggers with the word Trigger in the name.
 * Supplier of dynamic LED pattern.
 * Static LED pattern.
 * Restrict Subsystem Default Command to none until set once at any time and then unchangeable.
 * Controller subsystem scheduled by a command to reach a Goal.
 * Default commands can either run or not run within a sequential group depending on how the group is defined using Proxy.
 * Commands run in sequence by triggering successive commands.
 * Use of Time.
 * Use of sequential and parallel composed command groups to perform tasks.
 * Use of a reusable Moore-Like FSM structure of current state, trigger, new state transitions.
 * Use of a perpetually running command to accept "goals".
 * Use of Alerts.
 */

/*
 * Default Commands can be useful but they normally do not run within grouped commands even if their
 * associated subsystem is not active at all times within the group.
 *
 * There are several possibilities to accommodate that restriction:
 *  1. do without default commands at any time.
 *  2. do without the default command only within the group.
 *  3. manually code the function of the default command within a group.
 *  4. break groups into smaller groups and use Triggers to sequence multiple groups.
 *  5. use Proxy branching out of the group restriction.
 *
 * Using Triggers to sequence successive commands may help better organize the command flow and
 * isolate some subsystem requirements so the default command can run. That’s okay and is preferred
 * to using proxy commands.
 *
 * Usage of Proxies to hide the subsystem requirements from normal checks and thus allow the
 * default command to activate could be useful but should be used extremely sparingly by an
 * experienced user.
 *
 * The possibility of unintended consequences is very high if bypassing the normal checks of
 * requirements. Usage should be limited to when you can easily understand what exactly you’re
 * doing by invoking them.
 *
 * Judicious use of asProxy() on a group’s interior commands can often allow default commands to
 * run correctly within groups. Incorrect application of Proxy results in an extremely difficult
 * problem to debug.
 *
 * Slapping an asProxy() around a composed command isn’t sufficient. You have to use proxy on the
 * inner commands, also or instead, and any new library commands to ease the use of Proxy aren’t
 * recursive to inner layers.
 *
 * After thoroughly understanding the structure of your groups extremely carefully add asProxy() to
 * as few interior commands as possible to accomplish what you need.
 *
 * Known problems:
 *
 * Proxies break the sanity check that warn of a subsystem running in parallel commands. There is no
 * warning - just wrong results (a warning might be added to the new combined Proxy library
 * functions but that helps only if you use those functions correctly and nothing says you have to).
 *
 * repeatedly() doesn’t repeat correctly due to a bug in the WPILib. (The repeat is different than
 * the original proxy command).
 *
 * andThen() doesn’t work but that can be circumvented by using sequence().
 */

/*
 * Uses of a Subsystem's Default Command that can cause unexpected results:
 *
 * Default Command normally does't run in a group of commands. Not using default commands can
 * prevent that inconsistency but then you lose the benefits of default commands.
 *
 * Default Command can be set more than once but only the last one set is active. It might not be
 * obvious which Default Command is being used. Never setting more than one default command can
 * prevent that confusion.
 *
 * A default Command doesn't have to be set and can be removed after setting. Again it might not be
 * obvious what default command is active. Consider mistake-proofing strategies to prevent
 * confusion.
 */

/*
 * This example program runs in real or simulated mode of the 2024 WPILib.
 *
 * This is a refactor and extension of code donated by ChiefDelphi @illinar. It is intended to
 * demonstrate good programming based on @Oblarg's rules and comments by @Amicus1.
 *
 * The use of the InternalTrigger to sequence command runs was donated by ChiefDelphi @bovlb
 *
 * Errors and confusions are the fault of ChiefDelphi @SLAB-Mr.Thomas; github tom131313.
 */

/*
 * Warning:
 * 
 * WPILib examples often have a cancelAll() for commands. This program uses a perpetually running
 * command for one example and cancelling that command is fatal for that controller example.
 */
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private RobotContainer m_robotContainer = new RobotContainer();
  private Command m_autonomousSignal;

  public Robot() {
    CommandsTriggers.create(m_robotContainer);
  }

  @Override
  public void robotPeriodic() {
    // get a consistent set of all inputs including non-subsystems not in scheduler run
    m_robotContainer.runBeforeCommands(); // this is essentially similar to running the scheduler
                                          // Subsystem.periodic()

    // check all triggers and run all scheduled commands; all Subsystem.periodic() are run first
    CommandScheduler.getInstance().run();

    // write outputs like logging, dashboards, indicators, meh - goal-oriented subsystem periodic
    m_robotContainer.runAfterCommands();
  }

  @Override
  public void disabledInit() {} // Commands running from another mode haven't been cancelled.

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    // Commands running from another mode haven't been cancelled directly but may be interrupted by
    // this command.
    m_autonomousSignal = CommandsTriggers.setAutonomousSignal();
    m_autonomousSignal.schedule();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {
    m_autonomousSignal.cancel(); // cancel in case still running
  }

  @Override
  public void teleopInit() {
    // Commands running from another mode haven't been cancelled directly except the one below.
    if(m_autonomousSignal == null) { // check null in case not initialized in auto mode
      m_autonomousSignal.cancel(); // cancel in case still running
    }
    CommandsTriggers.getDisjointedSequenceTest().schedule();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
