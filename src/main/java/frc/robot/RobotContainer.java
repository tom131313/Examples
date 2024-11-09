package frc.robot;

/**
 * Create subsystems, triggers, and commands; bind buttons to commands and triggers; define command
 * logging; manage the details of what is periodically processed before and after the command
 * scheduler loop; - everything until it got too big and some logical splits to other classes had to
 * be made.
 */

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.print;

import frc.robot.subsystems.AchieveHueGoal;
import frc.robot.subsystems.GroupDisjointTest;
import frc.robot.subsystems.HistoryFSM;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.MooreLikeFSM;
import frc.robot.subsystems.MooreLikeFSMMultiCommand;
import frc.robot.subsystems.RobotSignals;
import frc.robot.subsystems.RobotSignals.LEDPatternSupplier;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import java.lang.invoke.MethodHandles;
import java.util.Optional;

public class RobotContainer {
  private static final String fullClassName = MethodHandles.lookup().lookupClass().getCanonicalName();
  static
  {
    System.out.println("Loading: " + fullClassName);
    System.out.println("WPILib version " + edu.wpi.first.wpilibj.util.WPILibVersion.Version);
  }

  // options for logging
  private boolean useConsole            = false;
  private boolean useDataLog            = true;
  private boolean useShuffleBoardLog    = false;

  // options to select desired demonstrations
  private boolean useAchieveHueGoal           = true;
  private boolean useGroupDisjointTest        = true;
  private boolean useHistoryFSM               = true;
  private boolean useIntake                   = true;
  private boolean useMooreLikeFSM             = true;
  private boolean useMooreLikeFSMMultiCommand = true;
  private boolean useAutonomousSignal         = true;
  private boolean useColorWheel               = true;
  private boolean useMainDefault              = true;
  private boolean useEnableDisable            = true;

  private final CommandXboxController m_operatorController;
  private CommandSchedulerLog schedulerLog;

  // define all the subsystems
  private final RobotSignals m_robotSignals; // container and creator of all the LEDView subsystems
  private Optional<Intake>                   m_intake             = Optional.empty();
  private Optional<HistoryFSM>               m_historyFSM         = Optional.empty();
  private Optional<AchieveHueGoal>           m_achieveHueGoal     = Optional.empty();
  private Optional<MooreLikeFSM>             m_mooreLikeFSMtop    = Optional.empty();
  private Optional<MooreLikeFSMMultiCommand> m_mooreLikeFSMbottom = Optional.empty();
  private Optional<GroupDisjointTest>        m_groupDisjointTest  = Optional.empty(); // container and creator of all
                                                                                      // the group/disjoint tests
  /**
   * Constructor creates most of the subsystems and operator controller bindings
   */
  public RobotContainer() {
 
    /* There are 10's of thousands of ways to do logging.
     * Here are 3 ways with options within the method.
     */
    configureCommandLogs(); // do early on otherwise log not ready for first commands

    final int operatorControllerPort = 0;
    m_operatorController = new CommandXboxController(operatorControllerPort);
    // subsystems
    m_robotSignals = new RobotSignals();

    // optional subsystems
    if(useIntake)                   m_intake             = Optional.of(new Intake(m_robotSignals.m_main));
    if(useHistoryFSM)               m_historyFSM         = Optional.of(new HistoryFSM(m_robotSignals.m_historyDemo));
    if(useAchieveHueGoal)           m_achieveHueGoal     = Optional.of(new AchieveHueGoal(m_robotSignals.m_achieveHueGoal));
    if(useMooreLikeFSM)             m_mooreLikeFSMtop    = Optional.of(new MooreLikeFSM(m_robotSignals.m_knightRider, 10.0, Color.kRed));
    if(useMooreLikeFSMMultiCommand) m_mooreLikeFSMbottom = Optional.of(new MooreLikeFSMMultiCommand(m_robotSignals.m_imposter, 9.9, Color.kOrange));
    if(useGroupDisjointTest)        m_groupDisjointTest  = Optional.of(new GroupDisjointTest());

    configureBindings();

    configureDefaultCommands();
  }

  /**
   * configure driver and operator controllers' buttons
   */
  private void configureBindings() {

    /**
     * Use operator "B" button for a fake indicator game piece is acquired
     */
    m_intake.ifPresent((x)->m_operatorController.b().whileTrue(x.gamePieceIsAcquired()));

    /**
     * Start History FSM Control with the operator "Y" button or it's time for a new color
     */
    m_historyFSM.ifPresent((x)->
    {
      var yButtonDebounceTime = Milliseconds.of(40.0);
      m_operatorController.y().debounce(yButtonDebounceTime.in(Seconds)).or(x::timesUp)
        .onTrue(x.newColor());
    });
    

    /**
     * Start a color wheel display with the operator "X" button
     */
    if(useColorWheel)
    {
      var xButtonDebounceTime = Milliseconds.of(30.0);
      m_operatorController
          .x()
          .debounce(xButtonDebounceTime.in(Seconds), DebounceType.kBoth)
          .onTrue(m_robotSignals.m_top.setSignal(colorWheel()));
    }

    /**
      * Goal setting demo control
      *
      * The PID controller is not running initially until a setpoint is set by moving the operator
      * right trigger axis past the threshold at which time a command runs to achieve that goal.
      */
    m_achieveHueGoal.ifPresent((x)->
    {
      var triggerHueGoalDeadBand = 0.05; //triggers if past a small threshold (scale of 0 to 1)
      m_operatorController.rightTrigger(triggerHueGoalDeadBand)
          .onTrue(
              x.achieveHue( // goal-acceptance command
                  () -> m_operatorController.getRightTriggerAxis()*180.0 // supplying the setpoint
                  // scale joystick's 0 to 1 to computer color wheel hue 0 to 180
                  ));

      // immediately stop controller
      m_operatorController.a()
          .onTrue(x.interrupt());
    });
  }

  /**
   * "color wheel" supplier runs when commanded
   *
   * @return LED pattern for the color selected by the operator controller
   */
  private RobotSignals.LEDPatternSupplier colorWheel() {
    // produce a color based on the timer current seconds of the minute
    return () ->
        LEDPattern.solid(
            Color.fromHSV(
                (int) (Timer.getFPGATimestamp() % 60.0 /* seconds of the minute */)
                    * 3 /* scale seconds to 180 hues per color wheel */,
                200,
                200));
  }

  /**
   * Configure some of the Default Commands
   *
   * <p>WARNING - heed the advice in the Robot.java comments about default commands
   */
  private void configureDefaultCommands() {
    final LEDPattern topDefaultSignal = LEDPattern.solid(new Color(0.0, 0.0, 1.0));
    final LEDPattern mainDefaultSignal = LEDPattern.solid(new Color(0.0, 1.0, 1.0));
    final LEDPattern disabled = LEDPattern.solid(Color.kRed).breathe(Seconds.of(2.0));
    final LEDPattern enabled = LEDPattern.solid(Color.kGreen).breathe(Seconds.of(2.0));
    final LEDPatternSupplier enableDisableDefaultSignal =
        () -> DriverStation.isDisabled() ? disabled : enabled;
    // Intended that hue controller display always be ON so make it noticeable that it's OFF
    // since this default command should never run
    final LEDPattern hueControllerDisplayOffSignal = LEDPattern.solid(Color.kWhiteSmoke)
        .blink(Seconds.of(0.09));

    if(useColorWheel)
    {
      final Command topDefault =
          m_robotSignals
              .m_top
              .setSignal(topDefaultSignal)
              .ignoringDisable(true)
              .withName("TopDefault");
      m_robotSignals.m_top.setDefaultCommand(topDefault);
    }

    if(useMainDefault)
    {
      final Command mainDefault =
          m_robotSignals
              .m_main
              .setSignal(mainDefaultSignal)
              .ignoringDisable(true)
              .withName("MainDefault");
      m_robotSignals.m_main.setDefaultCommand(mainDefault);
    }

    if(useEnableDisable)
    {
      final Command enableDisableDefault =
          m_robotSignals
              .m_enableDisable
              .setSignal(enableDisableDefaultSignal)
              .ignoringDisable(true)
              .withName("EnableDisableDefault");
      m_robotSignals.m_enableDisable.setDefaultCommand(enableDisableDefault);
    }

    m_achieveHueGoal.ifPresent((x)->
    {
      final Command hueControllerDisplayDefault =
          m_robotSignals
              .m_achieveHueGoal
              .setSignal(hueControllerDisplayOffSignal)
              .ignoringDisable(true)
              .withName("HueControllerDisplayOff");
      m_robotSignals.m_achieveHueGoal.setDefaultCommand(hueControllerDisplayDefault);
      x.achieveHueDisplay().schedule(); // Not strictly the default but it should
      // always be running and we never see the default. Using the default command for the perpetual
      // command may be more robust as it restarts if cancelled for any reason but that wasn't used
      // in this example. cancelAll() will kill this so don't! (It wouldn't kill a default command.)
    });
  }

  /**
   * Create a command to signal Autonomous mode
   *
   * <p>Example of setting two signals by contrived example of composed commands
   *
   * @return LED pattern signal for autonomous mode
   */
  public Command setAutonomousSignal() {
    if(useAutonomousSignal)
    {
      LEDPattern autoTopSignal =
            LEDPattern.solid(new Color(0.1, 0.2, 0.2))
            .blend(LEDPattern.solid(new Color(0.7, 0.2, 0.2)).blink(Seconds.of(0.1)));
            
      LEDPattern autoMainSignal = LEDPattern.solid(new Color(0.3, 1.0, 0.3));
      // statements before the return are run early at initialization time
      return
        // statements returned are run later when the command is scheduled
        parallel(
                // interrupting either of the two parallel commands with an external command interrupts
                // the group
                m_robotSignals.m_top.setSignal(autoTopSignal)
                    .withTimeout(6.0)/*.asProxy()*/ // timeout ends but the group continues and
                // the default command is not activated here with or without the ".andThen" command.
                // Use ".asProxy()" to disjoint the group and allow the "m_top" default command to run.
                // What happened to the ".andThen"? Beware using Proxy can cause surprising behavior!
                    .andThen(m_robotSignals.m_top.setSignal(autoTopSignal)),

                m_robotSignals.m_main.setSignal(autoMainSignal))
        .withName("AutoSignal");      
    }
    return print("Autonomous Signal not selected");
  }

  /**
   * Get disjointed sequence test from its creator for use by Robot - passing the reference up
   * 
   * @return Command to be scheduled to run disjointed sequence test
   */
  public Command getDisjointedSequenceTest() {
    if(m_groupDisjointTest.isPresent())
    {
      return m_groupDisjointTest.get().m_disjointedSequenceTest;
    }
    return print("Group Disjointed Test not selected");
  }

  /**
   * Configure Command logging to Console/Terminal, DataLog, or ShuffleBoard
   */
  public void configureCommandLogs()
  {
      if (useConsole || useDataLog || useShuffleBoardLog) {
        schedulerLog = new CommandSchedulerLog(useConsole, useDataLog, useShuffleBoardLog);
        schedulerLog.logCommandInitialize();
        schedulerLog.logCommandInterrupt();
        schedulerLog.logCommandFinish();
        schedulerLog.logCommandExecute();  // Can (optionally) generate a lot of output        
      }
  }

  /**
   * There are a variety of techniques to run I/O methods periodically and the example implemented
   * below in this code is a very simplistic start of a good possibility.
   * 
   * It demonstrates running before the scheduler loop to get a consistent set of sensor inputs.
   * After the scheduler loop completes all periodic outputs from subsystems are run such as data
   * logging and dashboards. (When enabled, the command scheduler runs its registered
   * subsystem.periodic() first but only for subsystems. Its use was threatened to be deprecated.)
   * (There is additional related discussion of periodic running in AchieveHueGoal.)
   *
   * There are clever ways to register classes say using a common "SubsystemTeam" class or
   * interface with a "register" method so they are automatically included in a list that can
   * easily be accessed with a loop. But this example is simplistic with no registration and no
   * loop - remember to type them in here and in any class that has multiple subsystems such as the 
   * example "GroupDisjointTest".
   * 
   * Security to prevent unauthorized running of periodic methods could be implemented in a variety
   * of ways but that error doesn't seem to happen so these examples have all "public" periodic
   * methods. Don't run them except in the designated places in the code.
   */

  /**
   * Run before commands and triggers from the Robot.periodic()
   *
   * <p>Run periodically before commands are run - read sensors, etc. Include all classes that have
   * periodic inputs or other need to run periodically.
   *
   */
  public void runBeforeCommands() {
    m_intake            .ifPresent((x)->x.runBeforeCommands());
    m_historyFSM        .ifPresent((x)->x.runBeforeCommands());
    m_achieveHueGoal    .ifPresent((x)->x.runBeforeCommands());
    m_mooreLikeFSMtop   .ifPresent((x)->x.runBeforeCommands());
    m_mooreLikeFSMbottom.ifPresent((x)->x.runBeforeCommands());
    m_groupDisjointTest .ifPresent((x)->x.runBeforeCommands());
    m_robotSignals                       .runBeforeCommands();
  }

  /**
   * Run after commands and triggers from the Robot.periodic()
   *
   * <p>Run periodically after commands are run - write logs, dashboards, indicators Include all
   * classes that have periodic outputs
   */
  public void runAfterCommands() {
    m_intake            .ifPresent((x)->x.runAfterCommands());
    m_historyFSM        .ifPresent((x)->x.runAfterCommands());
    m_achieveHueGoal    .ifPresent((x)->x.runAfterCommands());
    m_mooreLikeFSMtop   .ifPresent((x)->x.runAfterCommands());
    m_mooreLikeFSMbottom.ifPresent((x)->x.runAfterCommands());
    m_groupDisjointTest .ifPresent((x)->x.runAfterCommands());
    m_robotSignals                       .runAfterCommands();
  }
}
