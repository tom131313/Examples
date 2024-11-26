package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.print;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Milliseconds;

import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.AchieveHueGoal;
import frc.robot.subsystems.GroupDisjointTest;
import frc.robot.subsystems.HistoryFSM;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.RobotSignals;
import frc.robot.subsystems.RobotSignals.LEDPatternSupplier;

import java.util.Optional;

public abstract class CommandsTriggers {
  private static CommandXboxController              m_operatorController;
  private static RobotSignals                       m_robotSignals; // container and creator of all the LEDView subsystems
  private static Optional<AchieveHueGoal>           m_achieveHueGoal;
  private static Optional<GroupDisjointTest>        m_groupDisjointTest;
  private static Optional<HistoryFSM>               m_historyFSM;
  private static Optional<Intake>                   m_intake;
  private static Optional<Boolean>                  m_UseAutonomousSignal;
  private static Optional<Boolean>                  m_UseColorWheel;
  private static Optional<Boolean>                  m_UseMainDefault;
  private static Optional<Boolean>                  m_UseEnableDisable;

  
  private CommandsTriggers()
    {}

  public static void create(RobotContainer robotContainer)
  {
    m_operatorController = robotContainer.getM_operatorController();
    m_robotSignals = robotContainer.getM_robotSignals();
    m_achieveHueGoal = robotContainer.getM_achieveHueGoal();
    m_groupDisjointTest = robotContainer.getM_groupDisjointTest();
    m_historyFSM = robotContainer.getM_historyFSM();
    m_intake = robotContainer.getM_intake();
    m_UseAutonomousSignal = robotContainer.getM_autonomousSignal();
    m_UseColorWheel = robotContainer.getM_useColorWheel();
    m_UseMainDefault = robotContainer.getM_useMainDefault();
    m_UseEnableDisable = robotContainer.getM_useEnableDisable();

    configureSuppliers();
    configureGameControllersBindings();
    configureDefaultCommands();
  }

  private static RobotSignals.LEDPatternSupplier colorWheel;
  /**
   * Configure SUppliers
   */
  private static void configureSuppliers() {
    colorWheel = // produce a LED color pattern based on the timer current seconds of the minute
      () ->
        LEDPattern.solid(
            Color.fromHSV(
                (int) (Timer.getFPGATimestamp() % 60.0 /* seconds of the minute */)
                    * 3 /* scale seconds to 180 hues per color wheel */,
                200,
                200));
  }

  /**
   * Get disjointed sequence test from its creator for use by Robot - passing the reference up
   * 
   * @return Command to be scheduled to run disjointed sequence test
   */
  public static Command getDisjointedSequenceTest() {
    if(m_groupDisjointTest.isPresent())
    {
      return m_groupDisjointTest.get().m_disjointedSequenceTest;
    }
    else
    {
      return print("Group Disjointed Test not selected");
    }
  }

  /**
   * Create a command to signal Autonomous mode
   *
   * <p>Example of setting two signals by contrived example of composed commands
   *
   * @return LED pattern signal for autonomous mode
   */
  public static Command setAutonomousSignal() {
    if(m_UseAutonomousSignal.isPresent()) {
      LEDPattern autoTopSignal =
            LEDPattern.solid(new Color(0.1, 0.2, 0.2))
            .blend(LEDPattern.solid(new Color(0.7, 0.2, 0.2)).blink(Seconds.of(0.1)));
            
      LEDPattern autoMainSignal = LEDPattern.solid(new Color(0.3, 1.0, 0.3));
      // statements before the return are run early at initialization time
      return
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
    else {
      return
        print("Autonomous Signal not selected");
    }
  }

  /**
   * configure driver and operator controllers' buttons
   */
  private static void configureGameControllersBindings() {

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
    m_UseColorWheel.ifPresent((x) ->
    {
      var xButtonDebounceTime = Milliseconds.of(30.0);
      m_operatorController
          .x()
          .debounce(xButtonDebounceTime.in(Seconds), DebounceType.kBoth)
          .onTrue(m_robotSignals.m_top.setSignal(colorWheel.get()));
    });

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
   * Configure some of the Default Commands
   *
   * <p>WARNING - heed the advice in the Robot.java comments about default commands
   */
  private static void configureDefaultCommands() {
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

    m_UseColorWheel.ifPresent((x)->
    {
      final Command topDefault =
          m_robotSignals
              .m_top
              .setSignal(topDefaultSignal)
              .ignoringDisable(true)
              .withName("TopDefault");
      m_robotSignals.m_top.setDefaultCommand(topDefault);
    });

    m_UseMainDefault.ifPresent((x)->
    {
      final Command mainDefault =
          m_robotSignals
              .m_main
              .setSignal(mainDefaultSignal)
              .ignoringDisable(true)
              .withName("MainDefault");
      m_robotSignals.m_main.setDefaultCommand(mainDefault);
    });

    m_UseEnableDisable.ifPresent((x)->
    {
      final Command enableDisableDefault =
          m_robotSignals
              .m_enableDisable
              .setSignal(enableDisableDefaultSignal)
              .ignoringDisable(true)
              .withName("EnableDisableDefault");
      m_robotSignals.m_enableDisable.setDefaultCommand(enableDisableDefault);
    });

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
}
