package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;

public class LedSubsystem extends SubsystemBase {

  // Constants
  private static final int    CANDLE_ID      = 5;
  private static final int    LED_COUNT      = 58;
  private static final double BLINK_PERIOD   = 0.25;
  private static final double WARN_THRESHOLD = 7.0;

  private static final RGBWColor RED   = new RGBWColor(255, 0,   0, 0);
  private static final RGBWColor GREEN = new RGBWColor(0,   255, 0, 0);
  private static final RGBWColor OFF   = new RGBWColor(0,   0,   0, 0);
  private static final RGBWColor IDLE  = new RGBWColor(20,  20,  20, 0);

  private static final double[] SHIFT_ENDS = { 130.0, 105.0, 80.0, 55.0, 30.0 };

  private static final double TRANSITION_START = 140.0;
  private static final double TRANSITION_END   = 130.0;

  // Hardware
  private final CANdle           m_candle;
  private final SolidColor       m_solidControl   = new SolidColor(0, LED_COUNT);
  private final RainbowAnimation m_rainbowControl = new RainbowAnimation()
      .withBrightness(1.0)
      .withSpeed(0.5)
      .withLedCount(LED_COUNT)
      .withReversed(false);
  private final Timer m_timer = new Timer();

  private Boolean m_wonAuto = null;

  public LedSubsystem() {
    m_candle = new CANdle(CANDLE_ID, CANBus.roboRIO());

    CANdleConfiguration config = new CANdleConfiguration();
    config.LED.StripType        = StripTypeValue.GRB;
    config.LED.BrightnessScalar = 1.0;
    config.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;
    m_candle.getConfigurator().apply(config);

    m_timer.start();
  }

  // Periodic

  @Override
  public void periodic() {
    if (DriverStation.isDisabled()) {
      m_wonAuto = null;
      pulse(IDLE, 2.0);
      return;
    }

    if (m_wonAuto == null && DriverStation.isTeleopEnabled()) {
      m_wonAuto = wonAuto();
    }

    if (isTransitionShift()) {
      if (Boolean.TRUE.equals(m_wonAuto)) {
        rainbow();
      } else {
        setColor(GREEN);
      }
      return;
    }

    if (DriverStation.isAutonomousEnabled()) {
      setColor(GREEN);
      return;
    }

    if (!DriverStation.isTeleopEnabled()) {
      pulse(IDLE, 2.0);
      return;
    }

    boolean active  = isHubActive();
    boolean warning = isShiftChangeSoon();

    if      ( active && !warning) setColor(GREEN);
    else if ( active &&  warning) blink(GREEN);
    else if (!active && !warning) setColor(RED);
    else                          blink(RED);
  }

  // Game logic

  public boolean isTransitionShift() {
    if (!DriverStation.isTeleopEnabled()) return false;
    double matchTime = DriverStation.getMatchTime();
    if (matchTime < 0) return false;
    return matchTime > TRANSITION_END && matchTime <= TRANSITION_START;
  }

  public boolean wonAuto() {
    Optional<Alliance> allianceOpt = DriverStation.getAlliance();
    if (allianceOpt.isEmpty()) return false;
    Alliance alliance = allianceOpt.get();

    String gameData = DriverStation.getGameSpecificMessage();
    if (gameData == null || gameData.isEmpty()) return false;

    return switch (gameData.charAt(0)) {
      case 'R' -> alliance == Alliance.Red;
      case 'B' -> alliance == Alliance.Blue;
      default  -> false;
    };
  }

  public boolean isHubActive() {
    Optional<Alliance> allianceOpt = DriverStation.getAlliance();
    if (allianceOpt.isEmpty()) return true;
    Alliance alliance = allianceOpt.get();

    if (DriverStation.isAutonomousEnabled()) return true;
    if (!DriverStation.isTeleopEnabled())    return false;

    double matchTime = DriverStation.getMatchTime();
    if (matchTime < 0) return true;

    if (matchTime > TRANSITION_END || matchTime <= 30) return true;

    String gameData = DriverStation.getGameSpecificMessage();
    if (gameData == null || gameData.isEmpty()) return true;

    boolean redInactiveShift1 = switch (gameData.charAt(0)) {
      case 'R' -> true;
      case 'B' -> false;
      default  -> { yield true; }
    };

    boolean shift1InactiveForUs = (alliance == Alliance.Red) ? redInactiveShift1 : !redInactiveShift1;
    boolean inactiveThisShift   = (currentShift(matchTime) % 2 == 1) ? shift1InactiveForUs : !shift1InactiveForUs;
    return !inactiveThisShift;
  }

  public boolean isShiftChangeSoon() {
    if (!DriverStation.isTeleopEnabled()) return false;
    double matchTime = DriverStation.getMatchTime();
    if (matchTime < 0) return false;
    for (double boundary : SHIFT_ENDS) {
      if (matchTime > boundary && matchTime <= boundary + WARN_THRESHOLD) return true;
    }
    return false;
  }

  private int currentShift(double matchTime) {
    if (matchTime > TRANSITION_END || matchTime <= 30) return 0;
    if (matchTime > 105) return 1;
    if (matchTime > 80)  return 2;
    if (matchTime > 55)  return 3;
    return 4;
  }

  // Pattern helpers

  private void setColor(RGBWColor color) {
    m_candle.setControl(m_solidControl.withColor(color));
  }

  private void blink(RGBWColor color) {
    double phase = (m_timer.get() % BLINK_PERIOD) / BLINK_PERIOD;
    m_candle.setControl(m_solidControl.withColor(phase < 0.5 ? color : OFF));
  }

  private void pulse(RGBWColor base, double cycleSec) {
    double brightness = (1.0 - Math.cos((m_timer.get() % cycleSec) / cycleSec * 2.0 * Math.PI)) / 2.0;
    m_candle.setControl(m_solidControl.withColor(new RGBWColor(
        (int)(base.Red   * brightness),
        (int)(base.Green * brightness),
        (int)(base.Blue  * brightness),
        0)));
  }

  private void rainbow() {
    m_candle.setControl(m_rainbowControl);
  }
}