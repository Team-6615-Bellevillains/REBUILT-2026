package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.Optional;

public class LedSubsystem extends SubsystemBase {

  // Constants
  private static final int    CANDLE_ID      = 5;
  private static final int    LED_COUNT      = 58;
  private static final double BLINK_PERIOD   = 0.25;  // seconds per half-cycle
  private static final double WARN_THRESHOLD = 7.0;   // seconds before shift to start blinking

  // Alliance colors
  private static final RGBWColor RED   = new RGBWColor(255, 0, 0, 0);
  // private static final RGBWColor RED   = new RGBWColor(200, 50, 0, 0);
  private static final RGBWColor BLUE  = new RGBWColor(255, 0, 0, 0);

  // Active (hub open) color
  private static final RGBWColor GREEN = new RGBWColor(0, 255, 0, 0);

  // Off
  private static final RGBWColor OFF   = new RGBWColor(0, 0, 0, 0);
  // No-DS-connection idle color
  private static final RGBWColor IDLE  = new RGBWColor(20, 20, 20, 0);

  private static final double[] SHIFT_ENDS = { 130.0, 105.0, 80.0, 55.0, 30.0 };

  // Hardware
  private final CANdle     m_candle;
  private final SolidColor m_solidControl = new SolidColor(0, LED_COUNT);
  private final Timer      m_timer        = new Timer();

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
    Optional<Alliance> allianceOpt = DriverStation.getAlliance();

    if (allianceOpt.isEmpty()) {
      // DS not connected. dim pulse to confirm LEDs are alive
      pulse(IDLE, 2.0);
      return;
    }

    Alliance alliance = allianceOpt.get();
    boolean  active   = isHubActive(alliance);
    boolean  warning  = isShiftChangeSoon();

    if      ( active && !warning) setColor(GREEN);
    else if ( active &&  warning) blink(GREEN);
    else if (!active && !warning) setColor(alliance == Alliance.Red ? RED : BLUE);
    else                          blink(alliance == Alliance.Red ? RED : BLUE);
  }

  // Hub activity logic

  public boolean isHubActive(Alliance alliance) {
    if (DriverStation.isAutonomousEnabled()) return true;
    if (!DriverStation.isTeleopEnabled())    return false;

    double matchTime = DriverStation.getMatchTime();
    if (matchTime < 0) return true; // practice mode — no real match time

    if (matchTime > 130 || matchTime <= 30) return true; // Transition or End Game

    String gameData = DriverStation.getGameSpecificMessage();
    if (gameData == null || gameData.isEmpty()) return true;

    boolean redInactiveShift1;
    switch (gameData.charAt(0)) {
      case 'R' -> redInactiveShift1 = true;
      case 'B' -> redInactiveShift1 = false;
      default  -> { return true; }
    }

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
    if (matchTime > 130 || matchTime <= 30) return 0;
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
}