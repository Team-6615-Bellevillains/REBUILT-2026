package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
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
  private static final int CANDLE_ID = 5;
  private static final int LED_COUNT = 100;

  private static final double WARN_THRESHOLD = 10.0;
  private static final double SHOOT_ALERT_WINDOW = 2;
  private static final double TRANSITION_ALERT_WINDOW = 3.0;

  private static final double FLASH_PERIOD = 0.12;
  private static final double RAINBOW_PERIOD = 2.0;

  private static final double[] SHIFT_ENDS = {130.0, 105.0, 80.0, 55.0, 30.0};

  private static final double TRANSITION_START = 140.0;
  private static final double TRANSITION_END = 130.0;

  private static final RGBWColor RED = new RGBWColor(255, 0, 0, 0);
  private static final RGBWColor GREEN = new RGBWColor(0, 255, 0, 0);
  private static final RGBWColor OFF = new RGBWColor(0, 0, 0, 0);
  private static final RGBWColor IDLE = new RGBWColor(150, 10, 0, 0);
  private static final RGBWColor NO_COMMS = new RGBWColor(20, 20, 20, 0);

  // State Enum
  private enum LedState {
    DISCONNECTED,
    DISABLED,
    AUTO,
    TRANSITION,
    TELEOP_SAFE_ACTIVE,
    TELEOP_WARNING_ACTIVE,
    TELEOP_SAFE_INACTIVE,
    TELEOP_WARNING_INACTIVE
  }

  // Hardware
  private final CANdle m_candle;
  private final SolidColor m_solidControl = new SolidColor(0, LED_COUNT);
  private final Timer m_timer = new Timer();

  private Boolean m_wonAuto = null;

  // Constructor
  public LedSubsystem() {
    m_candle = new CANdle(CANDLE_ID, CANBus.roboRIO());

    CANdleConfiguration config = new CANdleConfiguration();
    config.LED.StripType = StripTypeValue.GRB;
    config.LED.BrightnessScalar = 1.0;
    config.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;

    m_candle.getConfigurator().apply(config);

    m_timer.start();

    // Boot flash
    RGBWColor white = new RGBWColor(150, 150, 150, 0);
    for (int i = 0; i < 3; i++) {
      m_candle.setControl(m_solidControl.withColor(white));
      Timer.delay(0.08);
      m_candle.setControl(m_solidControl.withColor(OFF));
      Timer.delay(0.08);
    }
  }

  // Periodic
  @Override
  public void periodic() {
    LedState state = getState();
    render(state);
  }

  // State Logic
  private LedState getState() {

    if (!DriverStation.isDSAttached()) return LedState.DISCONNECTED;

    if (DriverStation.isDisabled()) {
      m_wonAuto = null;
      return LedState.DISABLED;
    }

    if (m_wonAuto == null && DriverStation.isTeleopEnabled()) {
      m_wonAuto = wonAuto();
    }

    if (isTransitionShift()) {
      return LedState.TRANSITION;
    }

    if (DriverStation.isAutonomousEnabled()) {
      return LedState.AUTO;
    }

    if (!DriverStation.isTeleopEnabled()) {
      return LedState.DISABLED;
    }

    boolean active = isHubActive();
    boolean warning = isShiftChangeSoon();

    if (active && warning) return LedState.TELEOP_WARNING_ACTIVE;
    if (active) return LedState.TELEOP_SAFE_ACTIVE;
    if (warning) return LedState.TELEOP_WARNING_INACTIVE;
    return LedState.TELEOP_SAFE_INACTIVE;
  }

  // Rendering
  private void render(LedState state) {
    switch (state) {
      case DISCONNECTED -> pulse(NO_COMMS, 2.0);
      case DISABLED -> setColor(IDLE);
      case AUTO -> setColor(GREEN);

      case TRANSITION -> {
        double timeToEnd = getTimeToTransitionEnd();

        if (Boolean.TRUE.equals(m_wonAuto)) {
          if (timeToEnd <= TRANSITION_ALERT_WINDOW) {
            flash(GREEN);
          } else {
            rainbow();
          }
        } else {
          if (timeToEnd <= TRANSITION_ALERT_WINDOW) {
            flash(GREEN);
          } else {
            setColor(GREEN);
          }
        }
      }

      case TELEOP_SAFE_ACTIVE -> setColor(GREEN);
      case TELEOP_SAFE_INACTIVE -> setColor(RED);

      case TELEOP_WARNING_ACTIVE -> {
        if (getTimeToNextShift() <= SHOOT_ALERT_WINDOW) {
          flash(new RGBWColor(255, 255, 255, 0));
        } else {
          gradient(GREEN, getWarningProgress());
        }
      }

      case TELEOP_WARNING_INACTIVE -> {
        if (getTimeToNextShift() <= SHOOT_ALERT_WINDOW) {
          flash(new RGBWColor(255, 255, 255, 0));
        } else {
          gradient(RED, getWarningProgress());
        }
      }
    }
  }

  // Timing Helpers
  private double getTimeToNextShift() {
    double matchTime = DriverStation.getMatchTime();
    if (matchTime < 0) return Double.MAX_VALUE;

    for (double boundary : SHIFT_ENDS) {
      if (matchTime > boundary) {
        return matchTime - boundary;
      }
    }
    return Double.MAX_VALUE;
  }

  private double getTimeToTransitionEnd() {
    double matchTime = DriverStation.getMatchTime();
    if (matchTime < 0) return Double.MAX_VALUE;
    return matchTime - TRANSITION_END;
  }

  private double getWarningProgress() {
    double matchTime = DriverStation.getMatchTime();
    if (matchTime < 0) return 0.0;

    for (double boundary : SHIFT_ENDS) {
      if (matchTime > boundary && matchTime <= boundary + WARN_THRESHOLD) {
        return 1.0 - ((matchTime - boundary) / WARN_THRESHOLD);
      }
    }
    return 0.0;
  }

  // Visual Effects
  private void gradient(RGBWColor base, double progress) {
    progress = Math.max(0.0, Math.min(1.0, progress));
    double smooth = progress * progress * (3 - 2 * progress);

    m_candle.setControl(m_solidControl.withColor(new RGBWColor(
        (int)(base.Red * smooth),
        (int)(base.Green * smooth),
        (int)(base.Blue * smooth),
        0
    )));
  }

  private void flash(RGBWColor color) {
    double phase = (m_timer.get() % FLASH_PERIOD) / FLASH_PERIOD;
    boolean on = phase < 0.5;
    m_candle.setControl(m_solidControl.withColor(on ? color : OFF));
  }

  private void setColor(RGBWColor color) {
    m_candle.setControl(m_solidControl.withColor(color));
  }

  private void pulse(RGBWColor base, double cycleSec) {
    double brightness = (1.0 - Math.cos((m_timer.get() % cycleSec) / cycleSec * 2.0 * Math.PI)) / 2.0;

    m_candle.setControl(m_solidControl.withColor(new RGBWColor(
        (int)(base.Red * brightness),
        (int)(base.Green * brightness),
        (int)(base.Blue * brightness),
        0
    )));
  }

  private void rainbow() {
    double hue = (m_timer.get() % RAINBOW_PERIOD) / RAINBOW_PERIOD * 360.0;
    int[] rgb = hsvToRgb(hue, 1.0, 1.0);

    m_candle.setControl(m_solidControl.withColor(new RGBWColor(rgb[0], rgb[1], rgb[2], 0)));
  }

  private int[] hsvToRgb(double hue, double sat, double val) {
    double c = val * sat;
    double x = c * (1.0 - Math.abs((hue / 60.0) % 2.0 - 1.0));
    double m = val - c;

    double r, g, b;
    if (hue < 60) { r = c; g = x; b = 0; }
    else if (hue < 120) { r = x; g = c; b = 0; }
    else if (hue < 180) { r = 0; g = c; b = x; }
    else if (hue < 240) { r = 0; g = x; b = c; }
    else if (hue < 300) { r = x; g = 0; b = c; }
    else { r = c; g = 0; b = x; }

    return new int[]{
        (int)((r + m) * 255),
        (int)((g + m) * 255),
        (int)((b + m) * 255)
    };
  }

  // Game Logic
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
      default -> false;
    };
  }

  public boolean isHubActive() {
    Optional<Alliance> allianceOpt = DriverStation.getAlliance();
    if (allianceOpt.isEmpty()) return true;

    Alliance alliance = allianceOpt.get();

    if (DriverStation.isAutonomousEnabled()) return true;
    if (!DriverStation.isTeleopEnabled()) return false;

    double matchTime = DriverStation.getMatchTime();
    if (matchTime < 0) return true;

    if (matchTime > TRANSITION_END || matchTime <= 30) return true;

    String gameData = DriverStation.getGameSpecificMessage();
    if (gameData == null || gameData.isEmpty()) return true;

    boolean redInactiveShift1 = switch (gameData.charAt(0)) {
      case 'R' -> true;
      case 'B' -> false;
      default -> true;
    };

    boolean shift1InactiveForUs =
        (alliance == Alliance.Red) ? redInactiveShift1 : !redInactiveShift1;

    boolean inactiveThisShift =
        (currentShift(matchTime) % 2 == 1) ? shift1InactiveForUs : !shift1InactiveForUs;

    return !inactiveThisShift;
  }

  public boolean isShiftChangeSoon() {
    if (!DriverStation.isTeleopEnabled()) return false;

    double matchTime = DriverStation.getMatchTime();
    if (matchTime < 0) return false;

    for (double boundary : SHIFT_ENDS) {
      if (matchTime > boundary && matchTime <= boundary + WARN_THRESHOLD) {
        return true;
      }
    }

    return false;
  }

  private int currentShift(double matchTime) {
    if (matchTime > TRANSITION_END || matchTime <= 30) return 0;
    if (matchTime > 105) return 1;
    if (matchTime > 80) return 2;
    if (matchTime > 55) return 3;
    return 4;
  }
}