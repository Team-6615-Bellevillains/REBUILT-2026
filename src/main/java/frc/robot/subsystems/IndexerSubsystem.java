package frc.robot.subsystems;


import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {
    
    private final SparkMax spindexerMotor = new SparkMax(50, MotorType.kBrushless);
    private final SparkMax roadMotor = new SparkMax(52, MotorType.kBrushless);
    private State state = State.OFF;


    public IndexerSubsystem(){
        SparkMaxConfig spinConfig = new SparkMaxConfig();
        SparkMaxConfig roadConfig = new SparkMaxConfig();
        
        spinConfig.closedLoop
        .pid(0, 0, 0)
        .feedForward
        .kS(0.14)
        .kA(0)
        .kV(0.00202);

         
        roadConfig.closedLoop
        .pid(0, 0, 0)
        .feedForward
        .kS(0.25)
        .kA(0)
        .kV(0.00213);

        spinConfig.inverted(true);
        spindexerMotor.configure(spinConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        roadConfig.inverted(false);
        roadMotor.configure(roadConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        SmartDashboard.putString("indexer state", state.toString());
        switch (state) {
            case OFF:
                off();
                break;
        
            case INDEX:
                index();
                break;
            
            case SHOOT:
                shoot();
                break;
                
            case SLOW:
                slow();
                break;
            
            case REVERSE:
                reverse();
                break;
        }
        SmartDashboard.putNumber("spindexer rpm", spindexerMotor.getEncoder().getVelocity());
    }

    private void shoot(){
      spindexerMotor.getClosedLoopController()
      .setSetpoint(0.5,ControlType.kVelocity);
      roadMotor.getClosedLoopController()
      .setSetpoint(0.45, ControlType.kVelocity);
    }

    private void index(){
        spindexerMotor.getClosedLoopController()
        .setSetpoint(0.5, ControlType.kVelocity);
        roadMotor.stopMotor();
    }

    private void off(){
        spindexerMotor.stopMotor();
        roadMotor.stopMotor();
    }

    private void slow(){
        spindexerMotor.getClosedLoopController()
        .setSetpoint(.1, ControlType.kVelocity);
        roadMotor.stopMotor();
    }

    public void setState(State state){
        this.state = state;
    }

    private void reverse(){
        spindexerMotor.getClosedLoopController()
        .setSetpoint(-0.2, ControlType.kVelocity);
        roadMotor.stopMotor();
    }

    public enum State {
        OFF,
        INDEX,
        SHOOT,
        SLOW,
        REVERSE
    }

    public Command indexerRunCommand(){
        return this.runEnd(()->{
            this.setState(State.SLOW);
        }, ()->{
            this.setState(State.OFF);
        });
    }

    public Command indexerReverseCommand(){
        return this.runEnd(()->{
            this.setState(State.REVERSE);
        }, ()->{
            this.setState(State.OFF);
        });
    }
}
