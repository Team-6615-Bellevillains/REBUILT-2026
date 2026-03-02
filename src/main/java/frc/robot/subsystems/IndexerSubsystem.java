package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {
    
    private final SparkMax spindexerMotor = new SparkMax(50, MotorType.kBrushless);
    private final SparkMax roadMotor = new SparkMax(52, MotorType.kBrushless);
    private State state = State.OFF;


    public IndexerSubsystem(){
        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true);
        spindexerMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        config.inverted(false);
        roadMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
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
        spindexerMotor.set(0.3);
        roadMotor.set(0.3);
    }

    private void index(){
        spindexerMotor.set(0.3);
        roadMotor.stopMotor();
    }

    private void off(){
        spindexerMotor.stopMotor();
        roadMotor.stopMotor();
    }

    private void slow(){
        spindexerMotor.set(0.1);
        roadMotor.set(0);
    }

    public void setState(State state){
        this.state = state;
    }

    private void reverse(){
        spindexerMotor.set(-0.2);
        roadMotor.set(0);
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
