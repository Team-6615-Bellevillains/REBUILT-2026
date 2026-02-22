package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {
    
    private SparkMax spindexerMotor = new SparkMax(50, MotorType.kBrushless);
    private SparkMax roadMotor = new SparkMax(52, MotorType.kBrushless);
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
        }
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

    public void setState(State state){
        this.state = state;
    }

    public enum State {
        OFF,
        INDEX,
        SHOOT
    }

    public Command indexerRunCommand(){
        return this.runEnd(()->{
            this.setState(State.SHOOT);
        }, ()->{
            this.setState(State.OFF);
        });
    }
}
