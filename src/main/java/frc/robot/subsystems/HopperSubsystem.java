package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HopperSubsystem extends SubsystemBase{

    // put variables here.
    // TODO: put actual motor ID and proper motors.
    private SparkMax spindexerMotor = new SparkMax(0, MotorType.kBrushless);

    // this is the constructor. put hopper initialization code here. 
    public HopperSubsystem(){

    }

    // the periodic method. runs continuously.
    @Override
    public void periodic() {
        
    }
}
