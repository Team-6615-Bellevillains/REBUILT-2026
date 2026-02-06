package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShooterSubsystem extends SubsystemBase{
    
    // put variables here. 
    // TODO: put actual motor IDs
    private SparkMax leftMotor = new SparkMax(0, MotorType.kBrushless); 
    private SparkMax rightMotor = new SparkMax(0, MotorType.kBrushless); 


    // constructor. initialize shooter here.
    public ShooterSubsystem(){

    }

    // put code that needs to run continuously here. 
    @Override
    public void periodic() {
        
    }
}
