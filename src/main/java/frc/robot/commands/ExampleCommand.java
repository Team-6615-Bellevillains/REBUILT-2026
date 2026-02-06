package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

// example command to show how to use a custom command class. 
public class ExampleCommand extends Command{

    // Command constructor. DOES NOT RUN EVERY TIME THE COMMAND IS RUN. only runs when the command object is created.
    public ExampleCommand(){

    }

    // runs whenever a command starts executing. 
    @Override
    public void initialize() {
        
    }

    // called at the same frequency as the periodic() method in subsystems while the command is executing.
    @Override
    public void execute() {
        
    }

    // when this function returns true, the command finishes and end() is called.
    @Override
    public boolean isFinished() {
        return true;
    }

    // runs when when the command is finished. if the command was interrupted by another, interrupted is true.
    @Override
    public void end(boolean interrupted) {
        
    }
}
