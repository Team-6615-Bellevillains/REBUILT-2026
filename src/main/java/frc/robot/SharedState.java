package frc.robot;

// use this class to communicate between parts of the robot program. 
public class SharedState {
    //put shared variables here
    private boolean exampleBoolean;

    private static SharedState instance;

    private SharedState(){}

    // call this from anywhere with "SharedState.get()" to get the instance of the shared state.
    public static SharedState get(){
        if (instance == null){
            instance = new SharedState();
        }

        return instance;
    }

    // to set this value you would call "SharedState.get().setExampleBool(value)"
    public void setExampleBool(boolean exampleBool){
        this.exampleBoolean = exampleBool;
    }

    // to retrieve this value you would call "SharedState.get().getExampleBool()"
    public boolean getExampleBool(){
        return this.exampleBoolean;
    }
    
}
