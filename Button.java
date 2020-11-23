package org.firstinspires.ftc.teamcode;

public class Button {

    private boolean lastState=false;
    private boolean state=false;
    private int toggleMode=0;
    private int num=2;

    public Button() {
        num=2;
    }

    public Button(int number) {
        num=number;
    }

    public void update(boolean newState) {
        lastState=state;
        state=newState;
        if(lastState==false && state==true) {
            toggleMode=(toggleMode+1)%num;
        }
    }

    //This method is deprecated you fool
    public boolean getToggleMode() {
        return toggleMode!=0;
    }

    public boolean isToggleOn() { return toggleMode!=0; }

    public int getToggleNumber() { return toggleMode; }

    public boolean getState() {
        return state;
    }

    public boolean wasPressed() {
        return !lastState && state;
    }

    public boolean wasReleased() {
        return lastState && !state;
    }
}
