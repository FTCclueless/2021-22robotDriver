package org.firstinspires.ftc.teamcode.drive;

public class ButtonToggle {
    public boolean toggleState = false;
    public boolean lastButtonPressed = false;
    public void update(boolean buttonPressed) { // button is pressed = true

        if(buttonPressed && !lastButtonPressed){
            toggleState = !toggleState;
        }
        lastButtonPressed = buttonPressed;
    }
    public boolean getToggleState() {
        return toggleState;
    }
}