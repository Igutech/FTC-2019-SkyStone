package org.igutech.utils;

public class ButtonToggle {

    boolean press;

    public ButtonToggle(boolean isPressed) {
        press = isPressed;
    }

    public boolean isPressed() { return press; }

    public boolean isActive(boolean check) {
        return (check && press);
    }

    public void update(boolean isPressed) {
        press = isPressed;
    }

    public boolean equals(boolean compar) {
        return (press == compar);
    }

    public void toggle() {
        press = !press;
    }

}