package Components;

public class ButtonHandler {
    private boolean previouslyPressed = false;
    public boolean justPressed = false;
    public boolean justReleased = false;
    public boolean pressed = false;

    public ButtonHandler() {

    }

    public void update(boolean buttonPressed) {
        pressed = buttonPressed;

        // Not just pressed anymore if we already logged it as previously pressed.
        if (justPressed && previouslyPressed) {
            justPressed = false;
        }
        if (justReleased && !previouslyPressed) {
            justReleased = false;
        }

        // If it wasn't previously pressed that means we just pressed it.
        if (pressed && !previouslyPressed) {
            previouslyPressed = true;
            justPressed = true;
        }
        if (!pressed && previouslyPressed) {
            previouslyPressed = false;
            justReleased = true;
        }
    }
}
