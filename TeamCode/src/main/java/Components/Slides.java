package Components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Slides {

    public static final int minSlideTicks = 0;
    public static final int maxSlideTicks = -3800;
    public static final int topBarHeight = -1400;

    private float stayAtPosition = 0;
    private boolean slidesStopped = false;

    public DcMotor slideMotor = null;

    public Slides(DcMotor slideMotor) {
        this.slideMotor = slideMotor;
    }

    public void moveSlides(double power) {
        if (power == 0) {
            if (!slidesStopped) {
                slidesStopped = true;
                stayAtPosition = slideMotor.getCurrentPosition();
            }
            if (slideMotor.getCurrentPosition() > stayAtPosition) {
                power = -0.2;
            }
        }
        else {
            slidesStopped = false;
        }

        slideMotor.setPower(power);
        if (!((slideMotor.getCurrentPosition() < maxSlideTicks && power < 0) || (slideMotor.getCurrentPosition() > minSlideTicks && power > 0))) {
            slideMotor.setPower(power);
        }
    }

    public boolean isAtTop() {
        return slideMotor.getCurrentPosition() < maxSlideTicks;
    }

    public boolean isAtBottom() {
        return slideMotor.getCurrentPosition() > minSlideTicks;
    }

    public boolean isAboveHighBar() {
        return slideMotor.getCurrentPosition() < topBarHeight;
    }

    public void moveTowardHighBar() {
        if (!isAboveHighBar()) {
            moveSlides(-1);
        }
    }

    public boolean isAboveHighBasket() {
        return slideMotor.getCurrentPosition() < maxSlideTicks;
    }

    public void moveTowardHighBasket() {
        if (!isAboveHighBasket()) {
            moveSlides(-1);
        }
    }
}
