package Util;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Encoder {
    public enum Direction {
        FORWARD,
        REVERSE,
    }

    private DcMotorEx motor;
    private Direction direction;

    public Encoder(DcMotorEx motor) {
        this.motor = motor;
        this.direction = Direction.FORWARD;
    }

    public Direction getDirection() {
        return direction;
    }

    public void setDirection(Direction direction) {
        this.direction = direction;
    }

    public int getCurrentPosition() {
        return motor.getCurrentPosition() * (direction == Direction.FORWARD ? 1 : -1) *
                (motor.getDirection() == DcMotorSimple.Direction.FORWARD ? 1 : -1);
    }
}
