package Components;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

// Updated to use servo pulling w/ rope
public class Arm {
    public Servo armServo;
    public Servo clawServo;

    public enum ARM_STATE {
        UP,
        DOWN,
        SLIGHTLY_OVER,
    }
    public ARM_STATE armState = ARM_STATE.UP;

    public enum CLAW_STATE {
        OPEN,
        CLOSED,
    }
    public CLAW_STATE clawState = CLAW_STATE.CLOSED;

    public Arm(Servo armServo, Servo clawServo) {
        this.armServo = armServo;
        this.clawServo = clawServo;
    }

    public void debugPosition(Telemetry telemetry) {
        telemetry.addData("ARM: Arm position", armServo.getPosition());
        telemetry.addData("ARM: Claw position", clawServo.getPosition());
    }

    public void moveArmUp() {
        armState = ARM_STATE.UP;
        armServo.setPosition(0.3);
    }

    public void moveArmDown() {
        armState = ARM_STATE.DOWN;
        armServo.setPosition(0.85);
    }

    public void moveArmSlightlyOver() {
        armState = ARM_STATE.SLIGHTLY_OVER;
        armServo.setPosition(0.47);
    }

    public void openClaw() {
        clawState = CLAW_STATE.OPEN;
        clawServo.setPosition(1);
    }

    public void closeClaw() {
        clawState = CLAW_STATE.CLOSED;
        clawServo.setPosition(0.58);
    }

    public void setArmPosition(double angle) {
        armServo.setPosition(angle);
    }
}
