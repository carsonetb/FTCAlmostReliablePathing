package Components;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import Util.Vector2;

@Config
public class Hang {

    public DcMotor leftMotor;
    public DcMotor rightMotor;

    public Hang(DcMotor leftMotor, DcMotor rightMotor) {
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;

        leftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void goUp(double power) {
        setBothMotors(-power);
    }

    public void goDown(double power) {
        setBothMotors(power);
    }

    public void stop() {
        setBothMotors(0);
    }

    private void setBothMotors(double power) {
        this.leftMotor.setPower(power);
        this.rightMotor.setPower(power);
    }
}
