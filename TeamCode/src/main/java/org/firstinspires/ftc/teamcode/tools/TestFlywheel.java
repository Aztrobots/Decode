package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Test Flywheel")
public class TestFlywheel extends OpMode {

    private DcMotorEx flyWheel;
    private DcMotorEx fwl;

    @Override
    public void init() {
        flyWheel = hardwareMap.get(DcMotorEx.class, "flyWheel");
        fwl      = hardwareMap.get(DcMotorEx.class, "fwl");

        flyWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fwl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        fwl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void loop() {
        flyWheel.setPower(gamepad1.right_bumper ? 0.3 : 0);
        fwl.setPower(gamepad1.left_bumper  ? 0.3 : 0);

        telemetry.update();
    }
}