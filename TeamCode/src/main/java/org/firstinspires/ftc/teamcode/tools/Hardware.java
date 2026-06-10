package org.firstinspires.ftc.teamcode.tools;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class Hardware {

    private final OpMode opMode;


    public DcMotorEx flyWheel;
    public DcMotorEx fwl;
    public DcMotorEx intake;
    public DcMotorEx turret;
    public Servo     leftGate;

    public GoBildaPinpointDriver pinpoint;

    public static final double GATE_BLOCK = 0.5;
    public static final double GATE_OPEN  = 0.72;


    private final boolean ownsPinpoint;
    private final boolean resetPinpoint;


    public Hardware(OpMode opMode) {
        this(opMode, true, true);
    }

    /**
     * Constructor general.
     * @param ownsPinpoint  true  = Hardware inicializa el Pinpoint.
     *                      false = Pedro ya lo inicializó; Hardware solo obtiene la referencia.
     * @param resetPinpoint true  = llamar resetPosAndIMU() (solo si ownsPinpoint = true).
     */
    public Hardware(OpMode opMode, boolean ownsPinpoint, boolean resetPinpoint) {
        this.opMode        = opMode;
        this.ownsPinpoint  = ownsPinpoint;
        this.resetPinpoint = resetPinpoint;
    }

    public Hardware(OpMode opMode, boolean resetPinpoint) {
        this(opMode, resetPinpoint, resetPinpoint);
    }

    public OpMode opMode() { return opMode; }

    public void init() {
        // Motors
        flyWheel = opMode.hardwareMap.get(DcMotorEx.class, "flyWheel");
        flyWheel.setDirection(DcMotor.Direction.FORWARD);
        flyWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flyWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flyWheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        fwl = opMode.hardwareMap.get(DcMotorEx.class, "fwl");
        fwl.setDirection(DcMotor.Direction.REVERSE);
        fwl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fwl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fwl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intake = opMode.hardwareMap.get(DcMotorEx.class, "intake");
        intake.setDirection(DcMotor.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turret = opMode.hardwareMap.get(DcMotorEx.class, "turret");
        turret.setDirection(DcMotor.Direction.FORWARD);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftGate = opMode.hardwareMap.get(Servo.class, "servo");
        leftGate.setPosition(GATE_BLOCK);


        pinpoint = opMode.hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        if (ownsPinpoint) {

            pinpoint.setEncoderResolution(
                    GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
            pinpoint.setEncoderDirections(
                    GoBildaPinpointDriver.EncoderDirection.FORWARD,
                    GoBildaPinpointDriver.EncoderDirection.REVERSED);

            if (resetPinpoint) {
                pinpoint.resetPosAndIMU();
                try { Thread.sleep(500); }
                catch (InterruptedException e) { Thread.currentThread().interrupt(); }

                GoBildaPinpointDriver.DeviceStatus status = pinpoint.getDeviceStatus();
                if (status != GoBildaPinpointDriver.DeviceStatus.READY) {
                    opMode.telemetry.addData("PINPOINT ERROR", status.toString());
                    opMode.telemetry.update();
                }
            }
            // resetPinpoint=false: configura los parámetros sin resetear pose ni IMU.
            // Usado en TeleOp para heredar la pose acumulada del autónomo.
        }

    }
}