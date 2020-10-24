package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.HardwareMap;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;
        //import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Prototype Regular TeleOp", group = "Teleop")
public class VenomTeleopRegular extends OpMode {

    HardwareMap hardwareMap = new HardwareMap();
    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightBackMotor;

    //private Servo arm;


    @Override
    public void init(){
        leftFrontMotor = hardwareMap.dcMotor.get("leftFrontMotor");
        leftBackMotor = hardwareMap.dcMotor.get("leftBackMotor");
        rightFrontMotor = hardwareMap.dcMotor.get("rightFrontMotor");
        rightBackMotor = hardwareMap.dcMotor.get("rightBackMotor");

        leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
    }


    @Override
    public void loop(){
        doDrive();
        doTurn();
    }

    public void doDrive(){
        double power = gamepad1.left_stick_y;


        leftFrontMotor.setPower(power);
        leftBackMotor.setPower(power);
        rightFrontMotor.setPower(power);
        rightBackMotor.setPower(power);
    }

    public void doTurn(){
        double turnPower = gamepad1.right_stick_x;

        telemetry.addData("Turn Power", turnPower);

        if(turnPower > 0.5) {
            leftFrontMotor.setPower(turnPower);
            leftBackMotor.setPower(turnPower);
            rightFrontMotor.setPower(-turnPower);
            rightBackMotor.setPower(-turnPower);
        } else {
            leftFrontMotor.setPower(-turnPower);
            leftBackMotor.setPower(-turnPower);
            rightFrontMotor.setPower(turnPower);
            rightBackMotor.setPower(turnPower);
        }

    }

}
