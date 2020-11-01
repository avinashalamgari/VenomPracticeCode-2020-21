package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gyroscope;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Axis;
import java.lang.System;
import java.util.Set;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
//import com.qualcomm.robotcore.hardware.Servo;

public class VenomAuto extends LinearOpMode {
    boolean isBool = false;


    public void runOpMode(){
        telemetry.addData(">", "Press Play to Start");
        telemetry.update();
        waitForStart();

    }

    public void pidTurning(){

    }

    public void getWobbleGoal(){
//        while(){
//
//        }
    }
}
