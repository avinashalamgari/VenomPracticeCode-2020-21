/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class HardwareTest
{
    /* Public OpMode members. */
    public DcMotor leftBackMotor;
    public DcMotor rightBackMotor;
    public DcMotor leftFrontMotor;
    public DcMotor rightFrontMotor;
    public DcMotor shooter;
    public DcMotor intake;
    public DcMotor transitionBeltLeft;
    public DcMotor transitionBeltRight;

    public CRServo vertical;
    public Servo horizontal;
    public final static double HORIZONTAL_HOME = 0.0;
    public final static double HORIZONTAL_MIN_RANGE = 0.0;
    public final static double HORIZONTAL_MAX_RANGE = 0.5;




    public GyroSensor gyro;

    public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareTest(){

    }
    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        //gyro = hardwareMap.gyroSensor.get("gyro");
        gyro  = hwMap.gyroSensor.get("gyro");
        gyro = hwMap.get(GyroSensor.class, "gyro");
        // Define and Initialize Motors
        leftBackMotor  = hwMap.get(DcMotor.class, "leftBackDrive");
        leftFrontMotor = hwMap.get(DcMotor.class, "leftFrontDrive");
        rightBackMotor = hwMap.get(DcMotor.class, "rightBackDrive");
        rightFrontMotor = hwMap.get(DcMotor.class, "rightFrontDrive");
        shooter = hwMap.get(DcMotor.class, "shooter");
        intake = hwMap.get(DcMotor.class, "intake");
        transitionBeltLeft = hwMap.get(DcMotor.class, "transitionBeltLeft");
        transitionBeltRight = hwMap.get(DcMotor.class, "transitionBeltRight");

        vertical = hwMap.get(CRServo.class,  "verticalservo");

        horizontal = hwMap.get(Servo.class, "servo2");
        horizontal.setPosition(HORIZONTAL_HOME);


        //leftArm    = hwMap.get(DcMotor.class, "left_arm");
        leftBackMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        leftFrontMotor.setDirection(DcMotor.Direction.FORWARD);

        rightBackMotor.setDirection((DcMotor.Direction.REVERSE));
        rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        leftBackMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftFrontMotor.setPower(0);
        rightBackMotor.setPower(0);
        //leftArm.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // leftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

//        // Define and initialize ALL installed servos.
        Servo leftClaw  = hwMap.get(Servo.class, "left_hand");
        Servo rightClaw = hwMap.get(Servo.class, "right_hand");
//        leftClaw.setPosition(MID_SERVO);
//        rightClaw.setPosition(MID_SERVO);
    }
}

