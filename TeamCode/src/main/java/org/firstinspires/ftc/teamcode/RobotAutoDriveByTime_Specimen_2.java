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

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/*
 * This OpMode illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Double Specimen Hang", group="Robot")
public class RobotAutoDriveByTime_Specimen_2 extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    //telescoping arm
    private DcMotor armDrive = null;

    /*
    servo claw

    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position
    */

    // Define class members
    Servo clawServo;
    Servo hingeServo;

    double  clawPosition = 0.50;  //guessing middle is 0.50
    double  hingePosition = 0.0; // Standard servo TODO: Find right values through testing (value should be up)

    private IMU imu         = null;      // Control/Expansion Hub IMU

    // These variable are declared here (as class members) so they can be updated in various methods,
    // but still be displayed by sendTelemetry()
    private double  targetHeading = 0;
    private double  turnSpeed     = 0;

    private double          headingError  = 0;

    // These constants define the desired driving/control characteristics
    // They can/should be tweaked to suit the specific robot drive train.
    static final double     TURN_SPEED              = 0.8;     // Max turn speed to limit turn rate.
    static final double     HEADING_THRESHOLD       = 0.5 ;    // How close must the heading get to the target before moving to next step.
    // Requiring more accuracy (a smaller number) will often make the turn take longer to get into the final position.

    // Define the Proportional control coefficient (or GAIN) for "heading control".
    // We define one value when Turning (larger errors), and the other is used when Driving straight (smaller errors).
    // Increase these numbers if the heading does not correct strongly enough (eg: a heavy robot or using tracks)
    // Decrease these numbers if the heading does not settle on the correct value (eg: very agile robot with omni wheels)
    static final double     P_TURN_GAIN            = 0.5;     // Larger is more responsive, but also less stable.
    static final double     P_DRIVE_GAIN           = 0.03;     // Larger is more responsive, but also less stable.

    private final ElapsedTime     runtime = new ElapsedTime();

    // static final double     FORWARD_SPEED = 0.6;
    // static final double     TURN_SPEED    = 0.5;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        //add config for new motor (telescoping arm)
        armDrive = hardwareMap.get(DcMotor.class, "arm_drive");
        armDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armDrive.setTargetPosition(0);
        armDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armDrive.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        // Set direction for the telescoping arm
        armDrive.setDirection(DcMotor.Direction.REVERSE);

        /* The next two lines define Hub orientation.
         * The Default Orientation (shown) is when a hub is mounted horizontally with the printed logo pointing UP and the USB port pointing FORWARD.
         *
         * TODO:  EDIT these two lines to match YOUR mounting configuration.
         */
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        // Connect to servo
        clawServo = hardwareMap.get(Servo.class, "claw_hand");

        // Connect to servo
        hingeServo = hardwareMap.get(Servo.class, "arm_hinge");

        // Wait for the game to start (Display Gyro value while waiting)
        while (opModeInInit()) {
            telemetry.addData(">", "Robot Heading = %4.0f", getHeading());
            telemetry.update();
        }

        imu.resetYaw();

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Arm location starting at",  "%7d",
            armDrive.getCurrentPosition());
        telemetry.update();


        // Wait for the game to start (driver presses START)
        waitForStart();

        //int targetTicks = 0;

        // Step through each leg of the path, ensuring that the OpMode has not been stopped along the way.
        //          holdHeading() is used after turns to let the heading stabilize
        //          Add a sleep(2000) after any step to keep the telemetry data visible for review

        //calculate arm power based on joystick from gamepad2
        double armPower = 0.65;

        // Declare new target value for specimen hanging height
        int hangingSpecimenTicks = 1400; //TODO: test value

        // Declare new target value for arm bottom height
        int armDownTicks = 0;

        // Declare new target value for specimen lift-off height
        int specimenLiftTicks = 50; //TODO: test value

        // Step 1: Close claw
        clawPosition = 0.50;
        clawServo.setPosition(clawPosition);

        // Step 2: Lift arm
        encoderArm(armPower, hangingSpecimenTicks, 3);

        // Step 3: Drive forward for x amount of time and stop
        driveByTime(0.5, 0, 0, 2);

        // Step 4: Pull arm down
        encoderArm(armPower, armDownTicks, 1);

        // Step: Open claw
        clawPosition = 0.90;
        clawServo.setPosition(clawPosition);

        // Step: Strafe back halfway
        driveByTime(-0.5,0,0,0.7);

        // Step: Rotate 90 degrees clockwise
        turnToHeading( TURN_SPEED, -90.0);

        // Step: Lower arm hinge to specimen pick - up height
        hingePosition = 0.17;                       // TODO: Change values through testing
        hingeServo.setPosition(hingePosition);

        // Step: Drive forward TODO: drive with april tags
        driveByTime(0.6,0 , 0, 1.8);

        // Step: Rotate an additional 90 degrees clockwise
        turnToHeading( TURN_SPEED, -180.0);

        // Step: Drive forward into observation zone
        driveByTime(0.4,0,0,0.5); // TODO: Change values through testing

        // Step: Close claw
        clawPosition = 0.50;
        clawServo.setPosition(clawPosition);

        // Step: Lift arm slightly to disengage from wall
        encoderArm(armPower, specimenLiftTicks, 1);

        // Step: Raise arm hinge
        hingePosition = 1.0;                       // TODO: Change values through testing
        hingeServo.setPosition(hingePosition);

        // Step: Back up
        driveByTime(-0.4,0,0,0.5); // TODO: Change values through testing

        // Step: Rotate 90 degrees CW
        turnToHeading( TURN_SPEED, 90);

        // Step: drive forwards back to submersible
        driveByTime(0.6,0 , 0, 1.55);

        // Step: Raise arm
        encoderArm(armPower, hangingSpecimenTicks, 3);

        // Step: Rotate 90 degrees CW
        turnToHeading( TURN_SPEED, 0);

        // Step: Drive forwards
        driveByTime(0.5,0,0,0.7);

        // Step: Pull arm down
        encoderArm(armPower, armDownTicks, 1);

        // Step: Strafe back and to the right to observation zone
        driveByTime(-0.2, 0.6, 0, 2);

        // Step: Park and wait for TeleOp

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000); //TODO: Change value w/ testing
    }

/*
 *  Method to perform a move to an exact position, based on encoder counts.
 *  Move will stop if any of three conditions occur:
 *  1) Move gets to the desired position
 *  2) Move runs out of time
 *  3) Driver stops the OpMode running.
 */
    public void encoderArm(double speed,
                             int targetTicks,
                             double timeoutS) {
        // if you're already at the target do nothing
        if (armDrive.getCurrentPosition() == targetTicks) {
            return;
        }
        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            armDrive.setTargetPosition(targetTicks);

            // reset the timeout time and start motion.
            runtime.reset();
            armDrive.setPower(Math.abs(speed));

            //TODO: figure out how to include the while (opModeISActive() loop in code above
            //the encoder resolution is 537.7 PPR, this is for a 19.2:1 gear ratio
            //initial guess for fully extending the slide: about 15,000 - WAY TO MUCH

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    armDrive.isBusy()) {

                // Display it for the driver.
                telemetry.addData("Running telescoping arm to", " %7d", targetTicks);
                telemetry.addData("Currently at", " at %7d",
                        armDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            armDrive.setPower(0.05); // to prevent drift... ZeroPowerBehavior.BRAKE doesn't work

        }
    }

    /*
     *  Method to perform a move to an exact position, based on encoder counts.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     *
     *  TODO: Rename this. Driving wheels by time, not encoder.
     */
    public void driveByTime(double axial,
                             double lateral,
                             double yaw,
                             double timeout) {

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        // Set up a variable for each drive wheel to save the power level for telemetry.
        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral - yaw;

        // Normalize the values so no wheel power exceeds 100%
        // This ensures that the robot maintains the desired motion.
        double max;
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send calculated power to wheels to go
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < timeout)) {
            telemetry.addData("Path", "Leg 1: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Send calculated power to wheels to stop
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    public void turnToHeading(double maxTurnSpeed, double heading) {

        // Run getSteeringCorrection() once to pre-calculate the current error
        getSteeringCorrection(heading, P_DRIVE_GAIN); // TODO: check your args

        // keep looping while we are still active, and not on heading.
        while (opModeIsActive() && (Math.abs(headingError) > HEADING_THRESHOLD)) {

            // Determine required steering to keep on heading
            turnSpeed = getSteeringCorrection(heading, P_TURN_GAIN);

            // Clip the speed to the maximum permitted value.
            turnSpeed = Range.clip(turnSpeed, -maxTurnSpeed, maxTurnSpeed);

            // Pivot in place by applying the turning correction
            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower = -1 * turnSpeed;
            double rightFrontPower = turnSpeed;
            double leftBackPower = -1 * turnSpeed;
            double rightBackPower = turnSpeed;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            double max;
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // Send calculated power to wheels to go
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);


            // Display drive status for the driver.
            sendTelemetry(false);
        }

        // Stop all motion;
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }


    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        targetHeading = desiredHeading;  // Save for telemetry

        // Determine the heading current error
        headingError = targetHeading - getHeading();

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180)  headingError -= 360;
        while (headingError <= -180) headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(headingError * proportionalGain, -1, 1);
    }
    public double getHeading() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }
    private void sendTelemetry(boolean straight) {

        if (straight) {
            telemetry.addData("Motion", "Drive Straight");
        } else {
            telemetry.addData("Motion", "Turning");
        }

        telemetry.addData("Heading- Target : Current", "%5.2f : %5.0f", targetHeading, getHeading());
        telemetry.addData("Error  : Steer Pwr",  "%5.1f : %5.1f", headingError, turnSpeed);
        telemetry.addData("Threshold :",  "%5.1f", HEADING_THRESHOLD);
        telemetry.update();
    }
}
