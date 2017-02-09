package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.teamcode.vv_Constants.ANDYMARK_MOTOR_ENCODER_COUNTS_PER_REVOLUTION;
import static org.firstinspires.ftc.teamcode.vv_Constants.BACK_LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.vv_Constants.BACK_RIGHT_MOTOR;
import static org.firstinspires.ftc.teamcode.vv_Constants.DEBUG;
import static org.firstinspires.ftc.teamcode.vv_Constants.FRONT_LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.vv_Constants.FRONT_RIGHT_MOTOR;
import static org.firstinspires.ftc.teamcode.vv_Constants.MECCANUM_WHEEL_DIAMETER;
import static org.firstinspires.ftc.teamcode.vv_Constants.ROBOT_TRACK_DISTANCE;
import static org.firstinspires.ftc.teamcode.vv_Constants.TURN_POWER;

/**
 * Created by thomas on 9/25/2016.
 */

public class vv_DiagLib {

    protected vv_Robot robot;
    RobotTest robotTestArray[];

    public vv_DiagLib(vv_OpMode aOpMode)
            throws InterruptedException {
        robot = new vv_Robot();
        robot.init(aOpMode, aOpMode.hardwareMap);

        //initialize the array of tests

        //first lets create the space for the tests
        robotTestArray = new RobotTest[20];

        //now lets actually initialize the array with class instances.

        for (int i = 0; i < robotTestArray.length; i++) {
            robotTestArray[i] = new RobotTest();
        }
        //lets push the tests we have into the array.


        invalidateAllTests(aOpMode);

        createTestEntries(aOpMode);


        aOpMode.DBG("Before Invalidate all Results");

        invalidateAllTestResults(aOpMode);
    }

    public RobotTest findTestByName(vv_OpMode aOpMode, String name) {
        //could do this in a hashMap but since the number of records is so small we can use a simple loop.
        // the cost of a string compare is ok for our small number of records.

        for (int i = 0; i < robotTestArray.length; i++) {
            if (robotTestArray[i].getTestValidity(aOpMode)) {
                //the test is valid
                if (robotTestArray[i].getTestName(aOpMode).equals(name)) {
                    //found test
                    return (robotTestArray[i]);
                }
            }

        }
        //we have not found the record, return null.
        return null;

    }

    public void runAllTests(vv_OpMode aOpMode) throws InterruptedException {
        for (int i = 0; i < robotTestArray.length; i++) {
            if (robotTestArray[i].getTestValidity(aOpMode)) {
                //runnable test, it has been initialized
                //lets run and store the test.
                aOpMode.telemetryAddData("Running Test:", "Named:",
                        robotTestArray[i].getTestName(aOpMode) +
                                robotTestArray[i].getTestShortDescription(aOpMode));
                aOpMode.telemetryUpdate();
                robotTestArray[i].getTestRunnableTest(aOpMode).
                        runTest(aOpMode, robotTestArray[i]); //we expect the runTest itself will
                //set all the values of the record properly.
            }
        }

    }

    public void runAllAutomaticTests(vv_OpMode aOpMode) throws InterruptedException {
        for (int i = 0; i < robotTestArray.length; i++) {
            if ((robotTestArray[i].getTestValidity(aOpMode)) &&
                    (robotTestArray[i].getTestType(aOpMode) == TestType.AUTOMATIC)) {
                //runnable test, it has been initialized and it is an automatic test

                //lets run and store the test.
                aOpMode.telemetryAddData("Running Test:", "Named:",
                        robotTestArray[i].getTestName(aOpMode) +
                                robotTestArray[i].getTestShortDescription(aOpMode));
                aOpMode.telemetryUpdate();
                robotTestArray[i].getTestRunnableTest(aOpMode).
                        runTest(aOpMode, robotTestArray[i]); //we expect the runTest itself will
                //set all the values of the record properly.
            }
        }

    }

    public void initializeAllTests(vv_OpMode aOpMode) {
        for (int i = 0; i < robotTestArray.length; i++) {
            robotTestArray[i].setTestValidity(aOpMode, false); //invalidate each test
            robotTestArray[i].setTestResultValidity(aOpMode, false); //invalidate each test result
        }
    }

    public void invalidateAllTests(vv_OpMode aOpMode) {
        for (int i = 0; i < robotTestArray.length; i++) {
            robotTestArray[i].setTestValidity(aOpMode, false); //invalidate each test
        }
    }


    //These are the tests

    public void invalidateAllTestResults(vv_OpMode aOpMode) {
        for (int i = 0; i < robotTestArray.length; i++) {
            robotTestArray[i].setTestResultValidity(aOpMode, false); //invalidate each test result
        }
    }

    private void createTestEntries(vv_OpMode aOpMode) {
        robotTestArray[0].createTest(aOpMode, "testFrontLeftMotor", 0, "Test Front Left Motor",
                "Tests the front left motor by running it for a small duration",
                TestType.AUTOMATIC, new TestFrontLeftWheel());

        robotTestArray[1].createTest(aOpMode, "testFrontRightMotor", 1, "Test Front Right Motor",
                "Tests the front right motor by running it for a small duration",
                TestType.AUTOMATIC, new TestFrontRightWheel());

        robotTestArray[2].createTest(aOpMode, "testBackLeftMotor", 2, "Test Back Left Motor",
                "Tests the back left motor by running it for a small duration",
                TestType.AUTOMATIC, new TestBackLeftWheel());

        robotTestArray[3].createTest(aOpMode, "testBackRightMotor", 3, "Test Back Right Motor",
                "Tests the back right motor by running it for a small duration",
                TestType.AUTOMATIC, new TestBackRightWheel());

        robotTestArray[4].createTest(aOpMode, "testPlatformLeft", 4, "Test Platform Left",
                "Tests the Platform by moving it left for a small duration",
                TestType.AUTOMATIC, new TestPlatformLeft());

        robotTestArray[5].createTest(aOpMode, "testPlatformRight", 5, "Test Platform Right",
                "Tests the Platform by moving it right for a small duration",
                TestType.AUTOMATIC, new TestPlatformRight());

        robotTestArray[6].createTest(aOpMode, "testPlatformForward", 6, "Test Platform Forward",
                "Tests the Platform by moving it forward for a small duration",
                TestType.AUTOMATIC, new TestPlatformForward());

        robotTestArray[7].createTest(aOpMode, "testPlatformBackward", 7, "Test Platform Backward",
                "Tests the Platform by moving it backward for a small duration",
                TestType.AUTOMATIC, new TestPlatformBackward());

        robotTestArray[8].createTest(aOpMode, "testPlatformRotation", 8, "Test Platform Rotation",
                "Tests the Platform by rotating it for a fixed angle",
                TestType.AUTOMATIC, new TestPlatformRotation());

        robotTestArray[9].createTest(aOpMode, "testRangeSensor", 9, "Test Ultrasonic Range Sensor",
                "Tests the Range sensing by rotating the platform and checking for change in Range",
                TestType.AUTOMATIC, new TestRangeSensor());

    }

    public void analyzeTestResults(vv_OpMode aOpMode) throws InterruptedException {
        boolean noErrorsFound = true;

        for (int i = 0; i < robotTestArray.length; i++) {
            //list all tests and results.
            if (robotTestArray[i].getTestValidity(aOpMode)) {
                //its a valid test
                if (robotTestArray[i].getTestResultValidity(aOpMode)) {

                    //its a valid result
                    if (robotTestArray[i].getTestResult(aOpMode)) {
                        //this test passed.
                        //lets print this message out.
                        aOpMode.telemetryAddData("Test Result:",
                                robotTestArray[i].getTestName(aOpMode),
                                ":" + "Test Passed");
                        aOpMode.telemetryUpdate();

                    } else {
                        //this test failed.
                        //lets print out the message.
                        //flag the error in the boolean tracker
                        noErrorsFound = false;
                        //raise the flag.
                        robot.setBallFlagServoPosition(aOpMode,
                                vv_Constants.BALL_FLAG_SERVO_ALARM);

                        aOpMode.telemetryAddData("Test Result:",
                                robotTestArray[i].getTestName(aOpMode),
                                ":" + "Test Failed");
                        aOpMode.telemetryAddData("Test Result Message:", "",
                                robotTestArray[i].getTestResultMessage(aOpMode));

                        switch (robotTestArray[i].getTestSeverity(aOpMode)) {
                            case CRITICAL:
                                aOpMode.telemetryAddData("Test Result:",
                                        "This is a CRITICAL SEVERITY ERROR",
                                        "Error");
                                break;

                            case HIGH:
                                aOpMode.telemetryAddData("Test Result:",
                                        "This is a HIGH SEVERITY",
                                        "Error");
                                break;
                            case MEDIUM:
                                aOpMode.telemetryAddData("Test Result:",
                                        "This is a MEDIUM SEVERITY ERROR",
                                        "Error");
                                break;
                            case LOW:
                                break;
                            case INFO:
                                break;
                        }

                        aOpMode.telemetryAddData("Recommendation:", "try this:",
                                robotTestArray[i].getTestRecommendation(aOpMode));
                        aOpMode.telemetryUpdate();
                        //wait and display our errors if any
                        Thread.sleep(5000);

                    }

                }

            }

        }
    }

    public void turnAbsoluteMxpGyroDegrees(vv_OpMode aOpMode, float fieldReferenceDegrees) throws InterruptedException {
        //clockwise is represented by clockwise numbers.
        //counterclockwise by negative angle numbers in degrees.
        //the fieldReferenceDegrees parameters measures degrees off the initial reference frame when the robot is started and the gyro is
        //calibrated.
        // >> IMPORTANT: This depends on the zIntegratedHeading not being altered by relative turns !!!

        //first take the absolute degrees and modulus down to 0 and 359.

        float targetDegrees = fieldReferenceDegrees % 360;

        //compare to the current gyro zIntegrated heading and store the result.
        //the Integrated zValue returned is positive for clockwise turns
        //read the heading and store it.

        float startingHeading = robot.getMxpGyroSensorHeading(aOpMode);
        float turnDegrees = targetDegrees - startingHeading;

        //make the turn using encoders

        if (DEBUG) {
            aOpMode.telemetryAddData("targetDegrees", "Value",
                    ":" + targetDegrees);
            aOpMode.telemetryAddData("Starting Z", "Value",
                    ":" + startingHeading);
            aOpMode.telemetryAddData("Turn Degrees", "Value",
                    ":" + turnDegrees);

            aOpMode.telemetryUpdate();
        }

        //optimize the turn, so that direction of turn results in smallest turn needed.

        if (Math.abs(turnDegrees) > 180) {
            turnDegrees = Math.signum(turnDegrees) * -1 * (360 - Math.abs(turnDegrees));
        }

        turnUsingEncoders(aOpMode, TURN_POWER, Math.abs(turnDegrees),
                turnDegrees > 0 ? vv_Constants.TurnDirectionEnum.Clockwise :
                        vv_Constants.TurnDirectionEnum.Counterclockwise);

        float finalDegrees = robot.getMxpGyroSensorHeading(aOpMode);
        Thread.sleep(50); //cooling off after gyro read to prevent error in next run.

        if (DEBUG) {
            aOpMode.telemetryAddData("New Bearing Degrees", "Value:",
                    ":" + finalDegrees);
            aOpMode.telemetryAddData("Turn Error Degrees", "Value:",
                    ":" + (targetDegrees - finalDegrees));
            aOpMode.telemetryUpdate();
        }

    }

    public void turnUsingEncoders(vv_OpMode aOpMode, float power, float angle, vv_Constants.TurnDirectionEnum TurnDirection)
            throws InterruptedException {

        //calculate the turn distance to be used in terms of encoder clicks.
        //for Andymark encoders.

        int turnDistance = (int) (2 * ((ROBOT_TRACK_DISTANCE) * angle
                * ANDYMARK_MOTOR_ENCODER_COUNTS_PER_REVOLUTION) /
                (MECCANUM_WHEEL_DIAMETER * 360));


        switch (TurnDirection) {
            case Clockwise:
                robot.runRobotToPosition(aOpMode, power, power, power, power, turnDistance, -turnDistance, turnDistance, -turnDistance, true);
                break;
            case Counterclockwise:
                robot.runRobotToPosition(aOpMode, power, power, power, power, -turnDistance, turnDistance, -turnDistance, turnDistance, true);
                break;
        }

        //wait just a bit for the commands to complete
        Thread.sleep(50);
    }

    enum ResultSeverity {CRITICAL, HIGH, MEDIUM, LOW, INFO}

    enum TestType {AUTOMATIC, MANUAL}

    public interface RunnableTest {
        boolean runTest(vv_OpMode aOpMode, RobotTest robotTest) throws InterruptedException;
    }

    protected class RobotTest {
        String testElementName;
        int testElementId;
        String testShortDescription;
        String testLongDescription;
        boolean testResult;
        ResultSeverity testResultSeverity;
        String testResultMessage;
        String testRecommendation;
        long testTimeStamp;
        boolean testValidity; //validity of the test itself, set true if test initialized
        boolean testResultValidity; //validity of the results of the test, reset when running a new cycle.
        TestType testType;
        RunnableTest testRunMethod;

        public void setTestElementId(vv_OpMode aOpMode, int id) {
            testElementId = id;
        }

        public void setTestElementName(vv_OpMode aOpMode, String name) {
            testElementName = name;
        }

        public void setTestShortDescription(vv_OpMode aOpMode, String shortDescription) {
            testShortDescription = shortDescription;
        }

        public void setTestLongDescription(vv_OpMode aOpMode, String longDescription) {
            testLongDescription = longDescription;
        }

        public void setTestValidity(vv_OpMode aOpMode, boolean validity) {
            testValidity = validity;
        }

        public void setTestResultValidity(vv_OpMode aOpMode, boolean resultValidity) {
            testResultValidity = resultValidity;
        }

        public void setTestTimeStamp(vv_OpMode aOpMode, long timeStampMilliseconds) {
            testTimeStamp = timeStampMilliseconds;
        }

        public void setTestResult(vv_OpMode aOpMode, boolean result) {
            testResult = result;
        }

        public void setTestSeverity(vv_OpMode aOpMode, ResultSeverity resultSeverity) {
            testResultSeverity = resultSeverity;
        }

        public void setTestRecommendation(vv_OpMode aOpMode, String recommendation) {
            testRecommendation = recommendation;
        }

        public void setTestType(vv_OpMode aOpMode, TestType type) {
            testType = type;
        }

        public void setRunnableTest(vv_OpMode aOpMode, RunnableTest runnableTest) {
            testRunMethod = runnableTest;
        }

        public int getTestElementId(vv_OpMode aOpMode) {
            return testElementId;
        }

        public String getTestName(vv_OpMode aOpMode) {
            return testElementName;
        }

        public int getTestId(vv_OpMode aOpMode) {
            return testElementId;
        }

        public String getTestShortDescription(vv_OpMode aOpMode) {
            return testShortDescription;
        }

        public String getTestLongDescription(vv_OpMode aOpMode) {
            return testLongDescription;
        }

        public boolean getTestValidity(vv_OpMode aOpMode) {
            return testValidity;
        }

        public boolean getTestResultValidity(vv_OpMode aOpMode) {
            return testResultValidity;
        }

        public long getTestTimeStamp(vv_OpMode aOpMode) {
            return testTimeStamp;
        }

        public boolean getTestResult(vv_OpMode aOpMode) {
            return testResult;
        }

        public String getTestResultMessage(vv_OpMode aOpMode) {
            return testResultMessage;
        }

        public ResultSeverity getTestSeverity(vv_OpMode aOpMode) {
            return testResultSeverity;
        }

        public String getTestRecommendation(vv_OpMode aOpMode) {
            return testRecommendation;
        }

        public TestType getTestType(vv_OpMode aOpMode) {
            return testType;
        }

        public RunnableTest getTestRunnableTest(vv_OpMode aOpMode) {
            return testRunMethod;
        }

        public void createTest(vv_OpMode aOpMode, String name, int id, String shortDescription,
                               String longDescription, TestType testType, RunnableTest runnableTest) {

            //set values
            setTestElementId(aOpMode, id);
            setTestElementName(aOpMode, name);
            setTestShortDescription(aOpMode, shortDescription);
            setTestLongDescription(aOpMode, longDescription);
            setTestType(aOpMode, testType);
            setRunnableTest(aOpMode, runnableTest);
            setTestValidity(aOpMode, true);
            setTestResultValidity(aOpMode, false);
        }


    }

    class TestFrontLeftWheel implements RunnableTest {
        public boolean runTest(vv_OpMode aOpMode, RobotTest robotTest) throws InterruptedException {
            robot.setMotorMode(aOpMode, FRONT_LEFT_MOTOR, DcMotor.RunMode.RUN_USING_ENCODER);
            int motorPosition = robot.getMotorPosition(aOpMode, FRONT_LEFT_MOTOR);
            robot.testMotor(aOpMode, FRONT_LEFT_MOTOR, 0.5f, 1000);
            int newMotorPosition = robot.getMotorPosition(aOpMode, FRONT_LEFT_MOTOR);
            if (newMotorPosition == motorPosition) {
                //the motor encoder has not moved. we have a problem.
                robotTest.setTestResult(aOpMode, false);
                robotTest.setTestResultValidity(aOpMode, true);
                robotTest.setTestRecommendation(aOpMode, "Please examine electrical connections, front left motor or encoder failed");
                robotTest.setTestSeverity(aOpMode, ResultSeverity.HIGH);
                return false;

            }
            //there is no problem.
            robotTest.setTestResult(aOpMode, true);
            robotTest.setTestResultValidity(aOpMode, true);
            robotTest.setTestRecommendation(aOpMode, "No problem with Front Left Wheel");
            robotTest.setTestSeverity(aOpMode, ResultSeverity.HIGH);
            return true;

        }
    }

    class TestFrontRightWheel implements RunnableTest {
        public boolean runTest(vv_OpMode aOpMode, RobotTest robotTest) throws InterruptedException {
            robot.setMotorMode(aOpMode, FRONT_RIGHT_MOTOR, DcMotor.RunMode.RUN_USING_ENCODER);
            int motorPosition = robot.getMotorPosition(aOpMode, FRONT_RIGHT_MOTOR);
            robot.testMotor(aOpMode, FRONT_RIGHT_MOTOR, 0.5f, 1000);
            int newMotorPosition = robot.getMotorPosition(aOpMode, FRONT_RIGHT_MOTOR);
            if (newMotorPosition == motorPosition) {
                //the motor encoder has not moved. we have a problem.
                robotTest.setTestResult(aOpMode, false);
                robotTest.setTestResultValidity(aOpMode, true);
                robotTest.setTestRecommendation(aOpMode, "Please examine electrical connections, front right motor or encoder failed");
                robotTest.setTestSeverity(aOpMode, ResultSeverity.HIGH);
                return false;

            }
            //there is no problem.
            robotTest.setTestResult(aOpMode, true);
            robotTest.setTestResultValidity(aOpMode, true);
            robotTest.setTestRecommendation(aOpMode, "No problem with Front Right Wheel");
            robotTest.setTestSeverity(aOpMode, ResultSeverity.HIGH);
            return true;

        }
    }

    class TestBackLeftWheel implements RunnableTest {
        public boolean runTest(vv_OpMode aOpMode, RobotTest robotTest) throws InterruptedException {
            robot.setMotorMode(aOpMode, BACK_LEFT_MOTOR, DcMotor.RunMode.RUN_USING_ENCODER);
            int motorPosition = robot.getMotorPosition(aOpMode, BACK_LEFT_MOTOR);
            robot.testMotor(aOpMode, BACK_LEFT_MOTOR, 0.5f, 1000);
            int newMotorPosition = robot.getMotorPosition(aOpMode, BACK_LEFT_MOTOR);
            if (newMotorPosition == motorPosition) {
                //the motor encoder has not moved. we have a problem.
                robotTest.setTestResult(aOpMode, false);
                robotTest.setTestResultValidity(aOpMode, true);
                robotTest.setTestRecommendation(aOpMode, "Please examine electrical connections, back left motor or encoder failed");
                robotTest.setTestSeverity(aOpMode, ResultSeverity.HIGH);
                return false;

            }
            //there is no problem.
            robotTest.setTestResult(aOpMode, true);
            robotTest.setTestResultValidity(aOpMode, true);
            robotTest.setTestRecommendation(aOpMode, "No problem with Back Left Wheel");
            robotTest.setTestSeverity(aOpMode, ResultSeverity.HIGH);
            return true;

        }
    }

    class TestBackRightWheel implements RunnableTest {
        public boolean runTest(vv_OpMode aOpMode, RobotTest robotTest) throws InterruptedException {
            robot.setMotorMode(aOpMode, BACK_RIGHT_MOTOR, DcMotor.RunMode.RUN_USING_ENCODER);
            int motorPosition = robot.getMotorPosition(aOpMode, BACK_RIGHT_MOTOR);
            robot.testMotor(aOpMode, BACK_RIGHT_MOTOR, 0.5f, 1000);
            int newMotorPosition = robot.getMotorPosition(aOpMode, BACK_RIGHT_MOTOR);
            if (newMotorPosition == motorPosition) {
                //the motor encoder has not moved. we have a problem.
                robotTest.setTestResult(aOpMode, false);
                robotTest.setTestResultValidity(aOpMode, true);
                robotTest.setTestRecommendation(aOpMode, "Please examine electrical connections, Back Right motor or encoder failed");
                robotTest.setTestSeverity(aOpMode, ResultSeverity.HIGH);
                return false;

            }
            //there is no problem.
            robotTest.setTestResult(aOpMode, true);
            robotTest.setTestResultValidity(aOpMode, true);
            robotTest.setTestRecommendation(aOpMode, "No problem with Back Right Wheel");
            robotTest.setTestSeverity(aOpMode, ResultSeverity.HIGH);
            return true;

        }
    }

    class TestPlatformLeft implements RunnableTest {
        public boolean runTest(vv_OpMode aOpMode, RobotTest robotTest) throws InterruptedException {

            int motorPosition = robot.getMotorPosition(aOpMode, FRONT_LEFT_MOTOR); //representative motor
            float startingAngle = robot.getMxpGyroSensorHeading(aOpMode); //save starting angle.
            robot.universalMoveRobotForDuration(aOpMode, -0.5f, 0.0f, 0.0f, 0); //move robot to left
            int newMotorPosition = robot.getMotorPosition(aOpMode, FRONT_LEFT_MOTOR);
            float endingAngle = robot.getMxpGyroSensorHeading(aOpMode);
            if ((newMotorPosition == motorPosition) || (Math.abs(startingAngle - endingAngle) > 3)) {
                //the motor encoder has not moved. we have a problem.
                //or the platform has moved, but there is too much rotation
                robotTest.setTestResult(aOpMode, false);
                robotTest.setTestResultValidity(aOpMode, true);
                robotTest.setTestRecommendation(aOpMode, "Please examine electrical connections and encoder failure");
                robotTest.setTestSeverity(aOpMode, ResultSeverity.HIGH);
                return false;

            }
            //there is no problem.
            robotTest.setTestResult(aOpMode, true);
            robotTest.setTestResultValidity(aOpMode, true);
            robotTest.setTestRecommendation(aOpMode, "No problem with Platform Left");
            robotTest.setTestSeverity(aOpMode, ResultSeverity.HIGH);
            return true;

        }
    }

    class TestPlatformRight implements RunnableTest {
        public boolean runTest(vv_OpMode aOpMode, RobotTest robotTest) throws InterruptedException {

            int motorPosition = robot.getMotorPosition(aOpMode, FRONT_LEFT_MOTOR); //representative motor
            float startingAngle = robot.getMxpGyroSensorHeading(aOpMode); //save starting angle.
            robot.universalMoveRobotForDuration(aOpMode, 0.5f, 0.0f, 0.0f, 0); //move robot to left
            int newMotorPosition = robot.getMotorPosition(aOpMode, FRONT_LEFT_MOTOR);
            float endingAngle = robot.getMxpGyroSensorHeading(aOpMode);
            if ((newMotorPosition == motorPosition) || (Math.abs(startingAngle - endingAngle) > 3)) {
                //the motor encoder has not moved. we have a problem.
                //or the platform has moved, but there is too much rotation
                robotTest.setTestResult(aOpMode, false);
                robotTest.setTestResultValidity(aOpMode, true);
                robotTest.setTestRecommendation(aOpMode,
                        "Please examine electrical connections and encoder failure");
                robotTest.setTestSeverity(aOpMode, ResultSeverity.HIGH);
                return false;

            }
            //there is no problem.
            robotTest.setTestResult(aOpMode, true);
            robotTest.setTestResultValidity(aOpMode, true);
            robotTest.setTestRecommendation(aOpMode, "No problem with Platform Right");
            robotTest.setTestSeverity(aOpMode, ResultSeverity.HIGH);
            return true;

        }
    }

    class TestPlatformForward implements RunnableTest {
        public boolean runTest(vv_OpMode aOpMode, RobotTest robotTest) throws InterruptedException {

            int motorPosition = robot.getMotorPosition(aOpMode, FRONT_LEFT_MOTOR); //representative motor
            float startingAngle = robot.getMxpGyroSensorHeading(aOpMode); //save starting angle.
            robot.universalMoveRobotForDuration(aOpMode, 0.0f, 0.5f, 0.0f, 0); //move robot to left
            int newMotorPosition = robot.getMotorPosition(aOpMode, FRONT_LEFT_MOTOR);
            float endingAngle = robot.getMxpGyroSensorHeading(aOpMode);
            if ((newMotorPosition == motorPosition) || (Math.abs(startingAngle - endingAngle) > 3)) {
                //the motor encoder has not moved. we have a problem.
                //or the platform has moved, but there is too much rotation
                robotTest.setTestResult(aOpMode, false);
                robotTest.setTestResultValidity(aOpMode, true);
                robotTest.setTestRecommendation(aOpMode,
                        "Please examine electrical connections and encoder failure");
                robotTest.setTestSeverity(aOpMode, ResultSeverity.HIGH);
                return false;

            }
            //there is no problem.
            robotTest.setTestResult(aOpMode, true);
            robotTest.setTestResultValidity(aOpMode, true);
            robotTest.setTestRecommendation(aOpMode, "No problem with Platform Forward");
            robotTest.setTestSeverity(aOpMode, ResultSeverity.HIGH);
            return true;

        }
    }

    class TestPlatformBackward implements RunnableTest {
        public boolean runTest(vv_OpMode aOpMode, RobotTest robotTest) throws InterruptedException {

            int motorPosition = robot.getMotorPosition(aOpMode, FRONT_LEFT_MOTOR); //representative motor
            float startingAngle = robot.getMxpGyroSensorHeading(aOpMode); //save starting angle.
            robot.universalMoveRobotForDuration(aOpMode, 0.0f, -0.5f, 0.0f, 0); //move robot to left
            int newMotorPosition = robot.getMotorPosition(aOpMode, FRONT_LEFT_MOTOR);
            float endingAngle = robot.getMxpGyroSensorHeading(aOpMode);
            if ((newMotorPosition == motorPosition) || (Math.abs(startingAngle - endingAngle) > 3)) {
                //the motor encoder has not moved. we have a problem.
                //or the platform has moved, but there is too much rotation
                robotTest.setTestResult(aOpMode, false);
                robotTest.setTestResultValidity(aOpMode, true);
                robotTest.setTestRecommendation(aOpMode,
                        "Please examine electrical connections and encoder failure");
                robotTest.setTestSeverity(aOpMode, ResultSeverity.HIGH);
                return false;

            }
            //there is no problem.
            robotTest.setTestResult(aOpMode, true);
            robotTest.setTestResultValidity(aOpMode, true);
            robotTest.setTestRecommendation(aOpMode, "No problem with Platform Backward");
            robotTest.setTestSeverity(aOpMode, ResultSeverity.HIGH);
            return true;

        }
    }


    //== utility programs copied over from vvLib.

    class TestPlatformRotation implements RunnableTest {
        public boolean runTest(vv_OpMode aOpMode, RobotTest robotTest) throws InterruptedException {


            float startingAngle = robot.getMxpGyroSensorHeading(aOpMode); //save starting angle.
            turnAbsoluteMxpGyroDegrees(aOpMode, 90);
            float endingAngle = robot.getMxpGyroSensorHeading(aOpMode);
            float error = Math.abs((Math.abs(startingAngle - endingAngle) - 90));
            if (error > 3) {

                //We are off by more than 3 degrees in our gyro rotation
                robotTest.setTestResult(aOpMode, false);
                robotTest.setTestResultValidity(aOpMode, true);
                if (error < 5) {
                    robotTest.setTestRecommendation(aOpMode,
                            "Please check MXP gyro and calibration, error >3&<5 degrees");
                    robotTest.setTestSeverity(aOpMode, ResultSeverity.HIGH);
                }
                if (error >= 5) {
                    robotTest.setTestRecommendation(aOpMode,
                            "Please check MXP gyro and calibration, error >5 degrees");
                    robotTest.setTestSeverity(aOpMode, ResultSeverity.CRITICAL);
                }
                return false;

            }
            //there is no problem.
            robotTest.setTestResult(aOpMode, true);
            robotTest.setTestResultValidity(aOpMode, true);
            robotTest.setTestRecommendation(aOpMode, "No problem with Gyro turns");
            robotTest.setTestSeverity(aOpMode, ResultSeverity.HIGH);
            return true;

        }
    }

    class TestRangeSensor implements RunnableTest {
        public boolean runTest(vv_OpMode aOpMode, RobotTest robotTest) throws InterruptedException {


            boolean rangeHasChanged = false;
            for (int degrees = 0; degrees < 360; degrees += 20) {
                double startingRange = robot.getUltrasonicDistance(aOpMode);
                turnAbsoluteMxpGyroDegrees(aOpMode, degrees);
                Thread.sleep(100);
                double endingRange = robot.getUltrasonicDistance(aOpMode);
                if (startingRange != endingRange) {
                    //Range has changed. The ultrasonic sensor is working
                    rangeHasChanged = true;
                    break;
                }
            }
            if (!rangeHasChanged) {
                robotTest.setTestRecommendation(aOpMode,
                        "Please check Ultrasonic sensor and calibration, no range changes read");
                robotTest.setTestSeverity(aOpMode, ResultSeverity.CRITICAL);
                return false;
            }
            //there is no problem.
            robotTest.setTestResult(aOpMode, true);
            robotTest.setTestResultValidity(aOpMode, true);
            robotTest.setTestRecommendation(aOpMode, "No problem with Range Sensors");
            robotTest.setTestSeverity(aOpMode, ResultSeverity.HIGH);
            return true;

        }
    }

}
