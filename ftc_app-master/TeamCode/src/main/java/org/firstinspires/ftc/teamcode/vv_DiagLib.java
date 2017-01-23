package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import static org.firstinspires.ftc.teamcode.vv_Constants.BACK_LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.vv_Constants.BACK_RIGHT_MOTOR;
import static org.firstinspires.ftc.teamcode.vv_Constants.FRONT_LEFT_MOTOR;
import static org.firstinspires.ftc.teamcode.vv_Constants.FRONT_RIGHT_MOTOR;

/**
 * Created by thomas on 9/25/2016.
 */

public class vv_DiagLib {


    RobotTest robotTestArray[];
    TestFrontLeftWheel testFrontLeftWheel = new TestFrontLeftWheel();
    TestFrontRightWheel testFrontRightWheel = new TestFrontRightWheel();
    TestBackLeftWheel testBackLeftWheel = new TestBackLeftWheel();
    TestBackRightWheel testBackRightWheel = new TestBackRightWheel();
    private vv_Robot robot;

    public vv_DiagLib(vv_OpMode aOpMode)
            throws InterruptedException {
        robot = new vv_Robot();
        robot.init(aOpMode, aOpMode.hardwareMap);

        //initialize the array of tests

        robotTestArray = new RobotTest[20];
        //lets push the tests we have into the array.


        invalidateAllTests(aOpMode);

        robotTestArray[0].createTest(aOpMode, "testFrontLeftMotor", 0, "Test Front Left Motor",
                "Tests the front left motor by running it for a small duration", TestType.AUTOMATIC, testFrontLeftWheel);

        robotTestArray[1].createTest(aOpMode, "testFrontRightMotor", 1, "Test Front Right Motor",
                "Tests the front right motor by running it for a small duration", TestType.AUTOMATIC, testFrontRightWheel);

        robotTestArray[2].createTest(aOpMode, "testBackLeftMotor", 2, "Test Back Left Motor",
                "Tests the back left motor by running it for a small duration", TestType.AUTOMATIC, testBackLeftWheel);

        robotTestArray[3].createTest(aOpMode, "testBackRightMotor", 3, "Test Back Right Motor",
                "Tests the back right motor by running it for a small duration", TestType.AUTOMATIC, testBackRightWheel);


        invalidateAllTestResults(aOpMode);
    }

    public RobotTest findTestByName(vv_OpMode aOpMode, String name) {
        //could do this in a hashMap but since the number of records is so small we can use a simple loop.
        // the cost of a string compare is ok for our small number of records.

        for (int i = 0; i < robotTestArray.length; i++) {
            if (robotTestArray[i].getTestName(aOpMode).equals(name)) //found test
                return (robotTestArray[i]);
        }
        //we have not found the record, return null.
        return null;

    }

    public void runAllTests(vv_OpMode aOpMode) throws InterruptedException {
        for (int i = 0; i < robotTestArray.length; i++) {
            if (robotTestArray[i].getTestValidity(aOpMode))
                //runnable test, it has been initialized
                //lets run and store the test.
                aOpMode.telemetryAddData("Running Test:", "Named:", robotTestArray[i].getTestName(aOpMode));
            aOpMode.telemetryUpdate();
            robotTestArray[i].getTestRunnableTest(aOpMode).runTest(aOpMode, robotTestArray[i]); //we expect the runTest itself will
            //set all the values of the record properly.
        }

    }

    public void initializeAllTests(vv_OpMode aOpMode) {
        for (int i = 0; i < robotTestArray.length; i++) {
            robotTestArray[i].initializeTestRecord(aOpMode);
            robotTestArray[i].setTestValidity(aOpMode, false); //invalidate each test
            robotTestArray[i].setTestResultValidity(aOpMode, false); //invalidate each test result
        }
    }

    public void invalidateAllTests(vv_OpMode aOpMode) {
        for (int i = 0; i < robotTestArray.length; i++) {
            if (robotTestArray[i].testRecordEntry != null) {
                robotTestArray[i].setTestValidity(aOpMode, false); //invalidate each test
            }
        }
    }


    //These are the tests

    public void invalidateAllTestResults(vv_OpMode aOpMode) {
        for (int i = 0; i < robotTestArray.length; i++) {
            if (robotTestArray[i].testRecordEntry != null) {
                robotTestArray[i].setTestResultValidity(aOpMode, false); //invalidate each test result
            }
        }
    }

    enum ResultSeverity {CRITICAL, HIGH, MEDIUM, LOW, INFO}


    enum TestType {AUTOMATIC, MANUAL}

    public interface RunnableTest {
        boolean runTest(vv_OpMode aOpMode, RobotTest robotTest) throws InterruptedException;
    }

    protected class RobotTest {
        TestRecord testRecordEntry = null;

        public void setTestElementId(vv_OpMode aOpMode, int id) {
            testRecordEntry.testElementId = id;
        }

        public void setTestElementName(vv_OpMode aOpMode, String testName) {
            testRecordEntry.testElementName = testName;
        }

        public void setTestShortDescription(vv_OpMode aOpMode, String shortDescription) {
            testRecordEntry.testShortDescription = shortDescription;
        }

        public void setTestLongDescription(vv_OpMode aOpMode, String longDescription) {
            testRecordEntry.testShortDescription = longDescription;
        }

        public void setTestValidity(vv_OpMode aOpMode, boolean testValidity) {
            testRecordEntry.testValidity = testValidity;
        }

        public void setTestResultValidity(vv_OpMode aOpMode, boolean testResultValidity) {
            testRecordEntry.testResultValidity = testResultValidity;
        }

        public void setTestTimeStamp(vv_OpMode aOpMode, long timeStampMilliseconds) {
            testRecordEntry.testTimeStamp = timeStampMilliseconds;
        }

        public void setTestResult(vv_OpMode aOpMode, boolean testResult) {
            testRecordEntry.testResult = testResult;
        }

        public void setTestSeverity(vv_OpMode aOpMode, ResultSeverity resultSeverity) {
            testRecordEntry.testResultSeverity = resultSeverity;
        }

        public void setTestRecommendation(vv_OpMode aOpMode, String testRecommendation) {
            testRecordEntry.testRecommendation = testRecommendation;
        }

        public void setTestType(vv_OpMode aOpMode, TestType testType) {
            testRecordEntry.testType = testType;
        }

        public void setRunnableTest(vv_OpMode aOpMode, RunnableTest runnableTest) {
            testRecordEntry.testRunMethod = runnableTest;
        }

        public int getTestElementId(vv_OpMode aOpMode) {
            return testRecordEntry.testElementId;
        }

        public String getTestName(vv_OpMode aOpMode) {
            return testRecordEntry.testElementName;
        }

        public int getTestId(vv_OpMode aOpMode) {
            return testRecordEntry.testElementId;
        }

        public String getTestShortDescription(vv_OpMode aOpMode) {
            return testRecordEntry.testShortDescription;
        }

        public String getTestLongDescription(vv_OpMode aOpMode) {
            return testRecordEntry.testLongDescription;
        }

        public boolean getTestValidity(vv_OpMode aOpMode) {
            return testRecordEntry.testValidity;
        }

        public boolean getTestResultValidity(vv_OpMode aOpMode) {
            return testRecordEntry.testResultValidity;
        }

        public long getTestTimeStamp(vv_OpMode aOpMode) {
            return testRecordEntry.testTimeStamp;
        }

        public boolean getTestResult(vv_OpMode aOpMode) {
            return testRecordEntry.testResult;
        }

        public ResultSeverity getTestSeverity(vv_OpMode aOpMode) {
            return testRecordEntry.testResultSeverity;
        }

        public String getTestRecommendation(vv_OpMode aOpMode) {
            return testRecordEntry.testRecommendation;
        }

        public TestType getTestType(vv_OpMode aOpMode) {
            return testRecordEntry.testType;
        }

        public RunnableTest getTestRunnableTest(vv_OpMode aOpMode) {
            return testRecordEntry.testRunMethod;
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

        public void initializeTestRecord(vv_OpMode aOpMode) {
            testRecordEntry = new TestRecord();
        }

        private class TestRecord {
            String testElementName;
            int testElementId;
            String testShortDescription;
            String testLongDescription;
            boolean testResult;
            ResultSeverity testResultSeverity;
            String testRecommendation;
            long testTimeStamp;
            boolean testValidity; //validity of the test itself, set true if test initialized
            boolean testResultValidity; //validity of the results of the test, reset when running a new cycle.
            TestType testType;
            RunnableTest testRunMethod;
        }


    }

    class TestFrontLeftWheel implements RunnableTest {
        public boolean runTest(vv_OpMode aOpMode, RobotTest robotTest) throws InterruptedException {
            robot.setMotorMode(aOpMode, FRONT_LEFT_MOTOR, DcMotor.RunMode.RUN_USING_ENCODER);
            Thread.sleep(50); //to complete action.
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
            Thread.sleep(50); //to complete action.
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
            Thread.sleep(50); //to complete action.
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
            Thread.sleep(50); //to complete action.
            int motorPosition = robot.getMotorPosition(aOpMode, BACK_RIGHT_MOTOR);
            robot.testMotor(aOpMode, BACK_RIGHT_MOTOR, 0.5f, 1000);
            int newMotorPosition = robot.getMotorPosition(aOpMode, BACK_LEFT_MOTOR);
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

}
