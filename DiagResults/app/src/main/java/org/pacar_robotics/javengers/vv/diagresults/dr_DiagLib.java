package org.pacar_robotics.javengers.vv.diagresults;

import org.w3c.dom.Node;
import org.w3c.dom.NodeList;


/**
 * Created by thomas on 9/25/2016.
 */

public class dr_DiagLib {

   
    RobotTest robotTestArray[];

    public dr_DiagLib()
            throws InterruptedException {
        

        //initialize the array of tests

        //first lets create the space for the tests
        robotTestArray = new RobotTest[20];

        //now lets actually initialize the array with class instances.

        for (int i = 0; i < robotTestArray.length; i++) {
            robotTestArray[i] = new RobotTest();
        }
        //lets push the tests we have into the array.


        invalidateAllTests();
       
        invalidateAllTestResults();
    }

    public RobotTest findTestByName(String name) {
        //could do this in a hashMap but since the number of records is so small we can use a simple loop.
        // the cost of a string compare is ok for our small number of records.

        for (int i = 0; i < robotTestArray.length; i++) {
            if (robotTestArray[i].getTestValidity()) {
                //the test is valid
                if (robotTestArray[i].getTestName().equals(name)) {
                    //found test
                    return (robotTestArray[i]);
                }
            }

        }
        //we have not found the record, return null.
        return null;

    }

    public void runAllTests() throws InterruptedException {
        for (int i = 0; i < robotTestArray.length; i++) {
            if (robotTestArray[i].getTestValidity()) {
                //runnable test, it has been initialized
                //lets run and store the test.
             
                robotTestArray[i].getTestRunnableTest().
                        runTest(robotTestArray[i]); //we expect the runTest itself will
                //set all the values of the record properly.
            }
        }

    }

    public void runAllAutomaticTests() throws InterruptedException {
        int validTestCount = getValidTestCount();
        for (int i = 0; i < robotTestArray.length; i++) {
            if ((robotTestArray[i].getTestValidity()) &&
                    (robotTestArray[i].getTestType() == TestType.AUTOMATIC)) {
                //runnable test, it has been initialized and it is an automatic test

                //lets run and store the test.
               
                robotTestArray[i].getTestRunnableTest().
                        runTest( robotTestArray[i]); //we expect the runTest itself will
                //set all the values of the record properly.
            }
        }

    }

    public int getValidTestCount() {
        int validTestCount = 0;
        for (int i = 0; i < robotTestArray.length; i++) {
            if (robotTestArray[i].getTestValidity()) {
                validTestCount++;
            }
        }
        return validTestCount;
    }

    public void initializeAllTests( ) {
        for (int i = 0; i < robotTestArray.length; i++) {
            robotTestArray[i].setTestValidity( false); //invalidate each test
            robotTestArray[i].setTestResultValidity( false); //invalidate each test result
        }
    }

    public void invalidateAllTests( ) {
        for (int i = 0; i < robotTestArray.length; i++) {
            robotTestArray[i].setTestValidity( false); //invalidate each test
        }
    }


    //These are the tests

    public void invalidateAllTestResults( ) {
        for (int i = 0; i < robotTestArray.length; i++) {
            robotTestArray[i].setTestResultValidity( false); //invalidate each test result
        }
    }

   
    

    public void writeAllResults( ) throws InterruptedException {
        dr_XmlLib vvXmlLib = new dr_XmlLib();

        //initialize the XML tree for writing Diagnostic Results.
        vvXmlLib.initDiagResultsXmlForWrite();

        for (int i = 0; i < robotTestArray.length; i++) {
            //list all tests and results.
            if (robotTestArray[i].getTestValidity()) {
                //its a valid test
                if (robotTestArray[i].getTestResultValidity()) {
                    //its a valid result
                    //lets add it to the XML file
                    vvXmlLib.addRobotTestResultToDom( robotTestArray[i]);
                }

            }
        }
        //We have completed writing of all the tags in the XML DOM that have valid results.
        //lets write out the XML file.
        vvXmlLib.writeDiagResultsXML();
    }

    public void readAllResults( ) throws InterruptedException {
        dr_XmlLib vvXmlLib = new dr_XmlLib();

        //initialize the XML tree for reading Diagnostic Results.
        //this opens the DiagResults xml file and reads in the DOM.
        vvXmlLib.initDiagResultsXmlForRead();

        //read the Robot Test NodesList
        NodeList robotTestNodes = vvXmlLib.getRobotTestNodesFromDom();

        //now step through the list of RobotTestNodes and load the robotTestArray
        //but first initialize all tests to have clean data

        initializeAllTests();

        for (int i = 0; i < robotTestNodes.getLength() && i < robotTestArray.length; i++) {
            //lets step through each node.
            //first get a list of all child nodes of the test
            NodeList detailsNodeList = robotTestNodes.item(i).getChildNodes();
            for (int j = 0; j < detailsNodeList.getLength(); j++) {
                //depending on the type of node we need to assign it to the right Array
                if(detailsNodeList.item(j).getNodeType()== Node.ELEMENT_NODE) {
                    switch (detailsNodeList.item(j).getNodeName()) {
                        case "TestId":
                            robotTestArray[i].
                                    setTestElementId(Integer.valueOf(detailsNodeList.item(j).getTextContent()));
                            break;

                        case "TestName":
                            robotTestArray[i].
                                    setTestElementName(detailsNodeList.item(j).getTextContent());
                            break;
                        case "TestShortDescription":
                            robotTestArray[i].
                                    setTestShortDescription(detailsNodeList.item(j).getTextContent());
                            break;
                        case "TestLongDescription":
                            robotTestArray[i].
                                    setTestLongDescription(detailsNodeList.item(j).getTextContent());
                            break;
                        case "TestResult":
                            if (detailsNodeList.item(j).getTextContent().equals("Passed")) {
                                robotTestArray[i].
                                        setTestResult(true);
                            } else {
                                robotTestArray[i].
                                        setTestResult(false);
                            }
                            break;
                        case "TestResultMessage":
                            robotTestArray[i].
                                    setTestResultMessage(detailsNodeList.item(j).getTextContent());
                            break;
                        case "TestResultSeverity":
                            switch (detailsNodeList.item(j).getTextContent()) {
                                case "CRITICAL":
                                    robotTestArray[i].
                                            setTestResultSeverity(ResultSeverity.CRITICAL);
                                    break;

                                case "HIGH":
                                    robotTestArray[i].
                                            setTestResultSeverity(ResultSeverity.HIGH);
                                    break;
                                case "MEDIUM":
                                    robotTestArray[i].
                                            setTestResultSeverity(ResultSeverity.MEDIUM);
                                    break;
                                case "LOW":
                                    robotTestArray[i].
                                            setTestResultSeverity(ResultSeverity.LOW);
                                    break;
                                case "INFO":
                                    robotTestArray[i].
                                            setTestResultSeverity(ResultSeverity.INFO);
                                    break;
                                case "UNKNOWN":
                                    robotTestArray[i].
                                            setTestResultSeverity(ResultSeverity.UNKNOWN);
                                    break;
                                default:
                                    robotTestArray[i].
                                            setTestResultSeverity(ResultSeverity.UNKNOWN);
                                    break;
                            }

                        case "TestRecommendation":
                            robotTestArray[i].
                                    setTestRecommendation(detailsNodeList.item(j).getTextContent());
                        default:

                    }
                }
            }
            //mark the test as valid
            robotTestArray[i].testResultValidity=true;
            robotTestArray[i].testValidity=true;

        }

    }


   

    enum ResultSeverity {CRITICAL, HIGH, MEDIUM, LOW, INFO, UNKNOWN}

    enum TestType {AUTOMATIC, MANUAL}

    public interface RunnableTest {
        boolean runTest( RobotTest robotTest) throws InterruptedException;
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

        public void setTestElementId(int id) {
            testElementId = id;
        }

        public void setTestElementName(String name) {
            testElementName = name;
        }

        public void setTestShortDescription(String shortDescription) {
            testShortDescription = shortDescription;
        }

        public void setTestLongDescription(String longDescription) {
            testLongDescription = longDescription;
        }

        public void setTestValidity(boolean validity) {
            testValidity = validity;
        }

        public void setTestResultValidity(boolean resultValidity) {
            testResultValidity = resultValidity;
        }

        public void setTestTimeStamp(long timeStampMilliseconds) {
            testTimeStamp = timeStampMilliseconds;
        }

        public void setTestResult(boolean result) {
            testResult = result;
        }

        public void setTestResultMessage(String ResultMessage) {
            testResultMessage = ResultMessage;
        }

        public void setTestResultSeverity(ResultSeverity resultSeverity) {
            testResultSeverity = resultSeverity;
        }

        public void setTestRecommendation(String recommendation) {
            testRecommendation = recommendation;
        }

        public void setTestType(TestType type) {
            testType = type;
        }

        public void setRunnableTest(RunnableTest runnableTest) {
            testRunMethod = runnableTest;
        }

        public int getTestElementId() {
            return testElementId;
        }

        public String getTestName() {
            return testElementName;
        }

        public int getTestId() {
            return testElementId;
        }

        public String getTestShortDescription() {
            return testShortDescription;
        }

        public String getTestLongDescription() {
            return testLongDescription;
        }

        public boolean getTestValidity() {
            return testValidity;
        }

        public boolean getTestResultValidity() {
            return testResultValidity;
        }

        public long getTestTimeStamp() {
            return testTimeStamp;
        }

        public boolean getTestResult() {
            return testResult;
        }

        public String getTestResultMessage() {
            return testResultMessage;
        }

        public ResultSeverity getTestSeverity() {
            return testResultSeverity;
        }

        public String getTestRecommendation() {
            return testRecommendation;
        }

        public TestType getTestType() {
            return testType;
        }

        public RunnableTest getTestRunnableTest() {
            return testRunMethod;
        }

    }

}
