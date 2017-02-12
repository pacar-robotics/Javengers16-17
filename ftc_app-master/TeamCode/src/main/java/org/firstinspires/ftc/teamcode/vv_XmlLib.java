package org.firstinspires.ftc.teamcode;

//import java Libs we need to work with XML

import android.os.Environment;

import org.w3c.dom.Document;
import org.w3c.dom.Element;

import java.io.File;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import javax.xml.transform.Transformer;
import javax.xml.transform.TransformerConfigurationException;
import javax.xml.transform.TransformerException;
import javax.xml.transform.TransformerFactory;
import javax.xml.transform.dom.DOMSource;
import javax.xml.transform.stream.StreamResult;


public class vv_XmlLib {
    DocumentBuilderFactory docFactory;
    DocumentBuilder docBuilder;
    Document diagResultsDoc;
    Element diagResultsRoot;

    DOMSource diagResultsSource;
    StreamResult diagResultsResult;


    TransformerFactory transformerFactory;
    Transformer transformer;

    // root elements


    public vv_XmlLib(vv_OpMode aOpMode)
            throws InterruptedException {
        try {
            DocumentBuilderFactory docFactory = DocumentBuilderFactory.newInstance();
            DocumentBuilder docBuilder = docFactory.newDocumentBuilder();

            TransformerFactory transformerFactory = TransformerFactory.newInstance();
            Transformer transformer = transformerFactory.newTransformer();


        } catch (ParserConfigurationException PCE) {
            aOpMode.telemetryAddData("Parser Conf Error", "Parser", PCE.getMessage());
            aOpMode.telemetryUpdate();
            Thread.sleep(2000);

        } catch (TransformerConfigurationException TCE) {
            aOpMode.telemetryAddData("Parser Conf Error", "Parser", TCE.getMessage());
            aOpMode.telemetryUpdate();
            Thread.sleep(2000);

        }


    }

    protected void initDiagResultsXML(vv_OpMode aOpMode) {
        diagResultsDoc = docBuilder.newDocument();
        diagResultsRoot = diagResultsDoc.createElement("Velocity Vortex Results");
        diagResultsDoc.appendChild(diagResultsRoot);

        //get the current date and format it the way we want it.

        DateFormat df = new SimpleDateFormat("dd/MM/yy HH:mm:ss");
        Element timeStamp = diagResultsDoc.createElement(df.format(new Date()));

        //add the timestamp element into the XML tree
        diagResultsRoot.appendChild(timeStamp);

    }

    protected void addRobotTestResultXML(vv_OpMode aOpMode, vv_DiagLib.RobotTest robotTest) {
        //add the RobotTest result into the Results XML File.
        //first add a node for this robot test
        Element robotTestElement = diagResultsDoc.createElement("Robot Test");
        diagResultsRoot.appendChild(robotTestElement);

        //now add all the robot test elements under the Robot test element.

        //each element needs a new "XML Element" which creates the open and closed named tags
        //each element also needs a text node, which creates the actual values beween tags

        robotTestElement.appendChild(diagResultsDoc.createElement("TestId").
                appendChild(diagResultsDoc.createTextNode(String.valueOf(robotTest.getTestId(aOpMode)))));

        robotTestElement.appendChild(diagResultsDoc.createElement("TestName").
                appendChild(diagResultsDoc.createTextNode(robotTest.getTestName(aOpMode))));
        robotTestElement.appendChild(diagResultsDoc.createElement("TestShortDescription").
                appendChild(diagResultsDoc.createTextNode(robotTest.getTestShortDescription(aOpMode))));
        robotTestElement.appendChild(diagResultsDoc.createElement("TestLongDescription").
                appendChild(diagResultsDoc.createTextNode(robotTest.getTestLongDescription(aOpMode))));
        robotTestElement.appendChild(diagResultsDoc.createElement("TestResult").
                appendChild(diagResultsDoc.createTextNode(String.valueOf(robotTest.getTestResult(aOpMode)))));
        robotTestElement.appendChild(diagResultsDoc.createElement("TestResultMessage").
                appendChild(diagResultsDoc.createTextNode(robotTest.getTestResultMessage(aOpMode))));

        String Severity = null;
        switch (robotTest.getTestSeverity(aOpMode)) {
            case CRITICAL:
                Severity = "CRITICAL";
                break;
            case HIGH:
                Severity = "HIGH";
                break;
            case MEDIUM:
                Severity = "MEDIUM";
                break;
            case LOW:
                Severity = "LOW";
                break;
            case INFO:
                Severity = "INFO";
                break;
            default:
                Severity = "UNKNOWN";
        }
        robotTestElement.appendChild(diagResultsDoc.createElement("TestSeverity").
                appendChild(diagResultsDoc.createTextNode(Severity)));

        robotTestElement.appendChild(diagResultsDoc.createElement("TestRecommendation").
                appendChild(diagResultsDoc.createTextNode(String.valueOf(robotTest.getTestRecommendation(aOpMode)))));
    }


    protected void writeDiagResultsXML(vv_OpMode aOpMode) throws InterruptedException {
        //write out the DOM into an XML File.
        // write the content into xml file

        diagResultsSource = new DOMSource(diagResultsDoc);

        diagResultsResult =
                new StreamResult(new File(Environment.getExternalStorageDirectory().getPath() +
                        vv_Constants.DIAG_RESULTS_RELATIVE_FILE_PATH));
        try {
            transformer.transform(diagResultsSource, diagResultsResult);
        } catch (TransformerException TE) {
            aOpMode.telemetryAddData("XML Transformation Error", "Parser", TE.getMessage());
            aOpMode.telemetryUpdate();
            Thread.sleep(2000);
        }
    }


}
