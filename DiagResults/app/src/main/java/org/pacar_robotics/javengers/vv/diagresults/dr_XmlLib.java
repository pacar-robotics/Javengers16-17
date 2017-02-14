package org.pacar_robotics.javengers.vv.diagresults;

//import java Libs we need to work with XML

import android.os.Environment;

import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;
import org.xml.sax.SAXException;

import java.io.File;
import java.io.IOException;
import java.io.UnsupportedEncodingException;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.Date;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import javax.xml.transform.OutputKeys;
import javax.xml.transform.Transformer;
import javax.xml.transform.TransformerConfigurationException;
import javax.xml.transform.TransformerException;
import javax.xml.transform.TransformerFactory;
import javax.xml.transform.dom.DOMSource;
import javax.xml.transform.stream.StreamResult;


public class dr_XmlLib {
    DocumentBuilderFactory docFactory;
    DocumentBuilder docBuilder;
    Document diagResultsDoc;
    Element diagResultsRoot;
    Element diagResultsAutomaticRoot;
    Element diagResultsManualRoot;

    DOMSource diagResultsSource;
    StreamResult diagResultsResult;


    TransformerFactory transformerFactory;
    Transformer transformer;

    // root elements


    public dr_XmlLib()
            throws InterruptedException {
        try {
            docFactory = DocumentBuilderFactory.newInstance();
            docBuilder = docFactory.newDocumentBuilder();

            transformerFactory = TransformerFactory.newInstance();
            transformer = transformerFactory.newTransformer();


        } catch (ParserConfigurationException PCE) {
            
            Thread.sleep(2000);

        } catch (TransformerConfigurationException TCE) {
            
            Thread.sleep(2000);

        }


    }

    protected void initDiagResultsXmlForWrite() {
        diagResultsDoc = docBuilder.newDocument();
        diagResultsRoot = diagResultsDoc.createElement("VelocityVortexResults");
        diagResultsDoc.appendChild(diagResultsRoot);


        diagResultsAutomaticRoot = diagResultsDoc.createElement("Automatic");
        diagResultsManualRoot = diagResultsDoc.createElement("Manual");
        //add the timestamp element into the XML tree

        //get the current date and format it the way we want it.

        DateFormat df = new SimpleDateFormat("dd/MM/yy HH:mm:ss");

        diagResultsDoc.appendChild(diagResultsDoc.createElement("TimeStamp").
                appendChild(diagResultsDoc.createTextNode(df.format(new Date()))));

        diagResultsRoot.appendChild(diagResultsAutomaticRoot);
        diagResultsRoot.appendChild(diagResultsManualRoot);


    }

    protected void initDiagResultsXmlForRead() throws InterruptedException {

        try {
            //get the file handle to the results file.

            File xmlFile = new File(Environment.getExternalStorageDirectory().getPath() +
                    dr_Constants.DIAG_RESULTS_RELATIVE_FILE_PATH);
            //create a XML DOM document from file
            //and attach it to the results doc handle that is in the class.
            diagResultsDoc = docBuilder.parse(xmlFile);

        } catch (UnsupportedEncodingException UEE) {
            
            Thread.sleep(2000);

        } catch (SAXException SE) {
           
            Thread.sleep(2000);
        } catch (IOException IE) {
           
            Thread.sleep(2000);
        }

    }

    protected NodeList getRobotTestNodesFromDom() {
        //get and return list of Robot Test nodes
        //if these tags are not matched this may return null!

        return diagResultsDoc.getElementsByTagName("RobotTest");
    }


    protected void addRobotTestResultToDom(dr_DiagLib.RobotTest robotTest) {
        //add the RobotTest result into the Results XML File.
        //first add a node for this robot test
        Element robotTestElement = diagResultsDoc.createElement("RobotTest");
        //attach to either the Automatic or Manual Results tree.

        if (robotTest.getTestType() == dr_DiagLib.TestType.AUTOMATIC) {
            diagResultsAutomaticRoot.appendChild(robotTestElement);
        } else {
            diagResultsManualRoot.appendChild(robotTestElement);
        }

        //now add all the robot test elements under the Robot test element.

        //each element needs a new "XML Element" which creates the open and closed named tags
        //each element also needs a text node, which creates the actual values beween tags

        Element TestId = diagResultsDoc.createElement("TestId");
        TestId.appendChild(diagResultsDoc.createTextNode(String.valueOf(robotTest.getTestId())));
        robotTestElement.appendChild(TestId);

        Element TestName = diagResultsDoc.createElement("TestName");
        TestName.appendChild(diagResultsDoc.createTextNode(robotTest.getTestName()));
        robotTestElement.appendChild(TestName);

        Element TestShortDescription = diagResultsDoc.createElement("TestShortDescription");
        TestShortDescription.appendChild(diagResultsDoc.createTextNode(robotTest.getTestShortDescription()));
        robotTestElement.appendChild(TestShortDescription);

        Element TestLongDescription = diagResultsDoc.createElement("TestLongDescription");
        TestLongDescription.appendChild(diagResultsDoc.createTextNode(robotTest.getTestLongDescription()));
        robotTestElement.appendChild(TestLongDescription);

        Element TestResult = diagResultsDoc.createElement("TestResult");
        TestResult.appendChild(diagResultsDoc.
                createTextNode(robotTest.getTestResult() ? "Passed" : "Failed"));
        robotTestElement.appendChild(TestResult);

        Element TestResultMessage = diagResultsDoc.createElement("TestResultMessage");
        TestResultMessage.appendChild(diagResultsDoc.createTextNode(robotTest.getTestResultMessage()));
        robotTestElement.appendChild(TestResultMessage);


        String Severity = null;
        switch (robotTest.getTestSeverity()) {
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

        Element TestResultSeverity = diagResultsDoc.createElement("TestResultSeverity");
        TestResultSeverity.appendChild(diagResultsDoc.createTextNode(Severity));
        robotTestElement.appendChild(TestResultSeverity);

        Element TestRecommendation = diagResultsDoc.createElement("TestRecommendation");
        TestRecommendation.appendChild(diagResultsDoc.createTextNode(robotTest.getTestRecommendation()));
        robotTestElement.appendChild(TestRecommendation);

    }


    protected void writeDiagResultsXML() throws InterruptedException {
        //write out the DOM into an XML File.
        // write the content into xml file

        diagResultsSource = new DOMSource(diagResultsDoc);

        File xmlFile = new File(Environment.getExternalStorageDirectory().getPath() +
                dr_Constants.DIAG_RESULTS_RELATIVE_FILE_PATH);

        xmlFile.delete();

        diagResultsResult =
                new StreamResult(xmlFile);
        try {
            // Fix XML formatting
            transformer.setOutputProperty(OutputKeys.INDENT, "yes");
            transformer.setOutputProperty("{http://xml.apache.org/xslt}indent-amount", "2");
            transformer.transform(diagResultsSource, diagResultsResult);

            Thread.sleep(5000);
        } catch (TransformerException TE) {

            Thread.sleep(2000);
        }
    }


}
