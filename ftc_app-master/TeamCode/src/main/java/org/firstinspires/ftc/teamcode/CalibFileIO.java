package org.firstinspires.ftc.teamcode;

import android.os.Environment;
import android.util.Log;

import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.NodeList;
import org.xml.sax.SAXException;

import java.io.File;
import java.io.IOException;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import javax.xml.transform.OutputKeys;
import javax.xml.transform.Transformer;
import javax.xml.transform.TransformerFactory;
import javax.xml.transform.dom.DOMSource;
import javax.xml.transform.stream.StreamResult;
import javax.xml.xpath.XPath;
import javax.xml.xpath.XPathConstants;
import javax.xml.xpath.XPathExpression;
import javax.xml.xpath.XPathExpressionException;
import javax.xml.xpath.XPathFactory;

/**
 * Created by Rahul on 1/27/2017.
 */

public class CalibFileIO {
	private String filePath = Environment.getExternalStorageDirectory().getPath() +
			"/PACAR/Calib.xml";
	private static final String LOG_TAG = "CalibFileIO";

	public CalibFileIO(String fileName) {
		filePath = filePath.replace("Calib", fileName);
	}

	public void writeTextFile(float automaticFloorLightCalibrationValue) throws IOException {
		File file = new File(filePath);

		if (file.delete()) {
			Log.v(LOG_TAG, " File deleted:" + filePath);
		} else {
			Log.v(LOG_TAG, " File NOT deleted:" + filePath);
		}

		try {
			DocumentBuilderFactory docFactory = DocumentBuilderFactory.newInstance();
			DocumentBuilder docBuilder = docFactory.newDocumentBuilder();

			// Create root elements
			Document doc = docBuilder.newDocument();
			Element root = doc.createElement("LightCalib");
			doc.appendChild(root);

			Element i = doc.createElement("ICanSeeTheLight");
			i.appendChild(doc.createTextNode(Float.toString(automaticFloorLightCalibrationValue)));
			root.appendChild(i);

			// Write the content into xml file
			TransformerFactory transformerFactory = TransformerFactory.newInstance();
			Transformer transformer = transformerFactory.newTransformer();

			DOMSource source = new DOMSource(doc);

			StreamResult result = new StreamResult(file);

			// Fix XML formatting
			transformer.setOutputProperty(OutputKeys.INDENT, "yes");
			transformer.setOutputProperty("{http://xml.apache.org/xslt}indent-amount", "2");

			transformer.transform(source, result);
		} catch (Exception e) {
			Log.e(LOG_TAG, "File creation failed: " + e.getMessage());
			throw new IOException("Didn't write file or something");
		}
	}


	public float getCalibrationValue() throws IOException, NumberFormatException {
		String result = "null";

		try {
			File xmlFile = new File(filePath);
			DocumentBuilderFactory documentBuilderFactory = DocumentBuilderFactory.newInstance();
			DocumentBuilder documentBuilder = documentBuilderFactory.newDocumentBuilder();

			// Attempt to parse XML into DOM structure, initializing document
			Document document = documentBuilder.parse(xmlFile);

			// Find choices in XML DOM using XPATH
			XPath xPath = XPathFactory.newInstance().newXPath();

			// xpression is used to identify nodes. Needs to be compiled and stored
			XPathExpression expression = xPath.compile("/LightCalib/*");

			NodeList nodes = (NodeList) expression.evaluate(document, XPathConstants.NODESET);

			for (int i = 0; i < nodes.getLength(); i++) {
				result = nodes.item(i).getTextContent();
			}

			return Float.parseFloat(result);
		} catch (NumberFormatException e) {
			throw new NumberFormatException("Float Parsing of Node Item failed:" + result);
		} catch (SAXException e) {
			throw new IOException("Something with file reading went wrong");
		} catch (XPathExpressionException e) {
			throw new IOException("Something with file reading went wrong");
		} catch (ParserConfigurationException e) {
			throw new IOException("Something with file reading went wrong");
		} catch (IOException e) {
			throw new IOException("Something with file reading went wrong");
		}
	}
}
