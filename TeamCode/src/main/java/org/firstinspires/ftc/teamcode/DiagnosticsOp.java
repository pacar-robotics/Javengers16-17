package org.firstinspires.ftc.teamcode;

import android.os.Environment;
import android.util.Log;

import org.w3c.dom.Document;
import org.w3c.dom.NodeList;

import java.io.File;
import java.util.HashMap;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.xpath.XPath;
import javax.xml.xpath.XPathConstants;
import javax.xml.xpath.XPathExpression;
import javax.xml.xpath.XPathFactory;

/**
 * Created by Rahul on 11/27/2016.
 */

public class DiagnosticsOp extends vv_OpMode {


	// Code is based off of AutoXMLParser:
	// https://gist.github.com/rsquared226/21cf8b0d3e3476b38f22982f698d0388
	private static class XmlParser {
		private final String FILE_NAME = Environment.getExternalStorageDirectory().getPath() +
				"/PACAR/AutoChoices.xml";
		private final String LOG_TAG = "XmlParser";

		private HashMap<String, Boolean> choicesMap;

		public XmlParser() {
			choicesMap = parseXml();
		}

		private HashMap<String, Boolean> parseXml() {
			HashMap<String, Boolean> choicesMap = new HashMap<>();
			try {
				File xmlFile = new File(FILE_NAME);
				DocumentBuilderFactory documentBuilderFactory = DocumentBuilderFactory.newInstance();
				DocumentBuilder documentBuilder = documentBuilderFactory.newDocumentBuilder();

				// Attempt to parse XML into DOM structure, initializing document
				Document document = documentBuilder.parse(xmlFile);


				// Find choices in XML DOM using XPATH
				XPath xPath = XPathFactory.newInstance().newXPath();

				// xpression is used to identify nodes. Needs to be compiled and stored
				// Retrieve all elements that are children of Choices
				// TODO: Change root node in app. For now, node is AutoChoices because I forgot to change it
				XPathExpression expression = xPath.compile("/AutoChoices/*");

				NodeList nodes = (NodeList) expression.evaluate(document, XPathConstants.NODESET);

				for (int i = 0; i < nodes.getLength(); i++) {
					choicesMap.put(nodes.item(i).getNodeName(), Boolean.valueOf(nodes.item(i).getTextContent()));
				}

			} catch (Exception e) {
				Log.e(LOG_TAG, e.getMessage());
				return null;
			}

			return choicesMap;
		}

		public HashMap<String, Boolean> getChoicesMap() {
			return choicesMap;
		}
	}

	@Override
	public void runOpMode() throws InterruptedException {
		
	}
}
