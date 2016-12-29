package org.pacar_robotics.choicest;

import android.Manifest;
import android.app.Activity;
import android.content.pm.PackageManager;
import android.os.Build;
import android.support.annotation.NonNull;
import android.support.v4.app.ActivityCompat;
import android.support.v4.content.ContextCompat;
import android.util.Log;
import android.widget.Toast;

import org.w3c.dom.Document;
import org.w3c.dom.Element;

import java.io.File;
import java.util.HashMap;
import java.util.LinkedHashMap;
import java.util.Map;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.transform.OutputKeys;
import javax.xml.transform.Transformer;
import javax.xml.transform.TransformerFactory;
import javax.xml.transform.dom.DOMSource;
import javax.xml.transform.stream.StreamResult;

/**
 * Created by Rahul on 11/20/2016.
 */

class XmlWriter {

	private String filePath;
	private HashMap<String, String> autoChoicesMap;
	private LinkedHashMap<String, String> diagChoicesMap;
	private Activity activity;

	private static final int MY_PERMISSION_REQUEST_STORAGE = 101;
	private static final String LOG_TAG = "XmlWriter";

	public XmlWriter(String filePath, HashMap<String, String> autoChoicesMap,
	                 LinkedHashMap<String, String> diagChoicesMap, Activity activity) {
		this.filePath = filePath;
		this.autoChoicesMap = autoChoicesMap;
		this.diagChoicesMap = diagChoicesMap;
		this.activity = activity;

		getWritePermission();
	}

	private void writeXML() {
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
			Element root = doc.createElement("AutoChoices");
			doc.appendChild(root);

			for (Map.Entry<String, String> entry : (autoChoicesMap == null ? diagChoicesMap : autoChoicesMap).entrySet()) {
				Element i = doc.createElement(entry.getKey());
				i.appendChild(doc.createTextNode(entry.getValue()));
				root.appendChild(i);
			}

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
			Toast.makeText(activity, "Oh no! File creation failed!", Toast.LENGTH_LONG).show();
			Log.e(LOG_TAG, "File creation failed: " + e.getMessage());
		}
	}

	private void getWritePermission() {
		if ((Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) &&
				(ContextCompat.checkSelfPermission(activity, Manifest.permission.WRITE_EXTERNAL_STORAGE)
						!= PackageManager.PERMISSION_GRANTED)) {
			// If system OS is Marshmallow or greater, must check if have dangerous storage permission
			ActivityCompat.requestPermissions(activity, new String[]{Manifest.permission.WRITE_EXTERNAL_STORAGE},
					MY_PERMISSION_REQUEST_STORAGE);
		} else {
			// Permission is granted, move on
			Log.v(LOG_TAG, "External Storage Permission granted");
			writeXML();
			Log.v(LOG_TAG, "Finished writeXML function");
		}
	}

	public void onRequestPermissionResult(int requestCode, Activity activity,
	                                      @NonNull int[] grantResults) {
		switch (requestCode) {
			case MY_PERMISSION_REQUEST_STORAGE:
				if (grantResults[0] == PackageManager.PERMISSION_GRANTED) {
					// Permission has been granted
					// Go back into saveButtonClicked because it will go into else and save the file
					writeXML();
				} else {
					if (ActivityCompat.shouldShowRequestPermissionRationale(activity,
							Manifest.permission.WRITE_EXTERNAL_STORAGE)) {
						// Permission got denied but user did not check "do not show again"
						// They may have clicked the wrong button, so tell them they clicked the wrong one
						Toast.makeText(activity, "We need the storage permission to make an xml file\n" +
								"Please grant this permission", Toast.LENGTH_LONG).show();
					} else {
						// If they clicked "Do not show again", the user is not smart enough to be on this team
						Toast.makeText(activity, "You don't know how to use this app.\n" +
										"Ask Rahul the Tech Support God to help you",
								Toast.LENGTH_LONG).show();
					}
				}
				break;

			default:
				// There is no way the program can get to this statement
				// If it does, something is very, very wrong
				Log.e(LOG_TAG, "Reached default statement of onRequestPermissionResult");
				Toast.makeText(activity, "Something bad happened", Toast.LENGTH_SHORT).show();
				break;
		}
	}
}
