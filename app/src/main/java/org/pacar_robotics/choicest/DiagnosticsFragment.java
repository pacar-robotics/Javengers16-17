package org.pacar_robotics.choicest;


import android.Manifest;
import android.content.pm.PackageManager;
import android.os.Bundle;
import android.os.Environment;
import android.support.annotation.NonNull;
import android.support.v4.app.ActivityCompat;
import android.support.v4.app.Fragment;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.CheckBox;
import android.widget.Toast;

import java.util.List;

import butterknife.BindView;
import butterknife.BindViews;
import butterknife.ButterKnife;
import butterknife.OnCheckedChanged;
import butterknife.OnClick;
import butterknife.Unbinder;

/**
 * A simple {@link Fragment} subclass.
 */
public class DiagnosticsFragment extends Fragment {

	private static final int MY_PERMISSION_REQUEST_STORAGE = 101;
	private static final String LOG_TAG = "AutonomousFragment";
	private static final String FILE = Environment.getExternalStorageDirectory().getPath() +
			"/PACAR/DiagChoices.xml";

	@BindView(R.id.motors_check_all) CheckBox checkAllMotors;
	@BindView(R.id.servos_check_all) CheckBox checkAllServos;
	@BindView(R.id.sensors_check_all) CheckBox checkAllSensors;

	@BindViews({R.id.wheels_motor, R.id.worm_drive_motor, R.id.launcher_motor, R.id.intake_motor, R.id.lift_motor})
	List<CheckBox> motorsList;
	@BindViews({R.id.intake_gate_servo, R.id.beacon_servo, R.id.capball_servo})
	List<CheckBox> servosList;
	@BindViews({R.id.floor_color_sensor, R.id.beacon_touch_sensor, R.id.launcher_limit_touch_sensor, R.id.lift_limit_touch_sensor, R.id.ultrasonic_sensor, R.id.gyro_sensor})
	List<CheckBox> sensorsList;

	private Unbinder unbinder;

	public DiagnosticsFragment() {
		// Required empty public constructor
	}

	@OnCheckedChanged(R.id.motors_check_all)
	void changeCheckAllMotors(CheckBox motorCheckAllBox) {
		if (motorCheckAllBox.isChecked()) {
			ButterKnife.apply(motorsList, CHECK);
		} else {
			ButterKnife.apply(motorsList, UNCHECK);
		}
	}
	@OnCheckedChanged(R.id.servos_check_all)
	void changeCheckAllServos(CheckBox servoCheckAllBox) {
		if (servoCheckAllBox.isChecked()) {
			ButterKnife.apply(servosList, CHECK);
		} else {
			ButterKnife.apply(servosList, UNCHECK);
		}
	}
	@OnCheckedChanged(R.id.sensors_check_all)
	void changeCheckAllSensors(CheckBox sensorCheckAllBox) {
		if (sensorCheckAllBox.isChecked()) {
			ButterKnife.apply(sensorsList, CHECK);
		} else {
			ButterKnife.apply(sensorsList, UNCHECK);
		}
	}

	@OnClick(R.id.motors_header)
	void changeCheckMotors() {
		checkAllMotors.setChecked(!checkAllMotors.isChecked());
	}
	@OnClick(R.id.servos_header)
	void changeCheckServos() {
		checkAllServos.setChecked(!checkAllServos.isChecked());
	}
	@OnClick(R.id.sensors_header)
	void changeCheckSensors() {
		checkAllSensors.setChecked(!checkAllSensors.isChecked());
	}

	static final ButterKnife.Action<CheckBox> CHECK = new ButterKnife.Action<CheckBox>() {
		@Override
		public void apply(@NonNull CheckBox checkBox, int index) {
			checkBox.setChecked(true);
		}
	};
	static final ButterKnife.Action<CheckBox> UNCHECK = new ButterKnife.Action<CheckBox>() {
		@Override
		public void apply(@NonNull CheckBox checkBox, int index) {
			checkBox.setChecked(false);
		}
	};

	@Override
	public View onCreateView(LayoutInflater inflater, ViewGroup container,
	                         Bundle savedInstanceState) {
		View rootView = inflater.inflate(R.layout.fragment_diagnostics, container, false);
		unbinder = ButterKnife.bind(this, rootView);
		return rootView;
	}

	@Override
	public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions,
	                                       @NonNull int[] grantResults) {
		super.onRequestPermissionsResult(requestCode, permissions, grantResults);

		switch (requestCode) {
			case MY_PERMISSION_REQUEST_STORAGE:
				if (grantResults[0] == PackageManager.PERMISSION_GRANTED) {
					// Permission has been granted
					// Go back into saveButtonClicked because it will go into else and save the file
					// TODO: Update this: saveButtonClicked();
				} else {
					if (ActivityCompat.shouldShowRequestPermissionRationale(getActivity(),
							Manifest.permission.WRITE_EXTERNAL_STORAGE)) {
						// Permission got denied but user did not check "do not show again"
						// They may have clicked the wrong button, so tell them they clicked the wrong one
						Toast.makeText(getActivity(), "We need the storage permission to make an xml file\n" +
								"Please grant this permission", Toast.LENGTH_LONG).show();
					} else {
						// If they clicked "Do not show again", the user is not smart enough to be on this team
						Toast.makeText(getActivity(), "You don't know how to use this app.\n" +
										"Ask Rahul the Tech Support God to help you",
								Toast.LENGTH_LONG).show();
					}
				}
				break;

			default:
				// There is no way the program can get to this statement
				// If it does, something is very, very wrong
				Log.e(LOG_TAG, "Reached default statement of onRequestPermissionResult");
				Toast.makeText(getActivity(), "Something bad happened", Toast.LENGTH_SHORT).show();
				break;
		}
	}

	@Override
	public void onDestroyView() {
		super.onDestroyView();
		unbinder.unbind();
	}
}
