package org.pacar_robotics.choicest;


import android.os.Bundle;
import android.os.Environment;
import android.support.annotation.NonNull;
import android.support.v4.app.Fragment;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.CheckBox;

import java.util.HashMap;
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

	private static final String LOG_TAG = "AutonomousFragment";
	private static final String FILE = Environment.getExternalStorageDirectory().getPath() +
			"/PACAR/DiagChoices.xml";

	@BindView(R.id.motors_check_all) CheckBox checkAllMotors;
	@BindView(R.id.base_motor_check_all) CheckBox checkAllBaseMotors;
	@BindView(R.id.platform_movement_check_all) CheckBox checkAllPlatformMovement;
	@BindView(R.id.servos_check_all) CheckBox checkAllServos;
	@BindView(R.id.sensors_check_all) CheckBox checkAllSensors;

	@BindViews({R.id.front_right_wheel, R.id.front_left_wheel, R.id.back_right_wheel, R.id.back_left_wheel,
			R.id.forwards_platform, R.id.backwards_platform, R.id.right_platform, R.id.left_platform,
			R.id.worm_drive_motor, R.id.launcher_motor, R.id.intake_motor, R.id.lift_motor})
	List<CheckBox> motorsList;
	@BindViews({R.id.front_right_wheel, R.id.front_left_wheel, R.id.back_right_wheel, R.id.back_left_wheel})
	List<CheckBox> baseMotors;
	@BindViews({R.id.forwards_platform, R.id.backwards_platform, R.id.right_platform, R.id.left_platform})
	List<CheckBox> platformMotors;
	@BindViews({R.id.intake_gate_servo, R.id.beacon_servo, R.id.capball_servo})
	List<CheckBox> servosList;
	@BindViews({R.id.floor_color_sensor, R.id.beacon_touch_sensor, R.id.launcher_limit_touch_sensor, R.id.lift_limit_touch_sensor, R.id.ultrasonic_sensor, R.id.gyro_sensor})
	List<CheckBox> sensorsList;

	XmlWriter xmlWriter;

	private Unbinder unbinder;

	public DiagnosticsFragment() {
		// Required empty public constructor
	}

	@OnClick(R.id.btn_save)
	void saveButtonClicked() {
		xmlWriter = new XmlWriter(FILE, createHashMap(), getActivity());
	}

	private HashMap<String, String> createHashMap() {
		HashMap<String, String> choicesMap = new HashMap<>();

		choicesMap.putAll(addListsToMap(motorsList));
		choicesMap.putAll(addListsToMap(servosList));
		choicesMap.putAll(addListsToMap(sensorsList));

		return choicesMap;
	}

	private HashMap<String, String> addListsToMap(List<CheckBox> list) {
		HashMap<String, String> hashMap = new HashMap<>();

		for (CheckBox checkBox : list) {
			hashMap.put(checkBox.getText().toString().replace(" ", "").toLowerCase(),
					Boolean.toString(checkBox.isChecked()));
		}

		return hashMap;
	}


	@OnCheckedChanged(R.id.motors_check_all)
	void changeCheckAllMotors(CheckBox motorCheckAllBox) {
		if (motorCheckAllBox.isChecked()) {
			ButterKnife.apply(motorsList, CHECK);
			checkAllBaseMotors.setChecked(true);
			checkAllPlatformMovement.setChecked(true);
		} else {
			ButterKnife.apply(motorsList, UNCHECK);
			checkAllBaseMotors.setChecked(false);
			checkAllPlatformMovement.setChecked(false);
		}
	}
	@OnCheckedChanged(R.id.base_motor_check_all)
	void changeCheckAllBaseMotors(CheckBox baseMotorCheckAllBox) {
		if (baseMotorCheckAllBox.isChecked()) {
			ButterKnife.apply(baseMotors, CHECK);
		} else {
			ButterKnife.apply(baseMotors, UNCHECK);
		}
	}
	@OnCheckedChanged(R.id.platform_movement_check_all)
	void changeCheckAllPlatformMovement(CheckBox platformMovementCheckAll) {
		if (platformMovementCheckAll.isChecked()) {
			ButterKnife.apply(platformMotors, CHECK);
		} else {
			ButterKnife.apply(platformMotors, UNCHECK);
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

		xmlWriter.onRequestPermissionResult(requestCode, getActivity(), grantResults);
	}

	@Override
	public void onDestroyView() {
		super.onDestroyView();
		unbinder.unbind();
	}
}
