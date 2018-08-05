#-------------------------------------------------
#
# Project created by QtCreator 2018-01-09T14:13:32
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets  printsupport

TARGET = MyCom
TEMPLATE = app

//LIBS += "C:/Qt/Qt5.7.1/ctrl/Qt5PrintSupport.dll"
# The following define makes your compiler emit warnings if you use
# any feature of Qt which as been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0


SOURCES += main.cpp\
        mainwindow.cpp \
    qextserial/qextserialport.cpp \
    qcustomplot/qcustomplot.cpp \
    aboutdialog.cpp \
    aboutdialog.cpp \
    instrument/altimetro.cpp \
    instrument/qadi.cpp \
    instrument/qcompass.cpp

HEADERS  += mainwindow.h \
    qextserial/qextserialport.h \
    qextserial/qextserialport_global.h \
    qextserial/qextserialport_p.h \
    aboutdialog.h \
    aboutdialog.h \
    qcustomplot/qcustomplot.h \
    instrument/altimetro.h \
    instrument/qadi.h \
    instrument/qcompass.h \
    mavlink/v1.0/checksum.h \
    mavlink/v1.0/mavlink_conversions.h \
    mavlink/v1.0/mavlink_helpers.h \
    mavlink/v1.0/mavlink_types.h \
    mavlink/v1.0/protocol.h \
    mavlink/v1.0/common/common.h \
    mavlink/v1.0/common/mavlink.h \
    mavlink/v1.0/common/mavlink_msg_actuator_control_target.h \
    mavlink/v1.0/common/mavlink_msg_adsb_vehicle.h \
    mavlink/v1.0/common/mavlink_msg_altitude.h \
    mavlink/v1.0/common/mavlink_msg_att_pos_mocap.h \
    mavlink/v1.0/common/mavlink_msg_attitude.h \
    mavlink/v1.0/common/mavlink_msg_attitude_quaternion.h \
    mavlink/v1.0/common/mavlink_msg_attitude_quaternion_cov.h \
    mavlink/v1.0/common/mavlink_msg_attitude_target.h \
    mavlink/v1.0/common/mavlink_msg_auth_key.h \
    mavlink/v1.0/common/mavlink_msg_autopilot_version.h \
    mavlink/v1.0/common/mavlink_msg_battery_status.h \
    mavlink/v1.0/common/mavlink_msg_camera_trigger.h \
    mavlink/v1.0/common/mavlink_msg_change_operator_control.h \
    mavlink/v1.0/common/mavlink_msg_change_operator_control_ack.h \
    mavlink/v1.0/common/mavlink_msg_collision.h \
    mavlink/v1.0/common/mavlink_msg_command_ack.h \
    mavlink/v1.0/common/mavlink_msg_command_int.h \
    mavlink/v1.0/common/mavlink_msg_command_long.h \
    mavlink/v1.0/common/mavlink_msg_control_system_state.h \
    mavlink/v1.0/common/mavlink_msg_data_stream.h \
    mavlink/v1.0/common/mavlink_msg_data_transmission_handshake.h \
    mavlink/v1.0/common/mavlink_msg_debug.h \
    mavlink/v1.0/common/mavlink_msg_debug_vect.h \
    mavlink/v1.0/common/mavlink_msg_distance_sensor.h \
    mavlink/v1.0/common/mavlink_msg_encapsulated_data.h \
    mavlink/v1.0/common/mavlink_msg_estimator_status.h \
    mavlink/v1.0/common/mavlink_msg_extended_sys_state.h \
    mavlink/v1.0/common/mavlink_msg_file_transfer_protocol.h \
    mavlink/v1.0/common/mavlink_msg_follow_target.h \
    mavlink/v1.0/common/mavlink_msg_global_position_int.h \
    mavlink/v1.0/common/mavlink_msg_global_position_int_cov.h \
    mavlink/v1.0/common/mavlink_msg_global_vision_position_estimate.h \
    mavlink/v1.0/common/mavlink_msg_gps_global_origin.h \
    mavlink/v1.0/common/mavlink_msg_gps_inject_data.h \
    mavlink/v1.0/common/mavlink_msg_gps_input.h \
    mavlink/v1.0/common/mavlink_msg_gps_raw_int.h \
    mavlink/v1.0/common/mavlink_msg_gps_rtcm_data.h \
    mavlink/v1.0/common/mavlink_msg_gps_rtk.h \
    mavlink/v1.0/common/mavlink_msg_gps_status.h \
    mavlink/v1.0/common/mavlink_msg_gps2_raw.h \
    mavlink/v1.0/common/mavlink_msg_gps2_rtk.h \
    mavlink/v1.0/common/mavlink_msg_heartbeat.h \
    mavlink/v1.0/common/mavlink_msg_high_latency.h \
    mavlink/v1.0/common/mavlink_msg_highres_imu.h \
    mavlink/v1.0/common/mavlink_msg_hil_actuator_controls.h \
    mavlink/v1.0/common/mavlink_msg_hil_controls.h \
    mavlink/v1.0/common/mavlink_msg_hil_gps.h \
    mavlink/v1.0/common/mavlink_msg_hil_optical_flow.h \
    mavlink/v1.0/common/mavlink_msg_hil_rc_inputs_raw.h \
    mavlink/v1.0/common/mavlink_msg_hil_sensor.h \
    mavlink/v1.0/common/mavlink_msg_hil_state.h \
    mavlink/v1.0/common/mavlink_msg_hil_state_quaternion.h \
    mavlink/v1.0/common/mavlink_msg_home_position.h \
    mavlink/v1.0/common/mavlink_msg_landing_target.h \
    mavlink/v1.0/common/mavlink_msg_local_position_ned.h \
    mavlink/v1.0/common/mavlink_msg_local_position_ned_cov.h \
    mavlink/v1.0/common/mavlink_msg_local_position_ned_system_global_offset.h \
    mavlink/v1.0/common/mavlink_msg_log_data.h \
    mavlink/v1.0/common/mavlink_msg_log_entry.h \
    mavlink/v1.0/common/mavlink_msg_log_erase.h \
    mavlink/v1.0/common/mavlink_msg_log_request_data.h \
    mavlink/v1.0/common/mavlink_msg_log_request_end.h \
    mavlink/v1.0/common/mavlink_msg_log_request_list.h \
    mavlink/v1.0/common/mavlink_msg_manual_control.h \
    mavlink/v1.0/common/mavlink_msg_manual_setpoint.h \
    mavlink/v1.0/common/mavlink_msg_memory_vect.h \
    mavlink/v1.0/common/mavlink_msg_message_interval.h \
    mavlink/v1.0/common/mavlink_msg_mission_ack.h \
    mavlink/v1.0/common/mavlink_msg_mission_clear_all.h \
    mavlink/v1.0/common/mavlink_msg_mission_count.h \
    mavlink/v1.0/common/mavlink_msg_mission_current.h \
    mavlink/v1.0/common/mavlink_msg_mission_item.h \
    mavlink/v1.0/common/mavlink_msg_mission_item_int.h \
    mavlink/v1.0/common/mavlink_msg_mission_item_reached.h \
    mavlink/v1.0/common/mavlink_msg_mission_request.h \
    mavlink/v1.0/common/mavlink_msg_mission_request_int.h \
    mavlink/v1.0/common/mavlink_msg_mission_request_list.h \
    mavlink/v1.0/common/mavlink_msg_mission_request_partial_list.h \
    mavlink/v1.0/common/mavlink_msg_mission_set_current.h \
    mavlink/v1.0/common/mavlink_msg_mission_write_partial_list.h \
    mavlink/v1.0/common/mavlink_msg_named_value_float.h \
    mavlink/v1.0/common/mavlink_msg_named_value_int.h \
    mavlink/v1.0/common/mavlink_msg_nav_controller_output.h \
    mavlink/v1.0/common/mavlink_msg_optical_flow.h \
    mavlink/v1.0/common/mavlink_msg_optical_flow_rad.h \
    mavlink/v1.0/common/mavlink_msg_param_map_rc.h \
    mavlink/v1.0/common/mavlink_msg_param_request_list.h \
    mavlink/v1.0/common/mavlink_msg_param_request_read.h \
    mavlink/v1.0/common/mavlink_msg_param_set.h \
    mavlink/v1.0/common/mavlink_msg_param_value.h \
    mavlink/v1.0/common/mavlink_msg_ping.h \
    mavlink/v1.0/common/mavlink_msg_position_target_global_int.h \
    mavlink/v1.0/common/mavlink_msg_position_target_local_ned.h \
    mavlink/v1.0/common/mavlink_msg_power_status.h \
    mavlink/v1.0/common/mavlink_msg_radio_status.h \
    mavlink/v1.0/common/mavlink_msg_raw_imu.h \
    mavlink/v1.0/common/mavlink_msg_raw_pressure.h \
    mavlink/v1.0/common/mavlink_msg_rc_channels.h \
    mavlink/v1.0/common/mavlink_msg_rc_channels_override.h \
    mavlink/v1.0/common/mavlink_msg_rc_channels_raw.h \
    mavlink/v1.0/common/mavlink_msg_rc_channels_scaled.h \
    mavlink/v1.0/common/mavlink_msg_request_data_stream.h \
    mavlink/v1.0/common/mavlink_msg_resource_request.h \
    mavlink/v1.0/common/mavlink_msg_safety_allowed_area.h \
    mavlink/v1.0/common/mavlink_msg_safety_set_allowed_area.h \
    mavlink/v1.0/common/mavlink_msg_scaled_imu.h \
    mavlink/v1.0/common/mavlink_msg_scaled_imu2.h \
    mavlink/v1.0/common/mavlink_msg_scaled_imu3.h \
    mavlink/v1.0/common/mavlink_msg_scaled_pressure.h \
    mavlink/v1.0/common/mavlink_msg_scaled_pressure2.h \
    mavlink/v1.0/common/mavlink_msg_scaled_pressure3.h \
    mavlink/v1.0/common/mavlink_msg_serial_control.h \
    mavlink/v1.0/common/mavlink_msg_servo_output_raw.h \
    mavlink/v1.0/common/mavlink_msg_set_actuator_control_target.h \
    mavlink/v1.0/common/mavlink_msg_set_attitude_target.h \
    mavlink/v1.0/common/mavlink_msg_set_gps_global_origin.h \
    mavlink/v1.0/common/mavlink_msg_set_home_position.h \
    mavlink/v1.0/common/mavlink_msg_set_mode.h \
    mavlink/v1.0/common/mavlink_msg_set_position_target_global_int.h \
    mavlink/v1.0/common/mavlink_msg_set_position_target_local_ned.h \
    mavlink/v1.0/common/mavlink_msg_sim_state.h \
    mavlink/v1.0/common/mavlink_msg_statustext.h \
    mavlink/v1.0/common/mavlink_msg_sys_status.h \
    mavlink/v1.0/common/mavlink_msg_system_time.h \
    mavlink/v1.0/common/mavlink_msg_terrain_check.h \
    mavlink/v1.0/common/mavlink_msg_terrain_data.h \
    mavlink/v1.0/common/mavlink_msg_terrain_report.h \
    mavlink/v1.0/common/mavlink_msg_terrain_request.h \
    mavlink/v1.0/common/mavlink_msg_timesync.h \
    mavlink/v1.0/common/mavlink_msg_v2_extension.h \
    mavlink/v1.0/common/mavlink_msg_vfr_hud.h \
    mavlink/v1.0/common/mavlink_msg_vibration.h \
    mavlink/v1.0/common/mavlink_msg_vicon_position_estimate.h \
    mavlink/v1.0/common/mavlink_msg_vision_position_estimate.h \
    mavlink/v1.0/common/mavlink_msg_vision_speed_estimate.h \
    mavlink/v1.0/common/mavlink_msg_wind_cov.h \
    mavlink/v1.0/common/testsuite.h \
    mavlink/v1.0/common/version.h \
    mavlink/v1.0/matrixpilot/matrixpilot.h \
    mavlink/v1.0/matrixpilot/mavlink.h \
    mavlink/v1.0/matrixpilot/mavlink_msg_airspeeds.h \
    mavlink/v1.0/matrixpilot/mavlink_msg_altitudes.h \
    mavlink/v1.0/matrixpilot/mavlink_msg_flexifunction_buffer_function.h \
    mavlink/v1.0/matrixpilot/mavlink_msg_flexifunction_buffer_function_ack.h \
    mavlink/v1.0/matrixpilot/mavlink_msg_flexifunction_command.h \
    mavlink/v1.0/matrixpilot/mavlink_msg_flexifunction_command_ack.h \
    mavlink/v1.0/matrixpilot/mavlink_msg_flexifunction_directory.h \
    mavlink/v1.0/matrixpilot/mavlink_msg_flexifunction_directory_ack.h \
    mavlink/v1.0/matrixpilot/mavlink_msg_flexifunction_read_req.h \
    mavlink/v1.0/matrixpilot/mavlink_msg_flexifunction_set.h \
    mavlink/v1.0/matrixpilot/mavlink_msg_serial_udb_extra_f2_a.h \
    mavlink/v1.0/matrixpilot/mavlink_msg_serial_udb_extra_f2_b.h \
    mavlink/v1.0/matrixpilot/mavlink_msg_serial_udb_extra_f4.h \
    mavlink/v1.0/matrixpilot/mavlink_msg_serial_udb_extra_f5.h \
    mavlink/v1.0/matrixpilot/mavlink_msg_serial_udb_extra_f6.h \
    mavlink/v1.0/matrixpilot/mavlink_msg_serial_udb_extra_f7.h \
    mavlink/v1.0/matrixpilot/mavlink_msg_serial_udb_extra_f8.h \
    mavlink/v1.0/matrixpilot/mavlink_msg_serial_udb_extra_f13.h \
    mavlink/v1.0/matrixpilot/mavlink_msg_serial_udb_extra_f14.h \
    mavlink/v1.0/matrixpilot/mavlink_msg_serial_udb_extra_f15.h \
    mavlink/v1.0/matrixpilot/mavlink_msg_serial_udb_extra_f16.h \
    mavlink/v1.0/matrixpilot/testsuite.h \
    mavlink/v1.0/matrixpilot/version.h \
    mavlink/v1.0/ardupilotmega/ardupilotmega.h \
    mavlink/v1.0/ardupilotmega/mavlink.h \
    mavlink/v1.0/ardupilotmega/mavlink_msg_ahrs.h \
    mavlink/v1.0/ardupilotmega/mavlink_msg_ahrs2.h \
    mavlink/v1.0/ardupilotmega/mavlink_msg_ahrs3.h \
    mavlink/v1.0/ardupilotmega/mavlink_msg_airspeed_autocal.h \
    mavlink/v1.0/ardupilotmega/mavlink_msg_ap_adc.h \
    mavlink/v1.0/ardupilotmega/mavlink_msg_autopilot_version_request.h \
    mavlink/v1.0/ardupilotmega/mavlink_msg_battery2.h \
    mavlink/v1.0/ardupilotmega/mavlink_msg_camera_feedback.h \
    mavlink/v1.0/ardupilotmega/mavlink_msg_camera_status.h \
    mavlink/v1.0/ardupilotmega/mavlink_msg_compassmot_status.h \
    mavlink/v1.0/ardupilotmega/mavlink_msg_data16.h \
    mavlink/v1.0/ardupilotmega/mavlink_msg_data32.h \
    mavlink/v1.0/ardupilotmega/mavlink_msg_data64.h \
    mavlink/v1.0/ardupilotmega/mavlink_msg_data96.h \
    mavlink/v1.0/ardupilotmega/mavlink_msg_digicam_configure.h \
    mavlink/v1.0/ardupilotmega/mavlink_msg_digicam_control.h \
    mavlink/v1.0/ardupilotmega/mavlink_msg_ekf_status_report.h \
    mavlink/v1.0/ardupilotmega/mavlink_msg_fence_fetch_point.h \
    mavlink/v1.0/ardupilotmega/mavlink_msg_fence_point.h \
    mavlink/v1.0/ardupilotmega/mavlink_msg_fence_status.h \
    mavlink/v1.0/ardupilotmega/mavlink_msg_gimbal_axis_calibration_progress.h \
    mavlink/v1.0/ardupilotmega/mavlink_msg_gimbal_control.h \
    mavlink/v1.0/ardupilotmega/mavlink_msg_gimbal_erase_firmware_and_config.h \
    mavlink/v1.0/ardupilotmega/mavlink_msg_gimbal_factory_parameters_loaded.h \
    mavlink/v1.0/ardupilotmega/mavlink_msg_gimbal_home_offset_calibration_result.h \
    mavlink/v1.0/ardupilotmega/mavlink_msg_gimbal_perform_factory_tests.h \
    mavlink/v1.0/ardupilotmega/mavlink_msg_gimbal_report.h \
    mavlink/v1.0/ardupilotmega/mavlink_msg_gimbal_report_axis_calibration_status.h \
    mavlink/v1.0/ardupilotmega/mavlink_msg_gimbal_report_factory_tests_progress.h \
    mavlink/v1.0/ardupilotmega/mavlink_msg_gimbal_request_axis_calibration.h \
    mavlink/v1.0/ardupilotmega/mavlink_msg_gimbal_request_axis_calibration_status.h \
    mavlink/v1.0/ardupilotmega/mavlink_msg_gimbal_reset.h \
    mavlink/v1.0/ardupilotmega/mavlink_msg_gimbal_set_factory_parameters.h \
    mavlink/v1.0/ardupilotmega/mavlink_msg_gimbal_set_home_offsets.h \
    mavlink/v1.0/ardupilotmega/mavlink_msg_gimbal_torque_cmd_report.h \
    mavlink/v1.0/ardupilotmega/mavlink_msg_gopro_get_request.h \
    mavlink/v1.0/ardupilotmega/mavlink_msg_gopro_get_response.h \
    mavlink/v1.0/ardupilotmega/mavlink_msg_gopro_heartbeat.h \
    mavlink/v1.0/ardupilotmega/mavlink_msg_gopro_set_request.h \
    mavlink/v1.0/ardupilotmega/mavlink_msg_gopro_set_response.h \
    mavlink/v1.0/ardupilotmega/mavlink_msg_hwstatus.h \
    mavlink/v1.0/ardupilotmega/mavlink_msg_led_control.h \
    mavlink/v1.0/ardupilotmega/mavlink_msg_limits_status.h \
    mavlink/v1.0/ardupilotmega/mavlink_msg_mag_cal_progress.h \
    mavlink/v1.0/ardupilotmega/mavlink_msg_mag_cal_report.h \
    mavlink/v1.0/ardupilotmega/mavlink_msg_meminfo.h \
    mavlink/v1.0/ardupilotmega/mavlink_msg_mount_configure.h \
    mavlink/v1.0/ardupilotmega/mavlink_msg_mount_control.h \
    mavlink/v1.0/ardupilotmega/mavlink_msg_mount_status.h \
    mavlink/v1.0/ardupilotmega/mavlink_msg_pid_tuning.h \
    mavlink/v1.0/ardupilotmega/mavlink_msg_radio.h \
    mavlink/v1.0/ardupilotmega/mavlink_msg_rally_fetch_point.h \
    mavlink/v1.0/ardupilotmega/mavlink_msg_rally_point.h \
    mavlink/v1.0/ardupilotmega/mavlink_msg_rangefinder.h \
    mavlink/v1.0/ardupilotmega/mavlink_msg_remote_log_block_status.h \
    mavlink/v1.0/ardupilotmega/mavlink_msg_remote_log_data_block.h \
    mavlink/v1.0/ardupilotmega/mavlink_msg_rpm.h \
    mavlink/v1.0/ardupilotmega/mavlink_msg_sensor_offsets.h \
    mavlink/v1.0/ardupilotmega/mavlink_msg_set_mag_offsets.h \
    mavlink/v1.0/ardupilotmega/mavlink_msg_simstate.h \
    mavlink/v1.0/ardupilotmega/mavlink_msg_wind.h \
    mavlink/v1.0/ardupilotmega/testsuite.h \
    mavlink/v1.0/ardupilotmega/version.h \
    mavlink/v1.0/pixhawk/mavlink.h \
    mavlink/v1.0/pixhawk/mavlink_msg_attitude_control.h \
    mavlink/v1.0/pixhawk/mavlink_msg_brief_feature.h \
    mavlink/v1.0/pixhawk/mavlink_msg_data_transmission_handshake.h \
    mavlink/v1.0/pixhawk/mavlink_msg_encapsulated_data.h \
    mavlink/v1.0/pixhawk/mavlink_msg_image_available.h \
    mavlink/v1.0/pixhawk/mavlink_msg_image_trigger_control.h \
    mavlink/v1.0/pixhawk/mavlink_msg_image_triggered.h \
    mavlink/v1.0/pixhawk/mavlink_msg_marker.h \
    mavlink/v1.0/pixhawk/mavlink_msg_pattern_detected.h \
    mavlink/v1.0/pixhawk/mavlink_msg_point_of_interest.h \
    mavlink/v1.0/pixhawk/mavlink_msg_point_of_interest_connection.h \
    mavlink/v1.0/pixhawk/mavlink_msg_position_control_setpoint.h \
    mavlink/v1.0/pixhawk/mavlink_msg_raw_aux.h \
    mavlink/v1.0/pixhawk/mavlink_msg_set_cam_shutter.h \
    mavlink/v1.0/pixhawk/mavlink_msg_set_position_control_offset.h \
    mavlink/v1.0/pixhawk/mavlink_msg_watchdog_command.h \
    mavlink/v1.0/pixhawk/mavlink_msg_watchdog_heartbeat.h \
    mavlink/v1.0/pixhawk/mavlink_msg_watchdog_process_info.h \
    mavlink/v1.0/pixhawk/mavlink_msg_watchdog_process_status.h \
    mavlink/v1.0/pixhawk/pixhawk.h \
    mavlink/v1.0/pixhawk/testsuite.h \
    mavlink/v1.0/pixhawk/version.h \
    mavlink/v1.0/sensesoar/mavlink.h \
    mavlink/v1.0/sensesoar/mavlink_msg_cmd_airspeed_ack.h \
    mavlink/v1.0/sensesoar/mavlink_msg_cmd_airspeed_chng.h \
    mavlink/v1.0/sensesoar/mavlink_msg_filt_rot_vel.h \
    mavlink/v1.0/sensesoar/mavlink_msg_llc_out.h \
    mavlink/v1.0/sensesoar/mavlink_msg_obs_air_temp.h \
    mavlink/v1.0/sensesoar/mavlink_msg_obs_air_velocity.h \
    mavlink/v1.0/sensesoar/mavlink_msg_obs_attitude.h \
    mavlink/v1.0/sensesoar/mavlink_msg_obs_bias.h \
    mavlink/v1.0/sensesoar/mavlink_msg_obs_position.h \
    mavlink/v1.0/sensesoar/mavlink_msg_obs_qff.h \
    mavlink/v1.0/sensesoar/mavlink_msg_obs_velocity.h \
    mavlink/v1.0/sensesoar/mavlink_msg_obs_wind.h \
    mavlink/v1.0/sensesoar/mavlink_msg_pm_elec.h \
    mavlink/v1.0/sensesoar/mavlink_msg_sys_stat.h \
    mavlink/v1.0/sensesoar/sensesoar.h \
    mavlink/v1.0/sensesoar/testsuite.h \
    mavlink/v1.0/sensesoar/version.h \
    mavlink/v1.0/uAvionix/mavlink.h \
    mavlink/v1.0/uAvionix/testsuite.h \
    mavlink/v1.0/uAvionix/uAvionix.h \
    mavlink/v1.0/uAvionix/version.h

win32{
//如果win环境加载
SOURCES += qextserial/qextserialport_win.cpp
}

unix{
//如果unix环境加载
SOURCES += qextserial/qextserialport_unix.cpp \
}

FORMS    += mainwindow.ui \
    aboutdialog.ui \
    aboutdialog.ui \
    altimetro.ui \
    qadi.ui \
    qcompass.ui

RESOURCES += \
    images.qrc
RC_FILE = myico.rc
