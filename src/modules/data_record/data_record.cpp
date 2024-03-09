/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "data_record.hpp"

#include <drivers/drv_hrt.h>

using namespace time_literals;

DataRecord::DataRecord()
	: ModuleParams(nullptr),
        ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
	//parameters_update();
	_data_record_pub.advertise();
}

DataRecord::~DataRecord()
{
	perf_free(_loop_perf);
	perf_free(_loop_err_perf);
	_data_record_pub.unadvertise();
}

void DataRecord::parameters_update()
{
}

bool DataRecord::init()
{
        ScheduleOnInterval(2500_us); // 2500 us interval, 400 Hz rate

	return true;
}


void DataRecord::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	//perf_count(_loop_err_perf);

	// Check if parameters have changed
	/*if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);

		updateParams();
		parameters_update();
	}*/

	data_record_s &report = _data_record_pub.get();

	if (_data_record_sub.updated()){
		data_record_s last_data;
		_data_record_sub.update(&last_data);

		// report.timestamp_sample_ctl = last_data.timestamp_sample_ctl;
		// for(int i=0;i<4;i++){
		// 	report.control[i] = last_data.control[i];
		// }

		report.timestamp_sample_pwm = last_data.timestamp_sample_pwm;
		for(int i=0;i<4;i++){
			report.output[i] = last_data.output[i];
		}

                report.timestamp_sample_mot = last_data.timestamp_sample_mot;
                for(int i=0;i<4;i++){
                report.motors[i] = last_data.motors[i];
                }

                report.timestamp_sample_thr = last_data.timestamp_sample_thr;
                for(int i=0;i<3;i++){
                report.thrust[i] = last_data.thrust[i];
                }

                report.timestamp_sample_tor = last_data.timestamp_sample_tor;
                for(int i=0;i<3;i++){
                report.torque[i] = last_data.output[i];
                }

                        report.timestamp_sample_att = last_data.timestamp_sample_att;
                        for(int i=0;i<3;i++){
                                report.vehicle_attitude_ned[i] = last_data.vehicle_attitude_ned[i];
                        }

                report.timestamp_sample_imu = last_data.timestamp_sample_imu;
                for(int i=0;i<3;i++){
                        report.vehicle_angular_velocity_xyz[i] = last_data.vehicle_angular_velocity_xyz[i];
                        report.vehicle_acceleration_xyz[i] = last_data.vehicle_acceleration_xyz[i];
                }

                // report.timestamp_sample_jy901b = last_data.timestamp_sample_jy901b;
                // for(int i=0;i<3;i++){
                //         report.jy901b_acc[i] = last_data.jy901b_acc[i];
                //         report.jy901b_gyro[i] = last_data.jy901b_gyro[i];
                //         report.jy901b_euler[i] = last_data.jy901b_euler[i];
                //         report.jy901b_q[i] = last_data.jy901b_q[i];
                // }
                //         report.jy901b_q[3] = last_data.jy901b_q[3];
                //         report.jy901b_baro = last_data.jy901b_baro;
                //         report.jy901b_temp = last_data.jy901b_temp;

                // report.timestamp_sample_bias = last_data.timestamp_sample_bias;
                // for(int i=0;i<3;i++){
                //         report.gyro_bias[i] = last_data.gyro_bias[i];
                //         report.accel_bias[i] = last_data.accel_bias[i];
                // }

                // report.timestamp_sample_external_vision =  last_data.timestamp_sample_external_vision;
                // report.external_vision_position_ned[0] = last_data.external_vision_position_ned[0];
                // report.external_vision_position_ned[1] = last_data.external_vision_position_ned[1];
                // report.external_vision_position_ned[2] = last_data.external_vision_position_ned[2];

                report.timestamp_sample_ground_truth = last_data.timestamp_sample_ground_truth;
                report.ground_truth_acceleration_ned[0] = last_data.ground_truth_acceleration_ned[0];
                report.ground_truth_acceleration_ned[1] = last_data.ground_truth_acceleration_ned[1];
                report.ground_truth_acceleration_ned[2] = last_data.ground_truth_acceleration_ned[2];
                report.ground_truth_acceleration_xyz[0] = last_data.ground_truth_acceleration_xyz[0];
                report.ground_truth_acceleration_xyz[1] = last_data.ground_truth_acceleration_xyz[1];
                report.ground_truth_acceleration_xyz[2] = last_data.ground_truth_acceleration_xyz[2];
                report.ground_truth_position_ned[0] = last_data.ground_truth_position_ned[0];
                report.ground_truth_position_ned[1] = last_data.ground_truth_position_ned[1];
                report.ground_truth_position_ned[2] = last_data.ground_truth_position_ned[2];
                report.ground_truth_velocity_ned[0] = last_data.ground_truth_velocity_ned[0];
                report.ground_truth_velocity_ned[1] = last_data.ground_truth_velocity_ned[1];
                report.ground_truth_velocity_ned[2] = last_data.ground_truth_velocity_ned[2];
                report.ground_truth_velocity_xyz[0] = last_data.ground_truth_velocity_xyz[0];
                report.ground_truth_velocity_xyz[1] = last_data.ground_truth_velocity_xyz[1];
                report.ground_truth_velocity_xyz[2] = last_data.ground_truth_velocity_xyz[2];

                report.timestamp_sample_baro = last_data.timestamp_sample_baro;
                report.pressure = last_data.pressure;
                report.temperature = last_data.temperature;

                report.timestamp_sample_wind = last_data.timestamp_sample_wind;
                report.windspeed_x = last_data.windspeed_x;
                report.windspeed_y = last_data.windspeed_y;
                report.windspeed_z = last_data.windspeed_z;
                report.windspeed_zs = last_data.windspeed_zs;

                report.timestamp_sample_bat = last_data.timestamp_sample_bat;
                report.battery_scale = last_data.battery_scale;
	}

        if (_battery_sub.updated()){
                battery_status_s battery_status;
                _battery_sub.update(&battery_status);
                report.timestamp_sample_bat = battery_status.timestamp;
                report.battery_scale = battery_status.scale;
        }

//         if (_ev_odom_sub.updated()) {
//                 vehicle_odometry_s _ev_odom{};
//                 // copy both attitude & position, we need both to fill a single extVisionSample
//                 _ev_odom_sub.copy(&_ev_odom);
//                 report.timestamp_sample_external_vision =  _ev_odom.timestamp_sample;
//                 report.external_vision_position_ned[0] = _ev_odom.x;
//                 report.external_vision_position_ned[1] = _ev_odom.y;
//                 report.external_vision_position_ned[2] = _ev_odom.z;
//         }

	matrix::Quatf q;
	if (_vehicle_attitude_sub.updated()){
		vehicle_attitude_s vehicle_attitude;
		_vehicle_attitude_sub.update(&vehicle_attitude);
		report.timestamp_sample_att = vehicle_attitude.timestamp;
		q = matrix::Quatf(vehicle_attitude.q);
		report.vehicle_attitude_ned[0] = matrix::Eulerf(q).phi();
		report.vehicle_attitude_ned[1] = matrix::Eulerf(q).theta();
		report.vehicle_attitude_ned[2] = matrix::Eulerf(q).psi();

	}

        if (_vehicle_local_sub.updated()){
		vehicle_local_position_s vehicle_ground_truth;
		_vehicle_local_sub.update(&vehicle_ground_truth);
                report.timestamp_sample_ground_truth = vehicle_ground_truth.timestamp;
                report.ground_truth_acceleration_ned[0] = vehicle_ground_truth.ax;
                report.ground_truth_acceleration_ned[1] = vehicle_ground_truth.ay;
                report.ground_truth_acceleration_ned[2] = vehicle_ground_truth.az;
                report.ground_truth_position_ned[0] = vehicle_ground_truth.x;
                report.ground_truth_position_ned[1] = vehicle_ground_truth.y;
                report.ground_truth_position_ned[2] = vehicle_ground_truth.z;
                report.ground_truth_velocity_ned[0] = vehicle_ground_truth.vx;
                report.ground_truth_velocity_ned[1] = vehicle_ground_truth.vy;
                report.ground_truth_velocity_ned[2] = vehicle_ground_truth.vz;

                const matrix::Dcmf body_to_earth(q);
                const matrix::Vector3f v_r = matrix::Vector3f(body_to_earth.transpose() * matrix::Vector3f(vehicle_ground_truth.vx, vehicle_ground_truth.vy, vehicle_ground_truth.vz));
                const matrix::Vector3f a_r = matrix::Vector3f(body_to_earth.transpose() * matrix::Vector3f(vehicle_ground_truth.ax, vehicle_ground_truth.ay, vehicle_ground_truth.az));
                report.ground_truth_velocity_xyz[0] = v_r(0);
                report.ground_truth_velocity_xyz[1] = v_r(1);
                report.ground_truth_velocity_xyz[2] = v_r(2);
                report.ground_truth_acceleration_xyz[0] = a_r(0);
                report.ground_truth_acceleration_xyz[1] = a_r(1);
                report.ground_truth_acceleration_xyz[2] = a_r(2);
        }

        if (_sensor_baro_sub.updated()){
                sensor_baro_s sensor_baro;
                _sensor_baro_sub.update(&sensor_baro);
                report.timestamp_sample_baro = sensor_baro.timestamp;
                report.pressure = sensor_baro.pressure;
                report.temperature = sensor_baro.temperature;
        }

	if (_windspeed_sub.updated()){
		wind_speed_s windspeed;
		_windspeed_sub.update(&windspeed);
		report.timestamp_sample_wind = windspeed.timestamp;
		report.windspeed_x = windspeed.measurement_windspeed_x_m_s;
		report.windspeed_y = windspeed.measurement_windspeed_y_m_s;
                report.windspeed_z = windspeed.measurement_windspeed_z_m_s;
                report.windspeed_zs = windspeed.measurement_windspeed_zs_m_s;
	}

        // if (_actuator_controls_sub.updated()){
        //         actuator_controls_s actuator_controls;
        //         _actuator_controls_sub.update(&actuator_controls);
        //         report.timestamp_sample_ctl = actuator_controls.timestamp;
        //         for(int i=0;i<4;i++){
        //                 report.control[i] = actuator_controls.control[i];
        //         }
        // }

        if (_actuator_outputs_sub.updated()){
                actuator_outputs_s actuator_outputs;
                _actuator_outputs_sub.update(&actuator_outputs);
                report.timestamp_sample_pwm = actuator_outputs.timestamp;
                for(int i=0;i<4;i++){
                        report.output[i] = actuator_outputs.output[i];
                }
        }

        if (_actuator_motors_sub.updated()){
                actuator_motors_s actuator_motors;
                _actuator_motors_sub.update(&actuator_motors);
                report.timestamp_sample_mot = actuator_motors.timestamp;
                for(int i=0;i<4;i++){
                        report.motors[i] = actuator_motors.control[i];
                }
        }

        if (_vehicle_thrust_setpoint_sub.updated()){
                vehicle_thrust_setpoint_s vehicle_thrust_setpoint;
                _vehicle_thrust_setpoint_sub.update(&vehicle_thrust_setpoint);
                report.timestamp_sample_thr = vehicle_thrust_setpoint.timestamp;
                for(int i=0;i<3;i++){
                        report.thrust[i] = vehicle_thrust_setpoint.xyz[i];
                }
        }

        if (_vehicle_torque_setpoint_sub.updated()){
                vehicle_torque_setpoint_s vehicle_torque_setpoint;
                _vehicle_torque_setpoint_sub.update(&vehicle_torque_setpoint);
                report.timestamp_sample_tor = vehicle_torque_setpoint.timestamp;
                for(int i=0;i<3;i++){
                        report.torque[i] = vehicle_torque_setpoint.xyz[i];
                }
        }

        // if (_vehicle_acceleration_sub.updated() && _vehicle_rate_sub.updated()){
        //         vehicle_acceleration_s vehicle_acceleration;
        //         _vehicle_acceleration_sub.update(&vehicle_acceleration);
        //         vehicle_angular_velocity_s vehicle_angular_velocity;
        //         _vehicle_rate_sub.update(&vehicle_angular_velocity);
        //         report.timestamp_sample_imu = vehicle_acceleration.timestamp_sample;
        //         for(int i=0;i<3;i++){
        //                 report.vehicle_acceleration_xyz[i] = vehicle_acceleration.xyz[i];
        //                 report.vehicle_angular_velocity_xyz[i] = vehicle_angular_velocity.xyz[i];
        //         }
        // }

        // if (_estimator_sensor_bias_sub.updated()){
        //         estimator_sensor_bias_s estimator_sensor_bias;
        //         _estimator_sensor_bias_sub.update(&estimator_sensor_bias);
        //         report.timestamp_sample_bias = estimator_sensor_bias.timestamp;
        //         for(int i=0;i<3;i++){
        //                 report.gyro_bias[i] = estimator_sensor_bias.gyro_bias[i];
        //                 report.accel_bias[i] = estimator_sensor_bias.accel_bias[i];
        //         }
        // }

        if (_vehicle_sensor_combined_sub.updated()){
                sensor_combined_s sensor_combined;
                _vehicle_sensor_combined_sub.update(&sensor_combined);
                report.timestamp_sample_imu = sensor_combined.timestamp;
                for(int i=0;i<3;i++){
                        report.vehicle_angular_velocity_xyz[i] = sensor_combined.gyro_rad[i];
                        report.vehicle_acceleration_xyz[i] = sensor_combined.accelerometer_m_s2[i];
                }
        }

        // if (_jy901b_msg_sub.updated()){
        //         jy901b_msg_s jy901b_msg;
        //         _jy901b_msg_sub.update(&jy901b_msg);
        //         report.timestamp_sample_jy901b = jy901b_msg.timestamp;
        //         for(int i=0;i<3;i++){
        //                 report.jy901b_acc[i] = jy901b_msg.acc[i];
        //                 report.jy901b_gyro[i] = jy901b_msg.gyro[i];
        //                 report.jy901b_euler[i] = jy901b_msg.euler[i];
        //                 report.jy901b_q[i] = jy901b_msg.q[i];
        //         }
	// 	report.jy901b_q[3] = jy901b_msg.q[3];
	// 	report.jy901b_baro = jy901b_msg.baro;
	// 	report.jy901b_temp = jy901b_msg.temp;
        // }

	report.timestamp = hrt_absolute_time();
	_data_record_pub.update();

	perf_end(_loop_perf);
}

int DataRecord::task_spawn(int argc, char *argv[])
{
	DataRecord *instance = new DataRecord();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		} else {
			PX4_ERR("init failed");
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int DataRecord::print_status()
{
	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_err_perf);
	return 0;
}

int DataRecord::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int DataRecord::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Section that describes the provided module functionality.

This is a template for a module running as a task in the background with start/stop/status functionality.

### Implementation
Section describing the high-level implementation of this module.

### Examples
CLI usage example:
$ module start

)DESCR_STR");

        PRINT_MODULE_USAGE_NAME("data_record", "template");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int data_record_main(int argc, char *argv[])
{
	return DataRecord::main(argc, argv);
}
