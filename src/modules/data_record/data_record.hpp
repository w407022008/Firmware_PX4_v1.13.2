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

#include <perf/perf_counter.h>
#include <lib/matrix/matrix/math.hpp>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
//#include <uORB/topics/parameter_update.h>
// #include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>
#include <uORB/topics/vehicle_attitude.h>
// #include <uORB/topics/vehicle_angular_velocity.h>
// #include <uORB/topics/vehicle_acceleration.h>
#include <uORB/topics/sensor_combined.h>
// #include <uORB/topics/jy901b_msg.h>
//#include <uORB/topics/vehicle_odometry.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/wind_speed.h>
#include <uORB/topics/battery_status.h>
// #include <uORB/topics/estimator_sensor_bias.h>
#include <uORB/topics/sensor_baro.h>

#include <uORB/topics/data_record.h>

extern "C" __EXPORT int data_record_main(int argc, char *argv[]);


class DataRecord : public ModuleBase<DataRecord>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	DataRecord();

	~DataRecord() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

private:
	void Run() override;

	/**
	 * Check for parameter changes and update them if needed.
	 * @param parameter_update_sub uorb subscription to parameter_update
	 * @param force for a parameter update
	 */
	void parameters_update();

//    DEFINE_PARAMETERS(
//        (ParamFloat<px4::params::MC_ROLL_P>) _param_mc_roll_p
//	)

    // Subscriptions
//    uORB::Subscription	_parameter_update_sub{ORB_ID(parameter_update)};
    /* actuator */
//	uORB::Subscription 	_actuator_controls_sub{ORB_ID(actuator_controls_0)};
    uORB::Subscription 	_actuator_motors_sub{ORB_ID(actuator_motors),1};
    uORB::Subscription 	_vehicle_thrust_setpoint_sub{ORB_ID(vehicle_thrust_setpoint),1};
    uORB::Subscription 	_vehicle_torque_setpoint_sub{ORB_ID(vehicle_torque_setpoint),1};
    uORB::Subscription 	_actuator_outputs_sub{ORB_ID(actuator_outputs),1}; // 800hz defined by IMU_GYRO_RATEMAX
    /* flight state */
    // uORB::Subscription 	_vehicle_acceleration_sub{ORB_ID(vehicle_acceleration)};
    // uORB::Subscription 	_vehicle_rate_sub{ORB_ID(vehicle_angular_velocity)}; // Lp filter
    uORB::Subscription 	_vehicle_sensor_combined_sub{ORB_ID(sensor_combined)}; // 200hz defined by IMU_INTEG_RATE
//     uORB::Subscription 	_jy901b_msg_sub{ORB_ID(jy901b_msg)};
    uORB::Subscription 	_vehicle_attitude_sub{ORB_ID(vehicle_attitude)}; // 200hz
//    uORB::Subscription 	_ev_odom_sub{ORB_ID(vehicle_visual_odometry)};
    uORB::Subscription 	_vehicle_local_sub{ORB_ID(vehicle_local_position)}; // 100hz
    uORB::Subscription 	_sensor_baro_sub{ORB_ID(sensor_baro)}; // 150hz defined by SENS_BARO_RATE
    uORB::Subscription 	_windspeed_sub{ORB_ID(wind_speed)}; // 400hz
    uORB::Subscription 	_battery_sub{ORB_ID(battery_status)}; //
    uORB::Subscription 	_data_record_sub{ORB_ID(data_record)}; //400hz
//    uORB::Subscription _estimator_sensor_bias_sub{ORB_ID(estimator_sensor_bias)};

	// Publications
	uORB::PublicationData<data_record_s>	_data_record_pub{ORB_ID(data_record)};			/**< rate setpoint publication */

    perf_counter_t	_loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": normal_cycle")};
    perf_counter_t	_loop_err_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": err")};
};

