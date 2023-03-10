/****************************************************************************
 *
 *   Copyright (c) 2013-2020 PX4 Development Team. All rights reserved.
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

#include "sfm3000.hpp"


SFM3000::SFM3000(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	_px4_anemometer(get_device_id(), config.rotation)
{
    // up the retries since the device misses the first measure attempts
    I2C::_retries = 3;
}

SFM3000::~SFM3000()
{
    perf_free(_sample_perf);
    perf_free(_comms_errors);
}

int SFM3000::collect()
{
    uint8_t val[2] {};
    const hrt_abstime timestamp_sample = hrt_absolute_time();
    float measurement[4];
    float confidence[4];
    uint8_t orientation = 128;

    perf_begin(_sample_perf);

    // Increment the sensor index, (limited to the number of sensors connected).
    for (size_t index = 0; index < _sensor_count; index++) {
    
        if(enable_TCA9578A){
            set_device_address(TCA_ADDR);
            uint8_t cmd = 1 << _sensor_chanal[index]; // for port #_sensor_chanal[index] in tca9548a
            transfer(&cmd, 1, nullptr, 0);
        }
        // Set address of the current sensor to collect data from.
        set_device_address(SFM_BASEADDR);
        // Transfer data from the bus.
        //if (PX4_OK != measure()) return 0;
        int ret_val = transfer(nullptr, 0, &val[0], 2);

        if (ret_val != PX4_OK) {
            PX4_DEBUG("sensor #0%i reading failed, at chanal: #0%d (#0128: disable TCA)", index, _sensor_chanal[index]);
            perf_count(_comms_errors);
            perf_end(_sample_perf);
            return ret_val;
        }

        uint16_t speed = uint16_t(val[0]) << 8 | val[1];
        float speed_m_s = static_cast<float>(speed - SFM_OFFSET) / SFM_SCALE * SFM_SLM2MS;
        orientation = _sensor_rotations[index];

        switch (orientation) {
		    case wind_speed_s::ROTATION_FORWARD_FACING:{
		        measurement[0] = speed_m_s;
		        confidence[0] = 1;
		        break;
		        }

		    case wind_speed_s::ROTATION_RIGHT_FACING:{
		        measurement[1] = speed_m_s;
		        confidence[1] = 1;
		        break;
		        }

		    case wind_speed_s::ROTATION_DOWNWARD_FACING:{
		        measurement[2] = speed_m_s;
		        confidence[2] = 1;
		        break;
		        }

		    case wind_speed_s::ROTATION_DOWNWARD_FACING_SECOND:{
		        measurement[3] = speed_m_s;
		        confidence[3] = 1;
		        break;
		        }
        }
    }

    _px4_anemometer.update(timestamp_sample, measurement, confidence, orientation);


    perf_count(_sample_perf);
    perf_end(_sample_perf);

    return PX4_OK;
}

int SFM3000::init()
{
    int32_t hw_model = 0;
    param_get(param_find("SENS_EN_SFM3000"), &hw_model);

    switch (hw_model) {
    case 0: // Disabled
        PX4_WARN("Disabled");
        return PX4_ERROR;

    case 1: // Enable
        set_device_address(TCA_ADDR);
        
        if (I2C::init() != OK) {
            set_device_address(SFM_BASEADDR);
                
            if (I2C::init() != OK) {
                PX4_DEBUG(" initialisation failed");
                return PX4_ERROR;

            } else {
                enable_TCA9578A = false;
            }
        }
        
        if(enable_TCA9578A){
            // Check for connected anemometer on each i2c port,
            // starting from the base address 0x40 and incrementing
            int j=0;
            int i=0;
            while (i < TCA9578A_MAX_CHANAL) {
                // set TCA9578A chanal
                set_device_address(TCA_ADDR);
                uint8_t cmd = 1 << i; // for port #index in tca9548a
                transfer(&cmd, 1, nullptr, 0);

                // Check if a sensor is present.
                set_device_address(SFM_BASEADDR);
                if (probe() != PX4_OK) {
                    PX4_DEBUG("At tca9578a chanal #0%d: There is not any sensor connected", i);
                    i++;
                    continue;
                }

                // Store I2C address
                _sensor_chanal[j] = i;
                _sensor_rotations[j] = get_sensor_rotation(j);
                _sensor_count++;
                j++;
                i++;
            }
        }else{
            // Check if a sensor is present.
            set_device_address(SFM_BASEADDR);
            if (probe() != PX4_OK) {
                PX4_DEBUG("There is not any sensor connected");
            }
            _sensor_count=1;
            _sensor_rotations[0]=wind_speed_s::ROTATION_FORWARD_FACING;
            _sensor_chanal[0]=128;
        }
        break;

    default:
        PX4_ERR("invalid HW model %" PRId32 ".", hw_model);
        return PX4_ERROR;
    }

    start();

    return PX4_OK;
}

int
SFM3000::get_sensor_rotation(const size_t index)
{
    int32_t _q_sensor_x; // x - forward
    int32_t _q_sensor_y; // y - right
    int32_t _q_sensor_z; // z - down
    param_get(param_find("SFM_ROT_X"),&_q_sensor_x);
    param_get(param_find("SFM_ROT_Y"),&_q_sensor_y);
    param_get(param_find("SFM_ROT_Z"),&_q_sensor_z); // which one is connected on ch-index, x?y?z?
    switch (index) {
    case 0:
        return (_q_sensor_x == 1)?wind_speed_s::ROTATION_FORWARD_FACING:((_q_sensor_y == 1)?wind_speed_s::ROTATION_RIGHT_FACING:((_q_sensor_z == 1)?wind_speed_s::ROTATION_DOWNWARD_FACING:128));

    case 1:
        return (_q_sensor_x == 2)?wind_speed_s::ROTATION_FORWARD_FACING:((_q_sensor_y == 2)?wind_speed_s::ROTATION_RIGHT_FACING:((_q_sensor_z == 2)?wind_speed_s::ROTATION_DOWNWARD_FACING:128));

    case 2:
        return (_q_sensor_x == 3)?wind_speed_s::ROTATION_FORWARD_FACING:((_q_sensor_y == 3)?wind_speed_s::ROTATION_RIGHT_FACING:((_q_sensor_z == 3)?wind_speed_s::ROTATION_DOWNWARD_FACING:128));

    default: return wind_speed_s::ROTATION_DOWNWARD_FACING_SECOND;
    }
}

int SFM3000::measure()
{
    // Send the command to begin a measurement.
    uint8_t cmd[2] = {0x10,0x00};
    int ret_val = transfer(&cmd[0], 2, nullptr, 0);

    if (ret_val != PX4_OK) {
        perf_count(_comms_errors);
        PX4_DEBUG("i2c::transfer returned %d", ret_val);
        return ret_val;
    }

    return PX4_OK;
}

int SFM3000::probe()
{
    return measure();
}

void SFM3000::RunImpl()
{
    // Perform data collection.
    collect();
}

void SFM3000::start()
{

    // Schedule the driver to run on a set interval
    ScheduleOnInterval(SFM_MEASUREMENT_INTERVAL);
}

void SFM3000::print_status()
{
    for(size_t i=0;i<_sensor_count;i++){
        PX4_INFO("Sensor on address 0x%02x at tca9578a chanal #0%d: Connected", SFM_BASEADDR, _sensor_chanal[i]);
    }

    I2CSPIDriverBase::print_status();
    perf_print_counter(_sample_perf);
    perf_print_counter(_comms_errors);
}
