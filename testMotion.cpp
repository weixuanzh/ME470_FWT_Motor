/** Coordinated_trigger.cpp

*@brief This is a very simple projects that triggers a kinematic effect simultaneously between two actuators.
*
*   @author Rebecca McWilliam <rmcwilliam@irisdynamics.com>
*    @version 1.1
*
*    @copyright Copyright 2022 Iris Dynamics Ltd
*    Licensed under the Apache License, Version 2.0 (the "License");
*    you may not use this file except in compliance with the License.
*    You may obtain a copy of the License at
*
*    http://www.apache.org/licenses/LICENSE-2.0
*
*    Unless required by applicable law or agreed to in writing, software
*    distributed under the License is distributed on an "AS IS" BASIS,
*    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
*    See the License for the specific language governing permissions and
*    limitations under the License.
*
*    For questions or feedback on this file, please email <support@irisdynamics.com>.
*/
#define UNICODE
#define PI 3.1415926535
#include "dependency/library_linker.h"
#include "dependency/modbus_client/device_applications/actuator.h"
#include "dependency/simple_serial_port-master/simple-serial-port/simple-serial-port/SimpleSerial.cpp"

#include <iostream>
#include <conio.h>
#include <thread>
#include <cassert>
#include <atomic>
using namespace std;
using namespace std::literals::chrono_literals;

//Define Keyboad inputs to console
#define KEY_UP      72

#define NUM_MOTORS 3
// Platform parameters (lengths in mm)
float g = 145; // distance between bottom pin joints
float h = 150; // distance between top ball joints
float actuator_width = 60; // distance between pin center and actuator centerline
float aw2 = actuator_width * actuator_width;
float d_retracted = 213; // length of fully retracted actuator (from connection with pin joint to center of ball joint)

#define SWING_UP_MOTION 114
#define SWING_UP_TIME 3000
Actuator motors[NUM_MOTORS]{
  {0, "Orca A", 1}
, {0, "Orca B", 1}
, {0, "Orca C", 1}
};
// Position setpoint "buffers" in mm
alignas(64) atomic<int> d1 = 0;
alignas(64) atomic<int> d2 = 0;
alignas(64) atomic<int> d3 = 0;


Actuator::ConnectionConfig connection_config;
int port_number[NUM_MOTORS];

// Thread function to progress motor communications
void motor_comms() {
    while (1) {
        for (int j = 0; j < NUM_MOTORS; j++) {
            motors[j].run_in();
            motors[j].run_out();
           // motors[i].set_mode(Actuator::PositionMode);
        }
    }
}

// Thread function to continuously send position setpoints
void send_pos() {
    float d[3];
    while (1) {
        d[0] = d1.load();
        d[1] = d2.load();
        d[2] = d3.load();
        for (int i = 0; i < NUM_MOTORS; i++) {
            if (motors[i].get_mode() == Actuator::SleepMode) {
                motors[i].set_mode(Actuator::PositionMode);
            }
            motors[i].set_position_um(d[i] * 1000);           
        }
    }
}

// gets orientation and height measurements from MPU 6050 gyroscope
float* read_gyroscope() {
    return nullptr;
}

// Calculates the required actuator lengths given height and orientation
// h, g, d_retracted, aw2: same definition as in main
float* inverse_kinematics(float z_des, float pitch_des, float roll_des) {
    static float lengths[NUM_MOTORS];
    // convert yaw, pitch, roll to ZYZ Euler angles
    // solve yaw, find rotation matrix, then find angles
    float c1 = cos(pitch_des);
    float s1 = sin(pitch_des);
    float c2 = cos(roll_des);
    float s2 = sin(roll_des);
    // Solving yaw using atan and constraints on rotation matrix
    float yaw_des = atan2(s1 * s2, c1 + c2);
    float c3 = cos(yaw_des);
    float s3 = sin(yaw_des);
    // convert to ZYZ using rotation matrix
    float r13 = s1 * c2 * c3 + s2 * s3;
    float r23 = s1 * c2 * s3 - s2 * c3;
    float r33 = c1 * c2;
    float alpha = atan2(r23, r13);
    float beta = atan2(sqrt(pow(r13, 2) + pow(r23, 2)), r33);
    // inverse kinematics calculation
    float ca = cos(alpha);
    float sa = sin(alpha);
    float cb = cos(beta);
    float sb = sin(beta);
    float c2a = cos(2 * alpha);
    float s2a = sin(2 * alpha);
    float sq3 = sqrt(3);
    float ca2 = pow(ca, 2);
    float sa2 = pow(sa, 2);
    float cbm = cb - 1;
    // center position
    float px = -0.5 * h * (1 - cb) * c2a;
    float py = 0.5 * h * (1 - cb) * s2a;
    float dist_common = pow(g, 2) + pow(h, 2) + pow(px, 2) + pow(py, 2) + pow(z_des, 2);
    float trig_sq_common_1 = ca2 * cb + sa2;
    float trig_sq_common_2 = sa2 * cb + ca2;
    float x_common = h * (trig_sq_common_1 - sq3 * ca * sa * cbm) * (px + 0.5 * g);
    // actuator "raw" lengths
    float d1 = dist_common - 2 * g * px + 2 * h * trig_sq_common_1 * (px - g) + h * cbm * s2a * py - 2 * h * sb * ca * z_des;
    float d2 = dist_common + g * px - sq3 * g * py - x_common - h * (sa * ca * cbm - sq3 * trig_sq_common_2) * (py - sq3 * 0.5 * g) + h * sb * (ca - sq3 * sa) * z_des;
    float d3 = dist_common + g * px + sq3 * g * py - x_common - h * (sa * ca * cbm + sq3 * trig_sq_common_2) * (py + sq3 * 0.5 * g) + h * sb * (ca + sq3 * sa) * z_des;
    // check if lengths are reasonable
    assert(d1 > aw2);
    assert(d2 > aw2);
    assert(d3 > aw2);
    d1 = sqrt(d1 - aw2) - d_retracted;
    d2 = sqrt(d2 - aw2) - d_retracted;
    d3 = sqrt(d3 - aw2) - d_retracted;
    assert(d1 > 0);
    assert(d2 > 0);
    assert(d3 > 0);    
    lengths[0] = d1;
    lengths[1] = d2;
    lengths[2] = d3;
    return lengths;    
} 


// Moves the platform to a specified orientation smoothly and hold there
// Must be called after all motors are initialized and connected
// Use a PID force controller to get to the height
void swing_up(float z_initial, float pitch_initial, float roll_initial) {
    // Calculate required actuator lengths
    float* ds = inverse_kinematics(z_initial, pitch_initial, roll_initial);
    // position tolerance in micrometers
    float tolerance = 1000;
    // PID gains
    float Kp = 2.5;
    float Kd = 10;
    float Ki = 1;
    // motor positions
    float d1 = motors[0].get_position_um();
    float d2 = motors[1].get_position_um();
    float d3 = motors[2].get_position_um();
    // force inputs in mN
    float fs[NUM_MOTORS];
    float coeff;
    // get current time
    auto start = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point curr;
    while (d1 > tolerance || d2 > tolerance || d3 > tolerance) {
        // calculate required forces
        auto t = (std::chrono::steady_clock::now() - start).count()/1000000000;
        coeff = tanh(t);
        fs[0] = Kp * (ds[0] - d1) * coeff;
        fs[1] = Kp * (ds[1] - d2) * coeff;
        fs[2] = Kp * (ds[2] - d3) * coeff;
        // command forces
        for (int i = 0; i < NUM_MOTORS; i++) {
            if (motors[i].get_mode() == Actuator::SleepMode) {
                motors[i].set_mode(Actuator::ForceMode);
            }
            motors[i].set_force_mN(fs[i]);
        }
    }

    Sleep(SWING_UP_TIME);
}

int main()
{

    // test reading from gyroscope
    // char com_port[] = "\\\\.\\COM9";
    // DWORD COM_BAUD_RATE = CBR_115200;
    // SimpleSerial Serial(com_port, COM_BAUD_RATE);
    // while (1) {
    //     if(Serial.IsConnected()) {
    //         //do whatever
    //         Sleep(1);
    //         int reply_wait_time = 1;
    //         string syntax_type = "json";
    //         string incoming = Serial.ReadSerialPort(reply_wait_time, syntax_type);
    //         printf("%s\n", incoming.c_str());
    //     } else {
    //         std::cout << "not connected!" << std::endl;
    //         Sleep(1);
    //     }
    // }

    // store inverse kinematics results
    float* ds;

    // Hard-coded comport number
    port_number[0] = 3;
    port_number[1] = 6;
    port_number[2] = 8;    

    connection_config.target_baud_rate_bps = 1000000;// 500000;  //625000 //780000
    connection_config.target_delay_us = 0;
    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].set_connection_config(connection_config);
    }

    //establish hi speed modbus stream 
    // set motor operation mode
    for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].set_new_comport(port_number[i]);
        motors[i].init();
        motors[i].enable();
    }
    printf("\nmotors connected!");
    thread mthread(motor_comms); //process motor communications in seperate thread
    // Reset to sleep mode to clear comm timeout errors
     for (int i = 0; i < NUM_MOTORS; i++) {
        motors[i].set_mode(Actuator::SleepMode);
    }   

    // follow sine reference for height, pitch, and roll
    // height is in mm, angles are in rad, freq are in Hz
    float z_amp = 15;
    float z_offset = 320;
    float z_freq = 1.7;
    float pitch_amp = 5 * PI / 180;
    float pitch_offset = 0;
    float pitch_freq = sqrt(2);
    float roll_amp = 5 * PI / 180;
    float roll_offset = 0;
    float roll_freq = 3;

    // omega
    float z_omega = 2 * PI * z_freq;
    float pitch_omega = 2 * PI * pitch_freq;
    float roll_omega = 2 * PI * roll_freq;

    // swing up
    swing_up(z_offset, pitch_offset, roll_offset);

    // Continuous send position setpoints
    thread command_thread(send_pos);
    motors[0].set_mode(Actuator::PositionMode);
    motors[1].set_mode(Actuator::PositionMode);
    motors[2].set_mode(Actuator::PositionMode);
    
    // timer to get current setpoint
    using clock = std::chrono::steady_clock;
    clock::time_point start = clock::now();
    auto prev = start;
    while (1) {
        // desired setpoints at the current time step
        clock::time_point now = clock::now();
        float t = float((now - start).count())/1000000000;
        float z_des = z_amp * sin(z_omega * t) + z_offset;
        float pitch_des = pitch_amp * sin(pitch_omega * t) + pitch_offset;
        float roll_des = roll_amp * sin(roll_omega * t) + roll_offset;

        // Compute inverse kinematics
        ds = inverse_kinematics(z_des, pitch_des, roll_des);
        // Send commands to motors
        d1.store(ds[0]);
        d2.store(ds[1]);
        d3.store(ds[2]);
        
        prev = now;
    }

    return 1;
}
