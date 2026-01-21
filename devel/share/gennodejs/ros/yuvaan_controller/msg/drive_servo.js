// Auto-generated. Do not edit!

// (in-package yuvaan_controller.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class drive_servo {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.vel_linear_x = null;
      this.vel_angular_z = null;
      this.mode = null;
      this.servo_speed = null;
    }
    else {
      if (initObj.hasOwnProperty('vel_linear_x')) {
        this.vel_linear_x = initObj.vel_linear_x
      }
      else {
        this.vel_linear_x = 0;
      }
      if (initObj.hasOwnProperty('vel_angular_z')) {
        this.vel_angular_z = initObj.vel_angular_z
      }
      else {
        this.vel_angular_z = 0;
      }
      if (initObj.hasOwnProperty('mode')) {
        this.mode = initObj.mode
      }
      else {
        this.mode = 0;
      }
      if (initObj.hasOwnProperty('servo_speed')) {
        this.servo_speed = initObj.servo_speed
      }
      else {
        this.servo_speed = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type drive_servo
    // Serialize message field [vel_linear_x]
    bufferOffset = _serializer.int32(obj.vel_linear_x, buffer, bufferOffset);
    // Serialize message field [vel_angular_z]
    bufferOffset = _serializer.int32(obj.vel_angular_z, buffer, bufferOffset);
    // Serialize message field [mode]
    bufferOffset = _serializer.int32(obj.mode, buffer, bufferOffset);
    // Serialize message field [servo_speed]
    bufferOffset = _serializer.int32(obj.servo_speed, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type drive_servo
    let len;
    let data = new drive_servo(null);
    // Deserialize message field [vel_linear_x]
    data.vel_linear_x = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [vel_angular_z]
    data.vel_angular_z = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [mode]
    data.mode = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [servo_speed]
    data.servo_speed = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'yuvaan_controller/drive_servo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f038d4ff1f38bef575d3a34d6c844c6d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Drive and Servo Control Message
    # Combines differential drive control with continuous rotation servo control
    
    # Drive control
    int32 vel_linear_x    # Linear velocity (-255 to 255)
    int32 vel_angular_z   # Angular velocity (-255 to 255)
    int32 mode            # Drive mode (1=slow, 2=medium, 3=fast, 4=max)
    
    # 360° Servo control (microseconds for precision)
    int32 servo_speed     # Servo speed in microseconds (1000-2000, 1500=stop)
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new drive_servo(null);
    if (msg.vel_linear_x !== undefined) {
      resolved.vel_linear_x = msg.vel_linear_x;
    }
    else {
      resolved.vel_linear_x = 0
    }

    if (msg.vel_angular_z !== undefined) {
      resolved.vel_angular_z = msg.vel_angular_z;
    }
    else {
      resolved.vel_angular_z = 0
    }

    if (msg.mode !== undefined) {
      resolved.mode = msg.mode;
    }
    else {
      resolved.mode = 0
    }

    if (msg.servo_speed !== undefined) {
      resolved.servo_speed = msg.servo_speed;
    }
    else {
      resolved.servo_speed = 0
    }

    return resolved;
    }
};

module.exports = drive_servo;
