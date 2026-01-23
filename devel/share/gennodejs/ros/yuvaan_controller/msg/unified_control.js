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

class unified_control {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.vel_linear_x = null;
      this.vel_angular_z = null;
      this.mode = null;
      this.servo_dpad = null;
      this.servo_stick_x = null;
      this.servo_stick_y = null;
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
      if (initObj.hasOwnProperty('servo_dpad')) {
        this.servo_dpad = initObj.servo_dpad
      }
      else {
        this.servo_dpad = 0;
      }
      if (initObj.hasOwnProperty('servo_stick_x')) {
        this.servo_stick_x = initObj.servo_stick_x
      }
      else {
        this.servo_stick_x = 0;
      }
      if (initObj.hasOwnProperty('servo_stick_y')) {
        this.servo_stick_y = initObj.servo_stick_y
      }
      else {
        this.servo_stick_y = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type unified_control
    // Serialize message field [vel_linear_x]
    bufferOffset = _serializer.int32(obj.vel_linear_x, buffer, bufferOffset);
    // Serialize message field [vel_angular_z]
    bufferOffset = _serializer.int32(obj.vel_angular_z, buffer, bufferOffset);
    // Serialize message field [mode]
    bufferOffset = _serializer.int32(obj.mode, buffer, bufferOffset);
    // Serialize message field [servo_dpad]
    bufferOffset = _serializer.int32(obj.servo_dpad, buffer, bufferOffset);
    // Serialize message field [servo_stick_x]
    bufferOffset = _serializer.int32(obj.servo_stick_x, buffer, bufferOffset);
    // Serialize message field [servo_stick_y]
    bufferOffset = _serializer.int32(obj.servo_stick_y, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type unified_control
    let len;
    let data = new unified_control(null);
    // Deserialize message field [vel_linear_x]
    data.vel_linear_x = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [vel_angular_z]
    data.vel_angular_z = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [mode]
    data.mode = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [servo_dpad]
    data.servo_dpad = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [servo_stick_x]
    data.servo_stick_x = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [servo_stick_y]
    data.servo_stick_y = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a message object
    return 'yuvaan_controller/unified_control';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4f2a61c6bf7b2e257f911489b14b7b6b';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Unified Control Message
    # Combines drive control + 3 servos (1 D-pad + 2 analog stick)
    
    # Drive control
    int32 vel_linear_x    # Linear velocity (-255 to 255)
    int32 vel_angular_z   # Angular velocity (-255 to 255)
    int32 mode            # Drive mode (1=slow, 2=medium, 3=fast, 4=max)
    
    # Servo control (all in microseconds, 1500=stop)
    int32 servo_dpad      # D-pad controlled servo (1000-2000μs)
    int32 servo_stick_x   # Right stick X servo (1000-2000μs)
    int32 servo_stick_y   # Right stick Y servo (1000-2000μs)
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new unified_control(null);
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

    if (msg.servo_dpad !== undefined) {
      resolved.servo_dpad = msg.servo_dpad;
    }
    else {
      resolved.servo_dpad = 0
    }

    if (msg.servo_stick_x !== undefined) {
      resolved.servo_stick_x = msg.servo_stick_x;
    }
    else {
      resolved.servo_stick_x = 0
    }

    if (msg.servo_stick_y !== undefined) {
      resolved.servo_stick_y = msg.servo_stick_y;
    }
    else {
      resolved.servo_stick_y = 0
    }

    return resolved;
    }
};

module.exports = unified_control;
