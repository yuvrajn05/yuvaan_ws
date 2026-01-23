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

class dual_servo {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.servo1_speed = null;
      this.servo2_speed = null;
    }
    else {
      if (initObj.hasOwnProperty('servo1_speed')) {
        this.servo1_speed = initObj.servo1_speed
      }
      else {
        this.servo1_speed = 0;
      }
      if (initObj.hasOwnProperty('servo2_speed')) {
        this.servo2_speed = initObj.servo2_speed
      }
      else {
        this.servo2_speed = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type dual_servo
    // Serialize message field [servo1_speed]
    bufferOffset = _serializer.int32(obj.servo1_speed, buffer, bufferOffset);
    // Serialize message field [servo2_speed]
    bufferOffset = _serializer.int32(obj.servo2_speed, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type dual_servo
    let len;
    let data = new dual_servo(null);
    // Deserialize message field [servo1_speed]
    data.servo1_speed = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [servo2_speed]
    data.servo2_speed = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'yuvaan_controller/dual_servo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'af2b50820dddbd7e357854376715f6d2';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Dual Servo Control Message
    # Controls 2 servos independently using right analog stick
    
    int32 servo1_speed  # Servo 1 speed in microseconds (1000-2000, 1500=stop)
    int32 servo2_speed  # Servo 2 speed in microseconds (1000-2000, 1500=stop)
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new dual_servo(null);
    if (msg.servo1_speed !== undefined) {
      resolved.servo1_speed = msg.servo1_speed;
    }
    else {
      resolved.servo1_speed = 0
    }

    if (msg.servo2_speed !== undefined) {
      resolved.servo2_speed = msg.servo2_speed;
    }
    else {
      resolved.servo2_speed = 0
    }

    return resolved;
    }
};

module.exports = dual_servo;
