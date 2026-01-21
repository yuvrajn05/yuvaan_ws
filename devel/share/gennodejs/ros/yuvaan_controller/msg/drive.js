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

class drive {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.vel_linear_x = null;
      this.vel_angular_z = null;
      this.mode = null;
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
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type drive
    // Serialize message field [vel_linear_x]
    bufferOffset = _serializer.int32(obj.vel_linear_x, buffer, bufferOffset);
    // Serialize message field [vel_angular_z]
    bufferOffset = _serializer.int32(obj.vel_angular_z, buffer, bufferOffset);
    // Serialize message field [mode]
    bufferOffset = _serializer.int32(obj.mode, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type drive
    let len;
    let data = new drive(null);
    // Deserialize message field [vel_linear_x]
    data.vel_linear_x = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [vel_angular_z]
    data.vel_angular_z = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [mode]
    data.mode = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'yuvaan_controller/drive';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b1dd02d8c99c39b8457292a0b8bdd1a9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 vel_linear_x
    int32 vel_angular_z
    int32 mode
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new drive(null);
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

    return resolved;
    }
};

module.exports = drive;
