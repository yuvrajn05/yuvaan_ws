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

class yuvaan {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.vel_linear_x = null;
      this.vel_angular_z = null;
      this.mode = null;
      this.yaw_mode = null;
      this.roll_mode = null;
      this.ra_1 = null;
      this.ra_2 = null;
      this.ra_3 = null;
      this.ra_4 = null;
      this.ra_5 = null;
      this.ra_6 = null;
      this.ba_1 = null;
      this.ba_2 = null;
      this.ba_3 = null;
      this.ba_4 = null;
      this.ba_5 = null;
      this.ba_6 = null;
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
      if (initObj.hasOwnProperty('yaw_mode')) {
        this.yaw_mode = initObj.yaw_mode
      }
      else {
        this.yaw_mode = 0;
      }
      if (initObj.hasOwnProperty('roll_mode')) {
        this.roll_mode = initObj.roll_mode
      }
      else {
        this.roll_mode = 0;
      }
      if (initObj.hasOwnProperty('ra_1')) {
        this.ra_1 = initObj.ra_1
      }
      else {
        this.ra_1 = 0;
      }
      if (initObj.hasOwnProperty('ra_2')) {
        this.ra_2 = initObj.ra_2
      }
      else {
        this.ra_2 = 0;
      }
      if (initObj.hasOwnProperty('ra_3')) {
        this.ra_3 = initObj.ra_3
      }
      else {
        this.ra_3 = 0;
      }
      if (initObj.hasOwnProperty('ra_4')) {
        this.ra_4 = initObj.ra_4
      }
      else {
        this.ra_4 = 0;
      }
      if (initObj.hasOwnProperty('ra_5')) {
        this.ra_5 = initObj.ra_5
      }
      else {
        this.ra_5 = 0;
      }
      if (initObj.hasOwnProperty('ra_6')) {
        this.ra_6 = initObj.ra_6
      }
      else {
        this.ra_6 = 0;
      }
      if (initObj.hasOwnProperty('ba_1')) {
        this.ba_1 = initObj.ba_1
      }
      else {
        this.ba_1 = 0;
      }
      if (initObj.hasOwnProperty('ba_2')) {
        this.ba_2 = initObj.ba_2
      }
      else {
        this.ba_2 = 0;
      }
      if (initObj.hasOwnProperty('ba_3')) {
        this.ba_3 = initObj.ba_3
      }
      else {
        this.ba_3 = 0;
      }
      if (initObj.hasOwnProperty('ba_4')) {
        this.ba_4 = initObj.ba_4
      }
      else {
        this.ba_4 = 0;
      }
      if (initObj.hasOwnProperty('ba_5')) {
        this.ba_5 = initObj.ba_5
      }
      else {
        this.ba_5 = 0;
      }
      if (initObj.hasOwnProperty('ba_6')) {
        this.ba_6 = initObj.ba_6
      }
      else {
        this.ba_6 = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type yuvaan
    // Serialize message field [vel_linear_x]
    bufferOffset = _serializer.int32(obj.vel_linear_x, buffer, bufferOffset);
    // Serialize message field [vel_angular_z]
    bufferOffset = _serializer.int32(obj.vel_angular_z, buffer, bufferOffset);
    // Serialize message field [mode]
    bufferOffset = _serializer.int32(obj.mode, buffer, bufferOffset);
    // Serialize message field [yaw_mode]
    bufferOffset = _serializer.int32(obj.yaw_mode, buffer, bufferOffset);
    // Serialize message field [roll_mode]
    bufferOffset = _serializer.int32(obj.roll_mode, buffer, bufferOffset);
    // Serialize message field [ra_1]
    bufferOffset = _serializer.int32(obj.ra_1, buffer, bufferOffset);
    // Serialize message field [ra_2]
    bufferOffset = _serializer.int32(obj.ra_2, buffer, bufferOffset);
    // Serialize message field [ra_3]
    bufferOffset = _serializer.int32(obj.ra_3, buffer, bufferOffset);
    // Serialize message field [ra_4]
    bufferOffset = _serializer.int32(obj.ra_4, buffer, bufferOffset);
    // Serialize message field [ra_5]
    bufferOffset = _serializer.int32(obj.ra_5, buffer, bufferOffset);
    // Serialize message field [ra_6]
    bufferOffset = _serializer.int32(obj.ra_6, buffer, bufferOffset);
    // Serialize message field [ba_1]
    bufferOffset = _serializer.int32(obj.ba_1, buffer, bufferOffset);
    // Serialize message field [ba_2]
    bufferOffset = _serializer.int32(obj.ba_2, buffer, bufferOffset);
    // Serialize message field [ba_3]
    bufferOffset = _serializer.int32(obj.ba_3, buffer, bufferOffset);
    // Serialize message field [ba_4]
    bufferOffset = _serializer.int32(obj.ba_4, buffer, bufferOffset);
    // Serialize message field [ba_5]
    bufferOffset = _serializer.int32(obj.ba_5, buffer, bufferOffset);
    // Serialize message field [ba_6]
    bufferOffset = _serializer.int32(obj.ba_6, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type yuvaan
    let len;
    let data = new yuvaan(null);
    // Deserialize message field [vel_linear_x]
    data.vel_linear_x = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [vel_angular_z]
    data.vel_angular_z = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [mode]
    data.mode = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [yaw_mode]
    data.yaw_mode = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [roll_mode]
    data.roll_mode = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [ra_1]
    data.ra_1 = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [ra_2]
    data.ra_2 = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [ra_3]
    data.ra_3 = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [ra_4]
    data.ra_4 = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [ra_5]
    data.ra_5 = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [ra_6]
    data.ra_6 = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [ba_1]
    data.ba_1 = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [ba_2]
    data.ba_2 = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [ba_3]
    data.ba_3 = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [ba_4]
    data.ba_4 = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [ba_5]
    data.ba_5 = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [ba_6]
    data.ba_6 = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 68;
  }

  static datatype() {
    // Returns string type for a message object
    return 'yuvaan_controller/yuvaan';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'f9d9c81a204467304bb82823914c9c9d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 vel_linear_x
    int32 vel_angular_z
    int32 mode
    int32 yaw_mode
    int32 roll_mode
    int32 ra_1
    int32 ra_2
    int32 ra_3
    int32 ra_4
    int32 ra_5
    int32 ra_6
    int32 ba_1
    int32 ba_2
    int32 ba_3
    int32 ba_4
    int32 ba_5
    int32 ba_6
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new yuvaan(null);
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

    if (msg.yaw_mode !== undefined) {
      resolved.yaw_mode = msg.yaw_mode;
    }
    else {
      resolved.yaw_mode = 0
    }

    if (msg.roll_mode !== undefined) {
      resolved.roll_mode = msg.roll_mode;
    }
    else {
      resolved.roll_mode = 0
    }

    if (msg.ra_1 !== undefined) {
      resolved.ra_1 = msg.ra_1;
    }
    else {
      resolved.ra_1 = 0
    }

    if (msg.ra_2 !== undefined) {
      resolved.ra_2 = msg.ra_2;
    }
    else {
      resolved.ra_2 = 0
    }

    if (msg.ra_3 !== undefined) {
      resolved.ra_3 = msg.ra_3;
    }
    else {
      resolved.ra_3 = 0
    }

    if (msg.ra_4 !== undefined) {
      resolved.ra_4 = msg.ra_4;
    }
    else {
      resolved.ra_4 = 0
    }

    if (msg.ra_5 !== undefined) {
      resolved.ra_5 = msg.ra_5;
    }
    else {
      resolved.ra_5 = 0
    }

    if (msg.ra_6 !== undefined) {
      resolved.ra_6 = msg.ra_6;
    }
    else {
      resolved.ra_6 = 0
    }

    if (msg.ba_1 !== undefined) {
      resolved.ba_1 = msg.ba_1;
    }
    else {
      resolved.ba_1 = 0
    }

    if (msg.ba_2 !== undefined) {
      resolved.ba_2 = msg.ba_2;
    }
    else {
      resolved.ba_2 = 0
    }

    if (msg.ba_3 !== undefined) {
      resolved.ba_3 = msg.ba_3;
    }
    else {
      resolved.ba_3 = 0
    }

    if (msg.ba_4 !== undefined) {
      resolved.ba_4 = msg.ba_4;
    }
    else {
      resolved.ba_4 = 0
    }

    if (msg.ba_5 !== undefined) {
      resolved.ba_5 = msg.ba_5;
    }
    else {
      resolved.ba_5 = 0
    }

    if (msg.ba_6 !== undefined) {
      resolved.ba_6 = msg.ba_6;
    }
    else {
      resolved.ba_6 = 0
    }

    return resolved;
    }
};

module.exports = yuvaan;
