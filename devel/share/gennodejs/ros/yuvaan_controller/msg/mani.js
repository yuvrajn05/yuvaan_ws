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

class mani {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.yaw_mode = null;
      this.roll_mode = null;
      this.ra_1 = null;
      this.ra_2 = null;
      this.ra_3 = null;
      this.ra_4 = null;
      this.ra_5 = null;
      this.ra_6 = null;
    }
    else {
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
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type mani
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
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type mani
    let len;
    let data = new mani(null);
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
    return data;
  }

  static getMessageSize(object) {
    return 32;
  }

  static datatype() {
    // Returns string type for a message object
    return 'yuvaan_controller/mani';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'd4cb1b85a0be84892c13244d118cfb38';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int32 yaw_mode
    int32 roll_mode
    int32 ra_1
    int32 ra_2
    int32 ra_3
    int32 ra_4
    int32 ra_5
    int32 ra_6
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new mani(null);
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

    return resolved;
    }
};

module.exports = mani;
