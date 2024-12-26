// Auto-generated. Do not edit!

// (in-package testDrive.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class carInfo {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.mode = null;
      this.enu_east = null;
      this.enu_north = null;
      this.enu_up = null;
      this.yaw = null;
      this.utm_x = null;
      this.utm_y = null;
      this.heading = null;
    }
    else {
      if (initObj.hasOwnProperty('mode')) {
        this.mode = initObj.mode
      }
      else {
        this.mode = '';
      }
      if (initObj.hasOwnProperty('enu_east')) {
        this.enu_east = initObj.enu_east
      }
      else {
        this.enu_east = 0.0;
      }
      if (initObj.hasOwnProperty('enu_north')) {
        this.enu_north = initObj.enu_north
      }
      else {
        this.enu_north = 0.0;
      }
      if (initObj.hasOwnProperty('enu_up')) {
        this.enu_up = initObj.enu_up
      }
      else {
        this.enu_up = 0.0;
      }
      if (initObj.hasOwnProperty('yaw')) {
        this.yaw = initObj.yaw
      }
      else {
        this.yaw = 0.0;
      }
      if (initObj.hasOwnProperty('utm_x')) {
        this.utm_x = initObj.utm_x
      }
      else {
        this.utm_x = 0.0;
      }
      if (initObj.hasOwnProperty('utm_y')) {
        this.utm_y = initObj.utm_y
      }
      else {
        this.utm_y = 0.0;
      }
      if (initObj.hasOwnProperty('heading')) {
        this.heading = initObj.heading
      }
      else {
        this.heading = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type carInfo
    // Serialize message field [mode]
    bufferOffset = _serializer.string(obj.mode, buffer, bufferOffset);
    // Serialize message field [enu_east]
    bufferOffset = _serializer.float64(obj.enu_east, buffer, bufferOffset);
    // Serialize message field [enu_north]
    bufferOffset = _serializer.float64(obj.enu_north, buffer, bufferOffset);
    // Serialize message field [enu_up]
    bufferOffset = _serializer.float64(obj.enu_up, buffer, bufferOffset);
    // Serialize message field [yaw]
    bufferOffset = _serializer.float64(obj.yaw, buffer, bufferOffset);
    // Serialize message field [utm_x]
    bufferOffset = _serializer.float64(obj.utm_x, buffer, bufferOffset);
    // Serialize message field [utm_y]
    bufferOffset = _serializer.float64(obj.utm_y, buffer, bufferOffset);
    // Serialize message field [heading]
    bufferOffset = _serializer.float64(obj.heading, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type carInfo
    let len;
    let data = new carInfo(null);
    // Deserialize message field [mode]
    data.mode = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [enu_east]
    data.enu_east = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [enu_north]
    data.enu_north = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [enu_up]
    data.enu_up = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [yaw]
    data.yaw = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [utm_x]
    data.utm_x = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [utm_y]
    data.utm_y = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [heading]
    data.heading = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += _getByteLength(object.mode);
    return length + 60;
  }

  static datatype() {
    // Returns string type for a message object
    return 'testDrive/carInfo';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'e597445d8085e6b97cea6f509d947ddb';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # 모드
    string mode  # 사용할 좌표계: "ENU" 또는 "UTM"
    
    # ENU 좌표계
    float64 enu_east
    float64 enu_north
    float64 enu_up
    float64 yaw
    
    # WGS84 좌표계
    float64 utm_x
    float64 utm_y
    float64 heading
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new carInfo(null);
    if (msg.mode !== undefined) {
      resolved.mode = msg.mode;
    }
    else {
      resolved.mode = ''
    }

    if (msg.enu_east !== undefined) {
      resolved.enu_east = msg.enu_east;
    }
    else {
      resolved.enu_east = 0.0
    }

    if (msg.enu_north !== undefined) {
      resolved.enu_north = msg.enu_north;
    }
    else {
      resolved.enu_north = 0.0
    }

    if (msg.enu_up !== undefined) {
      resolved.enu_up = msg.enu_up;
    }
    else {
      resolved.enu_up = 0.0
    }

    if (msg.yaw !== undefined) {
      resolved.yaw = msg.yaw;
    }
    else {
      resolved.yaw = 0.0
    }

    if (msg.utm_x !== undefined) {
      resolved.utm_x = msg.utm_x;
    }
    else {
      resolved.utm_x = 0.0
    }

    if (msg.utm_y !== undefined) {
      resolved.utm_y = msg.utm_y;
    }
    else {
      resolved.utm_y = 0.0
    }

    if (msg.heading !== undefined) {
      resolved.heading = msg.heading;
    }
    else {
      resolved.heading = 0.0
    }

    return resolved;
    }
};

module.exports = carInfo;
