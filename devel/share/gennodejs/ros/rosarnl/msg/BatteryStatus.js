// Auto-generated. Do not edit!

// (in-package rosarnl.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class BatteryStatus {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.charging_state = null;
      this.charge_percent = null;
    }
    else {
      if (initObj.hasOwnProperty('charging_state')) {
        this.charging_state = initObj.charging_state
      }
      else {
        this.charging_state = 0;
      }
      if (initObj.hasOwnProperty('charge_percent')) {
        this.charge_percent = initObj.charge_percent
      }
      else {
        this.charge_percent = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type BatteryStatus
    // Serialize message field [charging_state]
    bufferOffset = _serializer.int8(obj.charging_state, buffer, bufferOffset);
    // Serialize message field [charge_percent]
    bufferOffset = _serializer.float32(obj.charge_percent, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type BatteryStatus
    let len;
    let data = new BatteryStatus(null);
    // Deserialize message field [charging_state]
    data.charging_state = _deserializer.int8(buffer, bufferOffset);
    // Deserialize message field [charge_percent]
    data.charge_percent = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 5;
  }

  static datatype() {
    // Returns string type for a message object
    return 'rosarnl/BatteryStatus';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a5ae24af1ef085b1c28fd0c2d4869c5f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    int8 CHARGING_UNKNOWN = -1
    int8 CHARGING_NOT = 0
    int8 CHARGING_BULK = 1
    int8 CHARGING_OVERCHARGE = 2
    int8 CHARGING_FLOAT = 3
    int8 CHARGING_BALANCE = 4
    
    int8 charging_state
    float32 charge_percent
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new BatteryStatus(null);
    if (msg.charging_state !== undefined) {
      resolved.charging_state = msg.charging_state;
    }
    else {
      resolved.charging_state = 0
    }

    if (msg.charge_percent !== undefined) {
      resolved.charge_percent = msg.charge_percent;
    }
    else {
      resolved.charge_percent = 0.0
    }

    return resolved;
    }
};

// Constants for message
BatteryStatus.Constants = {
  CHARGING_UNKNOWN: -1,
  CHARGING_NOT: 0,
  CHARGING_BULK: 1,
  CHARGING_OVERCHARGE: 2,
  CHARGING_FLOAT: 3,
  CHARGING_BALANCE: 4,
}

module.exports = BatteryStatus;
