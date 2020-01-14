// Auto-generated. Do not edit!

// (in-package cyborg_controller.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class SystemState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.event = null;
      this.from_system_state = null;
      this.to_system_state = null;
    }
    else {
      if (initObj.hasOwnProperty('event')) {
        this.event = initObj.event
      }
      else {
        this.event = '';
      }
      if (initObj.hasOwnProperty('from_system_state')) {
        this.from_system_state = initObj.from_system_state
      }
      else {
        this.from_system_state = '';
      }
      if (initObj.hasOwnProperty('to_system_state')) {
        this.to_system_state = initObj.to_system_state
      }
      else {
        this.to_system_state = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type SystemState
    // Serialize message field [event]
    bufferOffset = _serializer.string(obj.event, buffer, bufferOffset);
    // Serialize message field [from_system_state]
    bufferOffset = _serializer.string(obj.from_system_state, buffer, bufferOffset);
    // Serialize message field [to_system_state]
    bufferOffset = _serializer.string(obj.to_system_state, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type SystemState
    let len;
    let data = new SystemState(null);
    // Deserialize message field [event]
    data.event = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [from_system_state]
    data.from_system_state = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [to_system_state]
    data.to_system_state = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.event.length;
    length += object.from_system_state.length;
    length += object.to_system_state.length;
    return length + 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'cyborg_controller/SystemState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '33e4fca93c330d18c8a8d3b755c771c5';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string event
    string from_system_state
    string to_system_state
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new SystemState(null);
    if (msg.event !== undefined) {
      resolved.event = msg.event;
    }
    else {
      resolved.event = ''
    }

    if (msg.from_system_state !== undefined) {
      resolved.from_system_state = msg.from_system_state;
    }
    else {
      resolved.from_system_state = ''
    }

    if (msg.to_system_state !== undefined) {
      resolved.to_system_state = msg.to_system_state;
    }
    else {
      resolved.to_system_state = ''
    }

    return resolved;
    }
};

module.exports = SystemState;
