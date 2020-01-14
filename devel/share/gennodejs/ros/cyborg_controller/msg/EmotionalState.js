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

class EmotionalState {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.from_emotional_state = null;
      this.to_emotional_state = null;
      this.current_pleasure = null;
      this.current_arousal = null;
      this.current_dominance = null;
    }
    else {
      if (initObj.hasOwnProperty('from_emotional_state')) {
        this.from_emotional_state = initObj.from_emotional_state
      }
      else {
        this.from_emotional_state = '';
      }
      if (initObj.hasOwnProperty('to_emotional_state')) {
        this.to_emotional_state = initObj.to_emotional_state
      }
      else {
        this.to_emotional_state = '';
      }
      if (initObj.hasOwnProperty('current_pleasure')) {
        this.current_pleasure = initObj.current_pleasure
      }
      else {
        this.current_pleasure = 0.0;
      }
      if (initObj.hasOwnProperty('current_arousal')) {
        this.current_arousal = initObj.current_arousal
      }
      else {
        this.current_arousal = 0.0;
      }
      if (initObj.hasOwnProperty('current_dominance')) {
        this.current_dominance = initObj.current_dominance
      }
      else {
        this.current_dominance = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type EmotionalState
    // Serialize message field [from_emotional_state]
    bufferOffset = _serializer.string(obj.from_emotional_state, buffer, bufferOffset);
    // Serialize message field [to_emotional_state]
    bufferOffset = _serializer.string(obj.to_emotional_state, buffer, bufferOffset);
    // Serialize message field [current_pleasure]
    bufferOffset = _serializer.float32(obj.current_pleasure, buffer, bufferOffset);
    // Serialize message field [current_arousal]
    bufferOffset = _serializer.float32(obj.current_arousal, buffer, bufferOffset);
    // Serialize message field [current_dominance]
    bufferOffset = _serializer.float32(obj.current_dominance, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type EmotionalState
    let len;
    let data = new EmotionalState(null);
    // Deserialize message field [from_emotional_state]
    data.from_emotional_state = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [to_emotional_state]
    data.to_emotional_state = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [current_pleasure]
    data.current_pleasure = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [current_arousal]
    data.current_arousal = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [current_dominance]
    data.current_dominance = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.from_emotional_state.length;
    length += object.to_emotional_state.length;
    return length + 20;
  }

  static datatype() {
    // Returns string type for a message object
    return 'cyborg_controller/EmotionalState';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ff5f086e373f1e982e89fbc9298d99be';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string from_emotional_state
    string to_emotional_state
    float32 current_pleasure
    float32 current_arousal
    float32 current_dominance
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new EmotionalState(null);
    if (msg.from_emotional_state !== undefined) {
      resolved.from_emotional_state = msg.from_emotional_state;
    }
    else {
      resolved.from_emotional_state = ''
    }

    if (msg.to_emotional_state !== undefined) {
      resolved.to_emotional_state = msg.to_emotional_state;
    }
    else {
      resolved.to_emotional_state = ''
    }

    if (msg.current_pleasure !== undefined) {
      resolved.current_pleasure = msg.current_pleasure;
    }
    else {
      resolved.current_pleasure = 0.0
    }

    if (msg.current_arousal !== undefined) {
      resolved.current_arousal = msg.current_arousal;
    }
    else {
      resolved.current_arousal = 0.0
    }

    if (msg.current_dominance !== undefined) {
      resolved.current_dominance = msg.current_dominance;
    }
    else {
      resolved.current_dominance = 0.0
    }

    return resolved;
    }
};

module.exports = EmotionalState;
