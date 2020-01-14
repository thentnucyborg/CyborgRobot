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

class EmotionalFeedback {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.delta_pleasure = null;
      this.delta_arousal = null;
      this.delta_dominance = null;
    }
    else {
      if (initObj.hasOwnProperty('delta_pleasure')) {
        this.delta_pleasure = initObj.delta_pleasure
      }
      else {
        this.delta_pleasure = 0.0;
      }
      if (initObj.hasOwnProperty('delta_arousal')) {
        this.delta_arousal = initObj.delta_arousal
      }
      else {
        this.delta_arousal = 0.0;
      }
      if (initObj.hasOwnProperty('delta_dominance')) {
        this.delta_dominance = initObj.delta_dominance
      }
      else {
        this.delta_dominance = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type EmotionalFeedback
    // Serialize message field [delta_pleasure]
    bufferOffset = _serializer.float32(obj.delta_pleasure, buffer, bufferOffset);
    // Serialize message field [delta_arousal]
    bufferOffset = _serializer.float32(obj.delta_arousal, buffer, bufferOffset);
    // Serialize message field [delta_dominance]
    bufferOffset = _serializer.float32(obj.delta_dominance, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type EmotionalFeedback
    let len;
    let data = new EmotionalFeedback(null);
    // Deserialize message field [delta_pleasure]
    data.delta_pleasure = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [delta_arousal]
    data.delta_arousal = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [delta_dominance]
    data.delta_dominance = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 12;
  }

  static datatype() {
    // Returns string type for a message object
    return 'cyborg_controller/EmotionalFeedback';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6f61bf537c02522f197246d02ded0f2c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 delta_pleasure # Changes in pleasure
    float32 delta_arousal # Changes in arousal
    float32 delta_dominance #Changes in dominance
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new EmotionalFeedback(null);
    if (msg.delta_pleasure !== undefined) {
      resolved.delta_pleasure = msg.delta_pleasure;
    }
    else {
      resolved.delta_pleasure = 0.0
    }

    if (msg.delta_arousal !== undefined) {
      resolved.delta_arousal = msg.delta_arousal;
    }
    else {
      resolved.delta_arousal = 0.0
    }

    if (msg.delta_dominance !== undefined) {
      resolved.delta_dominance = msg.delta_dominance;
    }
    else {
      resolved.delta_dominance = 0.0
    }

    return resolved;
    }
};

module.exports = EmotionalFeedback;
