// Auto-generated. Do not edit!

// (in-package cyborg_controller.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class EmotionalStateServiceRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.nothing = null;
    }
    else {
      if (initObj.hasOwnProperty('nothing')) {
        this.nothing = initObj.nothing
      }
      else {
        this.nothing = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type EmotionalStateServiceRequest
    // Serialize message field [nothing]
    bufferOffset = _serializer.string(obj.nothing, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type EmotionalStateServiceRequest
    let len;
    let data = new EmotionalStateServiceRequest(null);
    // Deserialize message field [nothing]
    data.nothing = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += object.nothing.length;
    return length + 4;
  }

  static datatype() {
    // Returns string type for a service object
    return 'cyborg_controller/EmotionalStateServiceRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'ddfffbaa83f4c3ce761af512040042eb';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string nothing
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new EmotionalStateServiceRequest(null);
    if (msg.nothing !== undefined) {
      resolved.nothing = msg.nothing;
    }
    else {
      resolved.nothing = ''
    }

    return resolved;
    }
};

class EmotionalStateServiceResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.emotional_state = null;
      this.current_pleasure = null;
      this.current_arousal = null;
      this.current_dominance = null;
    }
    else {
      if (initObj.hasOwnProperty('emotional_state')) {
        this.emotional_state = initObj.emotional_state
      }
      else {
        this.emotional_state = '';
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
    // Serializes a message object of type EmotionalStateServiceResponse
    // Serialize message field [emotional_state]
    bufferOffset = _serializer.string(obj.emotional_state, buffer, bufferOffset);
    // Serialize message field [current_pleasure]
    bufferOffset = _serializer.float32(obj.current_pleasure, buffer, bufferOffset);
    // Serialize message field [current_arousal]
    bufferOffset = _serializer.float32(obj.current_arousal, buffer, bufferOffset);
    // Serialize message field [current_dominance]
    bufferOffset = _serializer.float32(obj.current_dominance, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type EmotionalStateServiceResponse
    let len;
    let data = new EmotionalStateServiceResponse(null);
    // Deserialize message field [emotional_state]
    data.emotional_state = _deserializer.string(buffer, bufferOffset);
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
    length += object.emotional_state.length;
    return length + 16;
  }

  static datatype() {
    // Returns string type for a service object
    return 'cyborg_controller/EmotionalStateServiceResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6c0110dd20a28b505d55f2e776c50490';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    string emotional_state
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
    const resolved = new EmotionalStateServiceResponse(null);
    if (msg.emotional_state !== undefined) {
      resolved.emotional_state = msg.emotional_state;
    }
    else {
      resolved.emotional_state = ''
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

module.exports = {
  Request: EmotionalStateServiceRequest,
  Response: EmotionalStateServiceResponse,
  md5sum() { return '68da7087fb08817fd317a07916f3d49f'; },
  datatype() { return 'cyborg_controller/EmotionalStateService'; }
};
