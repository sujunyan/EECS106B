// Auto-generated. Do not edit!

// (in-package intera_motion_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class TrackingOptions {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.use_min_time_rate = null;
      this.min_time_rate = null;
      this.use_max_time_rate = null;
      this.max_time_rate = null;
      this.use_time_rate_accel = null;
      this.time_rate_accel = null;
      this.goal_joint_tolerance = null;
      this.use_goal_time_tolerance = null;
      this.goal_time_tolerance = null;
    }
    else {
      if (initObj.hasOwnProperty('use_min_time_rate')) {
        this.use_min_time_rate = initObj.use_min_time_rate
      }
      else {
        this.use_min_time_rate = false;
      }
      if (initObj.hasOwnProperty('min_time_rate')) {
        this.min_time_rate = initObj.min_time_rate
      }
      else {
        this.min_time_rate = 0.0;
      }
      if (initObj.hasOwnProperty('use_max_time_rate')) {
        this.use_max_time_rate = initObj.use_max_time_rate
      }
      else {
        this.use_max_time_rate = false;
      }
      if (initObj.hasOwnProperty('max_time_rate')) {
        this.max_time_rate = initObj.max_time_rate
      }
      else {
        this.max_time_rate = 0.0;
      }
      if (initObj.hasOwnProperty('use_time_rate_accel')) {
        this.use_time_rate_accel = initObj.use_time_rate_accel
      }
      else {
        this.use_time_rate_accel = false;
      }
      if (initObj.hasOwnProperty('time_rate_accel')) {
        this.time_rate_accel = initObj.time_rate_accel
      }
      else {
        this.time_rate_accel = 0.0;
      }
      if (initObj.hasOwnProperty('goal_joint_tolerance')) {
        this.goal_joint_tolerance = initObj.goal_joint_tolerance
      }
      else {
        this.goal_joint_tolerance = [];
      }
      if (initObj.hasOwnProperty('use_goal_time_tolerance')) {
        this.use_goal_time_tolerance = initObj.use_goal_time_tolerance
      }
      else {
        this.use_goal_time_tolerance = false;
      }
      if (initObj.hasOwnProperty('goal_time_tolerance')) {
        this.goal_time_tolerance = initObj.goal_time_tolerance
      }
      else {
        this.goal_time_tolerance = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type TrackingOptions
    // Serialize message field [use_min_time_rate]
    bufferOffset = _serializer.bool(obj.use_min_time_rate, buffer, bufferOffset);
    // Serialize message field [min_time_rate]
    bufferOffset = _serializer.float64(obj.min_time_rate, buffer, bufferOffset);
    // Serialize message field [use_max_time_rate]
    bufferOffset = _serializer.bool(obj.use_max_time_rate, buffer, bufferOffset);
    // Serialize message field [max_time_rate]
    bufferOffset = _serializer.float64(obj.max_time_rate, buffer, bufferOffset);
    // Serialize message field [use_time_rate_accel]
    bufferOffset = _serializer.bool(obj.use_time_rate_accel, buffer, bufferOffset);
    // Serialize message field [time_rate_accel]
    bufferOffset = _serializer.float64(obj.time_rate_accel, buffer, bufferOffset);
    // Serialize message field [goal_joint_tolerance]
    bufferOffset = _arraySerializer.float64(obj.goal_joint_tolerance, buffer, bufferOffset, null);
    // Serialize message field [use_goal_time_tolerance]
    bufferOffset = _serializer.bool(obj.use_goal_time_tolerance, buffer, bufferOffset);
    // Serialize message field [goal_time_tolerance]
    bufferOffset = _serializer.float64(obj.goal_time_tolerance, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type TrackingOptions
    let len;
    let data = new TrackingOptions(null);
    // Deserialize message field [use_min_time_rate]
    data.use_min_time_rate = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [min_time_rate]
    data.min_time_rate = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [use_max_time_rate]
    data.use_max_time_rate = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [max_time_rate]
    data.max_time_rate = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [use_time_rate_accel]
    data.use_time_rate_accel = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [time_rate_accel]
    data.time_rate_accel = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [goal_joint_tolerance]
    data.goal_joint_tolerance = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [use_goal_time_tolerance]
    data.use_goal_time_tolerance = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [goal_time_tolerance]
    data.goal_time_tolerance = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 8 * object.goal_joint_tolerance.length;
    return length + 40;
  }

  static datatype() {
    // Returns string type for a message object
    return 'intera_motion_msgs/TrackingOptions';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '9db39097326cca64edfc125c068ee82f';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # Minimum trajectory tracking time rate:  (default = less than one)
    bool     use_min_time_rate
    float64  min_time_rate
    
    # Maximum trajectory tracking time rate:  (1.0 = real-time = default)
    bool     use_max_time_rate
    float64  max_time_rate
    
    # How quickly to change the tracking time rate
    bool     use_time_rate_accel
    float64  time_rate_accel
    
    # How close (in rad) each joint should be to the final goal
    float64[] goal_joint_tolerance
    
    # Settling time after reaching the end of the trajectory
    bool     use_goal_time_tolerance
    float64  goal_time_tolerance
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new TrackingOptions(null);
    if (msg.use_min_time_rate !== undefined) {
      resolved.use_min_time_rate = msg.use_min_time_rate;
    }
    else {
      resolved.use_min_time_rate = false
    }

    if (msg.min_time_rate !== undefined) {
      resolved.min_time_rate = msg.min_time_rate;
    }
    else {
      resolved.min_time_rate = 0.0
    }

    if (msg.use_max_time_rate !== undefined) {
      resolved.use_max_time_rate = msg.use_max_time_rate;
    }
    else {
      resolved.use_max_time_rate = false
    }

    if (msg.max_time_rate !== undefined) {
      resolved.max_time_rate = msg.max_time_rate;
    }
    else {
      resolved.max_time_rate = 0.0
    }

    if (msg.use_time_rate_accel !== undefined) {
      resolved.use_time_rate_accel = msg.use_time_rate_accel;
    }
    else {
      resolved.use_time_rate_accel = false
    }

    if (msg.time_rate_accel !== undefined) {
      resolved.time_rate_accel = msg.time_rate_accel;
    }
    else {
      resolved.time_rate_accel = 0.0
    }

    if (msg.goal_joint_tolerance !== undefined) {
      resolved.goal_joint_tolerance = msg.goal_joint_tolerance;
    }
    else {
      resolved.goal_joint_tolerance = []
    }

    if (msg.use_goal_time_tolerance !== undefined) {
      resolved.use_goal_time_tolerance = msg.use_goal_time_tolerance;
    }
    else {
      resolved.use_goal_time_tolerance = false
    }

    if (msg.goal_time_tolerance !== undefined) {
      resolved.goal_time_tolerance = msg.goal_time_tolerance;
    }
    else {
      resolved.goal_time_tolerance = 0.0
    }

    return resolved;
    }
};

module.exports = TrackingOptions;
