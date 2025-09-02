// Auto-generated. Do not edit!

// (in-package mpc_planner_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let Gaussian = require('./Gaussian.js');
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class ObstacleGMM {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.id = null;
      this.pose = null;
      this.gaussians = null;
      this.probabilities = null;
    }
    else {
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = 0;
      }
      if (initObj.hasOwnProperty('pose')) {
        this.pose = initObj.pose
      }
      else {
        this.pose = new geometry_msgs.msg.Pose();
      }
      if (initObj.hasOwnProperty('gaussians')) {
        this.gaussians = initObj.gaussians
      }
      else {
        this.gaussians = [];
      }
      if (initObj.hasOwnProperty('probabilities')) {
        this.probabilities = initObj.probabilities
      }
      else {
        this.probabilities = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ObstacleGMM
    // Serialize message field [id]
    bufferOffset = _serializer.int32(obj.id, buffer, bufferOffset);
    // Serialize message field [pose]
    bufferOffset = geometry_msgs.msg.Pose.serialize(obj.pose, buffer, bufferOffset);
    // Serialize message field [gaussians]
    // Serialize the length for message field [gaussians]
    bufferOffset = _serializer.uint32(obj.gaussians.length, buffer, bufferOffset);
    obj.gaussians.forEach((val) => {
      bufferOffset = Gaussian.serialize(val, buffer, bufferOffset);
    });
    // Serialize message field [probabilities]
    bufferOffset = _arraySerializer.float64(obj.probabilities, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ObstacleGMM
    let len;
    let data = new ObstacleGMM(null);
    // Deserialize message field [id]
    data.id = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [pose]
    data.pose = geometry_msgs.msg.Pose.deserialize(buffer, bufferOffset);
    // Deserialize message field [gaussians]
    // Deserialize array length for message field [gaussians]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.gaussians = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.gaussians[i] = Gaussian.deserialize(buffer, bufferOffset)
    }
    // Deserialize message field [probabilities]
    data.probabilities = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    object.gaussians.forEach((val) => {
      length += Gaussian.getMessageSize(val);
    });
    length += 8 * object.probabilities.length;
    return length + 68;
  }

  static datatype() {
    // Returns string type for a message object
    return 'mpc_planner_msgs/ObstacleGMM';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '43fafe3d1d2bd26d6dea45245b7944b1';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # ID for obstacle association
    int32 id
    
    # Current pose of the obstacle
    geometry_msgs/Pose pose
    
    # List of Gaussians and their probabilities
    mpc_planner_msgs/Gaussian[] gaussians
    float64[] probabilities
    
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    ================================================================================
    MSG: mpc_planner_msgs/Gaussian
    # Trajectory of the mean prediction
    nav_msgs/Path mean
    
    # Covariances decomposed into their major and minor axes
    float64[] major_semiaxis
    float64[] minor_semiaxis
    ================================================================================
    MSG: nav_msgs/Path
    #An array of poses that represents a Path for a robot to follow
    Header header
    geometry_msgs/PoseStamped[] poses
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    ================================================================================
    MSG: geometry_msgs/PoseStamped
    # A Pose with reference coordinate frame and timestamp
    Header header
    Pose pose
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ObstacleGMM(null);
    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = 0
    }

    if (msg.pose !== undefined) {
      resolved.pose = geometry_msgs.msg.Pose.Resolve(msg.pose)
    }
    else {
      resolved.pose = new geometry_msgs.msg.Pose()
    }

    if (msg.gaussians !== undefined) {
      resolved.gaussians = new Array(msg.gaussians.length);
      for (let i = 0; i < resolved.gaussians.length; ++i) {
        resolved.gaussians[i] = Gaussian.Resolve(msg.gaussians[i]);
      }
    }
    else {
      resolved.gaussians = []
    }

    if (msg.probabilities !== undefined) {
      resolved.probabilities = msg.probabilities;
    }
    else {
      resolved.probabilities = []
    }

    return resolved;
    }
};

module.exports = ObstacleGMM;
