// Auto-generated. Do not edit!

// (in-package mpc_planner_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let nav_msgs = _finder('nav_msgs');

//-----------------------------------------------------------

class Gaussian {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.mean = null;
      this.major_semiaxis = null;
      this.minor_semiaxis = null;
    }
    else {
      if (initObj.hasOwnProperty('mean')) {
        this.mean = initObj.mean
      }
      else {
        this.mean = new nav_msgs.msg.Path();
      }
      if (initObj.hasOwnProperty('major_semiaxis')) {
        this.major_semiaxis = initObj.major_semiaxis
      }
      else {
        this.major_semiaxis = [];
      }
      if (initObj.hasOwnProperty('minor_semiaxis')) {
        this.minor_semiaxis = initObj.minor_semiaxis
      }
      else {
        this.minor_semiaxis = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Gaussian
    // Serialize message field [mean]
    bufferOffset = nav_msgs.msg.Path.serialize(obj.mean, buffer, bufferOffset);
    // Serialize message field [major_semiaxis]
    bufferOffset = _arraySerializer.float64(obj.major_semiaxis, buffer, bufferOffset, null);
    // Serialize message field [minor_semiaxis]
    bufferOffset = _arraySerializer.float64(obj.minor_semiaxis, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Gaussian
    let len;
    let data = new Gaussian(null);
    // Deserialize message field [mean]
    data.mean = nav_msgs.msg.Path.deserialize(buffer, bufferOffset);
    // Deserialize message field [major_semiaxis]
    data.major_semiaxis = _arrayDeserializer.float64(buffer, bufferOffset, null)
    // Deserialize message field [minor_semiaxis]
    data.minor_semiaxis = _arrayDeserializer.float64(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += nav_msgs.msg.Path.getMessageSize(object.mean);
    length += 8 * object.major_semiaxis.length;
    length += 8 * object.minor_semiaxis.length;
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'mpc_planner_msgs/Gaussian';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '850460d51db9d70a66e94a860b9ab01d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Gaussian(null);
    if (msg.mean !== undefined) {
      resolved.mean = nav_msgs.msg.Path.Resolve(msg.mean)
    }
    else {
      resolved.mean = new nav_msgs.msg.Path()
    }

    if (msg.major_semiaxis !== undefined) {
      resolved.major_semiaxis = msg.major_semiaxis;
    }
    else {
      resolved.major_semiaxis = []
    }

    if (msg.minor_semiaxis !== undefined) {
      resolved.minor_semiaxis = msg.minor_semiaxis;
    }
    else {
      resolved.minor_semiaxis = []
    }

    return resolved;
    }
};

module.exports = Gaussian;
