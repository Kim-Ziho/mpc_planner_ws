// Auto-generated. Do not edit!

// (in-package roadmap_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let RoadPolyline = require('./RoadPolyline.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class RoadPolylineArray {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.road_polylines = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('road_polylines')) {
        this.road_polylines = initObj.road_polylines
      }
      else {
        this.road_polylines = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RoadPolylineArray
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [road_polylines]
    // Serialize the length for message field [road_polylines]
    bufferOffset = _serializer.uint32(obj.road_polylines.length, buffer, bufferOffset);
    obj.road_polylines.forEach((val) => {
      bufferOffset = RoadPolyline.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RoadPolylineArray
    let len;
    let data = new RoadPolylineArray(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [road_polylines]
    // Deserialize array length for message field [road_polylines]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.road_polylines = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.road_polylines[i] = RoadPolyline.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.road_polylines.forEach((val) => {
      length += RoadPolyline.getMessageSize(val);
    });
    return length + 4;
  }

  static datatype() {
    // Returns string type for a message object
    return 'roadmap_msgs/RoadPolylineArray';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4ef0cc5f23352e54e8abcc0a04846779';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    Header header
    
    roadmap_msgs/RoadPolyline[] road_polylines
    
    
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
    MSG: roadmap_msgs/RoadPolyline
    # Road line identifier
    int32 id
    
    # Type of road line 
    uint8 LANECENTER_FREEWAY=1
    uint8 LANECENTER_SURFACESTREET=2
    uint8 LANECENTER_BIKELANE=3
    uint8 ROADLINE_BROKENSINGLEWHITE=6
    uint8 ROADLINE_SOLIDSINGLEWHITE=7
    uint8 ROADLINE_SOLIDDOUBLEWHITE=8
    uint8 ROADLINE_BROKENSINGLEYELLOW=9
    uint8 ROADLINE_BROKENDOUBLEYELLOW=10
    uint8 ROADLINE_SOLIDSINGLEYELLOW=11
    uint8 ROADLINE_SOLIDDOUBLEYELLOW=12
    uint8 ROADLINE_PASSINGDOUBLEYELLOW=13
    uint8 ROADEDGEBOUNDARY=15
    uint8 ROADEDGEMEDIAN=16
    uint8 STOPSIGN=17
    uint8 CROSSWALK=18
    uint8 SPEEDBUMP=19
    
    uint8 type
    
    # Polyline coordinates
    geometry_msgs/Point[] coords
    
    
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new RoadPolylineArray(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.road_polylines !== undefined) {
      resolved.road_polylines = new Array(msg.road_polylines.length);
      for (let i = 0; i < resolved.road_polylines.length; ++i) {
        resolved.road_polylines[i] = RoadPolyline.Resolve(msg.road_polylines[i]);
      }
    }
    else {
      resolved.road_polylines = []
    }

    return resolved;
    }
};

module.exports = RoadPolylineArray;
