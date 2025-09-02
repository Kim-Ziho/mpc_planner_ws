// Auto-generated. Do not edit!

// (in-package roadmap_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class RoadPolyline {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.id = null;
      this.type = null;
      this.coords = null;
    }
    else {
      if (initObj.hasOwnProperty('id')) {
        this.id = initObj.id
      }
      else {
        this.id = 0;
      }
      if (initObj.hasOwnProperty('type')) {
        this.type = initObj.type
      }
      else {
        this.type = 0;
      }
      if (initObj.hasOwnProperty('coords')) {
        this.coords = initObj.coords
      }
      else {
        this.coords = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type RoadPolyline
    // Serialize message field [id]
    bufferOffset = _serializer.int32(obj.id, buffer, bufferOffset);
    // Serialize message field [type]
    bufferOffset = _serializer.uint8(obj.type, buffer, bufferOffset);
    // Serialize message field [coords]
    // Serialize the length for message field [coords]
    bufferOffset = _serializer.uint32(obj.coords.length, buffer, bufferOffset);
    obj.coords.forEach((val) => {
      bufferOffset = geometry_msgs.msg.Point.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type RoadPolyline
    let len;
    let data = new RoadPolyline(null);
    // Deserialize message field [id]
    data.id = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [type]
    data.type = _deserializer.uint8(buffer, bufferOffset);
    // Deserialize message field [coords]
    // Deserialize array length for message field [coords]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.coords = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.coords[i] = geometry_msgs.msg.Point.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 24 * object.coords.length;
    return length + 9;
  }

  static datatype() {
    // Returns string type for a message object
    return 'roadmap_msgs/RoadPolyline';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '35f06eaee5fd980da3b6582143b5a629';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new RoadPolyline(null);
    if (msg.id !== undefined) {
      resolved.id = msg.id;
    }
    else {
      resolved.id = 0
    }

    if (msg.type !== undefined) {
      resolved.type = msg.type;
    }
    else {
      resolved.type = 0
    }

    if (msg.coords !== undefined) {
      resolved.coords = new Array(msg.coords.length);
      for (let i = 0; i < resolved.coords.length; ++i) {
        resolved.coords[i] = geometry_msgs.msg.Point.Resolve(msg.coords[i]);
      }
    }
    else {
      resolved.coords = []
    }

    return resolved;
    }
};

// Constants for message
RoadPolyline.Constants = {
  LANECENTER_FREEWAY: 1,
  LANECENTER_SURFACESTREET: 2,
  LANECENTER_BIKELANE: 3,
  ROADLINE_BROKENSINGLEWHITE: 6,
  ROADLINE_SOLIDSINGLEWHITE: 7,
  ROADLINE_SOLIDDOUBLEWHITE: 8,
  ROADLINE_BROKENSINGLEYELLOW: 9,
  ROADLINE_BROKENDOUBLEYELLOW: 10,
  ROADLINE_SOLIDSINGLEYELLOW: 11,
  ROADLINE_SOLIDDOUBLEYELLOW: 12,
  ROADLINE_PASSINGDOUBLEYELLOW: 13,
  ROADEDGEBOUNDARY: 15,
  ROADEDGEMEDIAN: 16,
  STOPSIGN: 17,
  CROSSWALK: 18,
  SPEEDBUMP: 19,
}

module.exports = RoadPolyline;
