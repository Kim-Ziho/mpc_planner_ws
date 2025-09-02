;; Auto-generated. Do not edit!


(when (boundp 'roadmap_msgs::RoadPolyline)
  (if (not (find-package "ROADMAP_MSGS"))
    (make-package "ROADMAP_MSGS"))
  (shadow 'RoadPolyline (find-package "ROADMAP_MSGS")))
(unless (find-package "ROADMAP_MSGS::ROADPOLYLINE")
  (make-package "ROADMAP_MSGS::ROADPOLYLINE"))

(in-package "ROS")
;;//! \htmlinclude RoadPolyline.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))


(intern "*LANECENTER_FREEWAY*" (find-package "ROADMAP_MSGS::ROADPOLYLINE"))
(shadow '*LANECENTER_FREEWAY* (find-package "ROADMAP_MSGS::ROADPOLYLINE"))
(defconstant roadmap_msgs::RoadPolyline::*LANECENTER_FREEWAY* 1)
(intern "*LANECENTER_SURFACESTREET*" (find-package "ROADMAP_MSGS::ROADPOLYLINE"))
(shadow '*LANECENTER_SURFACESTREET* (find-package "ROADMAP_MSGS::ROADPOLYLINE"))
(defconstant roadmap_msgs::RoadPolyline::*LANECENTER_SURFACESTREET* 2)
(intern "*LANECENTER_BIKELANE*" (find-package "ROADMAP_MSGS::ROADPOLYLINE"))
(shadow '*LANECENTER_BIKELANE* (find-package "ROADMAP_MSGS::ROADPOLYLINE"))
(defconstant roadmap_msgs::RoadPolyline::*LANECENTER_BIKELANE* 3)
(intern "*ROADLINE_BROKENSINGLEWHITE*" (find-package "ROADMAP_MSGS::ROADPOLYLINE"))
(shadow '*ROADLINE_BROKENSINGLEWHITE* (find-package "ROADMAP_MSGS::ROADPOLYLINE"))
(defconstant roadmap_msgs::RoadPolyline::*ROADLINE_BROKENSINGLEWHITE* 6)
(intern "*ROADLINE_SOLIDSINGLEWHITE*" (find-package "ROADMAP_MSGS::ROADPOLYLINE"))
(shadow '*ROADLINE_SOLIDSINGLEWHITE* (find-package "ROADMAP_MSGS::ROADPOLYLINE"))
(defconstant roadmap_msgs::RoadPolyline::*ROADLINE_SOLIDSINGLEWHITE* 7)
(intern "*ROADLINE_SOLIDDOUBLEWHITE*" (find-package "ROADMAP_MSGS::ROADPOLYLINE"))
(shadow '*ROADLINE_SOLIDDOUBLEWHITE* (find-package "ROADMAP_MSGS::ROADPOLYLINE"))
(defconstant roadmap_msgs::RoadPolyline::*ROADLINE_SOLIDDOUBLEWHITE* 8)
(intern "*ROADLINE_BROKENSINGLEYELLOW*" (find-package "ROADMAP_MSGS::ROADPOLYLINE"))
(shadow '*ROADLINE_BROKENSINGLEYELLOW* (find-package "ROADMAP_MSGS::ROADPOLYLINE"))
(defconstant roadmap_msgs::RoadPolyline::*ROADLINE_BROKENSINGLEYELLOW* 9)
(intern "*ROADLINE_BROKENDOUBLEYELLOW*" (find-package "ROADMAP_MSGS::ROADPOLYLINE"))
(shadow '*ROADLINE_BROKENDOUBLEYELLOW* (find-package "ROADMAP_MSGS::ROADPOLYLINE"))
(defconstant roadmap_msgs::RoadPolyline::*ROADLINE_BROKENDOUBLEYELLOW* 10)
(intern "*ROADLINE_SOLIDSINGLEYELLOW*" (find-package "ROADMAP_MSGS::ROADPOLYLINE"))
(shadow '*ROADLINE_SOLIDSINGLEYELLOW* (find-package "ROADMAP_MSGS::ROADPOLYLINE"))
(defconstant roadmap_msgs::RoadPolyline::*ROADLINE_SOLIDSINGLEYELLOW* 11)
(intern "*ROADLINE_SOLIDDOUBLEYELLOW*" (find-package "ROADMAP_MSGS::ROADPOLYLINE"))
(shadow '*ROADLINE_SOLIDDOUBLEYELLOW* (find-package "ROADMAP_MSGS::ROADPOLYLINE"))
(defconstant roadmap_msgs::RoadPolyline::*ROADLINE_SOLIDDOUBLEYELLOW* 12)
(intern "*ROADLINE_PASSINGDOUBLEYELLOW*" (find-package "ROADMAP_MSGS::ROADPOLYLINE"))
(shadow '*ROADLINE_PASSINGDOUBLEYELLOW* (find-package "ROADMAP_MSGS::ROADPOLYLINE"))
(defconstant roadmap_msgs::RoadPolyline::*ROADLINE_PASSINGDOUBLEYELLOW* 13)
(intern "*ROADEDGEBOUNDARY*" (find-package "ROADMAP_MSGS::ROADPOLYLINE"))
(shadow '*ROADEDGEBOUNDARY* (find-package "ROADMAP_MSGS::ROADPOLYLINE"))
(defconstant roadmap_msgs::RoadPolyline::*ROADEDGEBOUNDARY* 15)
(intern "*ROADEDGEMEDIAN*" (find-package "ROADMAP_MSGS::ROADPOLYLINE"))
(shadow '*ROADEDGEMEDIAN* (find-package "ROADMAP_MSGS::ROADPOLYLINE"))
(defconstant roadmap_msgs::RoadPolyline::*ROADEDGEMEDIAN* 16)
(intern "*STOPSIGN*" (find-package "ROADMAP_MSGS::ROADPOLYLINE"))
(shadow '*STOPSIGN* (find-package "ROADMAP_MSGS::ROADPOLYLINE"))
(defconstant roadmap_msgs::RoadPolyline::*STOPSIGN* 17)
(intern "*CROSSWALK*" (find-package "ROADMAP_MSGS::ROADPOLYLINE"))
(shadow '*CROSSWALK* (find-package "ROADMAP_MSGS::ROADPOLYLINE"))
(defconstant roadmap_msgs::RoadPolyline::*CROSSWALK* 18)
(intern "*SPEEDBUMP*" (find-package "ROADMAP_MSGS::ROADPOLYLINE"))
(shadow '*SPEEDBUMP* (find-package "ROADMAP_MSGS::ROADPOLYLINE"))
(defconstant roadmap_msgs::RoadPolyline::*SPEEDBUMP* 19)

(defun roadmap_msgs::RoadPolyline-to-symbol (const)
  (cond
        ((= const 1) 'roadmap_msgs::RoadPolyline::*LANECENTER_FREEWAY*)
        ((= const 2) 'roadmap_msgs::RoadPolyline::*LANECENTER_SURFACESTREET*)
        ((= const 3) 'roadmap_msgs::RoadPolyline::*LANECENTER_BIKELANE*)
        ((= const 6) 'roadmap_msgs::RoadPolyline::*ROADLINE_BROKENSINGLEWHITE*)
        ((= const 7) 'roadmap_msgs::RoadPolyline::*ROADLINE_SOLIDSINGLEWHITE*)
        ((= const 8) 'roadmap_msgs::RoadPolyline::*ROADLINE_SOLIDDOUBLEWHITE*)
        ((= const 9) 'roadmap_msgs::RoadPolyline::*ROADLINE_BROKENSINGLEYELLOW*)
        ((= const 10) 'roadmap_msgs::RoadPolyline::*ROADLINE_BROKENDOUBLEYELLOW*)
        ((= const 11) 'roadmap_msgs::RoadPolyline::*ROADLINE_SOLIDSINGLEYELLOW*)
        ((= const 12) 'roadmap_msgs::RoadPolyline::*ROADLINE_SOLIDDOUBLEYELLOW*)
        ((= const 13) 'roadmap_msgs::RoadPolyline::*ROADLINE_PASSINGDOUBLEYELLOW*)
        ((= const 15) 'roadmap_msgs::RoadPolyline::*ROADEDGEBOUNDARY*)
        ((= const 16) 'roadmap_msgs::RoadPolyline::*ROADEDGEMEDIAN*)
        ((= const 17) 'roadmap_msgs::RoadPolyline::*STOPSIGN*)
        ((= const 18) 'roadmap_msgs::RoadPolyline::*CROSSWALK*)
        ((= const 19) 'roadmap_msgs::RoadPolyline::*SPEEDBUMP*)
        (t nil)))

(defclass roadmap_msgs::RoadPolyline
  :super ros::object
  :slots (_id _type _coords ))

(defmethod roadmap_msgs::RoadPolyline
  (:init
   (&key
    ((:id __id) 0)
    ((:type __type) 0)
    ((:coords __coords) ())
    )
   (send-super :init)
   (setq _id (round __id))
   (setq _type (round __type))
   (setq _coords __coords)
   self)
  (:id
   (&optional __id)
   (if __id (setq _id __id)) _id)
  (:type
   (&optional __type)
   (if __type (setq _type __type)) _type)
  (:coords
   (&rest __coords)
   (if (keywordp (car __coords))
       (send* _coords __coords)
     (progn
       (if __coords (setq _coords (car __coords)))
       _coords)))
  (:serialization-length
   ()
   (+
    ;; int32 _id
    4
    ;; uint8 _type
    1
    ;; geometry_msgs/Point[] _coords
    (apply #'+ (send-all _coords :serialization-length)) 4
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; int32 _id
       (write-long _id s)
     ;; uint8 _type
       (write-byte _type s)
     ;; geometry_msgs/Point[] _coords
     (write-long (length _coords) s)
     (dolist (elem _coords)
       (send elem :serialize s)
       )
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; int32 _id
     (setq _id (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; uint8 _type
     (setq _type (sys::peek buf ptr- :char)) (incf ptr- 1)
   ;; geometry_msgs/Point[] _coords
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _coords (let (r) (dotimes (i n) (push (instance geometry_msgs::Point :init) r)) r))
     (dolist (elem- _coords)
     (send elem- :deserialize buf ptr-) (incf ptr- (send elem- :serialization-length))
     ))
   ;;
   self)
  )

(setf (get roadmap_msgs::RoadPolyline :md5sum-) "35f06eaee5fd980da3b6582143b5a629")
(setf (get roadmap_msgs::RoadPolyline :datatype-) "roadmap_msgs/RoadPolyline")
(setf (get roadmap_msgs::RoadPolyline :definition-)
      "# Road line identifier
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

")



(provide :roadmap_msgs/RoadPolyline "35f06eaee5fd980da3b6582143b5a629")


