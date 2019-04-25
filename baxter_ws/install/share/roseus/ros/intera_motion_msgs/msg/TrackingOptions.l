;; Auto-generated. Do not edit!


(when (boundp 'intera_motion_msgs::TrackingOptions)
  (if (not (find-package "INTERA_MOTION_MSGS"))
    (make-package "INTERA_MOTION_MSGS"))
  (shadow 'TrackingOptions (find-package "INTERA_MOTION_MSGS")))
(unless (find-package "INTERA_MOTION_MSGS::TRACKINGOPTIONS")
  (make-package "INTERA_MOTION_MSGS::TRACKINGOPTIONS"))

(in-package "ROS")
;;//! \htmlinclude TrackingOptions.msg.html


(defclass intera_motion_msgs::TrackingOptions
  :super ros::object
  :slots (_use_min_time_rate _min_time_rate _use_max_time_rate _max_time_rate _use_time_rate_accel _time_rate_accel _goal_joint_tolerance _use_goal_time_tolerance _goal_time_tolerance ))

(defmethod intera_motion_msgs::TrackingOptions
  (:init
   (&key
    ((:use_min_time_rate __use_min_time_rate) nil)
    ((:min_time_rate __min_time_rate) 0.0)
    ((:use_max_time_rate __use_max_time_rate) nil)
    ((:max_time_rate __max_time_rate) 0.0)
    ((:use_time_rate_accel __use_time_rate_accel) nil)
    ((:time_rate_accel __time_rate_accel) 0.0)
    ((:goal_joint_tolerance __goal_joint_tolerance) (make-array 0 :initial-element 0.0 :element-type :float))
    ((:use_goal_time_tolerance __use_goal_time_tolerance) nil)
    ((:goal_time_tolerance __goal_time_tolerance) 0.0)
    )
   (send-super :init)
   (setq _use_min_time_rate __use_min_time_rate)
   (setq _min_time_rate (float __min_time_rate))
   (setq _use_max_time_rate __use_max_time_rate)
   (setq _max_time_rate (float __max_time_rate))
   (setq _use_time_rate_accel __use_time_rate_accel)
   (setq _time_rate_accel (float __time_rate_accel))
   (setq _goal_joint_tolerance __goal_joint_tolerance)
   (setq _use_goal_time_tolerance __use_goal_time_tolerance)
   (setq _goal_time_tolerance (float __goal_time_tolerance))
   self)
  (:use_min_time_rate
   (&optional __use_min_time_rate)
   (if __use_min_time_rate (setq _use_min_time_rate __use_min_time_rate)) _use_min_time_rate)
  (:min_time_rate
   (&optional __min_time_rate)
   (if __min_time_rate (setq _min_time_rate __min_time_rate)) _min_time_rate)
  (:use_max_time_rate
   (&optional __use_max_time_rate)
   (if __use_max_time_rate (setq _use_max_time_rate __use_max_time_rate)) _use_max_time_rate)
  (:max_time_rate
   (&optional __max_time_rate)
   (if __max_time_rate (setq _max_time_rate __max_time_rate)) _max_time_rate)
  (:use_time_rate_accel
   (&optional __use_time_rate_accel)
   (if __use_time_rate_accel (setq _use_time_rate_accel __use_time_rate_accel)) _use_time_rate_accel)
  (:time_rate_accel
   (&optional __time_rate_accel)
   (if __time_rate_accel (setq _time_rate_accel __time_rate_accel)) _time_rate_accel)
  (:goal_joint_tolerance
   (&optional __goal_joint_tolerance)
   (if __goal_joint_tolerance (setq _goal_joint_tolerance __goal_joint_tolerance)) _goal_joint_tolerance)
  (:use_goal_time_tolerance
   (&optional __use_goal_time_tolerance)
   (if __use_goal_time_tolerance (setq _use_goal_time_tolerance __use_goal_time_tolerance)) _use_goal_time_tolerance)
  (:goal_time_tolerance
   (&optional __goal_time_tolerance)
   (if __goal_time_tolerance (setq _goal_time_tolerance __goal_time_tolerance)) _goal_time_tolerance)
  (:serialization-length
   ()
   (+
    ;; bool _use_min_time_rate
    1
    ;; float64 _min_time_rate
    8
    ;; bool _use_max_time_rate
    1
    ;; float64 _max_time_rate
    8
    ;; bool _use_time_rate_accel
    1
    ;; float64 _time_rate_accel
    8
    ;; float64[] _goal_joint_tolerance
    (* 8    (length _goal_joint_tolerance)) 4
    ;; bool _use_goal_time_tolerance
    1
    ;; float64 _goal_time_tolerance
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _use_min_time_rate
       (if _use_min_time_rate (write-byte -1 s) (write-byte 0 s))
     ;; float64 _min_time_rate
       (sys::poke _min_time_rate (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _use_max_time_rate
       (if _use_max_time_rate (write-byte -1 s) (write-byte 0 s))
     ;; float64 _max_time_rate
       (sys::poke _max_time_rate (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; bool _use_time_rate_accel
       (if _use_time_rate_accel (write-byte -1 s) (write-byte 0 s))
     ;; float64 _time_rate_accel
       (sys::poke _time_rate_accel (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64[] _goal_joint_tolerance
     (write-long (length _goal_joint_tolerance) s)
     (dotimes (i (length _goal_joint_tolerance))
       (sys::poke (elt _goal_joint_tolerance i) (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
       )
     ;; bool _use_goal_time_tolerance
       (if _use_goal_time_tolerance (write-byte -1 s) (write-byte 0 s))
     ;; float64 _goal_time_tolerance
       (sys::poke _goal_time_tolerance (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _use_min_time_rate
     (setq _use_min_time_rate (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float64 _min_time_rate
     (setq _min_time_rate (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _use_max_time_rate
     (setq _use_max_time_rate (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float64 _max_time_rate
     (setq _max_time_rate (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; bool _use_time_rate_accel
     (setq _use_time_rate_accel (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float64 _time_rate_accel
     (setq _time_rate_accel (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64[] _goal_joint_tolerance
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _goal_joint_tolerance (instantiate float-vector n))
     (dotimes (i n)
     (setf (elt _goal_joint_tolerance i) (sys::peek buf ptr- :double)) (incf ptr- 8)
     ))
   ;; bool _use_goal_time_tolerance
     (setq _use_goal_time_tolerance (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float64 _goal_time_tolerance
     (setq _goal_time_tolerance (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get intera_motion_msgs::TrackingOptions :md5sum-) "9db39097326cca64edfc125c068ee82f")
(setf (get intera_motion_msgs::TrackingOptions :datatype-) "intera_motion_msgs/TrackingOptions")
(setf (get intera_motion_msgs::TrackingOptions :definition-)
      "# Minimum trajectory tracking time rate:  (default = less than one)
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

")



(provide :intera_motion_msgs/TrackingOptions "9db39097326cca64edfc125c068ee82f")


