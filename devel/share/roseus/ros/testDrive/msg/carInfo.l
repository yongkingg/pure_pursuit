;; Auto-generated. Do not edit!


(when (boundp 'testDrive::carInfo)
  (if (not (find-package "TESTDRIVE"))
    (make-package "TESTDRIVE"))
  (shadow 'carInfo (find-package "TESTDRIVE")))
(unless (find-package "TESTDRIVE::CARINFO")
  (make-package "TESTDRIVE::CARINFO"))

(in-package "ROS")
;;//! \htmlinclude carInfo.msg.html


(defclass testDrive::carInfo
  :super ros::object
  :slots (_mode _enu_east _enu_north _enu_up _yaw _utm_x _utm_y _heading ))

(defmethod testDrive::carInfo
  (:init
   (&key
    ((:mode __mode) "")
    ((:enu_east __enu_east) 0.0)
    ((:enu_north __enu_north) 0.0)
    ((:enu_up __enu_up) 0.0)
    ((:yaw __yaw) 0.0)
    ((:utm_x __utm_x) 0.0)
    ((:utm_y __utm_y) 0.0)
    ((:heading __heading) 0.0)
    )
   (send-super :init)
   (setq _mode (string __mode))
   (setq _enu_east (float __enu_east))
   (setq _enu_north (float __enu_north))
   (setq _enu_up (float __enu_up))
   (setq _yaw (float __yaw))
   (setq _utm_x (float __utm_x))
   (setq _utm_y (float __utm_y))
   (setq _heading (float __heading))
   self)
  (:mode
   (&optional __mode)
   (if __mode (setq _mode __mode)) _mode)
  (:enu_east
   (&optional __enu_east)
   (if __enu_east (setq _enu_east __enu_east)) _enu_east)
  (:enu_north
   (&optional __enu_north)
   (if __enu_north (setq _enu_north __enu_north)) _enu_north)
  (:enu_up
   (&optional __enu_up)
   (if __enu_up (setq _enu_up __enu_up)) _enu_up)
  (:yaw
   (&optional __yaw)
   (if __yaw (setq _yaw __yaw)) _yaw)
  (:utm_x
   (&optional __utm_x)
   (if __utm_x (setq _utm_x __utm_x)) _utm_x)
  (:utm_y
   (&optional __utm_y)
   (if __utm_y (setq _utm_y __utm_y)) _utm_y)
  (:heading
   (&optional __heading)
   (if __heading (setq _heading __heading)) _heading)
  (:serialization-length
   ()
   (+
    ;; string _mode
    4 (length _mode)
    ;; float64 _enu_east
    8
    ;; float64 _enu_north
    8
    ;; float64 _enu_up
    8
    ;; float64 _yaw
    8
    ;; float64 _utm_x
    8
    ;; float64 _utm_y
    8
    ;; float64 _heading
    8
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _mode
       (write-long (length _mode) s) (princ _mode s)
     ;; float64 _enu_east
       (sys::poke _enu_east (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _enu_north
       (sys::poke _enu_north (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _enu_up
       (sys::poke _enu_up (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _yaw
       (sys::poke _yaw (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _utm_x
       (sys::poke _utm_x (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _utm_y
       (sys::poke _utm_y (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _heading
       (sys::poke _heading (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _mode
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _mode (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _enu_east
     (setq _enu_east (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _enu_north
     (setq _enu_north (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _enu_up
     (setq _enu_up (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _yaw
     (setq _yaw (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _utm_x
     (setq _utm_x (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _utm_y
     (setq _utm_y (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _heading
     (setq _heading (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;;
   self)
  )

(setf (get testDrive::carInfo :md5sum-) "e597445d8085e6b97cea6f509d947ddb")
(setf (get testDrive::carInfo :datatype-) "testDrive/carInfo")
(setf (get testDrive::carInfo :definition-)
      "# 모드
string mode  # 사용할 좌표계: \"ENU\" 또는 \"UTM\"

# ENU 좌표계
float64 enu_east
float64 enu_north
float64 enu_up
float64 yaw

# WGS84 좌표계
float64 utm_x
float64 utm_y
float64 heading


")



(provide :testDrive/carInfo "e597445d8085e6b97cea6f509d947ddb")


