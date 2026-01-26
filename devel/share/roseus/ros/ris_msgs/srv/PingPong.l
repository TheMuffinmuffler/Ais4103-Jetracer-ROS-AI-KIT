;; Auto-generated. Do not edit!


(when (boundp 'ris_msgs::PingPong)
  (if (not (find-package "RIS_MSGS"))
    (make-package "RIS_MSGS"))
  (shadow 'PingPong (find-package "RIS_MSGS")))
(unless (find-package "RIS_MSGS::PINGPONG")
  (make-package "RIS_MSGS::PINGPONG"))
(unless (find-package "RIS_MSGS::PINGPONGREQUEST")
  (make-package "RIS_MSGS::PINGPONGREQUEST"))
(unless (find-package "RIS_MSGS::PINGPONGRESPONSE")
  (make-package "RIS_MSGS::PINGPONGRESPONSE"))

(in-package "ROS")





(defclass ris_msgs::PingPongRequest
  :super ros::object
  :slots (_input ))

(defmethod ris_msgs::PingPongRequest
  (:init
   (&key
    ((:input __input) "")
    )
   (send-super :init)
   (setq _input (string __input))
   self)
  (:input
   (&optional __input)
   (if __input (setq _input __input)) _input)
  (:serialization-length
   ()
   (+
    ;; string _input
    4 (length _input)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _input
       (write-long (length _input) s) (princ _input s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _input
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _input (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass ris_msgs::PingPongResponse
  :super ros::object
  :slots (_output ))

(defmethod ris_msgs::PingPongResponse
  (:init
   (&key
    ((:output __output) "")
    )
   (send-super :init)
   (setq _output (string __output))
   self)
  (:output
   (&optional __output)
   (if __output (setq _output __output)) _output)
  (:serialization-length
   ()
   (+
    ;; string _output
    4 (length _output)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _output
       (write-long (length _output) s) (princ _output s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _output
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _output (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(defclass ris_msgs::PingPong
  :super ros::object
  :slots ())

(setf (get ris_msgs::PingPong :md5sum-) "c63e85f503b805d84df783e71c6bb0d2")
(setf (get ris_msgs::PingPong :datatype-) "ris_msgs/PingPong")
(setf (get ris_msgs::PingPong :request) ris_msgs::PingPongRequest)
(setf (get ris_msgs::PingPong :response) ris_msgs::PingPongResponse)

(defmethod ris_msgs::PingPongRequest
  (:response () (instance ris_msgs::PingPongResponse :init)))

(setf (get ris_msgs::PingPongRequest :md5sum-) "c63e85f503b805d84df783e71c6bb0d2")
(setf (get ris_msgs::PingPongRequest :datatype-) "ris_msgs/PingPongRequest")
(setf (get ris_msgs::PingPongRequest :definition-)
      "string input
---
string output
")

(setf (get ris_msgs::PingPongResponse :md5sum-) "c63e85f503b805d84df783e71c6bb0d2")
(setf (get ris_msgs::PingPongResponse :datatype-) "ris_msgs/PingPongResponse")
(setf (get ris_msgs::PingPongResponse :definition-)
      "string input
---
string output
")



(provide :ris_msgs/PingPong "c63e85f503b805d84df783e71c6bb0d2")


