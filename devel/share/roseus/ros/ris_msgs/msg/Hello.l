;; Auto-generated. Do not edit!


(when (boundp 'ris_msgs::Hello)
  (if (not (find-package "RIS_MSGS"))
    (make-package "RIS_MSGS"))
  (shadow 'Hello (find-package "RIS_MSGS")))
(unless (find-package "RIS_MSGS::HELLO")
  (make-package "RIS_MSGS::HELLO"))

(in-package "ROS")
;;//! \htmlinclude Hello.msg.html


(defclass ris_msgs::Hello
  :super ros::object
  :slots (_text ))

(defmethod ris_msgs::Hello
  (:init
   (&key
    ((:text __text) "")
    )
   (send-super :init)
   (setq _text (string __text))
   self)
  (:text
   (&optional __text)
   (if __text (setq _text __text)) _text)
  (:serialization-length
   ()
   (+
    ;; string _text
    4 (length _text)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _text
       (write-long (length _text) s) (princ _text s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _text
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _text (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get ris_msgs::Hello :md5sum-) "74697ed3d931f6eede8bf3a8dfeca160")
(setf (get ris_msgs::Hello :datatype-) "ris_msgs/Hello")
(setf (get ris_msgs::Hello :definition-)
      "string text
")



(provide :ris_msgs/Hello "74697ed3d931f6eede8bf3a8dfeca160")


