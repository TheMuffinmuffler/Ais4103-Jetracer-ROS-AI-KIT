; Auto-generated. Do not edit!


(cl:in-package ris_msgs-srv)


;//! \htmlinclude PingPong-request.msg.html

(cl:defclass <PingPong-request> (roslisp-msg-protocol:ros-message)
  ((input
    :reader input
    :initarg :input
    :type cl:string
    :initform ""))
)

(cl:defclass PingPong-request (<PingPong-request>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PingPong-request>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PingPong-request)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ris_msgs-srv:<PingPong-request> is deprecated: use ris_msgs-srv:PingPong-request instead.")))

(cl:ensure-generic-function 'input-val :lambda-list '(m))
(cl:defmethod input-val ((m <PingPong-request>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ris_msgs-srv:input-val is deprecated.  Use ris_msgs-srv:input instead.")
  (input m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PingPong-request>) ostream)
  "Serializes a message object of type '<PingPong-request>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'input))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'input))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PingPong-request>) istream)
  "Deserializes a message object of type '<PingPong-request>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'input) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'input) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PingPong-request>)))
  "Returns string type for a service object of type '<PingPong-request>"
  "ris_msgs/PingPongRequest")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PingPong-request)))
  "Returns string type for a service object of type 'PingPong-request"
  "ris_msgs/PingPongRequest")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PingPong-request>)))
  "Returns md5sum for a message object of type '<PingPong-request>"
  "c63e85f503b805d84df783e71c6bb0d2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PingPong-request)))
  "Returns md5sum for a message object of type 'PingPong-request"
  "c63e85f503b805d84df783e71c6bb0d2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PingPong-request>)))
  "Returns full string definition for message of type '<PingPong-request>"
  (cl:format cl:nil "string input~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PingPong-request)))
  "Returns full string definition for message of type 'PingPong-request"
  (cl:format cl:nil "string input~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PingPong-request>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'input))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PingPong-request>))
  "Converts a ROS message object to a list"
  (cl:list 'PingPong-request
    (cl:cons ':input (input msg))
))
;//! \htmlinclude PingPong-response.msg.html

(cl:defclass <PingPong-response> (roslisp-msg-protocol:ros-message)
  ((output
    :reader output
    :initarg :output
    :type cl:string
    :initform ""))
)

(cl:defclass PingPong-response (<PingPong-response>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <PingPong-response>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'PingPong-response)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name ris_msgs-srv:<PingPong-response> is deprecated: use ris_msgs-srv:PingPong-response instead.")))

(cl:ensure-generic-function 'output-val :lambda-list '(m))
(cl:defmethod output-val ((m <PingPong-response>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader ris_msgs-srv:output-val is deprecated.  Use ris_msgs-srv:output instead.")
  (output m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <PingPong-response>) ostream)
  "Serializes a message object of type '<PingPong-response>"
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'output))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'output))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <PingPong-response>) istream)
  "Deserializes a message object of type '<PingPong-response>"
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'output) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'output) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<PingPong-response>)))
  "Returns string type for a service object of type '<PingPong-response>"
  "ris_msgs/PingPongResponse")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PingPong-response)))
  "Returns string type for a service object of type 'PingPong-response"
  "ris_msgs/PingPongResponse")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<PingPong-response>)))
  "Returns md5sum for a message object of type '<PingPong-response>"
  "c63e85f503b805d84df783e71c6bb0d2")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'PingPong-response)))
  "Returns md5sum for a message object of type 'PingPong-response"
  "c63e85f503b805d84df783e71c6bb0d2")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<PingPong-response>)))
  "Returns full string definition for message of type '<PingPong-response>"
  (cl:format cl:nil "string output~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'PingPong-response)))
  "Returns full string definition for message of type 'PingPong-response"
  (cl:format cl:nil "string output~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <PingPong-response>))
  (cl:+ 0
     4 (cl:length (cl:slot-value msg 'output))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <PingPong-response>))
  "Converts a ROS message object to a list"
  (cl:list 'PingPong-response
    (cl:cons ':output (output msg))
))
(cl:defmethod roslisp-msg-protocol:service-request-type ((msg (cl:eql 'PingPong)))
  'PingPong-request)
(cl:defmethod roslisp-msg-protocol:service-response-type ((msg (cl:eql 'PingPong)))
  'PingPong-response)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'PingPong)))
  "Returns string type for a service object of type '<PingPong>"
  "ris_msgs/PingPong")