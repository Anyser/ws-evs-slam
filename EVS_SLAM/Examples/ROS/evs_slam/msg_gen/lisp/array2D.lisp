; Auto-generated. Do not edit!


(cl:in-package orb_slam2-msg)


;//! \htmlinclude array2D.msg.html

(cl:defclass <array2D> (roslisp-msg-protocol:ros-message)
  ((data
    :reader data
    :initarg :data
    :type (cl:vector cl:fixnum)
   :initform (cl:make-array 0 :element-type 'cl:fixnum :initial-element 0)))
)

(cl:defclass array2D (<array2D>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <array2D>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'array2D)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name orb_slam2-msg:<array2D> is deprecated: use orb_slam2-msg:array2D instead.")))

(cl:ensure-generic-function 'data-val :lambda-list '(m))
(cl:defmethod data-val ((m <array2D>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader orb_slam2-msg:data-val is deprecated.  Use orb_slam2-msg:data instead.")
  (data m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <array2D>) ostream)
  "Serializes a message object of type '<array2D>"
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'data))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (cl:let* ((signed ele) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 256) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    ))
   (cl:slot-value msg 'data))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <array2D>) istream)
  "Deserializes a message object of type '<array2D>"
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'data) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'data)))
    (cl:dotimes (i __ros_arr_len)
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:aref vals i) (cl:if (cl:< unsigned 128) unsigned (cl:- unsigned 256)))))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<array2D>)))
  "Returns string type for a message object of type '<array2D>"
  "orb_slam2/array2D")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'array2D)))
  "Returns string type for a message object of type 'array2D"
  "orb_slam2/array2D")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<array2D>)))
  "Returns md5sum for a message object of type '<array2D>"
  "ac9c931aaf6ce145ea0383362e83c70b")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'array2D)))
  "Returns md5sum for a message object of type 'array2D"
  "ac9c931aaf6ce145ea0383362e83c70b")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<array2D>)))
  "Returns full string definition for message of type '<array2D>"
  (cl:format cl:nil "int8[] data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'array2D)))
  "Returns full string definition for message of type 'array2D"
  (cl:format cl:nil "int8[] data~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <array2D>))
  (cl:+ 0
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'data) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ 1)))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <array2D>))
  "Converts a ROS message object to a list"
  (cl:list 'array2D
    (cl:cons ':data (data msg))
))
