;; Auto-generated. Do not edit!


(when (boundp 'phidgets_ik::SetDigitalOutput)
  (if (not (find-package "PHIDGETS_IK"))
    (make-package "PHIDGETS_IK"))
  (shadow 'SetDigitalOutput (find-package "PHIDGETS_IK")))
(unless (find-package "PHIDGETS_IK::SETDIGITALOUTPUT")
  (make-package "PHIDGETS_IK::SETDIGITALOUTPUT"))
(unless (find-package "PHIDGETS_IK::SETDIGITALOUTPUTREQUEST")
  (make-package "PHIDGETS_IK::SETDIGITALOUTPUTREQUEST"))
(unless (find-package "PHIDGETS_IK::SETDIGITALOUTPUTRESPONSE")
  (make-package "PHIDGETS_IK::SETDIGITALOUTPUTRESPONSE"))

(in-package "ROS")





(defclass phidgets_ik::SetDigitalOutputRequest
  :super ros::object
  :slots (_index _state ))

(defmethod phidgets_ik::SetDigitalOutputRequest
  (:init
   (&key
    ((:index __index) 0)
    ((:state __state) nil)
    )
   (send-super :init)
   (setq _index (round __index))
   (setq _state __state)
   self)
  (:index
   (&optional __index)
   (if __index (setq _index __index)) _index)
  (:state
   (&optional __state)
   (if __state (setq _state __state)) _state)
  (:serialization-length
   ()
   (+
    ;; uint16 _index
    2
    ;; bool _state
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; uint16 _index
       (write-word _index s)
     ;; bool _state
       (if _state (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; uint16 _index
     (setq _index (sys::peek buf ptr- :short)) (incf ptr- 2)
   ;; bool _state
     (setq _state (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass phidgets_ik::SetDigitalOutputResponse
  :super ros::object
  :slots (_success ))

(defmethod phidgets_ik::SetDigitalOutputResponse
  (:init
   (&key
    ((:success __success) nil)
    )
   (send-super :init)
   (setq _success __success)
   self)
  (:success
   (&optional __success)
   (if __success (setq _success __success)) _success)
  (:serialization-length
   ()
   (+
    ;; bool _success
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; bool _success
       (if _success (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; bool _success
     (setq _success (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(defclass phidgets_ik::SetDigitalOutput
  :super ros::object
  :slots ())

(setf (get phidgets_ik::SetDigitalOutput :md5sum-) "8496af00b3dd95e3884fd81d8e38f019")
(setf (get phidgets_ik::SetDigitalOutput :datatype-) "phidgets_ik/SetDigitalOutput")
(setf (get phidgets_ik::SetDigitalOutput :request) phidgets_ik::SetDigitalOutputRequest)
(setf (get phidgets_ik::SetDigitalOutput :response) phidgets_ik::SetDigitalOutputResponse)

(defmethod phidgets_ik::SetDigitalOutputRequest
  (:response () (instance phidgets_ik::SetDigitalOutputResponse :init)))

(setf (get phidgets_ik::SetDigitalOutputRequest :md5sum-) "8496af00b3dd95e3884fd81d8e38f019")
(setf (get phidgets_ik::SetDigitalOutputRequest :datatype-) "phidgets_ik/SetDigitalOutputRequest")
(setf (get phidgets_ik::SetDigitalOutputRequest :definition-)
      "

uint16 index
bool state
---
bool success

")

(setf (get phidgets_ik::SetDigitalOutputResponse :md5sum-) "8496af00b3dd95e3884fd81d8e38f019")
(setf (get phidgets_ik::SetDigitalOutputResponse :datatype-) "phidgets_ik/SetDigitalOutputResponse")
(setf (get phidgets_ik::SetDigitalOutputResponse :definition-)
      "

uint16 index
bool state
---
bool success

")



(provide :phidgets_ik/SetDigitalOutput "8496af00b3dd95e3884fd81d8e38f019")


