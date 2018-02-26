;;; Copyright (c) 2014, Jan Winkler <winkler@cs.uni-bremen.de>
;;; All rights reserved.
;;; 
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;; 
;;;     * Redistributions of source code must retain the above copyright
;;;       notice, this list of conditions and the following disclaimer.
;;;     * Redistributions in binary form must reproduce the above copyright
;;;       notice, this list of conditions and the following disclaimer in the
;;;       documentation and/or other materials provided with the distribution.
;;;     * Neither the name of the Universitaet Bremen nor the names of its contributors 
;;;       may be used to endorse or promote products derived from this software 
;;;       without specific prior written permission.
;;; 
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.

(in-package :cram-moveit)

(defclass collision-matrix ()
  ((names :reader names :initarg :names)
   (entries :reader entries :initarg :entries)))

(defgeneric relative-collision-matrix (names-groups-1 names-groups-2 values &key matrix))
(defgeneric relative-collision-matrix-msg (names-groups-1 names-groups-2 values &key matrix))

(defun get-planning-scene (components)
  (call-service "/get_planning_scene"
                'moveit_msgs-srv:GetPlanningScene
                :components
                (make-message "moveit_msgs/PlanningSceneComponents"
                              :components components)))


(defun get-planning-scene-info (&key scene-settings robot-state
                                  robot-state-attached-objects world-object-names
                                  world-object-geometry octomap transforms
                                  allowed-collision-matrix link-padding-and-scaling
                                  object-colors)
  "Function to query the state of the planning scene.
What information is present depends on which &key parameters are set to non-nil.

The response is a list of pairs (string object),
where string names the information contained in object.
A list of possible queries follows.
:scene-settings T will result in (\"planning scene name\" ps-name-string)
 (\"robot model name\" rm-name-string) to be added to the response.
:robot-state-attached-objects T will result in
 (\"robot state with attached objects\" robot-state-msg) to be added to the response.
:robot-state T, if :robot-state-attached-objects is nil,
 will result in (\"robot state\" robot-state-msg) to be added to the response.
If both :robot-state and :robot-state-attached-objects are T,
 the response is as shown for :robot-state-attached-objects T.
:world-object-names T will result in (\"world object names\" list-of-strings)
 to be added to the response.
:world-object-geometry T will result in
 (\"world object geometry\" vector-of-collision-object-msgs) to be added to the response.
:octomap T will result in (\"octomap\" octomap-msg) to be added to the response.
:transforms T will result in (\"fixed frame transforms\" vector-of-pose-stamped-msgs)
 to be added to the response.
:allowed-collision-matrix T will result in
 (\"allowed collision matrix\" allowed-collision-matrix) to be added to the response.
Note, the response does not contain a ROS message.
:link-padding-and-scaling T will result in
 (\"link padding\" vector-of-link-padding-msg) (\"link scaling\" vector-of-link-scaling-msg)
 to be added to the response.
:object-colors T will result in (\"object colors\" vector-of-object-color-msg)
 to be added to the response."
  (let* ((components
           (apply #'logior
                  (mapcar (lambda (a b)
                            (if a (roslisp-msg-protocol:symbol-code
                                   'moveit_msgs-msg:planningscenecomponents b) 0))
                          (list scene-settings robot-state
                                robot-state-attached-objects world-object-names
                                world-object-geometry octomap transforms
                                allowed-collision-matrix link-padding-and-scaling
                                object-colors)
                          (list :scene_settings :robot_state
                                :robot_state_attached_objects :world_object_names 
                                :world_object_geometry :octomap :transforms
                                :allowed_collision_matrix :link_padding_and_scaling
                                :object_colors))))
        (result (get-planning-scene components)))
          (roslisp:with-fields ((allowed-collision-matrix-f (allowed_collision_matrix scene)) 
                                (fixed-frame-transforms-f (fixed_frame_transforms scene))
                                (link-padding-f (link_padding scene))
                                (link-scaling-f (link_scale scene))
                                (name-f (name scene))
                                (object-colors-f (object_colors scene))
                                (robot-model-name-f (robot_model_name scene))
                                (robot-state-f (robot_state scene))
                                (collision-objects-f (collision_objects world scene))
                                (octomap-f (octomap world scene))) result
            (let ((retq (append (if scene-settings
                                    (list (list "planning scene name" name-f)
                                          (list "robot model name" robot-model-name-f)))
                                (if robot-state-attached-objects
                                    (list (list "robot state with attached objects" robot-state-f))
                                    (if robot-state (list (list "robot state" robot-state-f))))
                                (if transforms (list (list "fixed frame transforms" fixed-frame-transforms-f)))
                                (if allowed-collision-matrix
                                    (list (list "allowed collision matrix"
                                                (collision-matrix-msg->collision-matrix allowed-collision-matrix-f))))
                                (if link-padding-and-scaling
                                    (list (list "link padding" link-padding-f)
                                          (list "link scaling" link-scaling-f)))
                                (if object-colors (list (list "object colors" object-colors-f)))
                                (if octomap (list (list "octomap" octomap-f)))
                                (if world-object-geometry
                                    (list (list "world object geometry" collision-objects-f)))
                                (if world-object-names
                                    (list (list "world object names"
                                                (mapcar (lambda (a)
                                                          (roslisp:with-fields ((name (id))) a name)) 
                                                        (coerce collision-objects-f 'list))))))))
              retq))))

(defun set-planning-scene-collision-matrix (matrix &optional quiet)
  (let* ((acm-msg (collision-matrix->msg matrix))
         (scene-msg (roslisp:make-msg "moveit_msgs/PlanningScene"
                                      allowed_collision_matrix acm-msg
                                      is_diff t)))
    (prog1 (roslisp:publish *planning-scene-publisher* scene-msg)
           (unless quiet
             (roslisp:ros-info (moveit)
                               "Updated allowed collision matrix in planning scene.")))))


(defun collision-matrix-msg->collision-matrix (msg)
  (with-fields (entry_names entry_values) msg
    (make-instance
     'collision-matrix
     :names (map 'list #'identity entry_names)
     :entries (make-array
               `(,(length entry_values)
                 ,(length entry_values))
               :initial-contents
               (map 'vector
                    (lambda (entry-line)
                      (with-fields (enabled) entry-line
                        enabled))
                    entry_values)))))

(defun get-allowed-collision-matrix ()
  (get-planning-scene
   (roslisp-msg-protocol:symbol-code
    'moveit_msgs-msg:planningscenecomponents
    :allowed_collision_matrix)))

(defun msg->collision-matrix (msg)
  (roslisp:with-fields ((acm (allowed_collision_matrix scene))) msg
    (collision-matrix-msg->collision-matrix acm)))

(defun get-collision-matrix-entry (matrix name-1 name-2)
  (let ((idx-1 (position name-1 (names matrix) :test #'string=))
        (idx-2 (position name-2 (names matrix) :test #'string=)))
    (when (and idx-1 idx-2)
      (aref (entries matrix) idx-1 idx-2))))

(defun set-collision-matrix-entry (matrix name-1 name-2 value)
  (let* ((old-names (names matrix))
         (new-names (remove-duplicates (append old-names `(,name-1 ,name-2))
                                       :test #'string=)))
    (make-instance
     'collision-matrix
     :names new-names
     :entries
     (make-array
      `(,(length new-names) ,(length new-names))
      :initial-contents
      (map 'vector
           (lambda (ref-name-1)
             (map 'vector
                  (lambda (ref-name-2)
                    (cond ((or (and (string= ref-name-1 name-1)
                                    (string= ref-name-2 name-2))
                               (and (string= ref-name-1 name-2)
                                    (string= ref-name-2 name-1)))
                           value)
                          (t (get-collision-matrix-entry
                              matrix ref-name-1 ref-name-2))))
                  new-names))
           new-names)))))

(defun collision-matrix->msg (matrix)
  (make-message
   "moveit_msgs/AllowedCollisionMatrix"
   :entry_names (map 'vector #'identity (names matrix))
   :entry_values
   (let* ((entries (entries matrix))
          (dimensions (array-dimensions entries)))
     (map 'vector #'identity
          (loop for i from 0 below (elt dimensions 0)
                collecting
                (make-message
                 "moveit_msgs/AllowedCollisionEntry"
                 :enabled
                 (map 'vector #'identity
                      (loop for j from 0 below (elt dimensions 1)
                            collecting (aref entries i j)))))))))

(defmethod relative-collision-matrix (names-groups-1 names-groups-2 values &key matrix)
  (cond (matrix
         (cond ((> (length names-groups-1) 0)
                (let ((new-matrix
                        (set-collision-matrix-entry
                         matrix
                         (first names-groups-1) (first names-groups-2) (first values))))
                  (relative-collision-matrix
                   (rest names-groups-1) (rest names-groups-2) (rest values)
                   :matrix new-matrix)))
               (t matrix)))
        (t (let* ((expanded-groups
                    (loop for i from 0 below (length names-groups-1)
                          appending
                          (loop for name-group-1 in (elt names-groups-1 i)
                                appending
                                (loop for name-group-2 in (elt names-groups-2 i)
                                      collecting
                                      `(,name-group-1 ,name-group-2 ,(elt values i))))))
                  (names-1 (mapcar (lambda (group) (first group)) expanded-groups))
                  (names-2 (mapcar (lambda (group) (second group)) expanded-groups))
                  (values (mapcar (lambda (group) (third group)) expanded-groups)))
             (relative-collision-matrix
              names-1 names-2 values
              :matrix (msg->collision-matrix
                       (get-allowed-collision-matrix)))))))

(defmethod relative-collision-matrix-msg (names-groups-1 names-groups-2 values &key matrix)
  (declare (ignore matrix))
  (collision-matrix->msg
   (combine-collision-matrices `(,@(generate-collision-matrices
                                    names-groups-1 names-groups-2 values)
                                 ,(msg->collision-matrix (get-allowed-collision-matrix))))))

(defun generate-collision-matrices (names-groups-1 names-groups-2 values)
  (mapcar (lambda (names-1 names-2 value)
            (let ((all-names (remove-duplicates
                              (append names-1 names-2)
                              :test #'string=)))
              (make-instance
               'collision-matrix
               :names all-names
               :entries
               (make-array
                `(,(length all-names) ,(length all-names))
                :initial-contents
                (map
                 'vector (lambda (name-1)
                           (map
                            'vector (lambda (name-2)
                                      (cond ((or (and (find name-1 names-1 :test #'string=)
                                                      (find name-2 names-2 :test #'string=))
                                                 (and (find name-1 names-2 :test #'string=)
                                                      (find name-2 names-1 :test #'string=)))
                                             value)
                                            (t :maybe)))
                            all-names))
                     all-names)))))
          names-groups-1 names-groups-2 values))

(defun combine-collision-matrices (matrices)
  (let ((all-names
          (remove-duplicates
           (loop for matrix in matrices
                 appending (names matrix))
           :test #'string=)))
    (make-instance
     'collision-matrix
     :names all-names
     :entries
     (make-array
      `(,(length all-names) ,(length all-names))
      :initial-contents
      (map
       'vector #'identity
       (loop for name-1 in all-names
             collecting
             (map
              'vector #'identity
              (loop for name-2 in all-names
                    collecting
                    (cond ((string= name-1 name-2) t)
                          (t (block check
                               (loop for matrix in matrices
                                     for names = (names matrix)
                                     for present = (and (find name-1 names :test #'string=)
                                                        (find name-2 names :test #'string=))
                                     when present
                                       do (let ((value (get-collision-matrix-entry
                                                        matrix name-1 name-2)))
                                            (when (not (eql value :maybe))
                                              (return-from check value)))))))))))))))
