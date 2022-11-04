(define (domain table-tamp)
    (:requirements :strips :equality)
    (:predicates
        (Conf ?q)
        (AtConf ?q)
        (ObjPose ?o ?p)
        (AtPose ?o ?p)
        (ObjConf ?o ?p)
        (AtObjConf ?o ?p)
        (Grasp ?o ?g)
        (On ?o ?r)
        (Kin ?o ?p ?g ?q ?t)
	    (KinOpen ?o ?p ?g ?q1 ?q2 ?t)
        (Holding ?o)
        (HandEmpty)
        (UprightObj ?o)
        (Trajectory ?q1 ?q2 ?t)
        (Traj ?t)
        (UnSafeTraj ?t)
        (CFreeTraj ?t ?o ?g)
        (Trajectory_Holding ?o ?g ?q1 ?q2 ?t)
        (Trajectory_Holding_Upright ?o ?g ?q1 ?q2 ?t)
        (Traj_Holding ?t ?o ?g)
        (UnSafeHolding ?t ?o ?g)
        (CFreeHolding ?t ?o ?g ?o2 ?p)
        (Region ?r)
        (Supported ?o ?p ?r)
        (AtGrasp ?o ?g)
        (InCollision ?o ?p)
        (Safe ?o ?p ?o2 ?p2)
        (CanMove)
        (Openable ?o ?k)
        (Open ?o ?k)
        ;(Open_Traj ?o ?g ?q1 ?q2 ?a ?i ?s ?t1 ?t2)
        (Open_Traj ?o ?g ?q1 ?q2 ?a ?i ?s ?t)
        (OpenAllAmount ?o ?a)
        (Open_Amount ?o ?a)
        (OpenAll ?o ?k)
        (Graspopenable ?o ?g ?k)
        (AtGraspOpenable ?o ?g ?k)
        (Placeable ?o)
	    (Graspable ?o)
	    (Holding_Openable ?o ?k)
    )

    (:action move_free
        :parameters (?q1 ?q2 ?t)
        :precondition (and (Trajectory ?q1 ?q2 ?t)
                            (Conf ?q1)
                            (Conf ?q2)
                            (AtConf ?q1)
                            (HandEmpty)
                            ;(not (UnSafeTraj ?t))
                      	    (CanMove))
        :effect (and (AtConf ?q2) (not (AtConf ?q1)) (not (CanMove)))
    )

    (:action move_holding
        :parameters (?o ?g ?q1 ?q2 ?t)
        :precondition (and (Trajectory_Holding ?o ?g ?q1 ?q2 ?t)
                            (Conf ?q1)
                            (Conf ?q2)
                            (AtConf ?q1)
                            (Holding ?o)
                            (Grasp ?o ?g)
                            (AtGrasp ?o ?g)
                            (not (UnSafeHolding ?t ?o ?g))
                            (CanMove)
                            (not (UprightObj ?o))
                            )
        :effect (and (AtConf ?q2) (not (AtConf ?q1)) (not (CanMove)))
    )

    (:action move_holding_upright
        :parameters (?o ?g ?q1 ?q2 ?t)
        :precondition (and (Trajectory_Holding_Upright ?o ?g ?q1 ?q2 ?t)
                            (Conf ?q1)
                            (Conf ?q2)
                            (AtConf ?q1)
                            (Holding ?o)
                            (Grasp ?o ?g)
                            (AtGrasp ?o ?g)
                            (not (UnSafeHolding ?t ?o ?g))
                            (CanMove)
                            (UprightObj ?o)
                            )
        :effect (and (AtConf ?q2) (not (AtConf ?q1)) (not (CanMove)))
    )

    ;(:action open_obj
     ;   :parameters (?o ?p1 ?p2 ?g ?k ?q1 ?q2 ?a ?i ?s ?t)
      ;  :precondition (and (Openable ?o ?k)
;			                (GraspOpenable ?o ?g ?k)
;			                (AtGraspOpenable ?o ?g ?k)
;		     	            (Holding ?o)
;			                (not (HandEmpty))
 ;                           (ObjConf ?o ?p1)
  ;                          (ObjConf ?o ?p2)
    ;                        (AtObjConf ?o ?p1)
   ;                         (Conf ?q1)
     ;                       (Conf ?q2)
      ;                      (AtConf ?q1)
       ;                     (Open_Traj ?o ?g ?q1 ?q2 ?a ?i ?s ?t)
        ;                    ;(Traj ?t)
          ;                  ;(Traj_Holding ?t ?o ?g)
         ;                   ;(not (UnSafeHolding ?t ?o ?g))
           ;                 ;(not (InCollision ?o ?p2))
	;			)
        ;:effect (and (not (AtObjConf ?o ?p1)) (not (Holding ?o)) (HandEmpty) (AtObjConf ?o ?p2) (not (AtConf ?q1)) (AtConf ?q2) (CanMove) (Open ?o ?k ?a) (not (AtGraspOpenable ?o ?g ?k)))
    ;)

    (:action open_obj
        :parameters (?o ?g ?k ?a ?q1)
        :precondition (and (Openable ?o ?k)
                            (GraspOpenable ?o ?g ?k)
                            (AtGraspOpenable ?o ?g ?k)
                            (not (HandEmpty))
			    (Open_Amount ?o ?a)
			    (Conf ?q1)
			    ;(Conf ?q2)
			    (AtConf ?q1)
			    ;(Open_Traj ?o ?g ?q1 ?q2 ?a ?i ?s ?t)
			    )
        :effect (and (not (Holding ?o)) (Open ?o ?k))
    )

    (:action grab
        :parameters (?o ?p ?g ?q ?t)
        :precondition (and (HandEmpty)
                          (Conf ?q)
                          (AtConf ?q)
                          (Kin ?o ?p ?g ?q ?t)
                          (ObjPose ?o ?p)
                          (AtPose ?o ?p)
                          (Grasp ?o ?g)
                          (Placeable ?o)
                          (not (UnSafeHolding ?t ?o ?g))
                          )
        :effect (and (Holding ?o) (not (HandEmpty)) (not (AtPose ?o ?p)) (AtGrasp ?o ?g) (CanMove))
    )

    (:action hold
        :parameters (?o ?p ?g ?q1 ?q2 ?k ?t)
        :precondition (and (HandEmpty)
                            (Conf ?q1)
                            (Conf ?q2)
                            (AtConf ?q1)
                            (KinOpen ?o ?p ?g ?q1 ?q2 ?t)
                            (ObjConf ?o ?p)
                            (AtObjConf ?o ?p)
                            (GraspOpenable ?o ?g ?k)
                            (Openable ?o ?k))
	:effect (and (not (AtConf ?q1)) (AtConf ?q2) (Holding ?o) (not (HandEmpty)) (AtGraspOpenable ?o ?g ?k))
    )

    (:action place
        :parameters (?o ?p ?g ?q ?t)
        :precondition (and (not (HandEmpty))
                          (Placeable ?o)
                          (ObjPose ?o ?p)
                          (Conf ?q)
                          (AtConf ?q)
                          (Kin ?o ?p ?g ?q ?t)
                          (Holding ?o)
                          (Grasp ?o ?g)
                          (AtGrasp ?o ?g)
                          (not (InCollision ?o ?p))
                          (not (UnSafeHolding ?t ?o ?g)))
        :effect (and (not (Holding ?o)) (HandEmpty) (AtPose ?o ?p) (not (AtGrasp ?o ?g)) (CanMove))
    )

    (:derived (On ?o ?r)
        (exists (?p) (and (Region ?r) (ObjPose ?o ?p) (AtPose ?o ?p) (Supported ?o ?p ?r)))
    )

    (:derived (InCollision ?o ?p)
        (exists (?o2 ?p2) (and (ObjPose ?o2 ?p2) (AtPose ?o2 ?p2) (not (Safe ?o ?p ?o2 ?p2))))
    )

    (:derived (UnSafeTraj ?t)
        (exists (?o ?p) (and (ObjPose ?o ?p) (AtPose ?o ?p) (not (CFreeTraj ?t ?o ?p))))
    )

    (:derived (UnSafeHolding ?t ?o ?g)
        (exists (?o2 ?p) (and (Grasp ?o ?g) (ObjPose ?o2 ?p) (AtGrasp ?o ?g) (AtPose ?o2 ?p) (not (CFreeHolding ?t ?o ?g ?o2 ?p))))
    )

    ;(:derived (OpenAll ?o ?k)
    ;    (exists (?a) (and (Openable ?o ?k) (Open ?o ?k ?a) (OpenAllAmount ?o ?a)))
    ;)
    
    (:derived (Holding_Openable ?o ?k)
	    (exists (?g) (and (Openable ?o ?k) (GraspOpenable ?o ?g ?k) (AtGraspOpenable ?o ?g ?k)))
    )
)
