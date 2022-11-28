(define (domain table-tamp)
    (:requirements :strips :equality)
    (:predicates
        (Conf ?q)
        (AtConf ?q)
        (ObjState ?o ?p)
        (AtObjState ?o ?s)
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
        (Openable ?o)
        (Open ?o ?h)
        (Open_Traj ?o ?g ?q1 ?q2 ?p2 ?i ?s ?t)
        (OpenEnough ?o ?p)
        (Graspopenable ?o ?g ?h)
        (AtGraspOpenable ?o ?g ?h)
        (Placeable ?o)
	    (Graspable ?o)
	    (Holding_Openable ?o ?h)
	    (Handle ?o ?h)
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

    (:action open_obj
        :parameters (?o ?p1 ?p2 ?g ?q1 ?q2 ?i ?s ?t)
        :precondition (and (Openable ?o)
                            (GraspOpenable ?o ?g ?h)
                            (AtGraspOpenable ?o ?g ?h)
                            (not (HandEmpty))
                            (Conf ?q1)
                            (Conf ?q2)
                            (AtConf ?q1)
                            (ObjState ?o ?p1)
                            (AtObjState ?o ?p1)
                            (ObjState ?o ?p2)
                            (Open_Traj ?o ?g ?q1 ?q2 ?p2 ?i ?s ?t)
                            (not (UnSafeHolding ?t ?o ?g))
			    )
        :effect (and (not (Holding ?o)) (HandEmpty) (AtObjState ?o ?p2)
                    (not (AtObjState ?o ?p1)) (AtConf ?q2) (not (AtConf ?q1)) (CanMove))
    )

    (:action grab
        :parameters (?o ?p ?g ?q ?t)
        :precondition (and (HandEmpty)
                          (Conf ?q)
                          (AtConf ?q)
                          (Kin ?o ?p ?g ?q ?t)
                          (ObjState ?o ?p)
                          (AtObjState ?o ?p)
                          (Grasp ?o ?g)
                          (Placeable ?o)
                          (not (UnSafeHolding ?t ?o ?g))
                          )
        :effect (and (Holding ?o) (not (HandEmpty)) (not (AtObjState ?o ?p)) (AtGrasp ?o ?g) (CanMove))
    )

    (:action hold
        :parameters (?o ?p ?g ?q1 ?q2 ?t)
        :precondition (and (HandEmpty)
                            (Conf ?q1)
                            (Conf ?q2)
                            (AtConf ?q1)
                            (KinOpen ?o ?p ?g ?q1 ?q2 ?t)
                            (ObjState ?o ?p)
                            (AtObjState ?o ?p)
                            (GraspOpenable ?o ?g ?h)
                            (Handle ?o ?h)
                            (Openable ?o))
	:effect (and (not (AtConf ?q1)) (AtConf ?q2) (Holding ?o) (not (HandEmpty)) (AtGraspOpenable ?o ?g ?h))
    )

    (:action place
        :parameters (?o ?p ?g ?q ?t)
        :precondition (and (not (HandEmpty))
                          (Placeable ?o)
                          (ObjState ?o ?p)
                          (Conf ?q)
                          (AtConf ?q)
                          (Kin ?o ?p ?g ?q ?t)
                          (Holding ?o)
                          (Grasp ?o ?g)
                          (AtGrasp ?o ?g)
                          (not (InCollision ?o ?p))
                          (not (UnSafeHolding ?t ?o ?g)))
        :effect (and (not (Holding ?o)) (HandEmpty) (AtObjState ?o ?p) (not (AtGrasp ?o ?g)) (CanMove))
    )

    (:derived (On ?o ?r)
        (exists (?p) (and (Region ?r) (ObjState ?o ?p) (AtObjState ?o ?p) (Supported ?o ?p ?r)))
    )

    (:derived (InCollision ?o ?p)
        (exists (?o2 ?p2) (and (ObjState ?o2 ?p2) (AtObjState ?o2 ?p2) (not (Safe ?o ?p ?o2 ?p2))))
    )

    (:derived (UnSafeTraj ?t)
        (exists (?o ?p) (and (ObjState ?o ?p) (AtObjState ?o ?p) (not (CFreeTraj ?t ?o ?p))))
    )

    (:derived (UnSafeHolding ?t ?o ?g)
        (exists (?o2 ?p) (and (Grasp ?o ?g) (ObjState ?o2 ?p) (AtGrasp ?o ?g) (AtObjState ?o2 ?p) (not (CFreeHolding ?t ?o ?g ?o2 ?p))))
    )

    (:derived (Open ?o ?h)
        (exists (?p) (and (Openable ?o) (Handle ?o ?h) (AtObjState ?o ?p) (OpenEnough ?o ?p)))
    )
    
    (:derived (Holding_Openable ?o ?h)
	    (exists (?g) (and (Openable ?o) (Handle ?o ?h) (GraspOpenable ?o ?g ?h) (AtGraspOpenable ?o ?g ?h)))
    )
)
