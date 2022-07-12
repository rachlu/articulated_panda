(define (domain table-tamp)
    (:requirements :strips :equality)
    (:predicates
        (Conf ?q)
        (AtConf ?q)
        (ObjPose ?o ?p)
        (AtPose ?o ?p)
        (Grasp ?o ?g)
        (On ?o ?r)
        (Kin ?o ?p ?g ?q ?t)
        (Holding ?o)
        (HandEmpty)
        (Graspable ?o)
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
	    (Open ?o)
	    (Open_Traj ?o ?g ?q1 ?q2 ?t)
    )

    (:action move_free
        :parameters (?q1 ?q2 ?t)
        :precondition (and (Trajectory ?q1 ?q2 ?t)
                            (Conf ?q1)
                            (Conf ?q2)
                            (AtConf ?q1)
                            (HandEmpty)
                            (not (UnSafeTraj ?t))
                      	    (CanMove)      
		      )
        :effect (and (AtConf ?q2) (not (AtConf ?q1)) (not (CanMove)))
    )

    (:action move_holding
        :parameters (?o ?g ?q1 ?q2 ?t)
        :precondition (and (Trajectory_Holding ?o ?g ?q1 ?q2 ?t)
                            (Conf ?q1)
                            (Conf ?q2)
                            (AtConf ?q1)
                            (Holding ?o)
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
                            (AtGrasp ?o ?g)
                            (not (UnSafeHolding ?t ?o ?g))
			                (CanMove)
			                (UprightObj ?o)
                            )
        :effect (and (AtConf ?q2) (not (AtConf ?q1)) (not (CanMove)))
    )

    (:action open_door
       :parameters (?o ?p ?g ?q1 ?q2 ?t)
       :precondition (and (not (HandEmpty))
			  (Holding ?o)
                          (AtConf ?q1)
                          (AtGrasp ?o ?g)
                          (Openable ?o)
                          (Open_Traj ?o ?g ?q1 ?q2 ?t)
                          (not (InCollision ?o ?p))
                        (not (UnSafeHolding ?t ?o ?g))
                    )
    :effect (and (not (Holding ?o)) (HandEmpty) (AtPose ?o ?p) (not (AtGrasp ?o ?g)) (not (AtConf ?q1)) (CanMove) (Open ?o) (AtConf ?q2))
    )

    (:action grab
        :parameters (?o ?p ?g ?q ?t)
        :precondition (and (HandEmpty)
                          (AtConf ?q)
                          (Kin ?o ?p ?g ?q ?t)
                          (AtPose ?o ?p)
                          (Grasp ?o ?g)
                          (Graspable ?o)
			              (not (UnSafeHolding ?t ?o ?g))
                          )
        :effect (and (Holding ?o) (not (HandEmpty)) (not (AtPose ?o ?p)) (AtGrasp ?o ?g) (CanMove))
    )

    (:action place
        :parameters (?o ?p ?g ?q ?t)
        :precondition (and (not (HandEmpty))
                          (ObjPose ?o ?p)
                          (AtConf ?q)
                          (Kin ?o ?p ?g ?q ?t)
                          (Holding ?o)
                          (AtGrasp ?o ?g)
                          (not (InCollision ?o ?p))
			              (not (UnSafeHolding ?t ?o ?g))
                          )
        :effect (and (not (Holding ?o)) (HandEmpty) (AtPose ?o ?p) (not (AtGrasp ?o ?g)) (CanMove))
    )

    (:derived (On ?o ?r)
        (exists (?p) (and (Region ?r) (AtPose ?o ?p) (Supported ?o ?p ?r)))
    )

    (:derived (InCollision ?o ?p)
        (exists (?o2 ?p2) (and (AtPose ?o2 ?p2) (not (Safe ?o ?p ?o2 ?p2))))
    )

    (:derived (UnSafeTraj ?t)
        (exists (?o ?p) (and (AtPose ?o ?p) (not (CFreeTraj ?t ?o ?p))))
    )

    (:derived (UnSafeHolding ?t ?o ?g)
        (exists (?o2 ?p) (and (AtGrasp ?o ?g) (AtPose ?o2 ?p) (not (CFreeHolding ?t ?o ?g ?o2 ?p))))
    )
)
