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
        (Open ?o ?a)
        (Open_Traj ?o ?g ?q1 ?q2 ?a ?i ?s ?t1 ?t2)
        (OpenAllAmount ?o ?a)
        (Open_Amount ?o ?a)
        (OpenAll ?o)
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
                            (not (UnSafeTraj ?t))
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
    ;   :parameters (?o ?p1 ?p2 ?g ?q1 ?q2 ?a ?i ?s ?t1 ?t2)
    ;   :precondition (and (HandEmpty)
    ;                        (ObjPose ?o ?p1)
    ;                        (ObjPose ?o ?p2)
    ;                        (AtPose ?o ?p1)
    ;                        (Grasp ?o ?g)
    ;                        (Conf ?q1)
    ;                        (Conf ?q2)
    ;                        (AtConf ?q1)
    ;                        (Openable ?o)
    ;                        (Open_Traj ?o ?g ?q1 ?q2 ?a ?i ?s ?t1 ?t2)
    ;                        (Traj ?t1)
    ;                        (Traj_Holding ?t2 ?o ?g)
    ;                        (not (UnSafeTraj ?t1))
    ;                        (not (InCollision ?o ?p2))
    ;                        (not (UnSafeHolding ?t2 ?o ?g))
    ;                        )
    ;    :effect (and (not (AtPose ?o ?p1)) (AtPose ?o ?p2) (not (AtConf ?q1)) (CanMove) (Open ?o ?a) (AtConf ?q2))
    ;)

    (:action grab
        :parameters (?o ?p ?g ?q ?t)
        :precondition (and (HandEmpty)
                          (Conf ?q)
                          (AtConf ?q)
                          (Kin ?o ?p ?g ?q ?t)
                          (ObjPose ?o ?p)
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
                          (Placeable ?o)
                          (ObjPose ?o ?p)
                          (Conf ?q)
                          (AtConf ?q)
                          (Kin ?o ?p ?g ?q ?t)
                          (Holding ?o)
                          (Grasp ?o ?g)
                          (AtGrasp ?o ?g)
                          (not (InCollision ?o ?p))
                          (not (UnSafeHolding ?t ?o ?g))
                      )
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

    (:derived (OpenAll ?o)
        (exists (?a) (and (Openable ?o) (Open ?o ?a) (OpenAllAmount ?o ?a)))
    )
)
