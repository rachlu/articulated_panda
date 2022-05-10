(define (domain table-tamp)
    (:requirements :strips :equality)
    (:predicates
        (Conf ?q)
        (ObjPose ?o ?p)
        (Grasp ?o ?g)
        (AtConf ?q)
        (AtPose ?o ?p)
        (On ?o ?r)
        (Kin ?o ?p ?q ?g)
        (Holding ?o)
        (HandEmpty)
        (Graspable ?o)
        (CF-Trajectory ?q1 ?q2 ?t)
        (Region ?r)
        (Supported ?o ?p ?r)
        (AtGrasp ?o ?g)
    )

    (:action move
        :parameters (?q1 ?q2 ?t)
        :precondition (and (CF-Trajectory ?q1 ?q2 ?t)
                            (Conf ?q1)
                            (Conf ?q2)
                            (AtConf ?q1))
        :effect (and (AtConf ?q2) (not (AtConf ?q1)))
    )

    (:action grab
        :parameters (?o ?p ?q ?g)
        :precondition (and (HandEmpty)
                          (AtConf ?q)
                          (Kin ?o ?p ?q ?g)
                          (AtPose ?o ?p)
                          (Grasp ?o ?g)
                          (Graspable ?o))
        :effect (and (Holding ?o) (not (HandEmpty)) (not (AtPose ?o ?p)) (AtGrasp ?o ?g))
    )

    (:action place
        :parameters (?o ?p ?q ?g)
        :precondition (and (not (HandEmpty))
                          (AtConf ?q)
                          (Kin ?o ?p ?q ?g)
                          (Holding ?o)
                          (AtGrasp ?o ?g))
        :effect (and (not (Holding ?o)) (HandEmpty) (AtPose ?o ?p) (not (AtGrasp ?o ?g)))
    )

    (:derived (On ?o ?r)
        (exists (?p) (and (Region ?r) (AtPose ?o ?p) (Supported ?o ?p ?r)))
    )
)