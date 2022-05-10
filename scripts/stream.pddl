(define (stream table-tamp)
    (:stream get_trajectory
        :inputs (?q1 ?q2)
        :domain (and (Conf ?q1) (Conf ?q2))
        :outputs (?t)
        :certified (CF-Trajectory ?q1 ?q2 ?t)
    )

    (:stream sampleGraspPose
        :inputs (?o ?p)
        :domain (and (Graspable ?o) (ObjPose ?o ?p))
        :outputs (?g)
        :certified (Grasp ?o ?g)
    )

    (:stream inverse-kinematics
        :inputs (?o ?p ?g)
        :domain (and (ObjPose ?o ?p) (Graspable ?o) (Grasp ?o ?g))
        :outputs (?q)
        :certified (and (Conf ?q) (Kin ?o ?p ?q ?g))
    )

    (:stream samplePlacePose
        :inputs (?o ?r)
        :domain (and (Graspable ?o) (Region ?r))
        :outputs (?p)
        :certified (and (ObjPose ?o ?p) (Supported ?o ?p ?r))
    )
)
