(define (stream table-tamp)
    (:stream get_trajectory
        :inputs (?q1 ?q2)
        :domain (and (Conf ?q1) (Conf ?q2))
        :outputs (?t)
        :certified (CF-Trajectory ?q1 ?q2 ?t)
    )

    (:stream sampleGraspPose
        :inputs (?o)
        :domain (Graspable ?o)
        :outputs (?p)
        :certified (and (Pose ?p) (GraspPose ?o ?p))
    )

    (:stream inverse-kinematics
        :inputs (?p)
        :domain (Pose ?p)
        :outputs (?q)
        :certified (and (Conf ?q) (Kin ?q ?p))
    )

    (:stream samplePlacePose
        :inputs (?o ?g)
        :domain (and (Graspable ?o) (GraspPose ?o ?g))
        :outputs (?p)
        :certified (and (Pose ?p) (PlacePose ?o ?g ?p))
    )
)
