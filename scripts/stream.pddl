(define (stream table-tamp)
    (:stream get_trajectory
        :inputs (?q1 ?q2)
        :domain (and (Conf ?q1) (Conf ?q2))
        :outputs (?t)
        :certified (CF-Trajectory ?q1 ?q2 ?t)
    )

    (:stream get_trajectory_holding
        :inputs (?q1 ?q2 ?o ?g)
        :domain (and (Conf ?q1) (Conf ?q2) (Graspable ?o) (Grasp ?o ?g))
        :outputs (?t)
        :certified (CF-Trajectory_Holding ?o ?g ?q1 ?q2 ?t)
    )

    (:stream sampleGraspPose
        :inputs (?o ?p)
        :domain (and (Graspable ?o) (ObjPose ?o ?p))
        ;:outputs (?g)
        :outputs (?g1 ?g2)
        ;:certified (Grasp ?o ?g)
        :certified (and (Grasp ?o ?g1) (Grasp ?o ?g2) (GraspSet ?g1 ?g2))
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

    (:stream sampleTable
        :inputs (?o ?p)
        :domain (and (Graspable ?o) (ObjPose ?o ?p))
        :outputs (?p2)
        :certified (ObjPose ?o ?p2)
    )

    (:stream collisionCheck
        :inputs (?o ?p ?o2 ?p2)
        :domain (and (ObjPose ?o ?p) (ObjPose ?o2 ?p2))
        :certified (Collision ?o ?p ?o2 ?p2)
    )
)
