(define (stream table-tamp)
    (:stream get_trajectory
        :inputs (?q1 ?q2)
        :domain (and (Conf ?q1) (Conf ?q2))
        :outputs (?t)
        :certified (and (Trajectory ?q1 ?q2 ?t) (Traj ?t))
    )

    (:stream get_trajectory_holding
        :inputs (?q1 ?q2 ?o ?g)
        :domain (and (Conf ?q1) (Conf ?q2) (Graspable ?o) (Grasp ?o ?g))
        :outputs (?t)
        :certified (and (Trajectory_Holding ?o ?g ?q1 ?q2 ?t) (Traj_Holding ?t ?o ?g))
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
        :outputs (?q ?t)
        :certified (and (Conf ?q) (Kin ?o ?p ?g ?q ?t) (Traj ?t))
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
        :certified (Safe ?o ?p ?o2 ?p2)
    )

    (:stream cfree
        :inputs (?t ?o ?p)
        :domain (and (ObjPose ?o ?p) (Traj ?t))
        :certified (CFreeTraj ?t ?o ?p)
    )

    (:stream cfreeholding
        :inputs (?t ?o ?g ?o2 ?p)
        :domain (and (ObjPose ?o2 ?p) (Traj_Holding ?t ?o ?g))
        :certified (CFreeHolding ?t ?o ?g ?o2 ?p)
    )
)
