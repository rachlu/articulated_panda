(define (stream table-tamp)
    (:stream get_trajectory
        :inputs (?q1 ?q2)
        :domain (and (Conf ?q1) (Conf ?q2))
        :outputs (?t)
        :certified (and (Trajectory ?q1 ?q2 ?t) (Traj ?t))
    )

    (:stream get_trajectory_holding
        :inputs (?q1 ?q2 ?o ?g)
        :domain (and (Conf ?q1) (Conf ?q2) (Placeable ?o) (Grasp ?o ?g))
        :outputs (?t)
        :certified (and (Trajectory_Holding ?o ?g ?q1 ?q2 ?t) (Traj_Holding ?t ?o ?g))
    )

    (:stream get_trajectory_holding_upright
        :inputs (?q1 ?q2 ?o ?g)
        :domain (and (Conf ?q1) (Conf ?q2) (Placeable ?o) (Grasp ?o ?g))
        :outputs (?t)
        :certified (and (Trajectory_Holding_Upright ?o ?g ?q1 ?q2 ?t) (Traj_Holding ?t ?o ?g))
    )

    ;(:stream open_traj
    ;    :inputs (?o ?q1 ?p1 ?a)
    ;    :domain (and (Conf ?q1) (Openable ?o) (ObjPose ?o ?p1) (Open_Amount ?o ?a))
    ;    :outputs (?t1 ?t2 ?q2 ?p2 ?g ?i ?s)
    ;    :certified (and (Open_Traj ?o ?g ?q1 ?q2 ?a ?i ?s ?t1 ?t2) (Traj_Holding ?t2 ?o ?g) (Traj ?t1) (ObjPose ?o ?p2) (Conf ?q2) (Grasp ?o ?g))
    ;)

    (:stream open_traj
        :inputs (?o ?p1 ?q1 ?g ?a ?k)
        :domain (and (Conf ?q1) (Openable ?o ?k) (GraspOpenable ?o ?g ?k) (ObjConf ?o ?p1) (Open_Amount ?o ?a))
        :outputs (?t ?q2 ?p2 ?i ?s)
        :certified (and (Open_Traj ?o ?g ?q1 ?q2 ?a ?i ?s ?t) (Traj_Holding ?t ?o ?g) (Traj ?t) (ObjConf ?o ?p2) (Conf ?q2))
    )

    (:stream sampleGraspOpenable
        :inputs (?o ?p ?k)
        :domain (and (Openable ?o ?k) (ObjConf ?o ?p))
        :outputs (?g)
        :certified (GraspOpenable ?o ?g ?k)
    )

    (:stream sampleGraspPose
        :inputs (?o ?p)
        :domain (and (Placeable ?o) (ObjPose ?o ?p))
        :outputs (?g)
        :certified (Grasp ?o ?g)
    )

    (:stream inverse-kinematics
        :inputs (?o ?p ?g)
        :domain (and (ObjPose ?o ?p) (Placeable ?o) (Grasp ?o ?g))
        :outputs (?q ?t)
        :certified (and (Conf ?q) (Kin ?o ?p ?g ?q ?t) (Traj ?t))
    )

    (:stream inverse-nonplaceable-kinematics
        :inputs (?o ?p ?g ?k)
        :domain (and (ObjConf ?o ?p) (Openable ?o ?k) (GraspOpenable ?o ?g ?k))
        :outputs (?q1 ?q2 ?t)
        :certified (and (Conf ?q1) (Conf ?q2) (KinOpen ?o ?p ?g ?q1 ?q2 ?t) (Traj ?t))
    )

    (:stream samplePlacePose
        :inputs (?o ?r)
        :domain (and (Placeable ?o) (Region ?r))
        :outputs (?p)
        :certified (and (ObjPose ?o ?p) (Supported ?o ?p ?r))
    )

    (:stream sampleTable
        :inputs (?o)
        :domain (Placeable ?o)
        :outputs (?p)
        :certified (ObjPose ?o ?p)
    )

    (:stream sampleOpenableConf
        :inputs (?o ?k)
        :domain (Openable ?o ?k)
        :outputs (?a ?p)
        :certified (and (ObjConf ?o ?p) (Open_Amount ?o ?a))
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
