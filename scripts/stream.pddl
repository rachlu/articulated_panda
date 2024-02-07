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

    (:stream open_traj
        :inputs (?o ?c1 ?c2 ?q1 ?h ?f)
        :domain (and (Conf ?q1) (Handle ?o ?h) (Openable ?o) (ObjState ?o ?c1) (ObjState ?o ?c2) (ForceNeeded ?o ?h ?f))
        :outputs (?q2 ?g ?t1 ?t2)
        :certified (and (Trajectory ?q1 ?q2 ?t1) (OpenGrasp ?o ?g ?h) (Traj ?t1) (Open_Traj ?o ?g ?q1 ?q2 ?c1 ?c2 ?h ?t1 ?t2) (Conf ?q2) (Traj_Holding ?t2 ?o ?g))
    )

    (:stream close_traj
        :inputs (?o ?c1 ?c2 ?q1 ?h ?f)
        :domain (and (Conf ?q1) (Handle ?o ?h) (Openable ?o) (ObjState ?o ?c1) (ObjState ?o ?c2) (ForceNeeded ?o ?h ?f))
        :outputs (?q2 ?g ?t1 ?t2)
        :certified (and (Trajectory ?q1 ?q2 ?t1) (CloseGrasp ?o ?g ?h) (Traj ?t1) (Close_Traj ?o ?g ?q1 ?q2 ?c1 ?c2 ?h ?t1 ?t2) (Conf ?q2) (Traj_Holding ?t2 ?o ?g))
    )

    (:stream randomRobotConf
        :outputs (?q)
        :certified (Conf ?q)
    )

    (:stream sampleGraspPose
        :inputs (?o ?p)
        :domain (and (Placeable ?o) (ObjState ?o ?p))
        :outputs (?g)
        :certified (Grasp ?o ?g)
    )

    (:stream inverse-kinematics
        :inputs (?o ?p ?g)
        :domain (and (ObjState ?o ?p) (Placeable ?o) (Grasp ?o ?g))
        :outputs (?q ?t)
        :certified (and (Conf ?q) (Kin ?o ?p ?g ?q ?t) (Traj ?t))
    )

    (:stream samplePlacePose
        :inputs (?o ?r)
        :domain (and (Placeable ?o) (Region ?r))
        :outputs (?p)
        :certified (and (ObjState ?o ?p) (Supported ?o ?p ?r))
    )

    (:stream samplePlaceCabinetPose
        :inputs (?o1 ?o2 ?r ?c)
        :domain (and (Placeable ?o1) 
                    (Handle ?o2 ?r) 
                    (Openable ?o2) 
                    (ObjState ?o2 ?c) 
                    (OpenEnough ?o2 ?c ?r) 
                    (CabinetRegion ?r))
        :outputs (?p)
        :certified (and (ObjState ?o1 ?p) (SupportedCabinet ?o1 ?p ?r ?c))   
    )
    
    (:stream sampleTable
        :inputs (?o)
        :domain (Placeable ?o)
        :outputs (?p)
        :certified (ObjState ?o ?p)
    )

    (:stream sampleOpenableConf
        :inputs (?o ?c ?h)
        :domain (and (Openable ?o) (ObjState ?o ?c) (Handle ?o ?h))
        :outputs (?c2)
        :certified (and (ObjState ?o ?c2) (OpenEnough ?o ?c2 ?h) (ValidStateTransition ?o ?c ?c2 ?h))
    )

    (:stream sampleCloseTransition
        :inputs (?o ?c ?h)
        :domain (and (Openable ?o) (ObjState ?o ?c) (Handle ?o ?h))
        :outputs (?c2)
        :certified (and (ObjState ?o ?c2) (ValidCloseTransition ?o ?c ?c2 ?h))
    )

    (:stream collisionCheck
        :inputs (?o ?p ?o2 ?p2)
        :domain (and (ObjState ?o ?p) (ObjState ?o2 ?p2))
        :certified (Safe ?o ?p ?o2 ?p2)
    )

    (:stream cfree
        :inputs (?t ?o ?p)
        :domain (and (ObjState ?o ?p) (Traj ?t))
        :certified (CFreeTraj ?t ?o ?p)
    )

    (:stream cfreeholding
        :inputs (?t ?o ?g ?o2 ?p)
        :domain (and (ObjState ?o2 ?p) (Traj_Holding ?t ?o ?g))
        :certified (CFreeHolding ?t ?o ?g ?o2 ?p)
    )
)
