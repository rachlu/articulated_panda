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
        :inputs (?o ?p1 ?p2 ?q1 ?g ?h)
        :domain (and (Conf ?q1) (Handle ?o ?h) (Openable ?o) (GraspOpenable ?o ?g ?h) (OpenGrasp ?o ?g ?h) (ObjState ?o ?p1) (ObjState ?o ?p2))
        :outputs (?t ?q2)
        :certified (and (Open_Traj ?o ?g ?q1 ?q2 ?p1 ?p2 ?h ?t) (Conf ?q2) (Traj_Holding ?t ?o ?g))
    )

   (:stream close_traj
        :inputs (?o ?p1 ?p2 ?q1 ?g ?h)
        :domain (and (Conf ?q1) (Handle ?o ?h) (Openable ?o) (GraspOpenable ?o ?g ?h) (CloseGrasp ?o ?g ?h) (ObjState ?o ?p1) (ObjState ?o ?p2))
        :outputs (?t ?q2)
        :certified (and (Close_Traj ?o ?g ?q1 ?q2 ?p1 ?p2 ?h ?t) (Conf ?q2) (Traj_Holding ?t ?o ?g))
    )

    (:stream sampleGraspOpenable
        :inputs (?o ?p ?h)
        :domain (and (Openable ?o) (ObjState ?o ?p) (Handle ?o ?h))
        :outputs (?g)
        :certified (and (GraspOpenable ?o ?g ?h) (OpenGrasp ?o ?g ?h))
    )

    (:stream sampleGraspCloseable
        :inputs (?o ?p ?h)
        :domain (and (Openable ?o) (ObjState ?o ?p) (Handle ?o ?h))
        :outputs (?g)
        :certified (and (GraspOpenable ?o ?g ?h) (CloseGrasp ?o ?g ?h))
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

    (:stream inverse-nonplaceable-kinematics
        :inputs (?o ?p ?g ?h)
        :domain (and (ObjState ?o ?p) (Handle ?o ?h) (Openable ?o) (GraspOpenable ?o ?g ?h))
        :outputs (?q1 ?q2 ?t)
        :certified (and (Conf ?q1) (Conf ?q2) (KinOpen ?o ?p ?g ?q1 ?q2 ?t) (Traj ?t))
    )

    (:stream samplePlacePose
        :inputs (?o ?r)
        :domain (and (Placeable ?o) (Region ?r))
        :outputs (?p)
        :certified (and (ObjState ?o ?p) (Supported ?o ?p ?r))
    )

    (:stream sampleTable
        :inputs (?o)
        :domain (Placeable ?o)
        :outputs (?p)
        :certified (ObjState ?o ?p)
    )

    (:stream sampleDeltaOpenableConf
        :inputs (?o ?h)
        :domain (and (Openable ?o) (Handle ?o ?h))
        :outputs (?d)
        :certified (DeltaObjState ?o ?d ?h)
    )

    (:stream sampleOpenableConf
        :inputs (?o ?p ?d ?h)
        :domain (and (Openable ?o) (ObjState ?o ?p) (DeltaObjState ?o ?d ?h))
        :outputs (?p1)
        :certified (and (ObjState ?o ?p1) (ValidStateTransition ?o ?p ?p1 ?h))
    )

    (:stream sampleCloseTransition
        :inputs (?o ?p ?h)
        :domain (and (Openable ?o) (ObjState ?o ?p) (Handle ?o ?h))
        :outputs (?p2)
        :certified (and (ObjState ?o ?p2) (ValidCloseTransition ?o ?p ?p2 ?h))
    )

    (:stream testOpenEnough
        :inputs (?o ?p ?h)
        :domain (and (Openable ?o) (Handle ?o ?h) (ObjState ?o ?p))
        :certified (OpenEnough ?o ?p ?h)
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
