from math import pi

task_poses = {
    "pins": [
        # Points are given in the form of [start_pose, start_approach_offset, end_pose, end_approach_offset].
        # Each offset is given in (distance, londitude, latitude).
        [[-0.204165, -0.276294, 0.237823, 0, pi/2, -pi/2], [0.03, 0, pi/2], [0.03, 0, pi/2], [0.067478, -0.471882, 0.260079, 0, pi/2, -pi/2], [0.03, 0, pi/2], [0.03, 0, pi/2]],
        [[-0.155527, -0.275257, 0.237579, 0, pi/2, -pi/2], [0.03, 0, pi/2], [0.03, 0, pi/2], [0.062748, -0.439779, 0.260819, 0, pi/2, -pi/2], [0.03, 0, pi/2], [0.03, 0, pi/2]],
        [[-0.107250, -0.273224, 0.235516, 0, pi/2, -pi/2], [0.03, 0, pi/2], [0.03, 0, pi/2], [-0.293883, -0.469849, 0.259035, 0, pi/2, -pi/2], [0.03, 0, pi/2], [0.03, 0, pi/2]],
        [[-0.059473, -0.273576, 0.233900, 0, pi/2, -pi/2], [0.03, 0, pi/2], [0.03, 0, pi/2], [-0.293159, -0.438014, 0.257588, 0, pi/2, -pi/2], [0.03, 0, pi/2], [0.03, 0, pi/2]]
    ],

    "rods_straight": [
        [[-0.100039, -0.186702, 0.230046, 0, pi/2, -pi/2], [0.15, 0, pi/2], [0.15, 0, pi/2], [0.029353, -0.272087, 0.138594, -1.596139, 0.017192, -1.403618], [0.15, 0, pi/2], [0.3, 0, pi/2]],
        [[-0.100039, -0.236702, 0.230046, 0, pi/2, -pi/2], [0.15, 0, pi/2], [0.15, 0, pi/2], [-0.296535, -0.260922, 0.134253, -1.639844, -0.005172, -1.593012], [0.15, 0, pi/2], [0.3, 0, pi/2]]
    ]
}
