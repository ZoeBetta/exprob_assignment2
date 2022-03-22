(define (problem task)
(:domain sherlockbot-domain)
(:objects
    wp1 wp2 wp3 wp4 - waypoint
    wphome - home
)
(:init



    (robot_at_home wphome)


)
(:goal (and
    (hypothesis_correct)
))
)
