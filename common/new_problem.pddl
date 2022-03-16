(define (problem task)
(:domain sherlockbot-domain)
(:objects
    wp1 wp2 wp3 wp4 - waypoint
    wphome - home
)
(:init


    (not (hypothesis_complete))

    (robot_at_home wphome)

    (not (hint_taken wp1))
    (not (hint_taken wp2))
    (not (hint_taken wp3))
    (not (hint_taken wp4))

)
(:goal (and
    (hypothesis_correct)
))
)
