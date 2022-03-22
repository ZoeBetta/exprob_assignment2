(define (problem task)
(:domain sherlockbot-domain)
(:objects
    wp1 wp2 wp3 wp4 - waypoint
    wphome - home
)
(:init


    (not (hypothesis_complete))

    (robot_at_home wphome)

    (hint_taken wp2)
    (hint_taken wp4)
    (hint_taken wp1)

)
(:goal (and
    (hypothesis_correct)
))
)
