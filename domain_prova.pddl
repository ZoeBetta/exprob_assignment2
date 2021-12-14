(define (domain sherlockbot-domain)

(:requirements :strips :typing :fluents :disjunctive-preconditions :durative-actions :adl)

(:types
	waypoint
	home
	robot
)

(:predicates
	(robot_at ?wp - waypoint)
	(hypothesis_correct)
	(hypothesis_complete)
	(robot_at_home ?h - home)
	(hint_taken ?wp -waypoint)
)


;; Move to any waypoint
(:durative-action goto_waypoint
	:parameters (?from ?to - waypoint)
	:duration ( = ?duration 60)
	:condition (and
		(at start(robot_at ?from))
		)
	:effect (and
		(at end(robot_at ?to))
		(at start (not (robot_at ?from)))
		)
		
)

(:durative-action take_hint
	:parameters (?wp - waypoint)
	:duration ( = ?duration 60)
	:condition (at start(robot_at ?wp))
	:effect (at end( hint_taken ?wp))
		
)

(:durative-action go_home
	:parameters ( ?from - waypoint ?to -home)
	:duration ( = ?duration 60)
	:condition (at start(robot_at ?from))
	:effect (and
		(at start(not (robot_at ?from)))
		(at end (robot_at_home ?to))
		)
)

(:durative-action move_from_home
	:parameters ( ?from - home ?to - waypoint)
	:duration ( = ?duration 60)
	:condition(and
				(at start (robot_at_home ?from))
				)
	:effect(and
		(at start (not(robot_at_home ?from)))
		(at end(robot_at ?to))
		)
)

(:durative-action check_complete
	:parameters(?h - home)
	:duration ( = ?duration 60)
	:condition (and 
				(at start (forall (?wp - waypoint) ( hint_taken ?wp)))
				)
	:effect ( and
			(at end (hypothesis_complete))
			)
)			

(:durative-action check_hypothesis
	:parameters(?h - home)
	:duration ( = ?duration 60)
	:condition (and 
				(at start(robot_at_home ?h))
				(at start(hypothesis_complete))
				)
				
	:effect (and
			(at end(hypothesis_correct))
			)
)

)
