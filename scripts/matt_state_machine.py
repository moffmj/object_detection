#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import smach
import smach_ros
import matt_perception_test as ps



def main():
    rospy.init_node('python_state_machine')

    sm = smach.StateMachine(outcomes=['mission_complete'])

    with sm:
        smach.StateMachine.add('GetObjectState', ps.matt_test_class(),
                               transitions={'FoundObjectTransition': 'TravelState'}, remapping={'xco_out':'sm_data_x','yco_out':'sm_data_y',})

        smach.StateMachine.add('TravelState', ps.navClass(), transitions={'Navigated': 'PickState'}, remapping={'xco_in':'sm_data_x','yco_in':'sm_data_y'})

	smach.StateMachine.add('PickState', ps.pickClass(), transitions={'Picked': 'NavToBinState','Failed':'PickState'})

	smach.StateMachine.add('NavToBinState', ps.navigateToBinClass(), transitions={'ReachedBin': 'mission_complete'})

    sis = smach_ros.IntrospectionServer('my_smach_introspection_server', sm, '/SM_ROOT')
    sis.start()

    outcome = sm.execute()

    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
