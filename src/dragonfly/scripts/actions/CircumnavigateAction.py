#! /usr/bin/env python
import math, rospy, rx, operator, time
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from sensor_msgs.msg import NavSatFix
from ActionState import ActionState
from rx.subjects import Subject
from rx.core import Observable

class CircumnavigateAction:

    SAMPLE_RATE = 100.0
    k0 = 0.45
    k1 = 0.5
    k2 = 1
    k3 = 2.5
    a = np.matrix([0, 0, 1]).transpose()
    a_a_transpose = a * a.transpose()
    rho = 4.2

    def __init__(self, id, local_setvelocity_publisher, target, flock):
        self.local_setvelocity_publisher = local_setvelocity_publisher
        self.id = id
        self.target = target
        self.started = False
        self.flock = flock

        self.twist = TwistStamped()
        self.previous_time = None

        self.n = len(flock)

        self.id_index = -1
        for i in range(self.n):
            if flock[i] == id:
                self.id_index = i

        # Ring topology adjacency matrix
        self.A = np.zeros(self.n)
        for j in range(self.n):
            offset = abs(self.id_index - j) % self.n
            if offset == 1 or offset == self.n - 1:
                self.A[j] = 1

        # Desired phasing angle between agents (thetaij may be different from thetaik)
        self.thetaij = 2 * math.pi / self.n

        # Desired inter-agent distance
        self.dij = 2 * self.rho * math.sin(self.thetaij / 2)

        # Matrix of inter-agent distances
        self.D = self.dij * self.A

        self.ros_subscriptions = []

        self.targetposition_subject = Subject()
        self.targetvelocity_subject = Subject()
        self.flockposition_subjects = []

        self.circumnavigate_subscription = Observable.empty().subscribe()


    def differenceInMeters(self, one, two):
        earthCircumference = 40008000
        return [
            ((one.longitude - two.longitude) * (earthCircumference / 360) * math.cos(one.latitude * 0.01745)),
            ((one.latitude - two.latitude) * (earthCircumference / 360)),
            one.altitude
        ]

    def format_velocities(self, twist):
        return [
            twist.twist.linear.x,
            twist.twist.linear.y,
            twist.twist.linear.z
        ]

    def relative_to_target(self, elements):
        targetLatLon = elements[1]

        converted_elements = [
            [self.differenceInMeters(element, targetLatLon) for element in elements[0]],
            [0, 0, targetLatLon.altitude],
            self.format_velocities(elements[2])]

        return converted_elements

    def velocity_vector(self, elements):

        pt = np.matrix(elements[1]).transpose()

        p_i = np.matrix(elements[0][self.id_index]).transpose()

        pit = p_i - pt

        DeliV1 = self.a_a_transpose * pit

        # Orthogonal projection matrix of a
        Pa = np.eye(3, 3) - self.a_a_transpose
        phii = (pit / np.linalg.norm(pit))
        phiia = (Pa * phii) / np.linalg.norm(Pa * phii)
        DeliV2 = (np.linalg.norm(Pa * pit) - self.rho) * phiia

        DeliV3 = np.matrix([0.0, 0.0, 0.0])


        for j in range(self.n):
            pj = np.matrix(elements[0][j]).transpose()
            pjt = pj - pt
            phij = pjt / np.linalg.norm(pjt)
            phija = Pa * phij / np.linalg.norm(Pa*phij)

            gammaij = self.A[j] * ((np.linalg.norm(phiia - phija) ** 2) - ((self.D[j] ** 2) / (self.rho ** 2))) * np.cross(self.a.transpose(), phiia.transpose()) * (phiia - phija)

            DeliV3 += (1 / np.linalg.norm(Pa * pit)) * gammaij * np.cross(self.a.transpose(), phiia.transpose())

        ui1 = -self.k1 * DeliV1
        ui2 = (-self.k2 * DeliV2) + (self.k0 * np.linalg.norm(Pa * pit) * np.cross(self.a.transpose(), phiia.transpose()).transpose())
        ui3 = -self.k3 * np.linalg.norm(Pa * pit) * DeliV3.transpose()

        ptdot = np.matrix(elements[2]).transpose()

        ui = ui1 + ui2 + ui3 + ptdot

        print "ui: {} + {} + {} + {}".format(ui1, ui2, ui3, ptdot)

        return np.asarray(ui).flatten()

    def navigate(self, input):

        twist = TwistStamped()
        twist.twist.linear.x = input[0]
        twist.twist.linear.y = input[1]
        twist.twist.linear.z = input[2]

        self.local_setvelocity_publisher.publish(twist)

    def setup_position_subscriber(self, flockposition_subject, subscription_name):
        self.ros_subscriptions.append(rospy.Subscriber(subscription_name, NavSatFix, lambda position: flockposition_subject.on_next(position)))
        print "Subscribed to {} with {}".format(subscription_name, flockposition_subject)

    def step(self):
        if not self.started:
            self.started = True

            print "Subscribing..."

            self.ros_subscriptions.append(rospy.Subscriber("{}/mavros/global_position/global".format(self.target), NavSatFix, lambda position: self.targetposition_subject.on_next(position)))
            print "Subscribed to {}/mavros/global_position/global".format(self.target)
            self.ros_subscriptions.append(rospy.Subscriber("{}/mavros/local_position/velocity_local".format(self.target), TwistStamped, lambda twist: self.targetvelocity_subject.on_next(twist)))
            print "Subscribed to {}/mavros/local_position/velocity_local".format(self.target)

            for name in self.flock:
                flockposition_subject = Subject()
                self.flockposition_subjects.append(flockposition_subject)
                self.setup_position_subscriber(flockposition_subject, "{}/mavros/global_position/global".format(name))

            self.circumnavigate_subscription = Observable.combine_latest(
                Observable.combine_latest(self.flockposition_subjects, lambda *positions: positions),
                self.targetposition_subject,
                self.targetvelocity_subject,
                lambda positions, target_position, target_velocity: [positions, target_position, target_velocity]) \
                .sample(self.SAMPLE_RATE) \
                .map(lambda elements: self.relative_to_target(elements)) \
                .map(lambda elements: self.velocity_vector(elements)) \
                .subscribe(on_next = lambda vectors: self.navigate(vectors))

            print "Setup finished"

        return ActionState.WORKING

    def stop(self):
        for subscription in self.ros_subscriptions:
            subscription.unregister()

        for subscription in self.flockposition_subjects:
            subscription.dispose()

        del self.ros_subscriptions

        self.targetposition_subject.dispose()
        self.targetvelocity_subject.dispose()

        self.circumnavigate_subscription.dispose()
