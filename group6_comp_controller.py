from kesslergame import KesslerController
from typing import Dict, Tuple
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import math
import numpy as np


class CompController(KesslerController):

    def __init__(self):
        self.eval_frames = 0 

        bullet_time = ctrl.Antecedent(np.arange(0, 1.0, 0.002), 'bullet_time')
        theta_delta = ctrl.Antecedent(np.arange(-1 * math.pi/30, math.pi/30, 0.1), 'theta_delta')
        ship_turn = ctrl.Consequent(np.arange(-180, 180, 1), 'ship_turn')
        ship_fire = ctrl.Consequent(np.arange(-1, 1, 0.1), 'ship_fire')

        bullet_time['S'] = fuzz.trimf(bullet_time.universe, [0, 0, 0.05])
        bullet_time['M'] = fuzz.trimf(bullet_time.universe, [0, 0.05, 0.1])
        bullet_time['L'] = fuzz.smf(bullet_time.universe, 0.0, 0.1)

        theta_delta['NL'] = fuzz.zmf(theta_delta.universe, -1*math.pi/30, -2*math.pi/90)
        theta_delta['NM'] = fuzz.trimf(theta_delta.universe, [-1*math.pi/30, -2*math.pi/90, -1*math.pi/90])
        theta_delta['NS'] = fuzz.trimf(theta_delta.universe, [-2*math.pi/90, -1*math.pi/90, math.pi/90])
        theta_delta['PS'] = fuzz.trimf(theta_delta.universe, [-1*math.pi/90, math.pi/90, 2*math.pi/90])
        theta_delta['PM'] = fuzz.trimf(theta_delta.universe, [math.pi/90, 2*math.pi/90, math.pi/30])
        theta_delta['PL'] = fuzz.smf(theta_delta.universe, 2*math.pi/90, math.pi/30)

        ship_turn['NL'] = fuzz.trimf(ship_turn.universe, [-180, -180, -120])
        ship_turn['NM'] = fuzz.trimf(ship_turn.universe, [-180, -120, -60])
        ship_turn['NS'] = fuzz.trimf(ship_turn.universe, [-120, -60, 60])
        ship_turn['PS'] = fuzz.trimf(ship_turn.universe, [-60, 60, 120])
        ship_turn['PM'] = fuzz.trimf(ship_turn.universe, [60, 120, 180])
        ship_turn['PL'] = fuzz.trimf(ship_turn.universe, [120, 180, 180])

        ship_fire['N'] = fuzz.trimf(ship_fire.universe, [-1, -1, -0.9])
        ship_fire['Y'] = fuzz.trimf(ship_fire.universe, [-0.5, 1, 1])

        rules = []
        rules.append(ctrl.Rule(bullet_time['L'] & theta_delta['NL'], (ship_turn['NL'], ship_fire['N'])))
        rules.append(ctrl.Rule(bullet_time['L'] & theta_delta['NM'], (ship_turn['NM'], ship_fire['N'])))
        rules.append(ctrl.Rule(bullet_time['L'] & theta_delta['NS'], (ship_turn['NS'], ship_fire['Y'])))
        rules.append(ctrl.Rule(bullet_time['L'] & theta_delta['PS'], (ship_turn['PS'], ship_fire['Y'])))
        rules.append(ctrl.Rule(bullet_time['L'] & theta_delta['PM'], (ship_turn['PM'], ship_fire['N'])))
        rules.append(ctrl.Rule(bullet_time['L'] & theta_delta['PL'], (ship_turn['PL'], ship_fire['N'])))

        rules.append(ctrl.Rule(bullet_time['M'] & theta_delta['NL'], (ship_turn['NL'], ship_fire['N'])))
        rules.append(ctrl.Rule(bullet_time['M'] & theta_delta['NM'], (ship_turn['NM'], ship_fire['N'])))
        rules.append(ctrl.Rule(bullet_time['M'] & theta_delta['NS'], (ship_turn['NS'], ship_fire['Y'])))
        rules.append(ctrl.Rule(bullet_time['M'] & theta_delta['PS'], (ship_turn['PS'], ship_fire['Y'])))
        rules.append(ctrl.Rule(bullet_time['M'] & theta_delta['PM'], (ship_turn['PM'], ship_fire['N'])))
        rules.append(ctrl.Rule(bullet_time['M'] & theta_delta['PL'], (ship_turn['PL'], ship_fire['N'])))

        rules.append(ctrl.Rule(bullet_time['S'] & theta_delta['NL'], (ship_turn['NL'], ship_fire['Y'])))
        rules.append(ctrl.Rule(bullet_time['S'] & theta_delta['NM'], (ship_turn['NM'], ship_fire['Y'])))
        rules.append(ctrl.Rule(bullet_time['S'] & theta_delta['NS'], (ship_turn['NS'], ship_fire['Y'])))
        rules.append(ctrl.Rule(bullet_time['S'] & theta_delta['PS'], (ship_turn['PS'], ship_fire['Y'])))
        rules.append(ctrl.Rule(bullet_time['S'] & theta_delta['PM'], (ship_turn['PM'], ship_fire['Y'])))
        rules.append(ctrl.Rule(bullet_time['S'] & theta_delta['PL'], (ship_turn['PL'], ship_fire['Y'])))

        self.targeting_control = ctrl.ControlSystem(rules)

        distance = ctrl.Antecedent(np.arange(0, 1500, 10), 'distance')
        ship_thrust = ctrl.Consequent(np.arange(-1000, 1000, 10), 'ship_thrust')

        distance['NEAR'] = fuzz.trimf(distance.universe, [0, 0, 250])
        distance['MID'] = fuzz.trimf(distance.universe, [100, 600, 1000])
        distance['FAR'] = fuzz.smf(distance.universe, 200, 1500)

        ship_thrust['LOW'] = fuzz.trimf(ship_thrust.universe, [-700, -10, -10])
        ship_thrust['MED'] = fuzz.trimf(ship_thrust.universe, [0, 0, 400])
        ship_thrust['HIGH'] = fuzz.trimf(ship_thrust.universe, [200, 1000, 1000])

        t_rules = [
            ctrl.Rule(distance['FAR'], ship_thrust['HIGH']),
            ctrl.Rule(distance['MID'], ship_thrust['MED']),
            ctrl.Rule(distance['NEAR'], ship_thrust['LOW'])
        ]
        self.motion_control = ctrl.ControlSystem(t_rules)



    def actions(self, ship_state, game_state):

        ship_pos_x = ship_state["position"][0]
        ship_pos_y = ship_state["position"][1]
        ship_heading_rad = math.radians(ship_state["heading"])

        closest = None
        for a in game_state["asteroids"]:
            dist = math.hypot(ship_pos_x - a["position"][0],
                              ship_pos_y - a["position"][1])
            if closest is None or dist < closest["dist"]:
                closest = {"aster": a, "dist": dist}

        if closest is None:
            return 0.0, 0.0, False, False

        dx_a = closest["aster"]["position"][0] - ship_pos_x
        dy_a = closest["aster"]["position"][1] - ship_pos_y
        angle_to_asteroid = math.atan2(dy_a, dx_a)
        rel_angle = (angle_to_asteroid - ship_heading_rad + math.pi) % (2*math.pi) - math.pi

        ax = closest["aster"]["position"][0]
        ay = closest["aster"]["position"][1]
        vx = closest["aster"]["velocity"][0]
        vy = closest["aster"]["velocity"][1]

        rel_x = ship_pos_x - ax
        rel_y = ship_pos_y - ay
        asteroid_dist = closest["dist"]

        asteroid_vel = math.hypot(vx, vy)
        bullet_speed = 800

        asteroid_dir = math.atan2(vy, vx)
        rel_dir = math.atan2(rel_y, rel_x)
        cos_term = math.cos(rel_dir - asteroid_dir)

        disc = (-2 * asteroid_dist * asteroid_vel * cos_term)**2 - \
               (4 * (asteroid_vel**2 - bullet_speed**2) * asteroid_dist**2)

        if disc < 0:
            bullet_t = 0.1
        else:
            sqrt_disc = math.sqrt(disc)
            t1 = ((2*asteroid_dist*asteroid_vel*cos_term) + sqrt_disc) / (2*(asteroid_vel**2 - bullet_speed**2))
            t2 = ((2*asteroid_dist*asteroid_vel*cos_term) - sqrt_disc) / (2*(asteroid_vel**2 - bullet_speed**2))
            ts = [t for t in (t1, t2) if t >= 0]
            bullet_t = min(ts) if ts else max(t1, t2)

        intr_x = ax + vx * (bullet_t + 1/30)
        intr_y = ay + vy * (bullet_t + 1/30)

        shooting_theta = math.atan2(intr_y - ship_pos_y, intr_x - ship_pos_x)
        shooting_theta = shooting_theta - ship_heading_rad
        shooting_theta = (shooting_theta + math.pi) % (2*math.pi) - math.pi

        shoot_sim = ctrl.ControlSystemSimulation(self.targeting_control, flush_after_run=1)
        shoot_sim.input['bullet_time'] = bullet_t
        shoot_sim.input['theta_delta'] = shooting_theta
        shoot_sim.compute()

        turn_rate = shoot_sim.output['ship_turn'] * 3
        turn_rate = float(max(-180, min(180, turn_rate)))

        fire = bool(shoot_sim.output['ship_fire'] >= 0)


        motion_sim = ctrl.ControlSystemSimulation(self.motion_control, flush_after_run=1)
        motion_sim.input['distance'] = asteroid_dist
        motion_sim.compute()
        thrust = float(motion_sim.output['ship_thrust'])


        drop_mine = False

        if asteroid_dist < 500:                                
            if abs(rel_angle) > math.radians(120):              
                if thrust > 200:                                
                    if self.eval_frames % 12 == 0:              
                        drop_mine = True

        drop_mine = bool(drop_mine)

        self.eval_frames += 1

        return thrust, turn_rate, fire, drop_mine


    @property
    def name(self):
        return "ProjController"
