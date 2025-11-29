# MODIFIED SCOTT_DICK_CONTROLLER
from pickle import FALSE
from kesslergame import KesslerController # In Eclipse, the name of the library is kesslergame, not src.kesslergame
from typing import Dict, Tuple
from cmath import sqrt
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import math
import numpy as np
import matplotlib as plt




class ProjController2point0(KesslerController):

    def __init__(self):
        self.eval_frames = 0 

        # self.targeting_control is the targeting rulebase, which is static in this controller.      
        # Declare variables
        bullet_time = ctrl.Antecedent(np.arange(0,1.0,0.002), 'bullet_time')
        theta_delta = ctrl.Antecedent(np.arange(-1*math.pi/30,math.pi/30,0.1), 'theta_delta') # Radians due to Python
        ship_turn = ctrl.Consequent(np.arange(-180,180,1), 'ship_turn') # Degrees due to Kessler
        ship_fire = ctrl.Consequent(np.arange(-1,1,0.1), 'ship_fire')
        
        #Declare fuzzy sets for bullet_time (how long it takes for the bullet to reach the intercept point)
        bullet_time['S'] = fuzz.trimf(bullet_time.universe,[0,0,0.05])
        bullet_time['M'] = fuzz.trimf(bullet_time.universe, [0,0.05,0.1])
        bullet_time['L'] = fuzz.smf(bullet_time.universe,0.0,0.1)
        
        # Declare fuzzy sets for theta_delta (degrees of turn needed to reach the calculated firing angle)
        # Hard-coded for a game step of 1/30 seconds
        theta_delta['NL'] = fuzz.zmf(theta_delta.universe, -1*math.pi/30,-2*math.pi/90)
        theta_delta['NM'] = fuzz.trimf(theta_delta.universe, [-1*math.pi/30, -2*math.pi/90, -1*math.pi/90])
        theta_delta['NS'] = fuzz.trimf(theta_delta.universe, [-2*math.pi/90,-1*math.pi/90,math.pi/90])
        # theta_delta['Z'] = fuzz.trimf(theta_delta.universe, [-1*math.pi/90,0,math.pi/90])
        theta_delta['PS'] = fuzz.trimf(theta_delta.universe, [-1*math.pi/90,math.pi/90,2*math.pi/90])
        theta_delta['PM'] = fuzz.trimf(theta_delta.universe, [math.pi/90,2*math.pi/90, math.pi/30])
        theta_delta['PL'] = fuzz.smf(theta_delta.universe,2*math.pi/90,math.pi/30)
        
        # Declare fuzzy sets for the ship_turn consequent; this will be returned as turn_rate.
        # Hard-coded for a game step of 1/30 seconds
        ship_turn['NL'] = fuzz.trimf(ship_turn.universe, [-180,-180,-120])
        ship_turn['NM'] = fuzz.trimf(ship_turn.universe, [-180,-120,-60])
        ship_turn['NS'] = fuzz.trimf(ship_turn.universe, [-120,-60,60])
        # ship_turn['Z'] = fuzz.trimf(ship_turn.universe, [-60,0,60])
        ship_turn['PS'] = fuzz.trimf(ship_turn.universe, [-60,60,120])
        ship_turn['PM'] = fuzz.trimf(ship_turn.universe, [60,120,180])
        ship_turn['PL'] = fuzz.trimf(ship_turn.universe, [120,180,180])
        
        #Declare singleton fuzzy sets for the ship_fire consequent; -1 -> don't fire, +1 -> fire; this will be  thresholded
        #   and returned as the boolean 'fire'
        ship_fire['N'] = fuzz.trimf(ship_fire.universe, [-1, -1, -0.9])
        ship_fire['Y'] = fuzz.trimf(ship_fire.universe, [-0.5, 1, 1])

                
        #Declare each fuzzy rule
        rule1 = ctrl.Rule(bullet_time['L'] & theta_delta['NL'], (ship_turn['NL'], ship_fire['N']))
        rule2 = ctrl.Rule(bullet_time['L'] & theta_delta['NM'], (ship_turn['NM'], ship_fire['N']))
        rule3 = ctrl.Rule(bullet_time['L'] & theta_delta['NS'], (ship_turn['NS'], ship_fire['Y']))
        # rule4 = ctrl.Rule(bullet_time['L'] & theta_delta['Z'], (ship_turn['Z'], ship_fire['Y']))
        rule5 = ctrl.Rule(bullet_time['L'] & theta_delta['PS'], (ship_turn['PS'], ship_fire['Y']))
        rule6 = ctrl.Rule(bullet_time['L'] & theta_delta['PM'], (ship_turn['PM'], ship_fire['N']))
        rule7 = ctrl.Rule(bullet_time['L'] & theta_delta['PL'], (ship_turn['PL'], ship_fire['N']))
        rule8 = ctrl.Rule(bullet_time['M'] & theta_delta['NL'], (ship_turn['NL'], ship_fire['N']))
        rule9 = ctrl.Rule(bullet_time['M'] & theta_delta['NM'], (ship_turn['NM'], ship_fire['N']))
        rule10 = ctrl.Rule(bullet_time['M'] & theta_delta['NS'], (ship_turn['NS'], ship_fire['Y']))
        # rule11 = ctrl.Rule(bullet_time['M'] & theta_delta['Z'], (ship_turn['Z'], ship_fire['Y']))
        rule12 = ctrl.Rule(bullet_time['M'] & theta_delta['PS'], (ship_turn['PS'], ship_fire['Y']))
        rule13 = ctrl.Rule(bullet_time['M'] & theta_delta['PM'], (ship_turn['PM'], ship_fire['N']))
        rule14 = ctrl.Rule(bullet_time['M'] & theta_delta['PL'], (ship_turn['PL'], ship_fire['N']))
        rule15 = ctrl.Rule(bullet_time['S'] & theta_delta['NL'], (ship_turn['NL'], ship_fire['Y']))
        rule16 = ctrl.Rule(bullet_time['S'] & theta_delta['NM'], (ship_turn['NM'], ship_fire['Y']))
        rule17 = ctrl.Rule(bullet_time['S'] & theta_delta['NS'], (ship_turn['NS'], ship_fire['Y']))
        # rule18 = ctrl.Rule(bullet_time['S'] & theta_delta['Z'], (ship_turn['Z'], ship_fire['Y']))
        rule19 = ctrl.Rule(bullet_time['S'] & theta_delta['PS'], (ship_turn['PS'], ship_fire['Y']))
        rule20 = ctrl.Rule(bullet_time['S'] & theta_delta['PM'], (ship_turn['PM'], ship_fire['Y']))
        rule21 = ctrl.Rule(bullet_time['S'] & theta_delta['PL'], (ship_turn['PL'], ship_fire['Y']))
   
      
             
        self.targeting_control = ctrl.ControlSystem()
        self.targeting_control.addrule(rule1)
        self.targeting_control.addrule(rule2)
        self.targeting_control.addrule(rule3)
        # self.targeting_control.addrule(rule4)
        self.targeting_control.addrule(rule5)
        self.targeting_control.addrule(rule6)
        self.targeting_control.addrule(rule7)
        self.targeting_control.addrule(rule8)
        self.targeting_control.addrule(rule9)
        self.targeting_control.addrule(rule10)
        # self.targeting_control.addrule(rule11)
        self.targeting_control.addrule(rule12)
        self.targeting_control.addrule(rule13)
        self.targeting_control.addrule(rule14)
        self.targeting_control.addrule(rule15)
        self.targeting_control.addrule(rule16)
        self.targeting_control.addrule(rule17)
        
        self.targeting_control.addrule(rule19)
        self.targeting_control.addrule(rule20)
        self.targeting_control.addrule(rule21)


        distance = ctrl.Antecedent(np.arange(0, 1500, 10), 'distance')
        ship_thrust = ctrl.Consequent(np.arange(-1000, 1000, 10), 'ship_thrust')


        # TRUST CONTROL
        distance['NEAR'] = fuzz.trimf(distance.universe, [0, 0, 150])
        distance['MID']  = fuzz.trimf(distance.universe, [100, 600, 1000])
        distance['FAR']  = fuzz.smf(distance.universe, 200, 1500)

        ship_distance = ctrl.Antecedent(np.arange(0, 2000, 10), 'ship_distance')
        ship_avoid_turn = ctrl.Consequent(np.arange(-180, 180, 1), 'ship_avoid_turn')

        ship_distance['DANGER'] = fuzz.trimf(ship_distance.universe, [0, 0, 300])
        ship_distance['CAUTION'] = fuzz.trimf(ship_distance.universe, [200, 600, 1000])
        ship_distance['CLEAR'] = fuzz.smf(ship_distance.universe, 800, 1500)

        ship_avoid_turn['LEFT']  = fuzz.trimf(ship_avoid_turn.universe, [-180, -120, -60])
        ship_avoid_turn['NONE']  = fuzz.trimf(ship_avoid_turn.universe, [-20, 0, 20])
        ship_avoid_turn['RIGHT'] = fuzz.trimf(ship_avoid_turn.universe, [60, 120, 180])


        ship_thrust['LOW']  = fuzz.trimf(ship_thrust.universe, [-450, -50, -10])  
        ship_thrust['MED']  = fuzz.trimf(ship_thrust.universe, [0, 0, 400])       
        ship_thrust['HIGH'] = fuzz.trimf(ship_thrust.universe, [200, 1000, 1000])     


        t_rule1 = ctrl.Rule(distance['FAR'],  ship_thrust['HIGH'])  
        t_rule2 = ctrl.Rule(distance['MID'],  ship_thrust['MED'])   
        t_rule3 = ctrl.Rule(distance['NEAR'], ship_thrust['LOW']) 


        self.motion_control = ctrl.ControlSystem([t_rule1, t_rule2, t_rule3])

        avoid_rule1 = ctrl.Rule(ship_distance['DANGER'], ship_avoid_turn['LEFT'])
        avoid_rule2 = ctrl.Rule(ship_distance['CAUTION'], ship_avoid_turn['NONE'])
        avoid_rule3 = ctrl.Rule(ship_distance['CLEAR'], ship_avoid_turn['NONE'])

        self.avoid_control = ctrl.ControlSystem([avoid_rule1, avoid_rule2, avoid_rule3])


        
        

    def actions(self, ship_state: Dict, game_state: Dict) -> Tuple[float, float, bool, bool]:

        ship_pos_x = ship_state["position"][0]     
        ship_pos_y = ship_state["position"][1]       
        closest_asteroid = None
        
        for a in game_state["asteroids"]:
            
            curr_dist = math.sqrt((ship_pos_x - a["position"][0])**2 + (ship_pos_y - a["position"][1])**2)
            if closest_asteroid is None :
                
                closest_asteroid = dict(aster = a, dist = curr_dist)
                
            else:    
               
                if closest_asteroid["dist"] > curr_dist:
                    
                    closest_asteroid["aster"] = a
                    closest_asteroid["dist"] = curr_dist
        
        
        asteroid_ship_x = ship_pos_x - closest_asteroid["aster"]["position"][0]
        asteroid_ship_y = ship_pos_y - closest_asteroid["aster"]["position"][1]
        
        asteroid_ship_theta = math.atan2(asteroid_ship_y,asteroid_ship_x)
        
        asteroid_direction = math.atan2(closest_asteroid["aster"]["velocity"][1], closest_asteroid["aster"]["velocity"][0]) # Velocity is a 2-element array [vx,vy].
        my_theta2 = asteroid_ship_theta - asteroid_direction
        cos_my_theta2 = math.cos(my_theta2)
        
        asteroid_vel = math.sqrt(closest_asteroid["aster"]["velocity"][0]**2 + closest_asteroid["aster"]["velocity"][1]**2)
        bullet_speed = 800 
        
        
        targ_det = (-2 * closest_asteroid["dist"] * asteroid_vel * cos_my_theta2)**2 - (4*(asteroid_vel**2 - bullet_speed**2) * (closest_asteroid["dist"]**2))
        
        
        intrcpt1 = ((2 * closest_asteroid["dist"] * asteroid_vel * cos_my_theta2) + math.sqrt(targ_det)) / (2 * (asteroid_vel**2 -bullet_speed**2))
        intrcpt2 = ((2 * closest_asteroid["dist"] * asteroid_vel * cos_my_theta2) - math.sqrt(targ_det)) / (2 * (asteroid_vel**2-bullet_speed**2))
        
       
        if intrcpt1 > intrcpt2:
            if intrcpt2 >= 0:
                bullet_t = intrcpt2
            else:
                bullet_t = intrcpt1
        else:
            if intrcpt1 >= 0:
                bullet_t = intrcpt1
            else:
                bullet_t = intrcpt2
                
        
        
        intrcpt_x = closest_asteroid["aster"]["position"][0] + closest_asteroid["aster"]["velocity"][0] * (bullet_t+1/30)
        intrcpt_y = closest_asteroid["aster"]["position"][1] + closest_asteroid["aster"]["velocity"][1] * (bullet_t+1/30)


        # --- Find closest other ship ---
        closest_ship = None
        for s in game_state["ships"]:
            # Skip the ship we are controlling
            if s is ship_state:
                continue

            d = math.dist(ship_state["position"], s["position"])
            if closest_ship is None or d < closest_ship["dist"]:
                closest_ship = dict(ship=s, dist=d)

        
        my_theta1 = math.atan2((intrcpt_y - ship_pos_y),(intrcpt_x - ship_pos_x))
        
    
        shooting_theta = my_theta1 - ((math.pi/180)*ship_state["heading"])
        
        
        shooting_theta = (shooting_theta + math.pi) % (2 * math.pi) - math.pi
        
        
        shooting = ctrl.ControlSystemSimulation(self.targeting_control,flush_after_run=1)
        
        shooting.input['bullet_time'] = bullet_t
        shooting.input['theta_delta'] = shooting_theta

        
        
        shooting.compute()
        turn_rate = shooting.output['ship_turn']
        

        if closest_ship is not None:
            avoid_sim = ctrl.ControlSystemSimulation(self.avoid_control, flush_after_run=1)
            avoid_sim.input['ship_distance'] = closest_ship["dist"]
            avoid_sim.compute()
            avoid_turn = avoid_sim.output['ship_avoid_turn']
        else:
            avoid_turn = 0

        attack_turn = shooting.output['ship_turn']

        # Add collision repulsion
        turn_rate = attack_turn * 2.5

        # smooth & clamp
        turn_rate = max(-180, min(180, turn_rate))

        
        # Get the defuzzified outputs
        # Get the defuzzified turn / fire outputs
        

        if True:#shooting.output['ship_fire'] >= 0:
            fire = True
        else:
            fire = False

       
        move_sim = ctrl.ControlSystemSimulation(self.motion_control, flush_after_run=1)
        move_sim.input['distance'] = closest_asteroid["dist"]
        move_sim.compute()
        thrust = move_sim.output['ship_thrust']

       
        drop_mine = False

        self.eval_frames += 1


   
        print(thrust, bullet_t, shooting_theta, turn_rate, fire)

        return thrust, turn_rate, fire, drop_mine


    @property
    def name(self) -> str:
        return "ProjController 2.0"