import math
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
from kesslergame import KesslerController, Scenario, KesslerGame, GraphicsType, TrainerEnvironment
import random
import EasyGA

#Our code uses to function to run GA. To rerun the GA, simply change run_GA to True, and a new best_chromosome will be found.
run_GA = False

def gene_generation():
    return sorted([random.random(), random.random(), random.random()])

def setup_controller_from_chromosome(chromosome):
    return GACompController(chromosome=chromosome)

def fitness(chromosome):
    controller = setup_controller_from_chromosome(chromosome)
    total = 0
    for _ in range(1):
        scenario = Scenario(
            name="GA Eval",
            num_asteroids=5,
            ship_states=[{'position': (400, 400), 'angle': 90, 'lives': 3, 'team': 1}],
            map_size=(1000, 800),
            time_limit=5,
            ammo_limit_multiplier=0,
            stop_if_no_ammo=False
        )
        settings = {
            'perf_tracker': False,
            'graphics_type': None,
            'realtime_multiplier': 0,
            'graphics_obj': None
        }
        game = TrainerEnvironment(settings=settings)
        score, _ = game.run(scenario=scenario, controllers=[controller])
        team = score.teams[0]
        s = team.asteroids_hit * 200 - team.deaths * 300 + team.accuracy * 20
        total += s
    return total

def run_ga(population_size, generations, gene_creator=gene_generation):
    ga = EasyGA.GA()
    ga.gene_impl = lambda: gene_creator()
    ga.chromosome_length = 3
    ga.population_size = population_size
    ga.generation_goal = generations
    ga.target_fitness_type = 'max'
    ga.fitness_function_impl = fitness
    ga.evolve()
    return [gene.value for gene in ga.population[0].gene_list]

class GACompController(KesslerController):

    def __init__(self, chromosome=None):
        if chromosome is None:
            chromosome = best_chromosome

        # Normalize EasyGA chromosome → list of lists
        if hasattr(chromosome, "gene_list"):
            chromosome = [g.value for g in chromosome.gene_list]
        elif hasattr(chromosome[0], "value"):
            chromosome = [g.value for g in chromosome]

        self.eval_frames = 0
        self.chromosome = chromosome

        bullet_time = ctrl.Antecedent(np.arange(0, 1.0, 0.002), 'bullet_time')
        theta_delta = ctrl.Antecedent(np.arange(-1 * math.pi/30, math.pi/30, 0.1), 'theta_delta')
        ship_turn = ctrl.Consequent(np.arange(-180, 180, 1), 'ship_turn')
        ship_fire = ctrl.Consequent(np.arange(-1, 1, 0.1), 'ship_fire')
        distance = ctrl.Antecedent(np.arange(0, 1500, 10), 'distance')
        ship_thrust = ctrl.Consequent(np.arange(-1000, 1000, 10), 'ship_thrust')

        bt1, bt2, bt3 = chromosome[0]
        td1, td2, td3 = chromosome[1]
        d1, d2, d3 = chromosome[2]
        bullet_time['S'] = fuzz.trimf(bullet_time.universe, [0, 0, bt1])
        bullet_time['M'] = fuzz.trimf(bullet_time.universe, [bt1, bt2, bt3])
        bullet_time['L'] = fuzz.smf(bullet_time.universe, bt2, 1.0)

        td1, td2, td3 = chromosome[1]
        td_min, td_max = -math.pi/30, math.pi/30
        a = td_min + td1 * (td_max - td_min)
        b = td_min + td2 * (td_max - td_min)
        c = td_min + td3 * (td_max - td_min)
        a, b, c = sorted([a, b, c])
        mid1 = (a + b) / 2
        mid2 = (b + c) / 2
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

        d1, d2, d3 = chromosome[2]
        d1 = max(0.1, d1)
        d2 = max(d1 + 0.1, d2)
        d3 = max(d2 + 0.1, d3)
        max_dist = 1500
        distance['NEAR'] = fuzz.trimf(distance.universe, [0, 0, max_dist * d1])
        distance['MID'] = fuzz.trimf(distance.universe, [d1 * max_dist, d2 * max_dist, d3 * max_dist])
        distance['FAR'] = fuzz.smf(distance.universe, d2 * max_dist, max_dist)

        ship_thrust['LOW'] = fuzz.trimf(ship_thrust.universe, [-700, -10, -10])
        ship_thrust['MED'] = fuzz.trimf(ship_thrust.universe, [0, 0, 400])
        ship_thrust['HIGH'] = fuzz.trimf(ship_thrust.universe, [200, 1000, 1000])

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
            dist = math.hypot(ship_pos_x - a["position"][0], ship_pos_y - a["position"][1])
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

        disc = (-2 * asteroid_dist * asteroid_vel * cos_term)**2 - (4 * (asteroid_vel**2 - bullet_speed**2) * asteroid_dist**2)
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
        fire = bool(shoot_sim.output['ship_fire'] >= -0.5)

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

        self.eval_frames += 1
        return thrust, turn_rate, fire, drop_mine

    @property
    def name(self):
        return "GAProjController"

print("Running Genetic Algorithm to find best chromosome...")

if run_GA:
    best_chromosome = run_ga(6, 3)
else:
    best_chromosome = [[0.006235545286981448, 0.2694790171799788, 0.40899082366816064], [0.0031061974781771973, 0.1412197337211607, 0.9603497921879489], [0.08359518852579406, 0.21191703216981628, 0.7117541988238603]]

print("Finished GA. Best chromosome:", best_chromosome)