import time
from kesslergame import Scenario, KesslerGame, GraphicsType
from test_controller import TestController
from scott_dick_controller import ScottDickController
from projcontroller import ProjController      # <-- your project controller
from graphics_both import GraphicsBoth
from GA_group6_controller import GACompController

# --- Define Scenario ---
my_test_scenario = Scenario(
    name='Test Scenario',
    num_asteroids=16,
    ship_states=[
        # Simple test controller (Team 1)
        {'position': (400, 400), 'angle': 90, 'lives': 3, 'team': 1},

        # Scott Dick controller (Team 2)
        {'position': (600, 400), 'angle': 90, 'lives': 3, 'team': 2},

        # Your project controller (Team 3)
        # You can tweak starting position / angle as you like
        #{'position': (500, 200), 'angle': 90, 'lives': 3, 'team': 3},
    ],
    map_size=(1000, 800),
    time_limit=60,
    ammo_limit_multiplier=0,
    stop_if_no_ammo=False
)

# --- Game Settings ---
game_settings = {
    'perf_tracker': True,
    'graphics_type': GraphicsType.Tkinter,  # Graphical mode
    'realtime_multiplier': 1,
    'graphics_obj': None
}

# Use this for visualization:
game = KesslerGame(settings=game_settings)

# For training-only (no graphics), use this instead:
# from kesslergame import TrainerEnvironment
# game = TrainerEnvironment(settings=game_settings)

# --- Run Game ---
pre = time.perf_counter()
score, perf_data = game.run(
    scenario=my_test_scenario,
    controllers=[
        ScottDickController(),
        GACompController(),          # Team 1
        #ProjController()           # Us
    ]
)

# --- Print Results ---
print("Scenario eval time:", time.perf_counter() - pre)
print("Stop reason:", score.stop_reason)
print("Asteroids hit:", [team.asteroids_hit for team in score.teams])
print("Deaths:", [team.deaths for team in score.teams])
print("Accuracy:", [team.accuracy for team in score.teams])
print("Mean eval time:", [team.mean_eval_time for team in score.teams])
# print("Evaluated frames:", [controller.eval_frames for controller in score.final_controllers])
