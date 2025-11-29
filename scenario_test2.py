import time
from kesslergame import Scenario, KesslerGame, GraphicsType
from test_controller import TestController
from projcontroller_backup import ProjController2point0
from projcontroller import ProjController      # <-- your project controller
from graphics_both import GraphicsBoth

# --- Define Scenario ---
my_test_scenario = Scenario(
    name='Test Scenario',
    num_asteroids=5,
    ship_states=[
        # Simple test controller (Team 1)
        {'position': (400, 400), 'angle': 90, 'lives': 3, 'team': 1, 'mines_remaining': 10},

        # Scott Dick controller (Team 2)
        {'position': (600, 400), 'angle': 90, 'lives': 3, 'team': 2, 'mines_remaining': 10},
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
        ProjController2point0(),          # Team 1
        ProjController()           # Us
    ]
)

# --- Print Results ---
print("Scenario eval time:", time.perf_counter() - pre)
print("Stop reason:", score.stop_reason)
print("Asteroids hit:", [team.asteroids_hit for team in score.teams])
print("Deaths:", [team.deaths for team in score.teams])
print("Accuracy:", [team.accuracy for team in score.teams])
print("Mean eval time:", [team.mean_eval_time for team in score.teams])
print("Evaluated frames:", [controller.eval_frames for controller in score.final_controllers])
