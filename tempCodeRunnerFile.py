14,
    ship_states=[
        # Simple test controller (Team 1)
        {'position': (400, 400), 'angle': 90, 'lives': 3, 'team': 1},

        # Scott Dick controller (Team 2)
        {'position': (600, 400), 'angle': 90, 'lives': 3, 'team': 2},

        # Your project controller (Team 3)
        # You can tweak starting position / angle as you like
        # {'position': (500, 200), 'angle': 90, 'lives': 3, 'team': 3},
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
        #ScottDickController(),
        GACompController(),          # Team 1