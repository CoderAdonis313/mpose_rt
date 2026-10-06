# CONFIGS TUNED FOR FULL HD
OUT_RES = (1920, 1080)
FPS = 15
IOU = 0.25
MPOSE_BATCH_SIZE = 32
N_WORKERS = 12
N_POINTS = 1000

COLOR_RANGES = {
    'bot1_marker': ((165, 80, 70), (179, 255, 255)),
    # 'bot2_marker': ((25, 135, 0), (35, 255, 255),),
    # 'bot3_marker': ((103, 133, 0), (115, 255, 255)),
    # 'arena1_marker': ((5, 100, 65), (17, 255, 255)),
    'arena1_marker': ((65, 139, 0), (89, 255, 255)),
}

MESH_PATHS = {
    'bot1_marker': 'models/PointerActual.obj',
    # 'bot2_marker': 'models/PointerActual.obj',
    # 'bot3_marker': 'models/PointerActual.obj',
    # 'arena1_marker': 'models/ArenaMarker.obj',
    'arena1_marker': 'models/ArenaMarker.obj'
}

