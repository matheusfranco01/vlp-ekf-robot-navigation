import numpy as np

from estimator.luminary import generate_single_luminaire
from estimator.utils import run_oconv, run_rtrace


def single_static_iluminance(path:str, wall_bounces:int, point:dict, pos:dict, base_lum_rad:str, idx:int) -> list:
    '''Calcula iluminância estática para uma luminária para cada ponto do grid'''
    
    lum_content = generate_single_luminaire(pos, base_lum_rad)
    cur_lum_path = f"{path}objects/current_lum.rad"
    oct_path = f"{path}output/scene_lum_{idx}.oct"
    room_path = f"{path}ambient/full_room.rad"
    
    with open(cur_lum_path, "w") as f:
        f.write(lum_content)

    # Compila Octree
    run_oconv(room_path, cur_lum_path, oct_path)

    # Roda rtrace
    lux_values = run_rtrace(oct_path, point, wall_bounces)

    return lux_values

def dynamic_illuminance(path:str, luminaries:dict, wall_bounces:int, 
                        current_time:float, point:dict) -> float:
    '''Calcula a iluminância instantânea no tempo atual para todas as luminárias.'''
    positions = luminaries['positions']
    frequencies = luminaries['modulation_frequencies']

    # Inicializa a iluminância acumulada como escalar.
    total_ilu = 0.0

    for i in range(len(positions)):
        
        # Iluminância estática para a luminária i
        ilu_static = np.asarray(
            single_static_iluminance(
                path, wall_bounces, point, positions[i],
                f"{path}objects/luminaire.rad", i
            ),
            dtype=float,
        ).reshape(-1)

        freq = frequencies[i]

        # Cálculo da iluminância dinâmica para a frequência i no tempo atual.
        tempo_mod = 1 + np.sign(np.sin(2 * np.pi * freq * current_time))
        ilu_dyn = ilu_static * tempo_mod

        total_ilu += float(np.sum(ilu_dyn))

    return float(total_ilu)


def dynamic_illuminance_window(path: str, luminaries: dict, wall_bounces: int,
                               start_time: float, point: dict,
                               sample_frequency: float, window_duration: float) -> np.ndarray:
    '''Calcula uma janela de iluminância dinâmica com amostragem uniforme.'''
    positions = luminaries['positions']
    frequencies = luminaries['modulation_frequencies']

    if sample_frequency is None or sample_frequency <= 0:
        raise ValueError('sample_frequency must be a positive number')

    sample_count = int(round(window_duration * sample_frequency))
    if sample_count <= 0:
        sample_count = 1

    sample_offsets = np.arange(sample_count, dtype=float) / sample_frequency
    sample_times = start_time + sample_offsets

    total_ilu = np.zeros(sample_count, dtype=float)

    for i in range(len(positions)):
        ilu_static = np.asarray(
            single_static_iluminance(
                path, wall_bounces, point, positions[i],
                f"{path}objects/luminaire.rad", i
            ),
            dtype=float,
        ).reshape(-1)

        static_illuminance = float(np.sum(ilu_static))
        freq = frequencies[i]
        tempo_mod = 1 + np.sign(np.sin(2 * np.pi * freq * sample_times))
        total_ilu += static_illuminance * tempo_mod

    return total_ilu
    