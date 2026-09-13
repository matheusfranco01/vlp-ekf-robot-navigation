import os
import subprocess

radiance_env = os.environ.copy()
radiance_env["RAYPATH"] = ".:/usr/local/lib/ray"

def run_ies2rad(ies_path, output_path):
    '''Converte um arquivo IES para formato rad'''
    cmd = [
        "ies2rad",
        "-dm",
        "-t", "default",
        "-o", output_path,
        ies_path
    ]
    try:
        subprocess.run(cmd, check=True, env=radiance_env)
    except subprocess.CalledProcessError as e:
        print("Erro ao executar ies2rad:", e)

def run_oconv(room_file, luminaire_file, output_oct):
    '''Compila a geometria e a iluminação em um Octree (.oct)'''
    cmd = f"oconv {room_file} {luminaire_file} > {output_oct}"
    subprocess.run(cmd, shell=True, check=True, env=radiance_env)

def run_rtrace(oct_file, point, wall_bounces=0):
    '''Executa o rtrace para calcular a iluminância'''
    # Formata os pontos para entrada do rtrace (x y z dx dy dz)
    # dx=0 dy=0 dz=1 (Normal apontando para cima)
    input_str = f"{point['x']} {point['y']} {point['z']} 0 0 1\n"

    cmd = ["rtrace", "-I", "-h", "-ov", "-ab", str(wall_bounces), oct_file]
    
    process = subprocess.Popen(cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, 
                               stderr=subprocess.PIPE, text=True, env=radiance_env)
    
    stdout, stderr = process.communicate(input=input_str)
    
    if stderr:
        print(f"Aviso rtrace: {stderr}")

    illuminance_values = []
    # Processa a saída RGB e converte para Lux
    # Lux = 179 * (0.265*R + 0.670*G + 0.065*B)

    for line in stdout.strip().split('\n'):
        if line:
            try:
                r, g, b = map(float, line.split())
                lux = 179 * (0.265 * r + 0.670 * g + 0.065 * b)
                illuminance_values.append(lux)
            except ValueError:
                illuminance_values.append(0.0)
                
    return illuminance_values

def get_elapsed_time(sample_frequency: float, total_time: float, 
                     frequencies: list) -> list:
    """Gera uma lista de instantes de tempo para amostragem"""
    if sample_frequency is None:
        return [0]
    
    dt = 1 / sample_frequency
    
    if total_time is None:
        if frequencies and len(frequencies) > 0:
            freq = min(frequencies)
            t = 1 / freq
        else:
            t = dt
    else:
        t = total_time
    
    n = round(t / dt)
    tau = [k * dt for k in range(n + 1)]
    return tau