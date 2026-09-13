import os
import subprocess

radiance_env = os.environ.copy()
radiance_env["RAYPATH"] = ".:/usr/local/lib/ray"

def wall_material(type:str, color_rgb:tuple, 
                      specularity: float, roughness: float) -> str:
    '''Gera string no formato RADIANCE para material da parede'''
    
    mat = f"""
    void {type} wall_mat
    0
    0
    5 {color_rgb[0]} {color_rgb[1]} {color_rgb[2]} {specularity} {roughness}
    """

    return mat

def generate_room(dimensions:dict, type:str, color_rgb:tuple,
                  specularity: float, roughness: float, path:str) -> str:
    
    '''Gera string no formato RADIANCE para o ambiente'''

    mat = wall_material(
        type=type,
        color_rgb=color_rgb,
        specularity=specularity,
        roughness=roughness
    )

    # Salva o material
    mat_path = path + "/materials/wall_mat.mat"
    mat_lines = [line.lstrip() for line in mat.strip().splitlines()]
    with open(mat_path, "w", encoding="utf-8") as f:
        f.write("\n".join(mat_lines) + "\n")

    output_path = f"{path}/ambient/room.rad"
    
    cmd = [
        "genbox",
        "wall_mat",
        "room",
        str(dimensions['x']),
        str(dimensions['y']),
        str(dimensions['z'])
    ]

    cmd_ = ["xform", "-I"]

    with open(output_path, "w") as f:
        p1 = subprocess.Popen(cmd, stdout=subprocess.PIPE, env=radiance_env)
        p2 = subprocess.Popen(cmd_, stdin=p1.stdout, stdout=f, env=radiance_env)
        
        p1.stdout.close()
        p2.communicate() 
    
    full_room_path = f"{path}ambient/full_room.rad"
    with open(full_room_path, 'w') as f:
        f.write(f"!xform {path}materials/wall_mat.mat\n")
        f.write(f"!xform {output_path}\n")
        
    return full_room_path

def grid_points(dimensions: dict, divisions:int, floor_level:float, origin: dict = None) -> list:
    '''Gera lista de coordenadas dos pontos do grid'''
    sx, sy = dimensions['x'], dimensions['y']
    ox = origin.get('x', 0.0) if origin else 0.0
    oy = origin.get('y', 0.0) if origin else 0.0
    step_x = sx / divisions
    step_y = sy / divisions
    
    points = []
    
    # Gera os pontos de cada eixo
    y_coords = [oy + k * step_y for k in range(divisions + 1)]
    x_coords = [ox + k * step_x for k in range(divisions + 1)]
    
    points = []
    
    # Combina os eixos para formar os pontos (x,y)
    for x in x_coords:
        for y in y_coords:
            points.append({
                'x': x, 
                'y': y, 
                'z': floor_level
            })
            
    return points