def generate_single_luminaire(position, base_rad_file):
    '''Gera string no formato RADIANCE para uma luminária'''
        
    content = f"""
    !xform -t {position['x']} {position['y']} {position['z']} {base_rad_file}
    """
    return content

