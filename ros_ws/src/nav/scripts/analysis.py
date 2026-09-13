import json
import numpy as np
import matplotlib.pyplot as plt

def load_json_data(filepath):
    with open(filepath, 'r') as f:
        return json.load(f)

def calculate_metrics(gt_x, gt_y, est_x, est_y):
    # Erros instantâneos por eixo
    err_x = est_x - gt_x
    err_y = est_y - gt_y
    
    # Erro euclidiano (distância linear absoluta)
    err_euclidean = np.sqrt(err_x**2 + err_y**2)
    
    metrics = {
        "MAE_X": np.mean(np.abs(err_x)),
        "MAE_Y": np.mean(np.abs(err_y)),
        "MAE_Euclidean": np.mean(err_euclidean),
        "RMSE_X": np.sqrt(np.mean(err_x**2)),
        "RMSE_Y": np.sqrt(np.mean(err_y**2)),
        "RMSE_Euclidean": np.sqrt(np.mean(err_euclidean**2)),
        "MAX_Error": np.max(err_euclidean)
    }
    return metrics, err_euclidean

# 1. Carregar dados dos arquivos salvos
try:
    gt_data = load_json_data('gt_gazebo.json')
    est_data = load_json_data('position.json')
except FileNotFoundError as e:
    print(f"Erro: Arquivo não encontrado. Garanta que rodou o script de coleta primeiro. [{e}]")
    exit()

# 2. Extrair vetores numéricos (garantindo alinhamento)
time = np.array([d['t'] for d in gt_data])
time = time - time[0] # Normalizar tempo iniciando em 0s

gt_x = np.array([d['x'] for d in    gt_data])
gt_y = np.array([d['y'] for d in gt_data])

est_x = np.array([d['x'] for d in est_data])
est_y = np.array([d['y'] for d in est_data])

# 3. Calcular Métricas de Erro
metrics, err_euclidean = calculate_metrics(gt_x, gt_y, est_x, est_y)

print("\n================ MÉTRICAS DE ERRO COM VLP + EKF ================")
print(f"Erro Máximo Absoluto (Max Error):  {metrics['MAX_Error']:.4f} m")
print(f"Erro Médio Absoluto (MAE) 2D:      {metrics['MAE_Euclidean']:.4f} m")
print(f"Erro Quadrático Médio (RMSE) 2D:   {metrics['RMSE_Euclidean']:.4f} m")
print("-----------------------------------------------------------------")
print(f"MAE Eixo X: {metrics['MAE_X']:.4f} m  |  RMSE Eixo X: {metrics['RMSE_X']:.4f} m")
print(f"MAE Eixo Y: {metrics['MAE_Y']:.4f} m  |  RMSE Eixo Y: {metrics['RMSE_Y']:.4f} m")
print("=================================================================\n")

# 4. Configuração dos Gráficos (Janela Dupla)
fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))

# Gráfico 1: Comparação de Trajetória 2D
ax1.plot(gt_x, gt_y, color='tab:orange', linestyle='--', linewidth=2, label='Ground Truth')
ax1.plot(est_x, est_y, color='tab:blue', linewidth=1.5, label='Estimativa')
ax1.scatter(gt_x[0], gt_y[0], color='green', marker='o', s=100, label='Início', zorder=5)
ax1.scatter(gt_x[-1], gt_y[-1], color='red', marker='X', s=100, label='Fim', zorder=5)
ax1.set_xlabel('X [m]')
ax1.set_ylabel('Y [m]')
ax1.set_aspect('equal', adjustable='box')
ax1.grid(True, alpha=0.3)
ax1.legend()

# Gráfico 2: Evolução do Erro Euclidiano no Tempo
ax2.plot(time, err_euclidean, color='tab:red', linewidth=1.5)
ax2.axhline(metrics['MAE_Euclidean'], color='black', linestyle=':', label=f"MAE ({metrics['MAE_Euclidean']:.3f}m)")
ax2.set_xlabel('Tempo [s]')
ax2.set_ylabel('MAE [m]')
ax2.grid(True, alpha=0.3)
ax2.legend()

plt.tight_layout()
plt.show()