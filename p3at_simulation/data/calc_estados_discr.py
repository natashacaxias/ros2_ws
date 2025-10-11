import numpy as np

def calc_estados_discr(pose_robo, faixas_x, faixas_y):
    """
    Calcula e discretiza o estado do robô.

    Parâmetros:
        pose_robo: array-like [x, y, theta] (no sistema do mundo)
        faixas_x: array-like -> limites para discretização em x
        faixas_y: array-like -> limites para discretização em y

    Retorna:
        estado_discreto_idx: tuple (idx_x, idx_y)
        erros_norm: np.array [erro_x_norm, erro_y_norm]
        dist_alvo: float -> distância até o objetivo
    """
    # Considera apenas x e y do pose (theta não usado aqui)
    erro_bruto = np.array([pose_robo[0], pose_robo[1]])

    # Distância até o alvo
    dist_alvo = np.sqrt(erro_bruto[0]**2 + erro_bruto[1]**2)

    # Evita divisão por zero
    if dist_alvo < 0.01:
        erros_norm = np.array([0.0, 0.0])
    else:
        erros_norm = erro_bruto / dist_alvo

    # Discretização dos erros normalizados
    idx_x = np.sum(erros_norm[0] > faixas_x) + 1
    idx_y = np.sum(erros_norm[1] > faixas_y) + 1

    estado_discreto_idx = (int(idx_x), int(idx_y))

    return estado_discreto_idx, erros_norm, dist_alvo
