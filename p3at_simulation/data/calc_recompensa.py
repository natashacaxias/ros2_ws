import numpy as np

def calc_recompensa(erros_norm, estado_proximo_discreto, estado_discreto, dist_tolerancia):
    """
    Calcula a recompensa para o passo atual.

    Parâmetros:
        erros_norm: array-like -> [erro_x_norm, erro_y_norm]
        estado_proximo_discreto: tuple/list -> (idx_x, idx_y) do próximo estado
        estado_discreto: tuple/list -> (idx_x, idx_y) do estado atual
        dist_tolerancia: float -> distância considerada como alvo alcançado

    Retorna:
        recompensa: float -> valor da recompensa
    """
    # Penalidade de tempo
    recompensa = -1

    # Distância euclidiana ao alvo
    distancia_ao_alvo = np.sqrt(erros_norm[0]**2 + erros_norm[1]**2)

    # Grande recompensa se atingir o alvo
    if estado_proximo_discreto == (6, 6):  # célula alvo
        recompensa = 100

    # Penalidade se ficou mais longe (pela soma dos índices)
    if sum(estado_proximo_discreto) > sum(estado_discreto):
        recompensa = -5

    # (Em um ambiente real, poderia adicionar penalidade por colisão aqui)
    return recompensa
